#include "TrafficManager.h"
#include "ArmyTruckSim.h"
#include "WorldGenerator.h"
#include "TrafficSignal.h"
#include "ArmyTruckPawn.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"

namespace
{
	constexpr double BigGap = 1.0e9;
}

ATrafficManager::ATrafficManager()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	VehicleClass = ATrafficVehicle::StaticClass();
}

void ATrafficManager::BeginPlay()
{
	Super::BeginPlay();
	if (!bInitialized)
	{
		if (AWorldGenerator* Gen = AWorldGenerator::Find(this))
		{
			if (!Gen->IsGenerated())
			{
				Gen->Generate();
			}
			Initialize(Gen);
		}
	}
}

void ATrafficManager::Initialize(AWorldGenerator* InGenerator)
{
	if (bInitialized || !InGenerator || InGenerator->GetLanes().Num() == 0)
	{
		return;
	}
	bInitialized = true;
	Generator = InGenerator;
	Rand.Initialize(Seed);

	UWorld* World = GetWorld();
	const FVector Avoid = Generator->GetPlayerSpawnTransform().GetLocation();
	UClass* Class = VehicleClass ? VehicleClass.Get() : ATrafficVehicle::StaticClass();

	static const FLinearColor Palette[] = {
		FLinearColor(0.6f, 0.05f, 0.04f), FLinearColor(0.05f, 0.1f, 0.35f), FLinearColor(0.7f, 0.7f, 0.7f),
		FLinearColor(0.03f, 0.03f, 0.03f), FLinearColor(0.85f, 0.85f, 0.82f), FLinearColor(0.1f, 0.3f, 0.12f),
		FLinearColor(0.75f, 0.45f, 0.05f), FLinearColor(0.35f, 0.35f, 0.38f), FLinearColor(0.3f, 0.05f, 0.25f)
	};

	FActorSpawnParameters Params;
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	Params.Owner = this;

	for (int32 i = 0; i < NumVehicles; ++i)
	{
		ATrafficVehicle* Vehicle = World->SpawnActor<ATrafficVehicle>(Class, FTransform(FVector(0.0, 0.0, -100000.0)), Params);
		if (!Vehicle)
		{
			continue;
		}
		const float KindRoll = Rand.FRand();
		const ETrafficVehicleKind Kind = KindRoll < 0.6f ? ETrafficVehicleKind::Car
			: KindRoll < 0.8f ? ETrafficVehicleKind::Van
			: KindRoll < 0.9f ? ETrafficVehicleKind::Bus
			: ETrafficVehicleKind::Truck;
		const FLinearColor Paint = Kind == ETrafficVehicleKind::Bus ? FLinearColor(0.8f, 0.55f, 0.05f) : Palette[Rand.RandRange(0, static_cast<int32>(UE_ARRAY_COUNT(Palette)) - 1)];
		Vehicle->Configure(Kind, Paint);
		const bool bHeavy = Kind == ETrafficVehicleKind::Bus || Kind == ETrafficVehicleKind::Truck;
		Vehicle->DesiredSpeedFactor = bHeavy ? Rand.FRandRange(0.7f, 0.85f) : Rand.FRandRange(0.85f, 1.1f);

		if (!PlaceVehicleRandomly(Vehicle, Avoid, PlayerSpawnClearance))
		{
			Vehicle->Destroy();
			continue;
		}
		ApplyTransform(Vehicle, 0.f, true);
		Vehicles.Add(Vehicle);
	}
	UE_LOG(LogTruckSim, Log, TEXT("Traffic manager spawned %d vehicles."), Vehicles.Num());
}

bool ATrafficManager::PlaceVehicleRandomly(ATrafficVehicle* Vehicle, const FVector& Avoid, double AvoidRadius)
{
	const TArray<FTrafficLane>& Lanes = Generator->GetLanes();
	double Total = 0.0;
	for (const FTrafficLane& Lane : Lanes)
	{
		Total += Lane.Path.Length();
	}
	if (Total <= 0.0)
	{
		return false;
	}

	for (int32 Try = 0; Try < 60; ++Try)
	{
		double Pick = Rand.FRand() * Total;
		int32 LaneIdx = 0;
		for (; LaneIdx < Lanes.Num() - 1; ++LaneIdx)
		{
			if (Pick < Lanes[LaneIdx].Path.Length())
			{
				break;
			}
			Pick -= Lanes[LaneIdx].Path.Length();
		}
		const FTrafficLane& Lane = Lanes[LaneIdx];
		const double Len = Lane.Path.Length();
		// Keep away from lane ends on open lanes (junction areas).
		const double D = Lane.Path.bClosed ? Pick : FMath::Clamp(Pick, 3000.0, FMath::Max(3000.0, Len - 3000.0));
		const FVector Loc = Lane.Path.GetLocationAtDistance(D);
		if (FVector::Dist2D(Loc, Avoid) < AvoidRadius)
		{
			continue;
		}
		bool bTooClose = false;
		for (const ATrafficVehicle* Other : Vehicles)
		{
			if (Other && Other != Vehicle && FVector::Dist(Other->GetActorLocation(), Loc) < 3500.0)
			{
				bTooClose = true;
				break;
			}
		}
		if (bTooClose)
		{
			continue;
		}
		Vehicle->LaneIndex = LaneIdx;
		Vehicle->Distance = D;
		Vehicle->Speed = Lane.GetProfileSpeed(D) * Vehicle->DesiredSpeedFactor * 0.6;
		Vehicle->BlendOffset = FVector::ZeroVector;
		Vehicle->StuckTime = 0.f;
		PlanExit(Vehicle);
		return true;
	}
	return false;
}

void ATrafficManager::PlanExit(ATrafficVehicle* Vehicle)
{
	const TArray<FTrafficLane>& Lanes = Generator->GetLanes();
	const FTrafficLane& Lane = Lanes[Vehicle->LaneIndex];
	const double Len = Lane.Path.Length();
	Vehicle->PlannedExit = INDEX_NONE;
	Vehicle->DistanceToExit = 0.0;

	TArray<TPair<double, int32>, TInlineAllocator<16>> Candidates;
	for (int32 e = 0; e < Lane.Exits.Num(); ++e)
	{
		const FLaneExit& Exit = Lane.Exits[e];
		if (Exit.bTerminal)
		{
			continue;
		}
		double Ahead = Exit.AtDistance - Vehicle->Distance;
		if (Lane.Path.bClosed)
		{
			if (Ahead < 50.0)
			{
				Ahead += Len;
			}
		}
		else if (Ahead < 50.0)
		{
			continue;
		}
		Candidates.Add(TPair<double, int32>(Ahead, e));
	}
	Candidates.Sort([](const TPair<double, int32>& A, const TPair<double, int32>& B) { return A.Key < B.Key; });
	for (const TPair<double, int32>& C : Candidates)
	{
		if (Rand.FRand() < Lane.Exits[C.Value].Probability)
		{
			Vehicle->PlannedExit = C.Value;
			Vehicle->DistanceToExit = C.Key;
			return;
		}
	}

	if (!Lane.Path.bClosed)
	{
		// Must take one of the terminal exits at the end of an open lane.
		double TotalWeight = 0.0;
		for (const FLaneExit& Exit : Lane.Exits)
		{
			TotalWeight += Exit.bTerminal ? Exit.Probability : 0.0;
		}
		double Pick = Rand.FRand() * TotalWeight;
		for (int32 e = 0; e < Lane.Exits.Num(); ++e)
		{
			const FLaneExit& Exit = Lane.Exits[e];
			if (!Exit.bTerminal)
			{
				continue;
			}
			// Remember the last terminal exit as a fallback against rounding.
			Vehicle->PlannedExit = e;
			Vehicle->DistanceToExit = FMath::Max(Exit.AtDistance - Vehicle->Distance, 0.0);
			Pick -= Exit.Probability;
			if (Pick <= 0.0)
			{
				break;
			}
		}
	}
}

void ATrafficManager::SampleAhead(const ATrafficVehicle* Vehicle, double Ahead, int32& OutLane, double& OutDistance) const
{
	const TArray<FTrafficLane>& Lanes = Generator->GetLanes();
	if (Vehicle->PlannedExit != INDEX_NONE && Ahead > Vehicle->DistanceToExit)
	{
		const FLaneExit& Exit = Lanes[Vehicle->LaneIndex].Exits[Vehicle->PlannedExit];
		OutLane = Exit.TargetLane;
		OutDistance = Lanes[OutLane].Path.WrapDistance(Exit.TargetDistance + (Ahead - Vehicle->DistanceToExit));
		return;
	}
	OutLane = Vehicle->LaneIndex;
	OutDistance = Lanes[OutLane].Path.WrapDistance(Vehicle->Distance + Ahead);
}

void ATrafficManager::AdvanceLanes(ATrafficVehicle* Vehicle)
{
	const TArray<FTrafficLane>& Lanes = Generator->GetLanes();
	for (int32 Guard = 0; Guard < 4; ++Guard)
	{
		const FTrafficLane& Lane = Lanes[Vehicle->LaneIndex];
		if (Vehicle->PlannedExit != INDEX_NONE && Vehicle->DistanceToExit <= 0.0)
		{
			const FLaneExit Exit = Lane.Exits[Vehicle->PlannedExit];
			const double Over = -Vehicle->DistanceToExit;
			// Keep the rendered position continuous across the lane switch.
			const FVector OldLoc = Lane.Path.GetLocationAtDistance(Exit.AtDistance + Over) + Vehicle->BlendOffset;
			Vehicle->LaneIndex = Exit.TargetLane;
			const FTrafficLane& Target = Lanes[Exit.TargetLane];
			Vehicle->Distance = Target.Path.WrapDistance(Exit.TargetDistance + Over);
			Vehicle->BlendOffset = OldLoc - Target.Path.GetLocationAtDistance(Vehicle->Distance);
			PlanExit(Vehicle);
			continue;
		}
		const double Len = Lane.Path.Length();
		if (Vehicle->Distance >= Len)
		{
			if (Lane.Path.bClosed)
			{
				Vehicle->Distance -= Len;
				if (Vehicle->PlannedExit == INDEX_NONE)
				{
					PlanExit(Vehicle);
				}
				continue;
			}
			// Dead end (should not happen with a connected network): respawn somewhere else.
			APawn* Player = UGameplayStatics::GetPlayerPawn(this, 0);
			PlaceVehicleRandomly(Vehicle, Player ? Player->GetActorLocation() : FVector::ZeroVector, 30000.0);
			ApplyTransform(Vehicle, 0.f, true);
		}
		break;
	}
}

void ATrafficManager::ApplyTransform(ATrafficVehicle* Vehicle, float DeltaSeconds, bool bTeleport)
{
	const FTrafficLane& Lane = Generator->GetLanes()[Vehicle->LaneIndex];
	const FVector Loc = Lane.Path.GetLocationAtDistance(Vehicle->Distance);
	const FVector Dir = Lane.Path.GetDirectionAtDistance(Vehicle->Distance, FMath::Max(150.0, Vehicle->HalfLength * 0.8));

	if (bTeleport)
	{
		Vehicle->BlendOffset = FVector::ZeroVector;
	}
	else
	{
		Vehicle->BlendOffset *= FMath::Exp(-2.5 * DeltaSeconds);
		if (Vehicle->BlendOffset.SizeSquared() < 1.0)
		{
			Vehicle->BlendOffset = FVector::ZeroVector;
		}
	}

	FRotator Target = Dir.Rotation();
	Target.Roll = 0.0;
	const FRotator Rot = bTeleport ? Target : FMath::RInterpTo(Vehicle->GetActorRotation(), Target, DeltaSeconds, 7.f);
	Vehicle->SetActorLocationAndRotation(Loc + Vehicle->BlendOffset, Rot, false, nullptr,
		bTeleport ? ETeleportType::TeleportPhysics : ETeleportType::None);
}

void ATrafficManager::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!Generator || Vehicles.Num() == 0 || DeltaSeconds <= 0.f)
	{
		return;
	}
	const TArray<FTrafficLane>& Lanes = Generator->GetLanes();
	const double Dt = FMath::Min(DeltaSeconds, 0.1f);

	// Bucket vehicles per lane.
	TArray<TArray<int32>> Buckets;
	Buckets.SetNum(Lanes.Num());
	for (int32 i = 0; i < Vehicles.Num(); ++i)
	{
		if (Vehicles[i] && Lanes.IsValidIndex(Vehicles[i]->LaneIndex))
		{
			Buckets[Vehicles[i]->LaneIndex].Add(i);
		}
	}

	// Player truck footprint.
	APawn* Player = UGameplayStatics::GetPlayerPawn(this, 0);
	FTransform PlayerTM = FTransform::Identity;
	FVector PlayerExtent(390.0, 125.0, 150.0);
	FVector PlayerVel = FVector::ZeroVector;
	if (Player)
	{
		PlayerTM = Player->GetActorTransform();
		PlayerVel = Player->GetVelocity();
		if (const AArmyTruckPawn* Truck = Cast<AArmyTruckPawn>(Player))
		{
			PlayerExtent = Truck->GetBodyHalfExtent();
		}
	}
	const FVector PlayerLoc = PlayerTM.GetLocation();

	TArray<double> Accel;
	Accel.SetNumZeroed(Vehicles.Num());

	for (int32 i = 0; i < Vehicles.Num(); ++i)
	{
		ATrafficVehicle* V = Vehicles[i];
		if (!V || !Lanes.IsValidIndex(V->LaneIndex))
		{
			continue;
		}
		const FTrafficLane& Lane = Lanes[V->LaneIndex];
		const double Len = Lane.Path.Length();
		const bool bClosed = Lane.Path.bClosed;
		const double Speed = V->Speed;
		const double AMax = V->MaxAccel;
		const double B = ComfortDecel;

		// Desired speed: speed limit and curvature ahead (brake early enough for the next curve).
		double V0 = Lane.SpeedLimit * V->DesiredSpeedFactor;
		const double Look = FMath::Max(4000.0, Speed * Speed / (2.0 * B) + 2500.0);
		for (double S = 0.0; S <= Look; S += 500.0)
		{
			int32 LI;
			double LD;
			SampleAhead(V, S, LI, LD);
			const double Vp = Lanes[LI].GetProfileSpeed(LD) * FMath::Min(V->DesiredSpeedFactor, 1.0);
			V0 = FMath::Min(V0, FMath::Sqrt(Vp * Vp + 2.0 * B * S));
		}
		V0 = FMath::Max(V0, 150.0);

		double Gap = BigGap;
		double LeadSpeed = 0.0;
		auto Consider = [&Gap, &LeadSpeed](double G, double LS)
		{
			if (G < Gap)
			{
				Gap = G;
				LeadSpeed = LS;
			}
		};
		const double ExitAhead = V->PlannedExit != INDEX_NONE ? V->DistanceToExit : BigGap;

		// Leader on the same lane.
		for (int32 j : Buckets[V->LaneIndex])
		{
			if (j == i)
			{
				continue;
			}
			const ATrafficVehicle* O = Vehicles[j];
			double Ahead = O->Distance - V->Distance;
			if (bClosed && Ahead < 0.0)
			{
				Ahead += Len;
			}
			if (Ahead < 0.0 || (Ahead == 0.0 && j < i))
			{
				continue;
			}
			if (Ahead > 15000.0 || Ahead > ExitAhead + O->HalfLength + V->HalfLength)
			{
				continue;
			}
			Consider(Ahead - V->HalfLength - O->HalfLength, O->Speed);
		}

		// Vehicles on the lane we are about to turn into (also yields to cars crossing the merge point).
		if (V->PlannedExit != INDEX_NONE && ExitAhead < 8000.0)
		{
			const FLaneExit& Exit = Lane.Exits[V->PlannedExit];
			const FTrafficLane& Target = Lanes[Exit.TargetLane];
			const double TL = Target.Path.Length();
			for (int32 j : Buckets[Exit.TargetLane])
			{
				if (j == i)
				{
					continue;
				}
				const ATrafficVehicle* O = Vehicles[j];
				double Rel = O->Distance - Exit.TargetDistance;
				if (Target.Path.bClosed)
				{
					if (Rel > TL * 0.5) { Rel -= TL; }
					if (Rel < -TL * 0.5) { Rel += TL; }
				}
				if (Rel < -(O->HalfLength + V->HalfLength + 200.0) || Rel > 15000.0)
				{
					continue;
				}
				Consider(ExitAhead + Rel - V->HalfLength - O->HalfLength, O->Speed);
			}
		}

		// Player truck overlapping the lane ahead.
		if (Player && FVector::DistSquared(PlayerLoc, V->GetActorLocation()) < FMath::Square(9000.0))
		{
			for (double S = 0.0; S <= 6000.0; S += 250.0)
			{
				int32 LI;
				double LD;
				SampleAhead(V, S, LI, LD);
				const FVector P = Lanes[LI].Path.GetLocationAtDistance(LD);
				const FVector Local = PlayerTM.InverseTransformPositionNoScale(P);
				const double DX = FMath::Max(FMath::Abs(Local.X) - PlayerExtent.X, 0.0);
				const double DY = FMath::Max(FMath::Abs(Local.Y) - PlayerExtent.Y, 0.0);
				if (FMath::Abs(Local.Z) < 500.0 && DX * DX + DY * DY < FMath::Square(V->HalfWidth + 80.0))
				{
					const FVector LaneDir = Lanes[LI].Path.GetDirectionAtDistance(LD);
					Consider(S - V->HalfLength, FMath::Max(0.0, FVector::DotProduct(PlayerVel, LaneDir)));
					break;
				}
			}
		}

		// Traffic lights.
		for (const FLaneStop& Stop : Lane.Stops)
		{
			double Ahead = Stop.Distance - V->Distance;
			if (bClosed && Ahead < -100.0)
			{
				Ahead += Len;
			}
			if (Ahead < -50.0 || Ahead > 9000.0 || Ahead > ExitAhead + 100.0)
			{
				continue;
			}
			const ATrafficSignal* Signal = Stop.Signal.Get();
			if (!Signal)
			{
				continue;
			}
			const ESignalState State = Signal->GetGroupState(Stop.Group);
			const double FrontToLine = Ahead - V->HalfLength;
			const bool bMustStop = State == ESignalState::Red
				? FrontToLine > -50.0
				: (State == ESignalState::Yellow && FrontToLine > Speed * Speed / (2.0 * B));
			if (bMustStop)
			{
				Consider(FrontToLine + MinGap * 0.7, 0.0);
			}
		}

		// Intelligent Driver Model.
		double Acc = AMax * (1.0 - FMath::Pow(Speed / V0, 4.0));
		if (Gap < BigGap * 0.5)
		{
			const double SStar = MinGap + FMath::Max(0.0, Speed * V->TimeHeadway + Speed * (Speed - LeadSpeed) / (2.0 * FMath::Sqrt(AMax * B)));
			Acc -= AMax * FMath::Square(SStar / FMath::Max(Gap, 10.0));
		}
		Accel[i] = FMath::Clamp(Acc, -MaxDecel, AMax);
	}

	// Integrate after all decisions so the update order does not matter.
	for (int32 i = 0; i < Vehicles.Num(); ++i)
	{
		ATrafficVehicle* V = Vehicles[i];
		if (!V || !Lanes.IsValidIndex(V->LaneIndex))
		{
			continue;
		}
		V->Speed = FMath::Max(0.0, V->Speed + Accel[i] * Dt);
		const double Travel = V->Speed * Dt;
		V->Distance += Travel;
		if (V->PlannedExit != INDEX_NONE)
		{
			V->DistanceToExit -= Travel;
		}
		AdvanceLanes(V);

		// Recycle vehicles stuck for a long time far from the player (e.g. gridlock behind a parked truck).
		V->StuckTime = V->Speed < 20.0 ? V->StuckTime + DeltaSeconds : 0.f;
		if (V->StuckTime > 60.f && FVector::Dist(V->GetActorLocation(), PlayerLoc) > 25000.0)
		{
			if (PlaceVehicleRandomly(V, PlayerLoc, 25000.0))
			{
				ApplyTransform(V, DeltaSeconds, true);
				continue;
			}
		}
		ApplyTransform(V, DeltaSeconds, false);
	}
}
