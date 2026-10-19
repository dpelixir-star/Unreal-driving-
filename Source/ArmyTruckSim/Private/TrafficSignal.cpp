#include "TrafficSignal.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/CollisionProfile.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
	const FLinearColor LampOff(0.02f, 0.02f, 0.02f);
	const FLinearColor LampRed(1.0f, 0.03f, 0.01f);
	const FLinearColor LampYellow(1.0f, 0.55f, 0.0f);
	const FLinearColor LampGreen(0.05f, 1.0f, 0.25f);
}

ATrafficSignal::ATrafficSignal()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickInterval = 0.1f;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeFinder(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderFinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereFinder(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	CubeMesh = CubeFinder.Object;
	CylinderMesh = CylinderFinder.Object;
	SphereMesh = SphereFinder.Object;
}

void ATrafficSignal::SetPhaseOffset(float Seconds)
{
	CycleTime = Seconds;
	RefreshLamps(true);
}

ESignalState ATrafficSignal::GetGroupState(int32 Group) const
{
	const float T = FMath::Fmod(CycleTime, CycleLength());
	const float MainEnd = MainGreenTime;
	const float MainYellowEnd = MainEnd + YellowTime;
	const float RedEnd1 = MainYellowEnd + AllRedTime;
	const float SideEnd = RedEnd1 + SideGreenTime;
	const float SideYellowEnd = SideEnd + YellowTime;

	if (Group == 0)
	{
		if (T < MainEnd) { return ESignalState::Green; }
		if (T < MainYellowEnd) { return ESignalState::Yellow; }
		return ESignalState::Red;
	}
	if (T >= RedEnd1 && T < SideEnd) { return ESignalState::Green; }
	if (T >= SideEnd && T < SideYellowEnd) { return ESignalState::Yellow; }
	return ESignalState::Red;
}

void ATrafficSignal::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	CycleTime = FMath::Fmod(CycleTime + DeltaSeconds, CycleLength());
	RefreshLamps(false);
}

void ATrafficSignal::RefreshLamps(bool bForce)
{
	for (int32 Group = 0; Group < 2; ++Group)
	{
		const ESignalState State = GetGroupState(Group);
		if (!bForce && State == LastStates[Group])
		{
			continue;
		}
		LastStates[Group] = State;
		for (int32 Head = 0; Head < HeadGroups.Num(); ++Head)
		{
			if (HeadGroups[Head] != Group || !LampMaterials.IsValidIndex(Head * 3 + 2))
			{
				continue;
			}
			auto SetLamp = [this, Head](int32 Lamp, const FLinearColor& Color)
			{
				if (UMaterialInstanceDynamic* Mat = LampMaterials[Head * 3 + Lamp])
				{
					Mat->SetVectorParameterValue(TEXT("Color"), Color);
				}
			};
			SetLamp(0, State == ESignalState::Red ? LampRed : LampOff);
			SetLamp(1, State == ESignalState::Yellow ? LampYellow : LampOff);
			SetLamp(2, State == ESignalState::Green ? LampGreen : LampOff);
		}
	}
}

void ATrafficSignal::AddSignalHead(const FVector& GroundLocation, const FVector& TrafficDirection, int32 Group)
{
	if (!HousingMaterial)
	{
		HousingMaterial = TruckSim::MakeColorMaterial(this, FLinearColor(0.03f, 0.03f, 0.025f));
	}
	FVector Facing = -TrafficDirection.GetSafeNormal2D(); // lamps face the approaching traffic
	if (Facing.IsNearlyZero())
	{
		Facing = FVector::ForwardVector;
	}
	const FRotator Rot = Facing.Rotation();

	auto MakePart = [&](UStaticMesh* Mesh, const FVector& WorldLoc, const FVector& Scale, UMaterialInterface* Material, bool bCollision)
	{
		UStaticMeshComponent* Part = NewObject<UStaticMeshComponent>(this);
		Part->SetStaticMesh(Mesh);
		Part->SetCollisionProfileName(bCollision ? UCollisionProfile::BlockAll_ProfileName : UCollisionProfile::NoCollision_ProfileName);
		Part->SetupAttachment(RootComponent);
		Part->RegisterComponent();
		Part->SetWorldLocationAndRotation(WorldLoc, Rot);
		Part->SetWorldScale3D(Scale);
		if (Material)
		{
			Part->SetMaterial(0, Material);
		}
		AddInstanceComponent(Part);
		return Part;
	};

	const FVector Up = FVector::UpVector;
	MakePart(CylinderMesh, GroundLocation + Up * 230.0, FVector(0.16, 0.16, 4.6), HousingMaterial, true);
	const FVector HousingLoc = GroundLocation + Up * 520.0;
	MakePart(CubeMesh, HousingLoc, FVector(0.38, 0.42, 1.15), HousingMaterial, false);

	const float Offsets[3] = { 35.f, 0.f, -35.f };
	for (int32 Lamp = 0; Lamp < 3; ++Lamp)
	{
		UMaterialInstanceDynamic* LampMat = TruckSim::MakeColorMaterial(this, LampOff);
		LampMaterials.Add(LampMat);
		MakePart(SphereMesh, HousingLoc + Facing * 20.0 + Up * Offsets[Lamp], FVector(0.27, 0.27, 0.27), LampMat, false);
	}
	HeadGroups.Add(Group);
	RefreshLamps(true);
}
