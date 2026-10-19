#include "WorldGenerator.h"
#include "ArmyTruckSim.h"
#include "RoadPathBuilder.h"
#include "TrafficSignal.h"

#include "ProceduralMeshComponent.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Engine/CollisionProfile.h"
#include "EngineUtils.h"
#include "Engine/DirectionalLight.h"
#include "Components/DirectionalLightComponent.h"
#include "Engine/SkyLight.h"
#include "Components/SkyLightComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Engine/ExponentialHeightFog.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "HAL/PlatformTime.h"
#include "Async/ParallelFor.h"

namespace
{
	constexpr double M2CM = 100.0;

	/** Vertex/index buffers for one procedural mesh section. */
	struct FMeshBuffers
	{
		TArray<FVector> Vertices;
		TArray<int32> Triangles;
		TArray<FVector> Normals;
		TArray<FVector2D> UVs;
		TArray<FLinearColor> Colors;

		bool IsEmpty() const { return Triangles.Num() == 0; }

		int32 AddVertex(const FVector& P, const FVector& N, const FVector2D& UV, const FLinearColor& C)
		{
			Vertices.Add(P);
			Normals.Add(N);
			UVs.Add(UV);
			Colors.Add(C);
			return Vertices.Num() - 1;
		}

		/**
		 * Quad between two cross section rows. A = left edge, B = right edge (seen in the direction of travel),
		 * 0 = current sample, 1 = next sample. The face normal is Forward x (B - A), i.e. up for a road surface.
		 */
		void AddQuad(const FVector& A0, const FVector& A1, const FVector& B0, const FVector& B1,
			const FVector& N0, const FVector& N1, double V0, double V1, double UA, double UB, const FLinearColor& C)
		{
			const int32 IA0 = AddVertex(A0, N0, FVector2D(UA, V0), C);
			const int32 IA1 = AddVertex(A1, N1, FVector2D(UA, V1), C);
			const int32 IB0 = AddVertex(B0, N0, FVector2D(UB, V0), C);
			const int32 IB1 = AddVertex(B1, N1, FVector2D(UB, V1), C);
			Triangles.Append({ IA0, IA1, IB0, IA1, IB1, IB0 });
		}

		void CreateSection(UProceduralMeshComponent* Mesh, int32 Index, bool bCollision) const
		{
			Mesh->CreateMeshSection_LinearColor(Index, Vertices, Triangles, Normals, UVs, Colors, TArray<FProcMeshTangent>(), bCollision);
		}
	};

	void SmoothProfile(TArray<double>& H, bool bClosed, int32 Radius, int32 Passes, const TArray<bool>& Pinned)
	{
		const int32 N = H.Num();
		if (N < 3 || Radius < 1)
		{
			return;
		}
		TArray<double> Tmp;
		Tmp.SetNum(N);
		for (int32 Pass = 0; Pass < Passes; ++Pass)
		{
			for (int32 i = 0; i < N; ++i)
			{
				if (Pinned[i])
				{
					Tmp[i] = H[i];
					continue;
				}
				double Sum = 0.0;
				int32 Count = 0;
				for (int32 k = -Radius; k <= Radius; ++k)
				{
					int32 j = i + k;
					if (bClosed)
					{
						j = ((j % N) + N) % N;
					}
					else if (j < 0 || j >= N)
					{
						continue;
					}
					Sum += H[j];
					++Count;
				}
				Tmp[i] = Count > 0 ? Sum / Count : H[i];
			}
			H = Tmp;
		}
	}

	/** Iteratively limits |dH/ds| to MaxGrade, splitting corrections between neighbours and respecting pins. */
	void RelaxGrade(TArray<double>& H, const TArray<double>& SegLen, double MaxGrade, bool bClosed, const TArray<bool>& Pinned, int32 MaxIterations)
	{
		const int32 N = H.Num();
		const int32 NumEdges = bClosed ? N : N - 1;
		if (NumEdges <= 0)
		{
			return;
		}
		auto Fix = [&](int32 E) -> bool
		{
			const int32 A = E;
			const int32 B = (E + 1) % N;
			const double Max = MaxGrade * SegLen[E];
			const double Diff = H[B] - H[A];
			if (FMath::Abs(Diff) <= Max + 0.01)
			{
				return false;
			}
			const double Excess = FMath::Abs(Diff) - Max;
			const double Sgn = Diff > 0.0 ? 1.0 : -1.0;
			if (!Pinned[A] && !Pinned[B])
			{
				H[A] += Sgn * Excess * 0.5;
				H[B] -= Sgn * Excess * 0.5;
			}
			else if (!Pinned[B])
			{
				H[B] -= Sgn * Excess;
			}
			else if (!Pinned[A])
			{
				H[A] += Sgn * Excess;
			}
			else
			{
				return false;
			}
			return true;
		};

		for (int32 It = 0; It < MaxIterations; ++It)
		{
			bool bChanged = false;
			for (int32 E = 0; E < NumEdges; ++E)
			{
				bChanged |= Fix(E);
			}
			for (int32 E = NumEdges - 1; E >= 0; --E)
			{
				bChanged |= Fix(E);
			}
			if (!bChanged)
			{
				break;
			}
		}
	}

	bool SegmentIntersect2D(const FVector2D& A, const FVector2D& B, const FVector2D& C, const FVector2D& D, double& OutT, double& OutU)
	{
		const FVector2D R = B - A;
		const FVector2D S = D - C;
		const double Den = R.X * S.Y - R.Y * S.X;
		if (FMath::Abs(Den) < 1e-9)
		{
			return false;
		}
		const FVector2D AC = C - A;
		OutT = (AC.X * S.Y - AC.Y * S.X) / Den;
		OutU = (AC.X * R.Y - AC.Y * R.X) / Den;
		return OutT >= 0.0 && OutT <= 1.0 && OutU >= 0.0 && OutU <= 1.0;
	}

	int32 FindClosestIndex2D(const TArray<FVector2D>& Pts, const FVector2D& P)
	{
		int32 Best = INDEX_NONE;
		double BestD = TNumericLimits<double>::Max();
		for (int32 i = 0; i < Pts.Num(); ++i)
		{
			const double D = FVector2D::DistSquared(Pts[i], P);
			if (D < BestD)
			{
				BestD = D;
				Best = i;
			}
		}
		return Best;
	}

	/** Junction point on a closed loop near the given polar angle, plus the heading pointing into the loop. */
	void FindLoopJunction(const TArray<FVector2D>& Loop, double AngleDeg, double Radius, FVector2D& OutPos, double& OutInwardHeading)
	{
		const int32 N = Loop.Num();
		const int32 Idx = FindClosestIndex2D(Loop, FRoadPathBuilder::HeadingToDir(AngleDeg) * Radius);
		OutPos = Loop[Idx];
		const FVector2D T = (Loop[(Idx + 1) % N] - Loop[(Idx - 1 + N) % N]).GetSafeNormal();
		FVector2D Nrm(-T.Y, T.X);
		if (FVector2D::DotProduct(Nrm, -OutPos) < 0.0)
		{
			Nrm = -Nrm;
		}
		OutInwardHeading = FRoadPathBuilder::DirToHeading(Nrm);
	}

	FVector2D To2D(const FVector& V) { return FVector2D(V.X, V.Y); }
}

// =============================================================================================

AWorldGenerator::AWorldGenerator()
{
	PrimaryActorTick.bCanEverTick = false;
	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;
}

AWorldGenerator* AWorldGenerator::Find(const UObject* WorldContext)
{
	UWorld* World = WorldContext ? WorldContext->GetWorld() : nullptr;
	if (!World)
	{
		return nullptr;
	}
	for (TActorIterator<AWorldGenerator> It(World); It; ++It)
	{
		return *It;
	}
	return nullptr;
}

void AWorldGenerator::BeginPlay()
{
	Super::BeginPlay();
	if (!bGenerated)
	{
		Generate();
	}
}

void AWorldGenerator::ClearGenerated()
{
	for (UProceduralMeshComponent* Mesh : MeshComponents)
	{
		if (Mesh)
		{
			Mesh->DestroyComponent();
		}
	}
	for (UHierarchicalInstancedStaticMeshComponent* Inst : InstanceComponents)
	{
		if (Inst)
		{
			Inst->DestroyComponent();
		}
	}
	for (AActor* Actor : SpawnedActors)
	{
		if (IsValid(Actor))
		{
			Actor->Destroy();
		}
	}
	MeshComponents.Reset();
	InstanceComponents.Reset();
	SpawnedActors.Reset();
	Materials.Reset();
	Roads.Reset();
	Lanes.Reset();
	Junctions.Reset();
	Crossings.Reset();
	RoadSamples.Reset();
	RoadGrid.Reset();
	Height = FTerrainHeightField();
	bGenerated = false;
}

void AWorldGenerator::Generate()
{
	const double StartTime = FPlatformTime::Seconds();
	ClearGenerated();

	Stream.Initialize(Seed);
	for (FVector2D& Offset : NoiseOffsets)
	{
		Offset = FVector2D(Stream.FRandRange(-1000.f, 1000.f), Stream.FRandRange(-1000.f, 1000.f));
	}

	BuildNaturalTerrain();
	BuildRoadLayout();
	ComputeRoadElevations();
	BuildRoadSampleGrid();
	ConformTerrainToRoads();
	BuildTerrainMesh();
	BuildRoadMeshes();
	if (bBuildGuardRails)
	{
		BuildGuardRails();
	}
	BuildTreesAndRocks();
	BuildLanes();

	UWorld* World = GetWorld();
	const bool bGameWorld = World && World->IsGameWorld();
	if (bGameWorld && bSpawnTrafficSignals)
	{
		BuildSignals();
	}
	if (bGameWorld && bSpawnEnvironment)
	{
		SpawnEnvironment();
	}

	// Player spawn: on the pass road, heading into the valley towards the mountain.
	if (Roads.IsValidIndex(PassRoad) && Lanes.IsValidIndex(Roads[PassRoad].ForwardLane))
	{
		const FTrafficLane& Lane = Lanes[Roads[PassRoad].ForwardLane];
		const double D = FMath::Min(9000.0, Lane.Path.Length() * 0.1);
		const FVector Loc = Lane.Path.GetLocationAtDistance(D);
		FRotator Rot = Lane.Path.GetDirectionAtDistance(D, 400.0).Rotation();
		Rot.Roll = 0.0;
		PlayerSpawn = FTransform(Rot, Loc + FVector(0.0, 0.0, 180.0));
	}

	bGenerated = true;

	double TotalRoad = 0.0;
	for (const FSimRoad& Road : Roads)
	{
		TotalRoad += Road.Center.Length();
	}
	UE_LOG(LogTruckSim, Log, TEXT("World generated in %.2f s: %d x %d terrain vertices, %d roads (%.1f km), %d lanes, %d junctions, %d crossings."),
		FPlatformTime::Seconds() - StartTime, Height.NumX, Height.NumY, Roads.Num(), TotalRoad / 100000.0, Lanes.Num(), Junctions.Num(), Crossings.Num());
}

// =============================================================================================
// Terrain
// =============================================================================================

double AWorldGenerator::NaturalHeightMeters(const FVector2D& P) const
{
	const double S = MapScale();

	// Rolling hills: fractal Perlin noise.
	double Fbm = 0.0;
	double Amp = 1.0;
	double Freq = 1.0 / (HillWavelengthMeters * S);
	double Norm = 0.0;
	for (int32 Octave = 0; Octave < 5; ++Octave)
	{
		Fbm += Amp * FMath::PerlinNoise2D(P * Freq + NoiseOffsets[Octave]);
		Norm += Amp;
		Amp *= 0.5;
		Freq *= 2.1;
	}
	Fbm /= Norm;
	double H = BaseHeightMeters + HillAmplitudeMeters * Fbm;

	// East-west mountain ridge with a saddle (the pass).
	const double Dy = (P.Y - RidgeCenterYMeters * S) / (RidgeWidthMeters * S);
	const double Dx = P.X / (RidgeHalfLengthMeters * S);
	const double Along = FMath::Exp(-(Dx * Dx * Dx * Dx));
	const double Sx = (P.X - SaddleCenterXMeters * S) / (SaddleWidthMeters * S);
	const double Saddle = 1.0 - SaddleDepth * FMath::Exp(-Sx * Sx);
	const double Crag = 1.0 + 0.18 * FMath::PerlinNoise2D(P / (230.0 * S) + NoiseOffsets[6]);
	H += RidgeHeightMeters * FMath::Exp(-Dy * Dy) * Along * Saddle * Crag;

	// Raised rim so the world feels enclosed.
	const double Edge = FMath::Max(FMath::Abs(P.X), FMath::Abs(P.Y)) / (MapSizeMeters * 0.5);
	const double RimT = FMath::Clamp((Edge - 0.9) / 0.1, 0.0, 1.0);
	H += RimHeightMeters * RimT * RimT * (3.0 - 2.0 * RimT);
	return H;
}

void AWorldGenerator::BuildNaturalTerrain()
{
	int32 Cells = FMath::Max(16, TruckSim::RoundI(MapSizeMeters / TerrainCellSizeMeters));
	Cells = FMath::Max(1, Cells / TilesPerSide) * TilesPerSide;
	const double Half = MapSizeMeters * 50.0;
	Height.Init(Cells + 1, Cells + 1, (MapSizeMeters * M2CM) / Cells, FVector2D(-Half, -Half));

	ParallelFor(Height.NumY, [this](int32 Y)
	{
		for (int32 X = 0; X < Height.NumX; ++X)
		{
			const FVector2D P = Height.GridToWorld(X, Y);
			Height.At(X, Y) = static_cast<float>(NaturalHeightMeters(P / M2CM) * M2CM);
		}
	});
}

// =============================================================================================
// Road layout
// =============================================================================================

void AWorldGenerator::BuildRoadLayout()
{
	const double S = MapScale();
	const double Step = RoadSampleSpacingMeters;
	auto M = [S](double X, double Y) { return FVector2D(X, Y) * S; };

	const double RingRadius = 1950.0 * S;
	const double JunctionAngles[4] = { -90.0, 90.0, 214.0, 326.0 }; // S, N, W, E

	// ---- Ring road: closed Catmull-Rom loop with random radial wiggles (sweeping curves).
	TArray<FVector2D> RingCtrl;
	const int32 NumRingCtrl = 20;
	for (int32 i = 0; i < NumRingCtrl; ++i)
	{
		const double Angle = 360.0 * i / NumRingCtrl;
		double MinJunctionDelta = 180.0;
		for (double JA : JunctionAngles)
		{
			MinJunctionDelta = FMath::Min(MinJunctionDelta, FMath::Abs(FMath::FindDeltaAngleDegrees(Angle, JA)));
		}
		const double Wiggle = MinJunctionDelta < 12.0 ? 0.0 : Stream.FRandRange(-110.f, 110.f) * S;
		RingCtrl.Add(FRoadPathBuilder::HeadingToDir(Angle) * (RingRadius + Wiggle));
	}
	const TArray<FVector2D> Ring2D = FRoadPathBuilder::ClosedLoop(RingCtrl, Step);

	FVector2D JS, JN, JW, JE;
	double HS, HN, HW, HE;
	FindLoopJunction(Ring2D, JunctionAngles[0], RingRadius, JS, HS);
	FindLoopJunction(Ring2D, JunctionAngles[1], RingRadius, JN, HN);
	FindLoopJunction(Ring2D, JunctionAngles[2], RingRadius, JW, HW);
	FindLoopJunction(Ring2D, JunctionAngles[3], RingRadius, JE, HE);

	// ---- Mountain pass road with switchbacks.
	FRoadPathBuilder Pass(JS, HS, Step);
	Pass.Straight(60.0);
	Pass.SmoothThrough({ M(40, -1550), M(-50, -1250), M(30, -950), M(-40, -650), M(-110, -400) }, 90.0);
	Pass.Arc(40.0, -75.0); // heading ~15 deg: traverse the slope eastwards
	const double LegLength = SwitchbackLegMeters * S;
	for (int32 i = 0; i < NumSwitchbackHairpins; ++i)
	{
		Pass.Straight(LegLength);
		Pass.Arc(HairpinRadiusMeters, (i % 2 == 0) ? 150.0 : -150.0); // hairpin
	}
	Pass.Straight(LegLength);
	Pass.Arc(60.0, (NumSwitchbackHairpins % 2 == 0) ? 75.0 : -75.0); // turn north towards the saddle
	Pass.Straight(120.0 * S);
	// North face: two more hairpins on the way down.
	Pass.Arc(60.0, -75.0);
	Pass.Straight(180.0 * S);
	Pass.Arc(HairpinRadiusMeters, 150.0);
	Pass.Straight(220.0 * S);
	Pass.Arc(HairpinRadiusMeters, -150.0);
	Pass.Straight(120.0 * S);
	Pass.Arc(80.0, 75.0);
	const double OutN = HN + 180.0;
	const FVector2D PassCur = Pass.GetPosition();
	const FVector2D PassApproach = JN - FRoadPathBuilder::HeadingToDir(OutN) * 60.0;
	Pass.SmoothThrough({
		FMath::Lerp(PassCur, PassApproach, 0.3) + M(-40, 0),
		FMath::Lerp(PassCur, PassApproach, 0.6) + M(-130, 0),
		FMath::Lerp(PassCur, PassApproach, 0.85) + M(30, 0),
		PassApproach }, OutN);
	Pass.Straight(60.0);

	// ---- Valley cross road (west -> east), crossing the pass road.
	const double OutE = HE + 180.0;
	FRoadPathBuilder Cross(JW, HW, Step);
	Cross.Straight(40.0);
	const FVector2D CrossApproach = JE - FRoadPathBuilder::HeadingToDir(OutE) * 40.0;
	Cross.SmoothThrough({ M(-1150, -1000), M(-600, -1180), M(-150, -1090), M(350, -1020), M(900, -1150), M(1300, -1080), CrossApproach }, OutE);
	Cross.Straight(40.0);

	// ---- Store roads (cm).
	auto MakeRoad = [&](const FString& Name, const TArray<FVector2D>& Pts, bool bClosed, double SpeedKmh)
	{
		FSimRoad Road;
		Road.Name = Name;
		Road.HalfWidth = RoadHalfWidthMeters * M2CM;
		Road.Shoulder = ShoulderWidthMeters * M2CM;
		Road.SpeedLimit = SpeedKmh * TruckSim::KmhToCms;
		Road.Center.bClosed = bClosed;
		Road.Center.Points.Reserve(Pts.Num());
		for (const FVector2D& P : Pts)
		{
			Road.Center.Points.Add(FVector(P.X * M2CM, P.Y * M2CM, 0.0));
		}
		Road.Center.Build();
		return Road;
	};
	Roads.Add(MakeRoad(TEXT("Ring Road"), Ring2D, true, RingSpeedLimitKmh));
	Roads.Add(MakeRoad(TEXT("Mountain Pass"), Pass.GetPoints(), false, PassSpeedLimitKmh));
	Roads.Add(MakeRoad(TEXT("Valley Road"), Cross.GetPoints(), false, CrossSpeedLimitKmh));

	auto AddJunction = [&](const FVector2D& PosM, int32 Branch, bool bStarts)
	{
		FJunctionInfo J;
		J.Pos = PosM * M2CM;
		J.MainRoad = RingRoad;
		J.BranchRoad = Branch;
		J.bBranchStartsHere = bStarts;
		Junctions.Add(J);
	};
	AddJunction(JS, PassRoad, true);
	AddJunction(JN, PassRoad, false);
	AddJunction(JW, CrossRoad, true);
	AddJunction(JE, CrossRoad, false);

	// ---- Find where the pass road and the valley road cross.
	const TArray<FVector>& PA = Roads[PassRoad].Center.Points;
	const TArray<FVector>& PB = Roads[CrossRoad].Center.Points;
	for (int32 i = 0; i + 1 < PA.Num(); ++i)
	{
		const FVector2D A0 = To2D(PA[i]);
		const FVector2D A1 = To2D(PA[i + 1]);
		for (int32 j = 0; j + 1 < PB.Num(); ++j)
		{
			double T, U;
			if (SegmentIntersect2D(A0, A1, To2D(PB[j]), To2D(PB[j + 1]), T, U))
			{
				const FVector2D X = FMath::Lerp(A0, A1, T);
				bool bNearJunction = false;
				for (const FJunctionInfo& J : Junctions)
				{
					bNearJunction |= FVector2D::Distance(J.Pos, X) < 5000.0;
				}
				if (!bNearJunction)
				{
					FCrossingInfo C;
					C.Pos = X;
					C.RoadA = PassRoad;
					C.RoadB = CrossRoad;
					Crossings.Add(C);
				}
			}
		}
	}
}

void AWorldGenerator::ElevateRoad(FSimRoad& Road, double MaxGrade, const TMap<int32, double>& Pins, int32 SmoothRadius)
{
	TArray<FVector>& P = Road.Center.Points;
	const int32 N = P.Num();
	const bool bClosed = Road.Center.bClosed;
	TArray<double> H;
	H.SetNum(N);
	TArray<bool> Pinned;
	Pinned.Init(false, N);
	for (int32 i = 0; i < N; ++i)
	{
		H[i] = NaturalHeightMeters(To2D(P[i]) / M2CM) * M2CM;
	}
	for (const TPair<int32, double>& Pin : Pins)
	{
		if (H.IsValidIndex(Pin.Key))
		{
			H[Pin.Key] = Pin.Value;
			Pinned[Pin.Key] = true;
		}
	}
	SmoothProfile(H, bClosed, SmoothRadius, 3, Pinned);

	const int32 NumEdges = bClosed ? N : N - 1;
	TArray<double> Seg;
	Seg.SetNum(FMath::Max(NumEdges, 0));
	for (int32 E = 0; E < NumEdges; ++E)
	{
		Seg[E] = FVector2D::Distance(To2D(P[E]), To2D(P[(E + 1) % N]));
	}
	RelaxGrade(H, Seg, MaxGrade, bClosed, Pinned, 800);
	SmoothProfile(H, bClosed, 4, 2, Pinned); // round off the vertical curves

	for (int32 i = 0; i < N; ++i)
	{
		P[i].Z = H[i];
	}
	Road.Center.Build();
}

void AWorldGenerator::ComputeRoadElevations()
{
	// Ring road first: free, closed profile.
	ElevateRoad(Roads[RingRoad], RingMaxGrade, TMap<int32, double>(), 20);

	auto RingHeightAt = [this](const FVector2D& Pos)
	{
		const FSimRoad& Ring = Roads[RingRoad];
		double Best = TNumericLimits<double>::Max();
		double Z = 0.0;
		for (const FVector& P : Ring.Center.Points)
		{
			const double D = FVector2D::DistSquared(To2D(P), Pos);
			if (D < Best)
			{
				Best = D;
				Z = P.Z;
			}
		}
		return Z;
	};

	auto PinEnds = [&](int32 RoadIdx, TMap<int32, double>& Pins)
	{
		const FSimRoad& Road = Roads[RoadIdx];
		for (const FJunctionInfo& J : Junctions)
		{
			if (J.BranchRoad != RoadIdx)
			{
				continue;
			}
			const double Z = RingHeightAt(J.Pos);
			for (int32 i = 0; i < Road.Center.Points.Num(); ++i)
			{
				if (FVector2D::Distance(To2D(Road.Center.Points[i]), J.Pos) < Road.HalfWidth + 300.0)
				{
					Pins.Add(i, Z);
				}
			}
		}
	};

	// Pass road: pinned to the ring at both ends.
	{
		TMap<int32, double> Pins;
		PinEnds(PassRoad, Pins);
		ElevateRoad(Roads[PassRoad], PassMaxGrade, Pins, 15);
	}

	// Valley road: pinned to the ring at both ends and to the pass road where they cross.
	{
		TMap<int32, double> Pins;
		PinEnds(CrossRoad, Pins);
		const FSimRoad& A = Roads[PassRoad];
		const FSimRoad& B = Roads[CrossRoad];
		for (const FCrossingInfo& C : Crossings)
		{
			const double DA = A.Center.FindClosestDistance(FVector(C.Pos, 0.0));
			const double Z = A.Center.GetLocationAtDistance(DA).Z;
			for (int32 i = 0; i < B.Center.Points.Num(); ++i)
			{
				if (FVector2D::Distance(To2D(B.Center.Points[i]), C.Pos) < A.HalfWidth + A.Shoulder + 200.0)
				{
					Pins.Add(i, Z);
				}
			}
		}
		ElevateRoad(Roads[CrossRoad], CrossMaxGrade, Pins, 15);
	}

	// Gaps so overlapping road surfaces don't z-fight.
	for (const FJunctionInfo& J : Junctions)
	{
		FSimRoad& Ring = Roads[J.MainRoad];
		FSimRoad& Branch = Roads[J.BranchRoad];
		const double RingD = Ring.Center.FindClosestDistance(FVector(J.Pos, 0.0));
		Ring.MarkingGaps.Add(FVector2D(RingD - Branch.HalfWidth - Branch.Shoulder - 100.0, RingD + Branch.HalfWidth + Branch.Shoulder + 100.0));
		const double Trim = Ring.HalfWidth + 10.0;
		const double L = Branch.Center.Length();
		if (J.bBranchStartsHere)
		{
			Branch.SurfaceGaps.Add(FVector2D(-1.0, Trim));
			Branch.MarkingGaps.Add(FVector2D(-1.0, Trim + Ring.Shoulder + 300.0));
		}
		else
		{
			Branch.SurfaceGaps.Add(FVector2D(L - Trim, L + 1.0));
			Branch.MarkingGaps.Add(FVector2D(L - Trim - Ring.Shoulder - 300.0, L + 1.0));
		}
	}
	for (const FCrossingInfo& C : Crossings)
	{
		FSimRoad& A = Roads[C.RoadA];
		FSimRoad& B = Roads[C.RoadB];
		const double DA = A.Center.FindClosestDistance(FVector(C.Pos, 0.0));
		const double DB = B.Center.FindClosestDistance(FVector(C.Pos, 0.0));
		A.MarkingGaps.Add(FVector2D(DA - B.HalfWidth - 150.0, DA + B.HalfWidth + 150.0));
		B.SurfaceGaps.Add(FVector2D(DB - A.HalfWidth - 20.0, DB + A.HalfWidth + 20.0));
		B.MarkingGaps.Add(FVector2D(DB - A.HalfWidth - 150.0, DB + A.HalfWidth + 150.0));
	}
}

void AWorldGenerator::BuildRoadSampleGrid()
{
	RoadSamples.Reset();
	RoadGrid.Reset();
	for (int32 R = 0; R < Roads.Num(); ++R)
	{
		const FSimRoad& Road = Roads[R];
		const double Core = Road.HalfWidth + Road.Shoulder + 300.0;
		for (const FVector& P : Road.Center.Points)
		{
			FRoadSampleRef Ref;
			Ref.P = To2D(P);
			Ref.Z = P.Z;
			Ref.Core = Core;
			Ref.Road = R;
			const int32 Idx = RoadSamples.Add(Ref);
			const FIntPoint Cell(TruckSim::FloorI(Ref.P.X / RoadGridCell), TruckSim::FloorI(Ref.P.Y / RoadGridCell));
			RoadGrid.FindOrAdd(Cell).Add(Idx);
		}
	}
}

double AWorldGenerator::DistanceToNearestRoad(const FVector2D& P, double MaxSearch) const
{
	const int32 Rings = FMath::Max(1, TruckSim::CeilI(MaxSearch / RoadGridCell));
	const FIntPoint C(TruckSim::FloorI(P.X / RoadGridCell), TruckSim::FloorI(P.Y / RoadGridCell));
	double Best = MaxSearch;
	for (int32 DY = -Rings; DY <= Rings; ++DY)
	{
		for (int32 DX = -Rings; DX <= Rings; ++DX)
		{
			if (const TArray<int32>* List = RoadGrid.Find(C + FIntPoint(DX, DY)))
			{
				for (int32 Idx : *List)
				{
					Best = FMath::Min(Best, FVector2D::Distance(P, RoadSamples[Idx].P));
				}
			}
		}
	}
	return Best;
}

void AWorldGenerator::ConformTerrainToRoads()
{
	const double Cut = TerrainCutMeters * M2CM;
	const double Blend = EmbankmentBlendMeters * M2CM;
	const int32 Rings = FMath::Max(1, TruckSim::CeilI((Blend + 1200.0) / RoadGridCell));

	ParallelFor(Height.NumY, [&](int32 Y)
	{
		for (int32 X = 0; X < Height.NumX; ++X)
		{
			const FVector2D P = Height.GridToWorld(X, Y);
			const FIntPoint C(TruckSim::FloorI(P.X / RoadGridCell), TruckSim::FloorI(P.Y / RoadGridCell));
			double WSum = 0.0;
			double HSum = 0.0;
			double MaxW = 0.0;
			for (int32 DY = -Rings; DY <= Rings; ++DY)
			{
				for (int32 DX = -Rings; DX <= Rings; ++DX)
				{
					const TArray<int32>* List = RoadGrid.Find(C + FIntPoint(DX, DY));
					if (!List)
					{
						continue;
					}
					for (int32 Idx : *List)
					{
						const FRoadSampleRef& S = RoadSamples[Idx];
						const double D = FVector2D::Distance(P, S.P);
						if (D >= S.Core + Blend)
						{
							continue;
						}
						double W;
						double Target;
						if (D <= S.Core)
						{
							W = 1.0;
							Target = S.Z - Cut;
						}
						else
						{
							const double T = (D - S.Core) / Blend;
							W = 1.0 - T * T * (3.0 - 2.0 * T);
							Target = S.Z - Cut * W;
						}
						const double W4 = W * W * W * W;
						HSum += W4 * Target;
						WSum += W4;
						MaxW = FMath::Max(MaxW, W);
					}
				}
			}
			if (WSum > 0.0)
			{
				const double H0 = Height.At(X, Y);
				Height.At(X, Y) = static_cast<float>(FMath::Lerp(H0, HSum / WSum, MaxW));
			}
		}
	});
}

// =============================================================================================
// Meshes
// =============================================================================================

UProceduralMeshComponent* AWorldGenerator::NewProcMesh(const FString& Name)
{
	UProceduralMeshComponent* Mesh = NewObject<UProceduralMeshComponent>(this, FName(*Name), RF_Transient);
	Mesh->bUseAsyncCooking = false; // collision must exist before the truck spawns
	Mesh->bUseComplexAsSimpleCollision = true;
	Mesh->SetCollisionProfileName(UCollisionProfile::BlockAll_ProfileName);
	Mesh->SetupAttachment(RootComponent);
	Mesh->RegisterComponent();
	AddInstanceComponent(Mesh);
	MeshComponents.Add(Mesh);
	return Mesh;
}

UHierarchicalInstancedStaticMeshComponent* AWorldGenerator::NewInstancer(const FString& Name, UStaticMesh* Mesh, UMaterialInterface* Material, bool bCollision)
{
	UHierarchicalInstancedStaticMeshComponent* Inst = NewObject<UHierarchicalInstancedStaticMeshComponent>(this, FName(*Name), RF_Transient);
	Inst->SetStaticMesh(Mesh);
	Inst->SetCollisionProfileName(bCollision ? UCollisionProfile::BlockAll_ProfileName : UCollisionProfile::NoCollision_ProfileName);
	Inst->SetupAttachment(RootComponent);
	if (Material)
	{
		Inst->SetMaterial(0, Material);
	}
	Inst->RegisterComponent();
	AddInstanceComponent(Inst);
	InstanceComponents.Add(Inst);
	return Inst;
}

UMaterialInterface* AWorldGenerator::ColorMaterial(const FLinearColor& Color, UMaterialInterface* Override)
{
	if (Override)
	{
		return Override;
	}
	UMaterialInstanceDynamic* MID = TruckSim::MakeColorMaterial(this, Color);
	if (MID)
	{
		Materials.Add(MID);
	}
	return MID;
}

void AWorldGenerator::BuildTerrainMesh()
{
	const int32 Cells = Height.NumX - 1;
	const int32 TileCells = FMath::Max(1, Cells / TilesPerSide);
	const int32 NumTiles = (Cells + TileCells - 1) / TileCells;

	UMaterialInterface* GrassMat = ColorMaterial(FLinearColor(0.10f, 0.17f, 0.05f), TerrainMaterial);
	UMaterialInterface* RockMat = ColorMaterial(FLinearColor(0.24f, 0.22f, 0.19f), TerrainMaterial);
	const double RockZ = RockSlopeNormalZ;

	for (int32 TY = 0; TY < NumTiles; ++TY)
	{
		for (int32 TX = 0; TX < NumTiles; ++TX)
		{
			const int32 X0 = TX * TileCells;
			const int32 Y0 = TY * TileCells;
			const int32 X1 = FMath::Min(X0 + TileCells, Cells);
			const int32 Y1 = FMath::Min(Y0 + TileCells, Cells);
			const int32 W = X1 - X0 + 1;
			const int32 H = Y1 - Y0 + 1;

			FMeshBuffers Sections[2];
			TArray<int32> Remap[2];
			Remap[0].Init(INDEX_NONE, W * H);
			Remap[1].Init(INDEX_NONE, W * H);

			auto GetVert = [&](int32 Sec, int32 GX, int32 GY) -> int32
			{
				int32& Slot = Remap[Sec][(GY - Y0) * W + (GX - X0)];
				if (Slot == INDEX_NONE)
				{
					const FVector2D P2 = Height.GridToWorld(GX, GY);
					const FVector P(P2.X, P2.Y, Height.At(GX, GY));
					const FVector N = Height.Normal(GX, GY);
					FLinearColor Color;
					if (Sec == 0)
					{
						const float Tint = 0.5f + 0.5f * FMath::PerlinNoise2D(P2 / 40000.0 + NoiseOffsets[7]);
						Color = FMath::Lerp(FLinearColor(0.16f, 0.26f, 0.08f), FLinearColor(0.32f, 0.30f, 0.15f), Tint);
					}
					else
					{
						Color = FLinearColor(0.33f, 0.31f, 0.28f);
					}
					Slot = Sections[Sec].AddVertex(P, N, P2 / 800.0, Color);
				}
				return Slot;
			};

			auto AddTri = [&](int32 AX, int32 AY, int32 BX, int32 BY, int32 CX, int32 CY)
			{
				const FVector2D A2 = Height.GridToWorld(AX, AY);
				const FVector2D B2 = Height.GridToWorld(BX, BY);
				const FVector2D C2 = Height.GridToWorld(CX, CY);
				const FVector A(A2.X, A2.Y, Height.At(AX, AY));
				const FVector B(B2.X, B2.Y, Height.At(BX, BY));
				const FVector C(C2.X, C2.Y, Height.At(CX, CY));
				const FVector Nf = FVector::CrossProduct(B - A, C - A).GetSafeNormal();
				const int32 Sec = Nf.Z < RockZ ? 1 : 0;
				Sections[Sec].Triangles.Add(GetVert(Sec, AX, AY));
				Sections[Sec].Triangles.Add(GetVert(Sec, BX, BY));
				Sections[Sec].Triangles.Add(GetVert(Sec, CX, CY));
			};

			for (int32 CY = Y0; CY < Y1; ++CY)
			{
				for (int32 CX = X0; CX < X1; ++CX)
				{
					// (x,y) -> (x+1,y) -> (x,y+1) faces up in Unreal's left handed frame.
					AddTri(CX, CY, CX + 1, CY, CX, CY + 1);
					AddTri(CX + 1, CY, CX + 1, CY + 1, CX, CY + 1);
				}
			}

			UProceduralMeshComponent* Mesh = NewProcMesh(FString::Printf(TEXT("Terrain_%d_%d"), TX, TY));
			for (int32 Sec = 0; Sec < 2; ++Sec)
			{
				if (!Sections[Sec].IsEmpty())
				{
					Sections[Sec].CreateSection(Mesh, Sec, true);
					Mesh->SetMaterial(Sec, Sec == 0 ? GrassMat : RockMat);
				}
			}
		}
	}
}

void AWorldGenerator::ComputeRoadFrames(const FSimRoad& Road, TArray<FVector>& OutRight, TArray<FVector>& OutUp, TArray<double>& OutCurvature) const
{
	const TArray<FVector>& P = Road.Center.Points;
	const int32 N = P.Num();
	const bool bClosed = Road.Center.bClosed;
	OutRight.SetNum(N);
	OutUp.SetNum(N);
	OutCurvature.SetNum(N);
	for (int32 i = 0; i < N; ++i)
	{
		const FVector& Prev = bClosed ? P[(i - 1 + N) % N] : P[FMath::Max(i - 1, 0)];
		const FVector& Next = bClosed ? P[(i + 1) % N] : P[FMath::Min(i + 1, N - 1)];
		const FVector Fwd = (Next - Prev).GetSafeNormal();
		const FVector Flat = FVector(Fwd.X, Fwd.Y, 0.0).GetSafeNormal();
		const FVector Right(-Flat.Y, Flat.X, 0.0); // Up x Forward
		OutRight[i] = Right;
		OutUp[i] = FVector::CrossProduct(Fwd, Right).GetSafeNormal();
		OutCurvature[i] = Road.Center.GetSignedCurvatureAtDistance(Road.Center.Distances[i], 1000.0);
	}
}

void AWorldGenerator::BuildRoadMeshes()
{
	UMaterialInterface* AsphaltMat = ColorMaterial(FLinearColor(0.035f, 0.035f, 0.04f), RoadMaterial);
	UMaterialInterface* ShoulderMat = ColorMaterial(FLinearColor(0.20f, 0.18f, 0.14f));
	UMaterialInterface* WhiteMat = ColorMaterial(FLinearColor(0.75f, 0.75f, 0.72f));
	UMaterialInterface* YellowMat = ColorMaterial(FLinearColor(0.80f, 0.55f, 0.05f));

	const FLinearColor Asphalt(0.05f, 0.05f, 0.055f);
	const FLinearColor Gravel(0.25f, 0.22f, 0.18f);
	const FLinearColor White(0.9f, 0.9f, 0.9f);
	const FLinearColor Yellow(0.9f, 0.65f, 0.05f);
	const double SkirtDepth = 250.0;
	const int32 ChunkSegments = 200;

	for (int32 RoadIdx = 0; RoadIdx < Roads.Num(); ++RoadIdx)
	{
		const FSimRoad& Road = Roads[RoadIdx];
		const TArray<FVector>& P = Road.Center.Points;
		const TArray<double>& D = Road.Center.Distances;
		const int32 N = P.Num();
		const int32 NumSeg = Road.Center.NumSegments();
		TArray<FVector> Right, Up;
		TArray<double> Curv;
		ComputeRoadFrames(Road, Right, Up, Curv);

		const double HW = Road.HalfWidth;
		const double SH = Road.Shoulder;

		for (int32 ChunkStart = 0; ChunkStart < NumSeg; ChunkStart += ChunkSegments)
		{
			FMeshBuffers AsphaltBuf, ShoulderBuf, WhiteBuf, YellowBuf;
			const int32 ChunkEnd = FMath::Min(ChunkStart + ChunkSegments, NumSeg);

			for (int32 i = ChunkStart; i < ChunkEnd; ++i)
			{
				const int32 j = (i + 1) % N;
				const double DMid = 0.5 * (D[i] + D[i + 1]);
				if (Road.IsInGap(Road.SurfaceGaps, DMid))
				{
					continue;
				}
				auto Off = [&](int32 K, double Lat, double Dz) { return P[K] + Right[K] * Lat + FVector(0.0, 0.0, Dz); };
				const double V0 = D[i] / (2.0 * HW);
				const double V1 = D[i + 1] / (2.0 * HW);

				// Asphalt.
				AsphaltBuf.AddQuad(Off(i, -HW, 0), Off(j, -HW, 0), Off(i, HW, 0), Off(j, HW, 0), Up[i], Up[j], V0, V1, 0.0, 1.0, Asphalt);
				// Shoulders (slightly lower, sloping away).
				ShoulderBuf.AddQuad(Off(i, -HW - SH, -12), Off(j, -HW - SH, -12), Off(i, -HW, -2), Off(j, -HW, -2), Up[i], Up[j], V0, V1, 0.0, 0.3, Gravel);
				ShoulderBuf.AddQuad(Off(i, HW, -2), Off(j, HW, -2), Off(i, HW + SH, -12), Off(j, HW + SH, -12), Up[i], Up[j], V0, V1, 0.7, 1.0, Gravel);
				// Skirts hide gaps where the road sits above the terrain. Left: A = bottom, Right: A = top.
				ShoulderBuf.AddQuad(Off(i, -HW - SH - 40, -SkirtDepth), Off(j, -HW - SH - 40, -SkirtDepth), Off(i, -HW - SH, -12), Off(j, -HW - SH, -12),
					-Right[i], -Right[j], V0, V1, 0.0, 0.1, Gravel);
				ShoulderBuf.AddQuad(Off(i, HW + SH, -12), Off(j, HW + SH, -12), Off(i, HW + SH + 40, -SkirtDepth), Off(j, HW + SH + 40, -SkirtDepth),
					Right[i], Right[j], V0, V1, 0.9, 1.0, Gravel);

				if (Road.IsInGap(Road.MarkingGaps, DMid))
				{
					continue;
				}
				auto AddLine = [&](FMeshBuffers& Buf, double CenterLat, double Width, const FLinearColor& Color)
				{
					Buf.AddQuad(Off(i, CenterLat - Width * 0.5, 3), Off(j, CenterLat - Width * 0.5, 3),
						Off(i, CenterLat + Width * 0.5, 3), Off(j, CenterLat + Width * 0.5, 3), Up[i], Up[j], V0, V1, 0.0, 1.0, Color);
				};
				// Edge lines.
				AddLine(WhiteBuf, -HW + 25.0, 15.0, White);
				AddLine(WhiteBuf, HW - 25.0, 15.0, White);
				// Centre line: double solid yellow on curves (no overtaking), dashed elsewhere.
				const double Kappa = FMath::Max(FMath::Abs(Curv[i]), FMath::Abs(Curv[j]));
				if (Kappa > 1.0 / 25000.0)
				{
					AddLine(YellowBuf, -11.0, 10.0, Yellow);
					AddLine(YellowBuf, 11.0, 10.0, Yellow);
				}
				else if (FMath::Fmod(DMid, 900.0) < 300.0)
				{
					AddLine(WhiteBuf, 0.0, 12.0, White);
				}
			}

			if (AsphaltBuf.IsEmpty())
			{
				continue;
			}
			UProceduralMeshComponent* Mesh = NewProcMesh(FString::Printf(TEXT("Road_%d_%d"), RoadIdx, ChunkStart / ChunkSegments));
			AsphaltBuf.CreateSection(Mesh, 0, true);
			Mesh->SetMaterial(0, AsphaltMat);
			if (!ShoulderBuf.IsEmpty())
			{
				ShoulderBuf.CreateSection(Mesh, 1, true);
				Mesh->SetMaterial(1, ShoulderMat);
			}
			if (!WhiteBuf.IsEmpty())
			{
				WhiteBuf.CreateSection(Mesh, 2, false);
				Mesh->SetMaterial(2, WhiteMat);
			}
			if (!YellowBuf.IsEmpty())
			{
				YellowBuf.CreateSection(Mesh, 3, false);
				Mesh->SetMaterial(3, YellowMat);
			}
		}
	}
}

// =============================================================================================
// Props
// =============================================================================================

void AWorldGenerator::BuildGuardRails()
{
	UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (!Cube)
	{
		return;
	}
	TArray<FTransform> Posts;
	TArray<FTransform> Rails;
	TArray<FTransform> Chevrons;
	const int32 PostEvery = FMath::Max(1, TruckSim::RoundI(4.0 / RoadSampleSpacingMeters));

	for (int32 RoadIdx = 0; RoadIdx < Roads.Num(); ++RoadIdx)
	{
		const FSimRoad& Road = Roads[RoadIdx];
		const TArray<FVector>& P = Road.Center.Points;
		const int32 N = P.Num();
		TArray<FVector> Right, Up;
		TArray<double> Curv;
		ComputeRoadFrames(Road, Right, Up, Curv);
		const double Lat = Road.HalfWidth + Road.Shoulder - 35.0;

		for (int32 SideIdx = 0; SideIdx < 2; ++SideIdx)
		{
			const double Side = SideIdx == 0 ? -1.0 : 1.0;
			int32 LastPost = INDEX_NONE;
			FVector LastPos = FVector::ZeroVector;
			int32 ChevronCooldown = 0;

			for (int32 i = 0; i < N; i += PostEvery)
			{
				const double Dist = Road.Center.Distances[i];
				bool bPlace = false;
				if (!Road.IsInGap(Road.MarkingGaps, Dist) && !Road.IsInGap(Road.SurfaceGaps, Dist))
				{
					// Outer side of tight curves (positive curvature = right turn, outer side = left).
					const double OuterSide = Curv[i] > 0.0 ? -1.0 : 1.0;
					const bool bTightCurve = FMath::Abs(Curv[i]) > 1.0 / 8000.0 && FMath::IsNearlyEqual(OuterSide, Side);
					// Drop off beside the road.
					const FVector Probe = P[i] + Right[i] * Side * (Road.HalfWidth + Road.Shoulder + 700.0);
					const bool bDrop = Height.Sample(To2D(Probe)) < P[i].Z - 250.0;
					bPlace = bTightCurve || bDrop;

					// Chevron boards on the outside of hairpins.
					if (bTightCurve && FMath::Abs(Curv[i]) > 1.0 / 3000.0 && ChevronCooldown <= 0)
					{
						const FVector Base = P[i] + Right[i] * Side * (Lat + 60.0);
						const FVector Facing = (-Right[i] * Side).GetSafeNormal2D();
						Chevrons.Add(FTransform(Facing.Rotation(), Base + FVector(0, 0, 130.0), FVector(0.04, 0.9, 0.6)));
						ChevronCooldown = 3;
					}
				}
				--ChevronCooldown;

				if (!bPlace)
				{
					LastPost = INDEX_NONE;
					continue;
				}
				const FVector Pos = P[i] + Right[i] * Side * Lat + FVector(0.0, 0.0, 33.0);
				Posts.Add(FTransform(FRotator::ZeroRotator, Pos, FVector(0.12, 0.12, 0.9)));
				if (LastPost != INDEX_NONE && i - LastPost == PostEvery)
				{
					const FVector A = LastPos + FVector(0.0, 0.0, 25.0);
					const FVector B = Pos + FVector(0.0, 0.0, 25.0);
					const FVector Mid = 0.5 * (A + B);
					const double Len = FVector::Dist(A, B);
					Rails.Add(FTransform((B - A).Rotation(), Mid - Right[i] * Side * 8.0, FVector(Len / 100.0 + 0.05, 0.05, 0.32)));
				}
				LastPost = i;
				LastPos = Pos;
			}
		}
	}

	if (Posts.Num() > 0)
	{
		NewInstancer(TEXT("GuardRailPosts"), Cube, ColorMaterial(FLinearColor(0.35f, 0.35f, 0.34f)), true)->AddInstances(Posts, false);
	}
	if (Rails.Num() > 0)
	{
		NewInstancer(TEXT("GuardRails"), Cube, ColorMaterial(FLinearColor(0.55f, 0.56f, 0.58f)), true)->AddInstances(Rails, false);
	}
	if (Chevrons.Num() > 0)
	{
		NewInstancer(TEXT("Chevrons"), Cube, ColorMaterial(FLinearColor(0.9f, 0.6f, 0.02f)), true)->AddInstances(Chevrons, false);
	}
}

void AWorldGenerator::BuildTreesAndRocks()
{
	UStaticMesh* Cylinder = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	UStaticMesh* Cone = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cone.Cone"));
	UStaticMesh* Sphere = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	if (!Cylinder || !Cone || !Sphere)
	{
		return;
	}

	const float HalfF = static_cast<float>(GetMapHalfSize() * 0.93);
	TArray<FTransform> Trunks, Canopies, CanopiesDark, Rocks;

	auto SlopeNormalAt = [this](const FVector2D& P)
	{
		const int32 X = FMath::Clamp(TruckSim::RoundI((P.X - Height.Origin.X) / Height.CellSize), 0, Height.NumX - 1);
		const int32 Y = FMath::Clamp(TruckSim::RoundI((P.Y - Height.Origin.Y) / Height.CellSize), 0, Height.NumY - 1);
		return Height.Normal(X, Y);
	};

	const int32 MaxAttempts = NumTrees * 4;
	for (int32 Attempt = 0; Attempt < MaxAttempts && Trunks.Num() < NumTrees; ++Attempt)
	{
		// Clustered forests: accept more readily where a low frequency noise is high.
		const FVector2D P(Stream.FRandRange(-HalfF, HalfF), Stream.FRandRange(-HalfF, HalfF));
		const float Forest = FMath::PerlinNoise2D(P / 120000.0 + NoiseOffsets[5]);
		if (Stream.FRand() > 0.25f + Forest * 1.2f)
		{
			continue;
		}
		const double Z = Height.Sample(P);
		if (Z > TreeLineMeters * M2CM || SlopeNormalAt(P).Z < 0.82)
		{
			continue;
		}
		if (DistanceToNearestRoad(P, 2500.0) < RoadHalfWidthMeters * M2CM + ShoulderWidthMeters * M2CM + 700.0)
		{
			continue;
		}
		const double TrunkH = Stream.FRandRange(2.5f, 5.0f);
		const double CanopyH = Stream.FRandRange(5.0f, 10.0f);
		const double CanopyD = Stream.FRandRange(2.8f, 4.8f);
		Trunks.Add(FTransform(FRotator::ZeroRotator, FVector(P, Z + TrunkH * 50.0 - 20.0), FVector(0.35, 0.35, TrunkH)));
		const FTransform Canopy(FRotator(0.0, Stream.FRandRange(0.f, 360.f), 0.0), FVector(P, Z + TrunkH * 80.0 + CanopyH * 50.0), FVector(CanopyD, CanopyD, CanopyH));
		(Stream.FRand() < 0.5f ? Canopies : CanopiesDark).Add(Canopy);
	}

	for (int32 Attempt = 0; Attempt < NumRocks * 6 && Rocks.Num() < NumRocks; ++Attempt)
	{
		const FVector2D P(Stream.FRandRange(-HalfF, HalfF), Stream.FRandRange(-HalfF, HalfF));
		const FVector N = SlopeNormalAt(P);
		if (N.Z > 0.9 && Stream.FRand() > 0.1f)
		{
			continue; // prefer slopes
		}
		if (DistanceToNearestRoad(P, 2500.0) < RoadHalfWidthMeters * M2CM + ShoulderWidthMeters * M2CM + 400.0)
		{
			continue;
		}
		const double Size = Stream.FRandRange(0.8f, 4.5f);
		const FVector Scale(Size * Stream.FRandRange(0.8f, 1.4f), Size * Stream.FRandRange(0.8f, 1.4f), Size * Stream.FRandRange(0.4f, 0.8f));
		const FRotator Rot(Stream.FRandRange(-15.f, 15.f), Stream.FRandRange(0.f, 360.f), Stream.FRandRange(-15.f, 15.f));
		Rocks.Add(FTransform(Rot, FVector(P, Height.Sample(P) + Scale.Z * 20.0), Scale));
	}

	if (Trunks.Num() > 0)
	{
		UHierarchicalInstancedStaticMeshComponent* TrunkInst = NewInstancer(TEXT("TreeTrunks"), Cylinder, ColorMaterial(FLinearColor(0.12f, 0.08f, 0.05f)), true);
		TrunkInst->SetCullDistances(0, 250000);
		TrunkInst->AddInstances(Trunks, false);
	}
	if (Canopies.Num() > 0)
	{
		UHierarchicalInstancedStaticMeshComponent* Inst = NewInstancer(TEXT("TreeCanopies"), Cone, ColorMaterial(FLinearColor(0.05f, 0.13f, 0.04f)), false);
		Inst->SetCullDistances(0, 300000);
		Inst->AddInstances(Canopies, false);
	}
	if (CanopiesDark.Num() > 0)
	{
		UHierarchicalInstancedStaticMeshComponent* Inst = NewInstancer(TEXT("TreeCanopiesDark"), Cone, ColorMaterial(FLinearColor(0.03f, 0.09f, 0.035f)), false);
		Inst->SetCullDistances(0, 300000);
		Inst->AddInstances(CanopiesDark, false);
	}
	if (Rocks.Num() > 0)
	{
		UHierarchicalInstancedStaticMeshComponent* Inst = NewInstancer(TEXT("Rocks"), Sphere, ColorMaterial(FLinearColor(0.28f, 0.27f, 0.25f)), true);
		Inst->SetCullDistances(0, 300000);
		Inst->AddInstances(Rocks, false);
	}
}

// =============================================================================================
// Traffic lanes & signals
// =============================================================================================

void AWorldGenerator::BuildLanes()
{
	Lanes.Reset();
	const double Side = bLeftHandTraffic ? -1.0 : 1.0;
	const double LateralAccel = TrafficLateralAccel * M2CM;

	for (int32 RoadIdx = 0; RoadIdx < Roads.Num(); ++RoadIdx)
	{
		FSimRoad& Road = Roads[RoadIdx];
		const TArray<FVector>& P = Road.Center.Points;
		const int32 N = P.Num();
		TArray<FVector> Right, Up;
		TArray<double> Curv;
		ComputeRoadFrames(Road, Right, Up, Curv);
		const double Offset = Road.HalfWidth * 0.5 * Side;

		FTrafficLane Fwd;
		FTrafficLane Bwd;
		Fwd.RoadIndex = Bwd.RoadIndex = RoadIdx;
		Fwd.SpeedLimit = Bwd.SpeedLimit = Road.SpeedLimit;
		Fwd.Path.bClosed = Bwd.Path.bClosed = Road.Center.bClosed;
		Fwd.Path.Points.Reserve(N);
		Bwd.Path.Points.Reserve(N);
		for (int32 i = 0; i < N; ++i)
		{
			Fwd.Path.Points.Add(P[i] + Right[i] * Offset);
		}
		for (int32 i = N - 1; i >= 0; --i)
		{
			Bwd.Path.Points.Add(P[i] - Right[i] * Offset);
		}
		Fwd.Path.Build();
		Bwd.Path.Build();
		Fwd.BuildSpeedProfile(LateralAccel);
		Bwd.BuildSpeedProfile(LateralAccel);
		Road.ForwardLane = Lanes.Add(MoveTemp(Fwd));
		Road.BackwardLane = Lanes.Add(MoveTemp(Bwd));
	}

	auto MakeExit = [](int32 Target, double At, double TargetDist, double Prob, bool bTerminal)
	{
		FLaneExit Exit;
		Exit.TargetLane = Target;
		Exit.AtDistance = At;
		Exit.TargetDistance = TargetDist;
		Exit.Probability = Prob;
		Exit.bTerminal = bTerminal;
		return Exit;
	};

	// T junctions on the ring road.
	for (FJunctionInfo& J : Junctions)
	{
		const FSimRoad& Main = Roads[J.MainRoad];
		const FSimRoad& Branch = Roads[J.BranchRoad];
		const FVector P3(J.Pos, 0.0);
		J.MainFwdLane = Main.ForwardLane;
		J.MainBwdLane = Main.BackwardLane;
		J.OutLane = J.bBranchStartsHere ? Branch.ForwardLane : Branch.BackwardLane;
		J.InLane = J.bBranchStartsHere ? Branch.BackwardLane : Branch.ForwardLane;
		J.MainFwdDist = Lanes[J.MainFwdLane].Path.FindClosestDistance(P3);
		J.MainBwdDist = Lanes[J.MainBwdLane].Path.FindClosestDistance(P3);
		J.OutDist = Lanes[J.OutLane].Path.FindClosestDistance(P3);
		J.InDist = Lanes[J.InLane].Path.FindClosestDistance(P3);

		Lanes[J.MainFwdLane].Exits.Add(MakeExit(J.OutLane, J.MainFwdDist, J.OutDist, TurnProbability, false));
		Lanes[J.MainBwdLane].Exits.Add(MakeExit(J.OutLane, J.MainBwdDist, J.OutDist, TurnProbability, false));
		Lanes[J.InLane].Exits.Add(MakeExit(J.MainFwdLane, J.InDist, J.MainFwdDist, 0.5, true));
		Lanes[J.InLane].Exits.Add(MakeExit(J.MainBwdLane, J.InDist, J.MainBwdDist, 0.5, true));
	}

	// 4 way crossings.
	for (FCrossingInfo& C : Crossings)
	{
		const FVector P3(C.Pos, 0.0);
		C.Lanes[0] = Roads[C.RoadA].ForwardLane;
		C.Lanes[1] = Roads[C.RoadA].BackwardLane;
		C.Lanes[2] = Roads[C.RoadB].ForwardLane;
		C.Lanes[3] = Roads[C.RoadB].BackwardLane;
		for (int32 k = 0; k < 4; ++k)
		{
			C.Dists[k] = Lanes[C.Lanes[k]].Path.FindClosestDistance(P3);
		}
		const double CrossTurn = TurnProbability * 0.5;
		for (int32 k = 0; k < 4; ++k)
		{
			const int32 OtherBase = k < 2 ? 2 : 0;
			for (int32 o = OtherBase; o < OtherBase + 2; ++o)
			{
				Lanes[C.Lanes[k]].Exits.Add(MakeExit(C.Lanes[o], C.Dists[k], C.Dists[o], CrossTurn, false));
			}
		}
	}

	for (FTrafficLane& Lane : Lanes)
	{
		Lane.Exits.Sort([](const FLaneExit& A, const FLaneExit& B) { return A.AtDistance < B.AtDistance; });
	}
}

void AWorldGenerator::BuildSignals()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}
	const double Side = bLeftHandTraffic ? -1.0 : 1.0;
	const double CurbOffset = RoadHalfWidthMeters * 50.0 + ShoulderWidthMeters * M2CM + 120.0;

	auto AddStop = [&](ATrafficSignal* Signal, int32 LaneIdx, double JunctionDist, double Setback, int32 Group)
	{
		FTrafficLane& Lane = Lanes[LaneIdx];
		const double StopD = Lane.Path.WrapDistance(JunctionDist - Setback);
		FLaneStop Stop;
		Stop.Distance = StopD;
		Stop.Signal = Signal;
		Stop.Group = Group;
		Lane.Stops.Add(Stop);

		const FVector Loc = Lane.Path.GetLocationAtDistance(StopD);
		const FVector Dir = Lane.Path.GetDirectionAtDistance(StopD, 300.0).GetSafeNormal2D();
		const FVector Right(-Dir.Y, Dir.X, 0.0);
		const FVector Ground = Loc + Right * Side * CurbOffset;
		Signal->AddSignalHead(FVector(Ground.X, Ground.Y, Loc.Z - 15.0), Dir, Group);
	};

	FActorSpawnParameters Params;
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	for (const FJunctionInfo& J : Junctions)
	{
		const FVector Loc(J.Pos, Lanes[J.MainFwdLane].Path.GetLocationAtDistance(J.MainFwdDist).Z);
		ATrafficSignal* Signal = World->SpawnActor<ATrafficSignal>(ATrafficSignal::StaticClass(), FTransform(Loc), Params);
		if (!Signal)
		{
			continue;
		}
		SpawnedActors.Add(Signal);
		const double Setback = RoadHalfWidthMeters * M2CM + 700.0;
		AddStop(Signal, J.MainFwdLane, J.MainFwdDist, Setback, 0);
		AddStop(Signal, J.MainBwdLane, J.MainBwdDist, Setback, 0);
		AddStop(Signal, J.InLane, J.InDist, RoadHalfWidthMeters * M2CM + 500.0, 1);
		Signal->SetPhaseOffset(Stream.FRandRange(0.f, 30.f));
	}
	for (const FCrossingInfo& C : Crossings)
	{
		const FVector Loc(C.Pos, Lanes[C.Lanes[0]].Path.GetLocationAtDistance(C.Dists[0]).Z);
		ATrafficSignal* Signal = World->SpawnActor<ATrafficSignal>(ATrafficSignal::StaticClass(), FTransform(Loc), Params);
		if (!Signal)
		{
			continue;
		}
		SpawnedActors.Add(Signal);
		const double Setback = RoadHalfWidthMeters * M2CM + 500.0;
		for (int32 k = 0; k < 4; ++k)
		{
			AddStop(Signal, C.Lanes[k], C.Dists[k], Setback, k < 2 ? 0 : 1);
		}
		Signal->SetPhaseOffset(Stream.FRandRange(0.f, 30.f));
	}
	for (FTrafficLane& Lane : Lanes)
	{
		Lane.Stops.Sort([](const FLaneStop& A, const FLaneStop& B) { return A.Distance < B.Distance; });
	}
}

void AWorldGenerator::SpawnEnvironment()
{
	UWorld* World = GetWorld();
	FActorSpawnParameters Params;
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	TActorIterator<ADirectionalLight> SunIt(World);
	if (!SunIt)
	{
		if (ADirectionalLight* Sun = World->SpawnActor<ADirectionalLight>(FVector(0.0, 0.0, 100000.0), FRotator(-38.0, 35.0, 0.0), Params))
		{
			if (UDirectionalLightComponent* Light = Cast<UDirectionalLightComponent>(Sun->GetLightComponent()))
			{
				Light->SetMobility(EComponentMobility::Movable);
				Light->SetIntensity(8.0f);
				Light->SetAtmosphereSunLight(true);
				Light->MarkRenderStateDirty();
			}
			SpawnedActors.Add(Sun);
		}
	}

	TActorIterator<ASkyAtmosphere> AtmoIt(World);
	if (!AtmoIt)
	{
		if (ASkyAtmosphere* Atmo = World->SpawnActor<ASkyAtmosphere>(FVector::ZeroVector, FRotator::ZeroRotator, Params))
		{
			SpawnedActors.Add(Atmo);
		}
	}

	TActorIterator<ASkyLight> SkyIt(World);
	if (!SkyIt)
	{
		if (ASkyLight* Sky = World->SpawnActor<ASkyLight>(FVector(0.0, 0.0, 50000.0), FRotator::ZeroRotator, Params))
		{
			if (USkyLightComponent* SkyComp = Sky->GetLightComponent())
			{
				SkyComp->SetMobility(EComponentMobility::Movable);
				SkyComp->bRealTimeCapture = true;
				SkyComp->SetIntensity(1.0f);
				SkyComp->MarkRenderStateDirty();
				SkyComp->RecaptureSky();
			}
			SpawnedActors.Add(Sky);
		}
	}

	TActorIterator<AExponentialHeightFog> FogIt(World);
	if (!FogIt)
	{
		if (AExponentialHeightFog* Fog = World->SpawnActor<AExponentialHeightFog>(FVector(0.0, 0.0, 0.0), FRotator::ZeroRotator, Params))
		{
			if (UExponentialHeightFogComponent* FogComp = Fog->GetComponent())
			{
				FogComp->SetFogDensity(0.01f);
				FogComp->SetFogHeightFalloff(0.05f);
			}
			SpawnedActors.Add(Fog);
		}
	}
}

// =============================================================================================
// Queries
// =============================================================================================

bool AWorldGenerator::FindNearestRoadPoint(const FVector& Location, int32& OutRoad, double& OutDistance, double& OutDistSq) const
{
	OutRoad = INDEX_NONE;
	OutDistance = 0.0;
	OutDistSq = TNumericLimits<double>::Max();
	const FVector2D L = To2D(Location);
	for (int32 R = 0; R < Roads.Num(); ++R)
	{
		const FSimRoad& Road = Roads[R];
		for (int32 i = 0; i < Road.Center.Points.Num(); ++i)
		{
			const double D = FVector2D::DistSquared(To2D(Road.Center.Points[i]), L);
			if (D < OutDistSq)
			{
				OutDistSq = D;
				OutRoad = R;
				OutDistance = Road.Center.Distances[i];
			}
		}
	}
	return OutRoad != INDEX_NONE;
}
