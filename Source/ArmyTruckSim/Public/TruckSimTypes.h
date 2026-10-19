#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"

class ATrafficSignal;
class UMaterialInterface;
class UMaterialInstanceDynamic;

namespace TruckSim
{
	inline int32 FloorI(double V) { return static_cast<int32>(FMath::FloorToDouble(V)); }
	inline int32 CeilI(double V) { return static_cast<int32>(FMath::CeilToDouble(V)); }
	inline int32 RoundI(double V) { return static_cast<int32>(FMath::RoundToDouble(V)); }

	/** km/h -> cm/s */
	constexpr double KmhToCms = 100000.0 / 3600.0;

	/** /Engine/BasicShapes/BasicShapeMaterial (has a "Color" vector parameter). */
	ARMYTRUCKSIM_API UMaterialInterface* GetBasicShapeMaterial();

	/** Creates a dynamic instance of Parent (BasicShapeMaterial if null) with its Color parameter set. */
	ARMYTRUCKSIM_API UMaterialInstanceDynamic* MakeColorMaterial(UObject* Outer, const FLinearColor& Color, UMaterialInterface* Parent = nullptr);
}

/** Polyline with cumulative arc length. All coordinates are Unreal units (cm). */
struct ARMYTRUCKSIM_API FSimPolyline
{
	TArray<FVector> Points;
	/** Cumulative distance at the start of every segment, plus the total length as the last entry. */
	TArray<double> Distances;
	bool bClosed = false;

	void Build();
	int32 NumSegments() const;
	double Length() const { return Distances.Num() > 0 ? Distances.Last() : 0.0; }
	double WrapDistance(double D) const;
	FVector GetLocationAtDistance(double D) const;
	FVector GetDirectionAtDistance(double D, double Window = 200.0) const;
	/** Signed horizontal curvature (1/cm). Positive = heading increasing = turning right (towards +Y). */
	double GetSignedCurvatureAtDistance(double D, double Window = 600.0) const;
	/** Distance along the polyline of the point closest to P. */
	double FindClosestDistance(const FVector& P, bool bIgnoreZ = true, double* OutDistSq = nullptr) const;

private:
	int32 FindSegment(double D, double& OutAlpha) const;
};

enum class ESignalState : uint8
{
	Green,
	Yellow,
	Red
};

/** A possible transition from one lane to another (turn at a junction or the lane end). */
struct FLaneExit
{
	int32 TargetLane = INDEX_NONE;
	double AtDistance = 0.0;
	double TargetDistance = 0.0;
	double Probability = 1.0;
	/** Terminal exits sit at the end of an open lane; a vehicle must take one of them. */
	bool bTerminal = false;
};

/** A stop line controlled by a signal group. */
struct FLaneStop
{
	double Distance = 0.0;
	TWeakObjectPtr<ATrafficSignal> Signal;
	int32 Group = 0;
};

/** One directional traffic lane. */
struct ARMYTRUCKSIM_API FTrafficLane
{
	FSimPolyline Path;
	int32 RoadIndex = INDEX_NONE;
	/** cm/s */
	double SpeedLimit = 1500.0;
	TArray<FLaneExit> Exits;
	TArray<FLaneStop> Stops;

	/** Curvature limited speed (cm/s) sampled every ProfileStep cm. */
	TArray<double> SpeedProfile;
	double ProfileStep = 500.0;

	void BuildSpeedProfile(double MaxLateralAccel);
	double GetProfileSpeed(double D) const;
};

/** Road centre line plus cross section information. */
struct ARMYTRUCKSIM_API FSimRoad
{
	FString Name;
	FSimPolyline Center;
	double HalfWidth = 375.0;
	double Shoulder = 150.0;
	double SpeedLimit = 1500.0;
	int32 ForwardLane = INDEX_NONE;
	int32 BackwardLane = INDEX_NONE;
	/** [Start, End] distance ranges where no asphalt/shoulder is generated (junction overlaps). */
	TArray<FVector2D> SurfaceGaps;
	/** [Start, End] distance ranges without lane markings. */
	TArray<FVector2D> MarkingGaps;

	bool IsInGap(const TArray<FVector2D>& Gaps, double D) const;
};

/** Where a branch road meets the ring road (T junction). */
struct FJunctionInfo
{
	FVector2D Pos = FVector2D::ZeroVector;
	int32 MainRoad = INDEX_NONE;
	int32 BranchRoad = INDEX_NONE;
	bool bBranchStartsHere = true;

	int32 MainFwdLane = INDEX_NONE;
	int32 MainBwdLane = INDEX_NONE;
	int32 OutLane = INDEX_NONE; // branch lane leaving the junction
	int32 InLane = INDEX_NONE;  // branch lane arriving at the junction
	double MainFwdDist = 0.0;
	double MainBwdDist = 0.0;
	double OutDist = 0.0;
	double InDist = 0.0;
};

/** Where two roads cross (4 way junction). Lane order: A fwd, A bwd, B fwd, B bwd. */
struct FCrossingInfo
{
	FVector2D Pos = FVector2D::ZeroVector;
	int32 RoadA = INDEX_NONE;
	int32 RoadB = INDEX_NONE;
	int32 Lanes[4] = { INDEX_NONE, INDEX_NONE, INDEX_NONE, INDEX_NONE };
	double Dists[4] = { 0.0, 0.0, 0.0, 0.0 };
};

/** Road sample used for terrain conforming and spatial queries. */
struct FRoadSampleRef
{
	FVector2D P = FVector2D::ZeroVector;
	double Z = 0.0;
	/** Half width of the fully flattened corridor (cm). */
	double Core = 0.0;
	int32 Road = INDEX_NONE;
};

/** Regular height grid (cm). */
struct ARMYTRUCKSIM_API FTerrainHeightField
{
	int32 NumX = 0;
	int32 NumY = 0;
	double CellSize = 1000.0;
	FVector2D Origin = FVector2D::ZeroVector;
	TArray<float> Heights;

	void Init(int32 InNumX, int32 InNumY, double InCellSize, const FVector2D& InOrigin);
	bool IsValid() const { return Heights.Num() > 0; }
	float& At(int32 X, int32 Y) { return Heights[Y * NumX + X]; }
	float At(int32 X, int32 Y) const { return Heights[Y * NumX + X]; }
	FVector2D GridToWorld(int32 X, int32 Y) const { return Origin + FVector2D(X * CellSize, Y * CellSize); }
	double Sample(const FVector2D& P) const;
	FVector Normal(int32 X, int32 Y) const;
};
