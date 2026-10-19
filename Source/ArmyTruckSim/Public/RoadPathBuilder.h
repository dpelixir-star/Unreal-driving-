#pragma once

#include "CoreMinimal.h"

/**
 * Turtle style 2D path builder used to design the road network.
 * Units are metres, headings are degrees. Increasing the heading turns from +X towards +Y,
 * which in Unreal's left handed world is a right hand turn.
 */
class ARMYTRUCKSIM_API FRoadPathBuilder
{
public:
	FRoadPathBuilder(const FVector2D& Start, double HeadingDeg, double InStep = 2.0);

	/** Drive straight ahead. */
	void Straight(double Length);

	/** Circular arc. Positive angle increases the heading. A 150-180 degree arc with a small radius is a hairpin. */
	void Arc(double Radius, double AngleDeg);

	/** Cubic Hermite curve to Target, arriving with TargetHeadingDeg. */
	void HermiteTo(const FVector2D& Target, double TargetHeadingDeg, double TangentScale = 1.0);

	/** Catmull-Rom like smooth curve through the waypoints, arriving at the last one with EndHeadingDeg. */
	void SmoothThrough(const TArray<FVector2D>& Waypoints, double EndHeadingDeg);

	const TArray<FVector2D>& GetPoints() const { return Points; }
	FVector2D GetPosition() const { return Pos; }
	double GetHeading() const { return Heading; }

	static FVector2D HeadingToDir(double HeadingDeg);
	static double DirToHeading(const FVector2D& Dir);

	/** Closed Catmull-Rom loop through the control points, uniformly resampled. The closing point is not duplicated. */
	static TArray<FVector2D> ClosedLoop(const TArray<FVector2D>& ControlPoints, double Step);

	/** Appends Dense resampled to (approximately) uniform Step spacing. The last dense point is always included. */
	static void ResampleAppend(const TArray<FVector2D>& Dense, double Step, TArray<FVector2D>& Out, bool bIncludeFirst);

private:
	TArray<FVector2D> Points;
	FVector2D Pos;
	double Heading;
	double Step;
};
