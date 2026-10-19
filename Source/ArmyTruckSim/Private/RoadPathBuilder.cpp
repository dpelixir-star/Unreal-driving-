#include "RoadPathBuilder.h"
#include "TruckSimTypes.h"

FVector2D FRoadPathBuilder::HeadingToDir(double HeadingDeg)
{
	const double R = FMath::DegreesToRadians(HeadingDeg);
	return FVector2D(FMath::Cos(R), FMath::Sin(R));
}

double FRoadPathBuilder::DirToHeading(const FVector2D& Dir)
{
	return FMath::RadiansToDegrees(FMath::Atan2(Dir.Y, Dir.X));
}

FRoadPathBuilder::FRoadPathBuilder(const FVector2D& Start, double HeadingDeg, double InStep)
	: Pos(Start)
	, Heading(HeadingDeg)
	, Step(FMath::Max(InStep, 0.25))
{
	Points.Add(Start);
}

void FRoadPathBuilder::Straight(double Length)
{
	if (Length <= 0.0)
	{
		return;
	}
	const int32 N = FMath::Max(1, TruckSim::CeilI(Length / Step));
	const FVector2D Dir = HeadingToDir(Heading);
	const FVector2D Start = Pos;
	for (int32 i = 1; i <= N; ++i)
	{
		Points.Add(Start + Dir * (Length * i / N));
	}
	Pos = Points.Last();
}

void FRoadPathBuilder::Arc(double Radius, double AngleDeg)
{
	if (FMath::IsNearlyZero(AngleDeg) || Radius <= 0.0)
	{
		return;
	}
	const double S = AngleDeg > 0.0 ? 1.0 : -1.0;
	const FVector2D Center = Pos + HeadingToDir(Heading + 90.0 * S) * Radius;
	const double Phi0 = Heading - 90.0 * S;
	const double ArcLen = FMath::DegreesToRadians(FMath::Abs(AngleDeg)) * Radius;
	const int32 N = FMath::Max(2, TruckSim::CeilI(ArcLen / Step));
	for (int32 i = 1; i <= N; ++i)
	{
		const double Phi = Phi0 + AngleDeg * i / N;
		Points.Add(Center + HeadingToDir(Phi) * Radius);
	}
	Pos = Points.Last();
	Heading += AngleDeg;
}

void FRoadPathBuilder::HermiteTo(const FVector2D& Target, double TargetHeadingDeg, double TangentScale)
{
	const FVector2D P0 = Pos;
	const FVector2D P1 = Target;
	const double Dist = FVector2D::Distance(P0, P1);
	if (Dist < 1e-3)
	{
		Heading = TargetHeadingDeg;
		return;
	}
	const FVector2D T0 = HeadingToDir(Heading) * (Dist * TangentScale);
	const FVector2D T1 = HeadingToDir(TargetHeadingDeg) * (Dist * TangentScale);
	const int32 NumDense = FMath::Max(32, TruckSim::CeilI(Dist / Step * 4.0));

	TArray<FVector2D> Dense;
	Dense.Reserve(NumDense + 1);
	for (int32 k = 0; k <= NumDense; ++k)
	{
		const double T = double(k) / NumDense;
		const double T2 = T * T;
		const double T3 = T2 * T;
		const double H00 = 2.0 * T3 - 3.0 * T2 + 1.0;
		const double H10 = T3 - 2.0 * T2 + T;
		const double H01 = -2.0 * T3 + 3.0 * T2;
		const double H11 = T3 - T2;
		Dense.Add(P0 * H00 + T0 * H10 + P1 * H01 + T1 * H11);
	}
	ResampleAppend(Dense, Step, Points, false);
	Pos = Target;
	Heading = TargetHeadingDeg;
}

void FRoadPathBuilder::SmoothThrough(const TArray<FVector2D>& Waypoints, double EndHeadingDeg)
{
	TArray<FVector2D> Pts;
	Pts.Add(Pos);
	Pts.Append(Waypoints);
	for (int32 i = 1; i < Pts.Num(); ++i)
	{
		const double H = (i < Pts.Num() - 1) ? DirToHeading(Pts[i + 1] - Pts[i - 1]) : EndHeadingDeg;
		HermiteTo(Pts[i], H, 1.0);
	}
}

TArray<FVector2D> FRoadPathBuilder::ClosedLoop(const TArray<FVector2D>& Ctrl, double Step)
{
	TArray<FVector2D> Out;
	const int32 N = Ctrl.Num();
	if (N < 3)
	{
		return Out;
	}
	TArray<FVector2D> Dense;
	for (int32 i = 0; i < N; ++i)
	{
		const FVector2D& P0 = Ctrl[i];
		const FVector2D& P1 = Ctrl[(i + 1) % N];
		const FVector2D T0 = (Ctrl[(i + 1) % N] - Ctrl[(i - 1 + N) % N]) * 0.5;
		const FVector2D T1 = (Ctrl[(i + 2) % N] - Ctrl[i]) * 0.5;
		const int32 K = FMath::Max(16, TruckSim::CeilI(FVector2D::Distance(P0, P1) / Step * 4.0));
		for (int32 k = 0; k < K; ++k)
		{
			const double T = double(k) / K;
			const double T2 = T * T;
			const double T3 = T2 * T;
			Dense.Add(P0 * (2.0 * T3 - 3.0 * T2 + 1.0) + T0 * (T3 - 2.0 * T2 + T) + P1 * (-2.0 * T3 + 3.0 * T2) + T1 * (T3 - T2));
		}
	}
	Dense.Add(Ctrl[0]);
	ResampleAppend(Dense, Step, Out, true);
	if (Out.Num() > 1)
	{
		Out.Pop();
	}
	return Out;
}

void FRoadPathBuilder::ResampleAppend(const TArray<FVector2D>& Dense, double Step, TArray<FVector2D>& Out, bool bIncludeFirst)
{
	if (Dense.Num() == 0)
	{
		return;
	}
	if (bIncludeFirst)
	{
		Out.Add(Dense[0]);
	}
	if (Dense.Num() < 2)
	{
		return;
	}
	TArray<double> Cum;
	Cum.SetNum(Dense.Num());
	Cum[0] = 0.0;
	for (int32 i = 1; i < Dense.Num(); ++i)
	{
		Cum[i] = Cum[i - 1] + FVector2D::Distance(Dense[i - 1], Dense[i]);
	}
	const double Total = Cum.Last();
	if (Total < 1e-6)
	{
		return;
	}
	const int32 N = FMath::Max(1, TruckSim::RoundI(Total / Step));
	int32 Seg = 0;
	for (int32 k = 1; k <= N; ++k)
	{
		const double Target = Total * k / N;
		while (Seg < Dense.Num() - 2 && Cum[Seg + 1] < Target)
		{
			++Seg;
		}
		const double SegLen = Cum[Seg + 1] - Cum[Seg];
		const double Alpha = SegLen > 0.0 ? FMath::Clamp((Target - Cum[Seg]) / SegLen, 0.0, 1.0) : 0.0;
		Out.Add(FMath::Lerp(Dense[Seg], Dense[Seg + 1], Alpha));
	}
}
