#include "TruckSimTypes.h"
#include "Algo/BinarySearch.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"

UMaterialInterface* TruckSim::GetBasicShapeMaterial()
{
	static TWeakObjectPtr<UMaterialInterface> Cached;
	if (!Cached.IsValid())
	{
		Cached = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	}
	return Cached.Get();
}

UMaterialInstanceDynamic* TruckSim::MakeColorMaterial(UObject* Outer, const FLinearColor& Color, UMaterialInterface* Parent)
{
	if (!Parent)
	{
		Parent = GetBasicShapeMaterial();
	}
	if (!Parent)
	{
		return nullptr;
	}
	UMaterialInstanceDynamic* MID = UMaterialInstanceDynamic::Create(Parent, Outer);
	if (MID)
	{
		MID->SetVectorParameterValue(TEXT("Color"), Color);
		MID->SetVectorParameterValue(TEXT("BaseColor"), Color);
	}
	return MID;
}

// ---------------------------------------------------------------------------------------------
// FSimPolyline
// ---------------------------------------------------------------------------------------------

int32 FSimPolyline::NumSegments() const
{
	if (Points.Num() < 2)
	{
		return 0;
	}
	return bClosed ? Points.Num() : Points.Num() - 1;
}

void FSimPolyline::Build()
{
	Distances.Reset(Points.Num() + 1);
	Distances.Add(0.0);
	const int32 NumSeg = NumSegments();
	for (int32 i = 0; i < NumSeg; ++i)
	{
		const FVector& A = Points[i];
		const FVector& B = Points[(i + 1) % Points.Num()];
		Distances.Add(Distances.Last() + FVector::Dist(A, B));
	}
}

double FSimPolyline::WrapDistance(double D) const
{
	const double L = Length();
	if (L <= 0.0)
	{
		return 0.0;
	}
	if (bClosed)
	{
		D = FMath::Fmod(D, L);
		if (D < 0.0)
		{
			D += L;
		}
		return D;
	}
	return FMath::Clamp(D, 0.0, L);
}

int32 FSimPolyline::FindSegment(double D, double& OutAlpha) const
{
	const int32 NumSeg = NumSegments();
	OutAlpha = 0.0;
	if (NumSeg <= 0 || Distances.Num() != NumSeg + 1)
	{
		return INDEX_NONE;
	}
	D = WrapDistance(D);
	int32 Idx = Algo::UpperBound(Distances, D) - 1;
	Idx = FMath::Clamp(Idx, 0, NumSeg - 1);
	const double SegLen = Distances[Idx + 1] - Distances[Idx];
	OutAlpha = SegLen > KINDA_SMALL_NUMBER ? FMath::Clamp((D - Distances[Idx]) / SegLen, 0.0, 1.0) : 0.0;
	return Idx;
}

FVector FSimPolyline::GetLocationAtDistance(double D) const
{
	if (Points.Num() == 0)
	{
		return FVector::ZeroVector;
	}
	if (Points.Num() == 1)
	{
		return Points[0];
	}
	double Alpha = 0.0;
	const int32 Idx = FindSegment(D, Alpha);
	if (Idx == INDEX_NONE)
	{
		return Points[0];
	}
	return FMath::Lerp(Points[Idx], Points[(Idx + 1) % Points.Num()], Alpha);
}

FVector FSimPolyline::GetDirectionAtDistance(double D, double Window) const
{
	if (Points.Num() < 2)
	{
		return FVector::ForwardVector;
	}
	const FVector A = GetLocationAtDistance(D - Window);
	const FVector B = GetLocationAtDistance(D + Window);
	FVector Dir = (B - A).GetSafeNormal();
	if (Dir.IsNearlyZero())
	{
		double Alpha = 0.0;
		const int32 Idx = FMath::Max(FindSegment(D, Alpha), 0);
		Dir = (Points[(Idx + 1) % Points.Num()] - Points[Idx]).GetSafeNormal();
	}
	return Dir.IsNearlyZero() ? FVector::ForwardVector : Dir;
}

double FSimPolyline::GetSignedCurvatureAtDistance(double D, double Window) const
{
	const double D0 = bClosed ? D - Window : WrapDistance(D - Window);
	const double D1 = bClosed ? D + Window : WrapDistance(D + Window);
	const double Span = D1 - D0;
	if (Span < 1.0)
	{
		return 0.0;
	}
	const FVector A = GetDirectionAtDistance(D0, Window * 0.5);
	const FVector B = GetDirectionAtDistance(D1, Window * 0.5);
	const double YawA = FMath::Atan2(A.Y, A.X);
	const double YawB = FMath::Atan2(B.Y, B.X);
	return FMath::FindDeltaAngleRadians(YawA, YawB) / Span;
}

double FSimPolyline::FindClosestDistance(const FVector& P, bool bIgnoreZ, double* OutDistSq) const
{
	const int32 NumSeg = NumSegments();
	double Best = TNumericLimits<double>::Max();
	double BestD = 0.0;
	for (int32 i = 0; i < NumSeg; ++i)
	{
		FVector A = Points[i];
		FVector B = Points[(i + 1) % Points.Num()];
		FVector Q = P;
		if (bIgnoreZ)
		{
			A.Z = B.Z = Q.Z = 0.0;
		}
		const FVector AB = B - A;
		const double Len2 = AB.SizeSquared();
		const double T = Len2 > KINDA_SMALL_NUMBER ? FMath::Clamp(FVector::DotProduct(Q - A, AB) / Len2, 0.0, 1.0) : 0.0;
		const double DistSq = FVector::DistSquared(Q, A + AB * T);
		if (DistSq < Best)
		{
			Best = DistSq;
			BestD = Distances.IsValidIndex(i + 1) ? FMath::Lerp(Distances[i], Distances[i + 1], T) : 0.0;
		}
	}
	if (OutDistSq)
	{
		*OutDistSq = Best;
	}
	return BestD;
}

// ---------------------------------------------------------------------------------------------
// FTrafficLane
// ---------------------------------------------------------------------------------------------

void FTrafficLane::BuildSpeedProfile(double MaxLateralAccel)
{
	const double L = Path.Length();
	const int32 Num = FMath::Max(2, TruckSim::CeilI(L / ProfileStep) + 1);
	TArray<double> Raw;
	Raw.SetNum(Num);
	for (int32 k = 0; k < Num; ++k)
	{
		const double Kappa = FMath::Abs(Path.GetSignedCurvatureAtDistance(k * ProfileStep, 600.0));
		const double VCurve = Kappa > 1e-7 ? FMath::Sqrt(MaxLateralAccel / Kappa) : SpeedLimit;
		Raw[k] = FMath::Min(SpeedLimit, VCurve);
	}
	// Take the minimum of each sample and its neighbours to be conservative.
	SpeedProfile.SetNum(Num);
	for (int32 k = 0; k < Num; ++k)
	{
		double V = Raw[k];
		if (k > 0) { V = FMath::Min(V, Raw[k - 1]); }
		if (k < Num - 1) { V = FMath::Min(V, Raw[k + 1]); }
		SpeedProfile[k] = FMath::Max(V, 300.0);
	}
}

double FTrafficLane::GetProfileSpeed(double D) const
{
	if (SpeedProfile.Num() == 0)
	{
		return SpeedLimit;
	}
	const double Wrapped = Path.WrapDistance(D);
	const int32 Idx = FMath::Clamp(TruckSim::RoundI(Wrapped / ProfileStep), 0, SpeedProfile.Num() - 1);
	return SpeedProfile[Idx];
}

// ---------------------------------------------------------------------------------------------
// FSimRoad
// ---------------------------------------------------------------------------------------------

bool FSimRoad::IsInGap(const TArray<FVector2D>& Gaps, double D) const
{
	const double L = Center.Length();
	for (const FVector2D& Gap : Gaps)
	{
		if (D >= Gap.X && D <= Gap.Y)
		{
			return true;
		}
		if (Center.bClosed && ((D + L >= Gap.X && D + L <= Gap.Y) || (D - L >= Gap.X && D - L <= Gap.Y)))
		{
			return true;
		}
	}
	return false;
}

// ---------------------------------------------------------------------------------------------
// FTerrainHeightField
// ---------------------------------------------------------------------------------------------

void FTerrainHeightField::Init(int32 InNumX, int32 InNumY, double InCellSize, const FVector2D& InOrigin)
{
	NumX = InNumX;
	NumY = InNumY;
	CellSize = InCellSize;
	Origin = InOrigin;
	Heights.SetNumZeroed(NumX * NumY);
}

double FTerrainHeightField::Sample(const FVector2D& P) const
{
	if (!IsValid())
	{
		return 0.0;
	}
	const double FX = FMath::Clamp((P.X - Origin.X) / CellSize, 0.0, double(NumX - 1) - 1e-4);
	const double FY = FMath::Clamp((P.Y - Origin.Y) / CellSize, 0.0, double(NumY - 1) - 1e-4);
	const int32 X = TruckSim::FloorI(FX);
	const int32 Y = TruckSim::FloorI(FY);
	const double TX = FX - X;
	const double TY = FY - Y;
	const double H00 = At(X, Y);
	const double H10 = At(X + 1, Y);
	const double H01 = At(X, Y + 1);
	const double H11 = At(X + 1, Y + 1);
	return FMath::Lerp(FMath::Lerp(H00, H10, TX), FMath::Lerp(H01, H11, TX), TY);
}

FVector FTerrainHeightField::Normal(int32 X, int32 Y) const
{
	const int32 XL = FMath::Max(X - 1, 0);
	const int32 XR = FMath::Min(X + 1, NumX - 1);
	const int32 YD = FMath::Max(Y - 1, 0);
	const int32 YU = FMath::Min(Y + 1, NumY - 1);
	const double DzDx = (double(At(XR, Y)) - double(At(XL, Y))) / (FMath::Max(XR - XL, 1) * CellSize);
	const double DzDy = (double(At(X, YU)) - double(At(X, YD))) / (FMath::Max(YU - YD, 1) * CellSize);
	return FVector(-DzDx, -DzDy, 1.0).GetSafeNormal();
}
