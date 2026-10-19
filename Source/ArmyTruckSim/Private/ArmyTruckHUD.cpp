#include "ArmyTruckHUD.h"
#include "ArmyTruckPawn.h"
#include "WorldGenerator.h"
#include "TrafficManager.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "EngineUtils.h"

namespace
{
	const FLinearColor PanelColor(0.0f, 0.0f, 0.0f, 0.45f);
	const FLinearColor TextColor(0.92f, 0.92f, 0.85f);
	const FLinearColor Amber(1.0f, 0.7f, 0.1f);
	const FLinearColor Warn(1.0f, 0.25f, 0.15f);
	const FLinearColor OliveHud(0.55f, 0.65f, 0.3f);
}

void AArmyTruckHUD::Text(const FString& Str, float X, float Y, const FLinearColor& Color, float Scale, bool bLarge)
{
	UFont* Font = bLarge ? GEngine->GetLargeFont() : GEngine->GetMediumFont();
	DrawText(Str, Color, X, Y, Font, Scale);
}

void AArmyTruckHUD::DrawHUD()
{
	Super::DrawHUD();
	if (!Canvas)
	{
		return;
	}
	const float S = Canvas->ClipY / 1080.f;

	if (!Generator.IsValid())
	{
		Generator = AWorldGenerator::Find(this);
	}
	if (!Traffic.IsValid() && GetWorld())
	{
		for (TActorIterator<ATrafficManager> It(GetWorld()); It; ++It)
		{
			Traffic = *It;
			break;
		}
	}

	const AArmyTruckPawn* Truck = Cast<AArmyTruckPawn>(GetOwningPawn());
	if (Truck)
	{
		DrawDashboard(Truck, S);
		DrawAdvisories(Truck, S);
	}
	if (bShowMap)
	{
		DrawMap(Truck, S);
	}
	if (bShowHelp)
	{
		DrawHelp(S);
	}
	else
	{
		Text(TEXT("H: controls"), 20.f * S, 20.f * S, FLinearColor(1.f, 1.f, 1.f, 0.6f), 1.0f * S);
	}
}

void AArmyTruckHUD::DrawDashboard(const AArmyTruckPawn* Truck, float S)
{
	const float W = 430.f * S;
	const float H = 190.f * S;
	const float X = 20.f * S;
	const float Y = Canvas->ClipY - H - 20.f * S;
	DrawRect(PanelColor, X, Y, W, H);

	// Speed.
	const int32 Kmh = FMath::RoundToInt(static_cast<float>(FMath::Abs(Truck->GetSpeedKmh())));
	Text(FString::Printf(TEXT("%d"), Kmh), X + 18.f * S, Y + 8.f * S, TextColor, 3.0f * S, true);
	Text(TEXT("km/h"), X + 18.f * S, Y + 88.f * S, TextColor, 1.0f * S);

	// Gear.
	FString Gear;
	const int32 G = Truck->GetGear();
	if (G < 0) { Gear = TEXT("R"); }
	else if (G == 0) { Gear = TEXT("N"); }
	else { Gear = Truck->IsAutomatic() ? FString::Printf(TEXT("D%d"), G) : FString::Printf(TEXT("%d"), G); }
	Text(Gear, X + 170.f * S, Y + 8.f * S, Amber, 2.4f * S, true);
	Text(Truck->IsAutomatic() ? TEXT("AUTO") : TEXT("MANUAL"), X + 170.f * S, Y + 75.f * S, TextColor, 1.0f * S);
	Text(Truck->IsLowRange() ? TEXT("LOW RANGE") : TEXT("HIGH RANGE"), X + 170.f * S, Y + 95.f * S, Truck->IsLowRange() ? Amber : TextColor, 1.0f * S);

	// RPM bar.
	const double RPM = Truck->GetEngineRPM();
	const float BarX = X + 18.f * S;
	const float BarY = Y + 125.f * S;
	const float BarW = W - 36.f * S;
	const float BarH = 16.f * S;
	DrawRect(FLinearColor(0.15f, 0.15f, 0.15f, 0.8f), BarX, BarY, BarW, BarH);
	const float Frac = FMath::Clamp(static_cast<float>(RPM / 2800.0), 0.f, 1.f);
	DrawRect(Frac > 0.85f ? Warn : OliveHud, BarX, BarY, BarW * Frac, BarH);
	Text(FString::Printf(TEXT("%4.0f rpm"), RPM), BarX, BarY + 20.f * S, TextColor, 1.0f * S);

	// Status column.
	const double Grade = Truck->GetGradePercent();
	const float CX = X + 300.f * S;
	Text(FString::Printf(TEXT("Grade %+.0f%%"), Grade), CX, Y + 12.f * S, FMath::Abs(Grade) > 8.0 ? Amber : TextColor, 1.0f * S);
	Text(FString::Printf(TEXT("Mass %.1f t"), Truck->GetTotalMassKg() / 1000.0), CX, Y + 32.f * S, TextColor, 1.0f * S);
	if (Truck->IsHandbrakeOn())
	{
		Text(TEXT("(P) PARK BRAKE"), CX, Y + 52.f * S, Warn, 1.0f * S);
	}
	if (Truck->AreLightsOn())
	{
		Text(TEXT("LIGHTS"), CX, Y + 72.f * S, OliveHud, 1.0f * S);
	}
	if (Truck->IsAnyWheelSlipping())
	{
		Text(TEXT("WHEEL SLIP"), CX, Y + 92.f * S, Amber, 1.0f * S);
	}
}

void AArmyTruckHUD::DrawAdvisories(const AArmyTruckPawn* Truck, float S)
{
	const float CX = Canvas->ClipX * 0.5f;
	float Y = 40.f * S;

	if (Truck->IsFlipped())
	{
		Text(TEXT("TRUCK OVERTURNED - press R to recover"), CX - 220.f * S, Y, Warn, 1.6f * S, true);
		Y += 50.f * S;
	}

	AWorldGenerator* Gen = Generator.Get();
	if (!Gen)
	{
		return;
	}
	int32 RoadIdx;
	double Dist, DistSq;
	if (!Gen->FindNearestRoadPoint(Truck->GetActorLocation(), RoadIdx, Dist, DistSq))
	{
		return;
	}
	const FSimRoad& Road = Gen->GetRoads()[RoadIdx];
	const double Off = FMath::Sqrt(DistSq);
	if (Off > Road.HalfWidth + Road.Shoulder + 400.0)
	{
		Text(TEXT("OFF ROAD - press R to return to the road"), CX - 200.f * S, Y, Amber, 1.2f * S, true);
		return;
	}
	Text(Road.Name, Canvas->ClipX - 320.f * S, Canvas->ClipY - 50.f * S, TextColor, 1.2f * S);

	// Look ahead along the road in the truck's direction of travel for tight curves.
	const FVector RoadDir = Road.Center.GetDirectionAtDistance(Dist, 300.0);
	const double Sign = FVector::DotProduct(RoadDir, Truck->GetActorForwardVector()) >= 0.0 ? 1.0 : -1.0;
	double MinRadius = TNumericLimits<double>::Max();
	double AtDistance = 0.0;
	for (double Ahead = 1500.0; Ahead <= 15000.0; Ahead += 1000.0)
	{
		const double Kappa = FMath::Abs(Road.Center.GetSignedCurvatureAtDistance(Dist + Sign * Ahead, 800.0));
		if (Kappa > 1e-7)
		{
			const double Radius = 1.0 / Kappa;
			if (Radius < MinRadius)
			{
				MinRadius = Radius;
				AtDistance = Ahead;
			}
		}
	}
	if (MinRadius < 4500.0)
	{
		Text(FString::Printf(TEXT("HAIRPIN AHEAD  %.0f m  -  slow down, use low gear"), AtDistance / 100.0), CX - 260.f * S, Y, Warn, 1.4f * S, true);
	}
	else if (MinRadius < 12000.0)
	{
		Text(FString::Printf(TEXT("SHARP CURVE  %.0f m"), AtDistance / 100.0), CX - 110.f * S, Y, Amber, 1.3f * S, true);
	}
}

void AArmyTruckHUD::DrawMap(const AArmyTruckPawn* Truck, float S)
{
	AWorldGenerator* Gen = Generator.Get();
	if (!Gen || !Gen->IsGenerated())
	{
		return;
	}
	const float Size = 300.f * S;
	const float X0 = Canvas->ClipX - Size - 20.f * S;
	const float Y0 = 20.f * S;
	DrawRect(FLinearColor(0.05f, 0.07f, 0.04f, 0.65f), X0, Y0, Size, Size);

	const double Half = Gen->GetMapHalfSize();
	// Top-down with world +X up and +Y right (matches Unreal's top view).
	auto ToMap = [&](const FVector& P)
	{
		return FVector2D(X0 + (P.Y / Half * 0.5 + 0.5) * Size, Y0 + (0.5 - P.X / Half * 0.5) * Size);
	};

	const FLinearColor RoadColors[3] = { FLinearColor(0.85f, 0.85f, 0.8f), Amber, FLinearColor(0.6f, 0.75f, 0.9f) };
	const TArray<FSimRoad>& Roads = Gen->GetRoads();
	for (int32 R = 0; R < Roads.Num(); ++R)
	{
		const TArray<FVector>& P = Roads[R].Center.Points;
		const int32 N = P.Num();
		const int32 Last = Roads[R].Center.bClosed ? N : N - 1;
		const int32 Step = 8;
		for (int32 i = 0; i < Last; i += Step)
		{
			const int32 j = FMath::Min(i + Step, Last);
			const FVector2D A = ToMap(P[i]);
			const FVector2D B = ToMap(P[j % N]);
			DrawLine(A.X, A.Y, B.X, B.Y, RoadColors[R % 3], 2.f * S);
		}
	}

	if (ATrafficManager* TM = Traffic.Get())
	{
		for (const ATrafficVehicle* V : TM->GetVehicles())
		{
			if (V)
			{
				const FVector2D M = ToMap(V->GetActorLocation());
				DrawRect(FLinearColor(0.2f, 0.6f, 1.0f), M.X - 1.5f * S, M.Y - 1.5f * S, 3.f * S, 3.f * S);
			}
		}
	}

	if (Truck)
	{
		const FVector2D M = ToMap(Truck->GetActorLocation());
		const FVector F = Truck->GetActorForwardVector().GetSafeNormal2D();
		const FVector2D Fwd(F.Y, -F.X); // map space
		const FVector2D Right(-Fwd.Y, Fwd.X);
		const float L = 9.f * S;
		const FVector2D Tip = M + Fwd * L;
		const FVector2D BL = M - Fwd * L * 0.6f - Right * L * 0.6f;
		const FVector2D BR = M - Fwd * L * 0.6f + Right * L * 0.6f;
		DrawLine(Tip.X, Tip.Y, BL.X, BL.Y, Warn, 2.5f * S);
		DrawLine(Tip.X, Tip.Y, BR.X, BR.Y, Warn, 2.5f * S);
		DrawLine(BL.X, BL.Y, BR.X, BR.Y, Warn, 2.5f * S);
	}
	Text(TEXT("5 km x 5 km   (M: map)"), X0 + 6.f * S, Y0 + Size + 4.f * S, TextColor, 0.9f * S);
}

void AArmyTruckHUD::DrawHelp(float S)
{
	static const TCHAR* Lines[] = {
		TEXT("ARMY TRUCK SIMULATOR - 5 ton 6x6"),
		TEXT("W / S          accelerate / brake (hold S when stopped = reverse)"),
		TEXT("A / D          steer"),
		TEXT("Space          handbrake"),
		TEXT("T              automatic / manual gearbox"),
		TEXT("E / Q          shift up / down (manual)"),
		TEXT("L              low range (only below ~9 km/h) - use on hairpins"),
		TEXT("C              chase / cab camera,  mouse = look around"),
		TEXT("F              headlights"),
		TEXT("R              recover truck onto the nearest road"),
		TEXT("M / H          toggle map / this help"),
		TEXT("Follow the amber road on the map up the mountain pass hairpins."),
	};
	const float X = 20.f * S;
	const float Y = 20.f * S;
	const float LineH = 22.f * S;
	const int32 NumLines = static_cast<int32>(UE_ARRAY_COUNT(Lines));
	DrawRect(PanelColor, X, Y, 640.f * S, LineH * NumLines + 16.f * S);
	for (int32 i = 0; i < NumLines; ++i)
	{
		Text(Lines[i], X + 10.f * S, Y + 8.f * S + i * LineH, i == 0 ? Amber : TextColor, 1.0f * S);
	}
}
