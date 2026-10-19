#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "ArmyTruckHUD.generated.h"

class AArmyTruckPawn;
class AWorldGenerator;
class ATrafficManager;

/** Canvas HUD: dashboard, road advisories (hairpins, grade), overview map and help panel. */
UCLASS()
class ARMYTRUCKSIM_API AArmyTruckHUD : public AHUD
{
	GENERATED_BODY()

public:
	virtual void DrawHUD() override;

	void ToggleHelp() { bShowHelp = !bShowHelp; }
	void ToggleMap() { bShowMap = !bShowMap; }

private:
	void DrawDashboard(const AArmyTruckPawn* Truck, float S);
	void DrawAdvisories(const AArmyTruckPawn* Truck, float S);
	void DrawMap(const AArmyTruckPawn* Truck, float S);
	void DrawHelp(float S);
	void Text(const FString& Str, float X, float Y, const FLinearColor& Color, float Scale, bool bLarge = false);

	TWeakObjectPtr<AWorldGenerator> Generator;
	TWeakObjectPtr<ATrafficManager> Traffic;
	bool bShowHelp = true;
	bool bShowMap = true;
	float HelpTimer = 0.f;
};
