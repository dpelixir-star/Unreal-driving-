#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TruckSimTypes.h"
#include "TrafficSignal.generated.h"

class UStaticMesh;
class UMaterialInstanceDynamic;

/**
 * Two phase traffic light controller for a junction.
 * Group 0 (main road) and group 1 (side road) alternate: green -> yellow -> all red.
 * Signal heads (pole + housing + 3 lamps) are added by the world generator for every controlled approach.
 */
UCLASS()
class ARMYTRUCKSIM_API ATrafficSignal : public AActor
{
	GENERATED_BODY()

public:
	ATrafficSignal();

	virtual void Tick(float DeltaSeconds) override;

	/** Adds a signal head at GroundLocation for traffic travelling along TrafficDirection. */
	void AddSignalHead(const FVector& GroundLocation, const FVector& TrafficDirection, int32 Group);

	ESignalState GetGroupState(int32 Group) const;

	/** Shifts the cycle so neighbouring junctions are not synchronised. */
	void SetPhaseOffset(float Seconds);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Signal")
	float MainGreenTime = 22.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Signal")
	float SideGreenTime = 14.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Signal")
	float YellowTime = 3.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Signal")
	float AllRedTime = 2.f;

private:
	void RefreshLamps(bool bForce);
	float CycleLength() const { return MainGreenTime + SideGreenTime + 2.f * (YellowTime + AllRedTime); }

	UPROPERTY(VisibleAnywhere, Category = "Signal")
	TObjectPtr<USceneComponent> Root;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UMaterialInstanceDynamic>> LampMaterials; // 3 per head: red, yellow, green

	UPROPERTY(Transient)
	TObjectPtr<UMaterialInstanceDynamic> HousingMaterial;

	UPROPERTY()
	TObjectPtr<UStaticMesh> CubeMesh;

	UPROPERTY()
	TObjectPtr<UStaticMesh> CylinderMesh;

	UPROPERTY()
	TObjectPtr<UStaticMesh> SphereMesh;

	TArray<int32> HeadGroups;
	float CycleTime = 0.f;
	ESignalState LastStates[2] = { ESignalState::Red, ESignalState::Red };
};
