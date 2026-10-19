#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TrafficVehicle.generated.h"

class UBoxComponent;
class UStaticMeshComponent;
class UStaticMesh;

UENUM(BlueprintType)
enum class ETrafficVehicleKind : uint8
{
	Car,
	Van,
	Bus,
	Truck
};

/**
 * Kinematic AI vehicle. It does not tick itself: ATrafficManager drives the simulation state below
 * and moves the actor along its lane. The collision box blocks the player's physics truck.
 */
UCLASS()
class ARMYTRUCKSIM_API ATrafficVehicle : public AActor
{
	GENERATED_BODY()

public:
	ATrafficVehicle();

	/** Sets body shape, dimensions, colours and driving characteristics. */
	void Configure(ETrafficVehicleKind InKind, const FLinearColor& Paint);

	ETrafficVehicleKind GetKind() const { return Kind; }

	// ---- Simulation state (owned by ATrafficManager), cm and cm/s.
	int32 LaneIndex = INDEX_NONE;
	double Distance = 0.0;
	double Speed = 0.0;
	double DesiredSpeedFactor = 1.0;
	int32 PlannedExit = INDEX_NONE;
	double DistanceToExit = 0.0;
	FVector BlendOffset = FVector::ZeroVector;
	float StuckTime = 0.f;

	double HalfLength = 220.0;
	double HalfWidth = 90.0;
	double MaxAccel = 160.0;
	double TimeHeadway = 1.3;

protected:
	UStaticMeshComponent* MakePart(const TCHAR* Name, UStaticMesh* Mesh);
	void SetPart(UStaticMeshComponent* Part, const FVector& LocM, const FVector& SizeM, const FLinearColor& Color);

	UPROPERTY(VisibleAnywhere, Category = "Traffic")
	TObjectPtr<USceneComponent> Root;

	UPROPERTY(VisibleAnywhere, Category = "Traffic")
	TObjectPtr<UBoxComponent> Collision;

	UPROPERTY(VisibleAnywhere, Category = "Traffic")
	TObjectPtr<UStaticMeshComponent> Body;

	UPROPERTY(VisibleAnywhere, Category = "Traffic")
	TObjectPtr<UStaticMeshComponent> Upper;

	UPROPERTY(VisibleAnywhere, Category = "Traffic")
	TObjectPtr<UStaticMeshComponent> Glass;

	UPROPERTY(VisibleAnywhere, Category = "Traffic")
	TObjectPtr<UStaticMeshComponent> LightsFront;

	UPROPERTY(VisibleAnywhere, Category = "Traffic")
	TObjectPtr<UStaticMeshComponent> LightsRear;

	UPROPERTY(VisibleAnywhere, Category = "Traffic")
	TArray<TObjectPtr<UStaticMeshComponent>> Wheels;

	ETrafficVehicleKind Kind = ETrafficVehicleKind::Car;
};
