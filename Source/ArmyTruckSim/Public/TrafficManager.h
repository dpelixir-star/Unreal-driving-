#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TrafficVehicle.h"
#include "TrafficManager.generated.h"

class AWorldGenerator;

/**
 * Spawns and simulates all AI traffic on the generator's lane graph.
 *
 * Every vehicle uses the Intelligent Driver Model (IDM) for longitudinal control. The "leader" is the
 * closest of: the next vehicle on its lane, a vehicle on the lane it is about to turn into,
 * the player's truck if it overlaps the lane ahead, and a red/yellow stop line.
 * Desired speed is limited by the lane speed limit and by curvature ahead, so traffic slows down
 * for curves and crawls around hairpins.
 */
UCLASS()
class ARMYTRUCKSIM_API ATrafficManager : public AActor
{
	GENERATED_BODY()

public:
	ATrafficManager();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	/** Spawns the traffic. Safe to call once the generator has generated. */
	void Initialize(AWorldGenerator* InGenerator);

	const TArray<TObjectPtr<ATrafficVehicle>>& GetVehicles() const { return Vehicles; }

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "0", ClampMax = "400"))
	int32 NumVehicles = 45;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic")
	int32 Seed = 4242;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic")
	TSubclassOf<ATrafficVehicle> VehicleClass;

	/** IDM comfortable deceleration (cm/s^2). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Driver Model")
	double ComfortDecel = 280.0;

	/** Hard braking limit (cm/s^2). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Driver Model")
	double MaxDecel = 900.0;

	/** Standstill bumper-to-bumper gap (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Driver Model")
	double MinGap = 300.0;

	/** Vehicles do not spawn closer than this to the player (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic")
	double PlayerSpawnClearance = 8000.0;

private:
	bool PlaceVehicleRandomly(ATrafficVehicle* Vehicle, const FVector& Avoid, double AvoidRadius);
	void PlanExit(ATrafficVehicle* Vehicle);
	void SampleAhead(const ATrafficVehicle* Vehicle, double Ahead, int32& OutLane, double& OutDistance) const;
	void AdvanceLanes(ATrafficVehicle* Vehicle);
	void ApplyTransform(ATrafficVehicle* Vehicle, float DeltaSeconds, bool bTeleport);

	UPROPERTY(Transient)
	TObjectPtr<AWorldGenerator> Generator;

	UPROPERTY(Transient)
	TArray<TObjectPtr<ATrafficVehicle>> Vehicles;

	FRandomStream Rand;
	bool bInitialized = false;
};
