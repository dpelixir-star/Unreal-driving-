#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "ArmyTruckGameMode.generated.h"

class AWorldGenerator;
class ATrafficManager;

/**
 * Generates the world (or uses a generator placed in the level), starts the traffic and then
 * spawns the player's truck on the mountain pass road. Works with any map, including an empty one.
 */
UCLASS()
class ARMYTRUCKSIM_API AArmyTruckGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AArmyTruckGameMode();

	virtual void StartPlay() override;
	virtual void RestartPlayer(AController* NewPlayer) override;

	UFUNCTION(BlueprintPure, Category = "Truck Sim")
	AWorldGenerator* GetWorldGenerator() const { return WorldGenerator; }

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Truck Sim")
	TSubclassOf<AWorldGenerator> WorldGeneratorClass;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Truck Sim")
	TSubclassOf<ATrafficManager> TrafficManagerClass;

private:
	UPROPERTY(Transient)
	TObjectPtr<AWorldGenerator> WorldGenerator;

	UPROPERTY(Transient)
	TObjectPtr<ATrafficManager> TrafficManager;

	TArray<TWeakObjectPtr<AController>> PendingPlayers;
	bool bWorldReady = false;
};
