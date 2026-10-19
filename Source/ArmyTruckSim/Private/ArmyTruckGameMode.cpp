#include "ArmyTruckGameMode.h"
#include "ArmyTruckSim.h"
#include "ArmyTruckPawn.h"
#include "ArmyTruckHUD.h"
#include "WorldGenerator.h"
#include "TrafficManager.h"
#include "GameFramework/Controller.h"
#include "Engine/World.h"
#include "EngineUtils.h"

AArmyTruckGameMode::AArmyTruckGameMode()
{
	DefaultPawnClass = AArmyTruckPawn::StaticClass();
	HUDClass = AArmyTruckHUD::StaticClass();
	WorldGeneratorClass = AWorldGenerator::StaticClass();
	TrafficManagerClass = ATrafficManager::StaticClass();
}

void AArmyTruckGameMode::StartPlay()
{
	UWorld* World = GetWorld();
	FActorSpawnParameters Params;
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	for (TActorIterator<AWorldGenerator> It(World); It; ++It)
	{
		WorldGenerator = *It;
		break;
	}
	if (!WorldGenerator)
	{
		UClass* GenClass = WorldGeneratorClass ? WorldGeneratorClass.Get() : AWorldGenerator::StaticClass();
		WorldGenerator = World->SpawnActor<AWorldGenerator>(GenClass, FTransform::Identity, Params);
	}
	if (WorldGenerator && !WorldGenerator->IsGenerated())
	{
		WorldGenerator->Generate();
	}

	for (TActorIterator<ATrafficManager> It(World); It; ++It)
	{
		TrafficManager = *It;
		break;
	}
	if (!TrafficManager)
	{
		UClass* TrafficClass = TrafficManagerClass ? TrafficManagerClass.Get() : ATrafficManager::StaticClass();
		TrafficManager = World->SpawnActor<ATrafficManager>(TrafficClass, FTransform::Identity, Params);
	}
	if (TrafficManager && WorldGenerator)
	{
		TrafficManager->Initialize(WorldGenerator);
	}

	bWorldReady = true;
	Super::StartPlay();

	// Players that logged in before the world existed.
	TArray<TWeakObjectPtr<AController>> Pending = MoveTemp(PendingPlayers);
	for (const TWeakObjectPtr<AController>& Controller : Pending)
	{
		if (Controller.IsValid() && !Controller->GetPawn())
		{
			RestartPlayer(Controller.Get());
		}
	}
}

void AArmyTruckGameMode::RestartPlayer(AController* NewPlayer)
{
	if (!NewPlayer)
	{
		return;
	}
	if (!bWorldReady || !WorldGenerator)
	{
		PendingPlayers.AddUnique(NewPlayer);
		return;
	}
	RestartPlayerAtTransform(NewPlayer, WorldGenerator->GetPlayerSpawnTransform());
}
