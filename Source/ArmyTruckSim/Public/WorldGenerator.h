#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TruckSimTypes.h"
#include "WorldGenerator.generated.h"

class UProceduralMeshComponent;
class UHierarchicalInstancedStaticMeshComponent;
class UMaterialInterface;
class UMaterialInstanceDynamic;
class UStaticMesh;

/**
 * Procedurally builds the whole driving area:
 *  - a 5 km x 5 km hilly heightfield with a mountain ridge, a saddle pass and a raised rim,
 *  - a road network: a curvy ring road, a mountain pass road with a flight of hairpin switchbacks
 *    and a valley cross road, all with grade limited elevation profiles,
 *  - terrain cut/fill around the roads, road meshes with shoulders and lane markings,
 *  - guard rails on curves and drop offs, trees and rocks,
 *  - the traffic lane graph and signalised junctions used by ATrafficManager,
 *  - a sun, sky atmosphere, sky light and fog when the level has none.
 *
 * Map layout (metres, map centre at the origin, +Y is "north"):
 *   Ring road  radius ~1950 m around the map.
 *   Pass road  from the south ring junction through the valley, up 8 hairpins on the
 *              south face of the ridge, over the saddle, 2 hairpins down the north face
 *              and on to the north ring junction.
 *   Cross road west-east through the southern valley, crossing the pass road at a 4 way junction.
 */
UCLASS()
class ARMYTRUCKSIM_API AWorldGenerator : public AActor
{
	GENERATED_BODY()

public:
	AWorldGenerator();

	virtual void BeginPlay() override;

	/** Builds (or rebuilds) the whole world. Called automatically by the game mode. */
	UFUNCTION(CallInEditor, BlueprintCallable, Category = "World Generation")
	void Generate();

	/** Removes everything generated. */
	UFUNCTION(CallInEditor, BlueprintCallable, Category = "World Generation")
	void ClearGenerated();

	bool IsGenerated() const { return bGenerated; }

	/** Returns the first generator in the world (or null). */
	static AWorldGenerator* Find(const UObject* WorldContext);

	const TArray<FSimRoad>& GetRoads() const { return Roads; }
	const TArray<FTrafficLane>& GetLanes() const { return Lanes; }
	FTransform GetPlayerSpawnTransform() const { return PlayerSpawn; }

	/** Half size of the map in cm. */
	double GetMapHalfSize() const { return MapSizeMeters * 50.0; }

	/** Final (road conformed) terrain height in cm. */
	double GetTerrainHeight(const FVector2D& P) const { return Height.Sample(P); }

	/** Finds the closest road centre line sample. Returns false if there are no roads. */
	bool FindNearestRoadPoint(const FVector& Location, int32& OutRoad, double& OutDistance, double& OutDistSq) const;

	// ---------------------------------------------------------------- General
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "World Generation")
	int32 Seed = 1979;

	/** Side length of the square map. The road layout scales with it. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "World Generation", meta = (ClampMin = "2000", ClampMax = "10000"))
	double MapSizeMeters = 5000.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "World Generation")
	bool bSpawnEnvironment = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "World Generation")
	bool bSpawnTrafficSignals = true;

	// ---------------------------------------------------------------- Terrain
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Terrain", meta = (ClampMin = "4", ClampMax = "50"))
	double TerrainCellSizeMeters = 10.0;

	/** The terrain is split into TilesPerSide x TilesPerSide mesh components for culling. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Terrain", meta = (ClampMin = "1", ClampMax = "32"))
	int32 TilesPerSide = 10;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Terrain")
	double BaseHeightMeters = 40.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Terrain")
	double HillAmplitudeMeters = 90.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Terrain")
	double HillWavelengthMeters = 900.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Terrain")
	double RidgeHeightMeters = 380.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Terrain")
	double RidgeCenterYMeters = 700.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Terrain")
	double RidgeWidthMeters = 650.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Terrain")
	double RidgeHalfLengthMeters = 1800.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Terrain")
	double SaddleCenterXMeters = 100.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Terrain")
	double SaddleWidthMeters = 450.0;

	/** 0 = no saddle, 1 = the ridge disappears at the saddle. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Terrain", meta = (ClampMin = "0", ClampMax = "1"))
	double SaddleDepth = 0.4;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Terrain")
	double RimHeightMeters = 140.0;

	/** Triangles whose normal Z is below this are rendered as rock. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Terrain", meta = (ClampMin = "0", ClampMax = "1"))
	double RockSlopeNormalZ = 0.8;

	/** Optional material for the terrain (vertex colours carry grass/rock tint). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Terrain")
	TObjectPtr<UMaterialInterface> TerrainMaterial;

	// ---------------------------------------------------------------- Roads
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Roads")
	double RoadHalfWidthMeters = 3.75;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Roads")
	double ShoulderWidthMeters = 1.5;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Roads", meta = (ClampMin = "0.5", ClampMax = "5"))
	double RoadSampleSpacingMeters = 2.0;

	/** Centre line radius of the hairpin turns. A loaded 5 ton truck needs ~12 m. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Roads", meta = (ClampMin = "12", ClampMax = "60"))
	double HairpinRadiusMeters = 18.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Roads", meta = (ClampMin = "60", ClampMax = "500"))
	double SwitchbackLegMeters = 220.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Roads", meta = (ClampMin = "2", ClampMax = "14"))
	int32 NumSwitchbackHairpins = 8;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Roads")
	double RingMaxGrade = 0.07;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Roads")
	double PassMaxGrade = 0.095;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Roads")
	double CrossMaxGrade = 0.08;

	/** Depth the terrain is cut below the road surface so it never pokes through. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Roads")
	double TerrainCutMeters = 0.5;

	/** Width of the cut/fill embankment blend either side of the road. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Roads")
	double EmbankmentBlendMeters = 30.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Roads")
	bool bBuildGuardRails = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Roads")
	TObjectPtr<UMaterialInterface> RoadMaterial;

	// ---------------------------------------------------------------- Traffic
	/** Drive on the left (UK/India style) instead of the right. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Traffic")
	bool bLeftHandTraffic = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Traffic")
	double RingSpeedLimitKmh = 70.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Traffic")
	double PassSpeedLimitKmh = 45.0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Traffic")
	double CrossSpeedLimitKmh = 55.0;

	/** Lateral acceleration AI vehicles accept in curves (m/s^2). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Traffic")
	double TrafficLateralAccel = 2.2;

	/** Probability that an AI vehicle turns at a junction it passes. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Traffic", meta = (ClampMin = "0", ClampMax = "1"))
	double TurnProbability = 0.3;

	// ---------------------------------------------------------------- Props
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Props", meta = (ClampMin = "0", ClampMax = "50000"))
	int32 NumTrees = 6000;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Props", meta = (ClampMin = "0", ClampMax = "10000"))
	int32 NumRocks = 700;

	/** Trees do not grow above this height. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Props")
	double TreeLineMeters = 330.0;

private:
	// Generation steps.
	void BuildNaturalTerrain();
	void BuildRoadLayout();
	void ComputeRoadElevations();
	void ElevateRoad(FSimRoad& Road, double MaxGrade, const TMap<int32, double>& Pins, int32 SmoothRadius);
	void BuildRoadSampleGrid();
	void ConformTerrainToRoads();
	void BuildTerrainMesh();
	void BuildRoadMeshes();
	void BuildGuardRails();
	void BuildTreesAndRocks();
	void BuildLanes();
	void BuildSignals();
	void SpawnEnvironment();

	double NaturalHeightMeters(const FVector2D& PMeters) const;
	double MapScale() const { return MapSizeMeters / 5000.0; }
	double DistanceToNearestRoad(const FVector2D& P, double MaxSearch) const;
	void ComputeRoadFrames(const FSimRoad& Road, TArray<FVector>& OutRight, TArray<FVector>& OutUp, TArray<double>& OutCurvature) const;

	UProceduralMeshComponent* NewProcMesh(const FString& Name);
	UHierarchicalInstancedStaticMeshComponent* NewInstancer(const FString& Name, UStaticMesh* Mesh, UMaterialInterface* Material, bool bCollision);
	UMaterialInterface* ColorMaterial(const FLinearColor& Color, UMaterialInterface* Override = nullptr);

	UPROPERTY(VisibleAnywhere, Category = "World Generation")
	TObjectPtr<USceneComponent> Root;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UProceduralMeshComponent>> MeshComponents;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UHierarchicalInstancedStaticMeshComponent>> InstanceComponents;

	UPROPERTY(Transient)
	TArray<TObjectPtr<AActor>> SpawnedActors;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UMaterialInstanceDynamic>> Materials;

	FTerrainHeightField Height;
	TArray<FSimRoad> Roads;
	TArray<FTrafficLane> Lanes;
	TArray<FJunctionInfo> Junctions;
	TArray<FCrossingInfo> Crossings;
	TArray<FRoadSampleRef> RoadSamples;
	TMap<FIntPoint, TArray<int32>> RoadGrid;
	double RoadGridCell = 5000.0;

	FTransform PlayerSpawn;
	FRandomStream Stream;
	FVector2D NoiseOffsets[8];
	bool bGenerated = false;

	static constexpr int32 RingRoad = 0;
	static constexpr int32 PassRoad = 1;
	static constexpr int32 CrossRoad = 2;
};
