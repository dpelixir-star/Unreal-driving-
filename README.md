# Army Truck Simulator (Unreal Engine 5)

A driving simulator for a **5 ton 6x6 army cargo truck** (M939 style) in a **5 km x 5 km hilly area**
with a **mountain pass full of hairpin bends**, sweeping curves, signalised junctions and **AI traffic**.

The whole project is C++ and needs no imported assets. Terrain, roads, truck, traffic, signals, trees,
guard rails and lighting are all generated at runtime from engine basic shapes and procedural meshes,
so the repository contains only text files.

## Requirements

* Unreal Engine **5.4** (5.3 – 5.5 should work; change `EngineAssociation` in `ArmyTruckSim.uproject`)
* Visual Studio 2022 / Rider (Windows), Xcode (macOS) or clang (Linux) with the UE toolchain
* Plugin: `ProceduralMeshComponent` (ships with the engine, enabled in the `.uproject`)

## Getting started

1. Right click `ArmyTruckSim.uproject` -> **Generate Visual Studio project files**
   (or run `GenerateProjectFiles` / let Rider open the `.uproject` directly).
2. Build the `ArmyTruckSimEditor` target (Development Editor), then open the project.
3. Press **Play**. The default map is the engine's empty `Entry` map. The game mode
   (`ArmyTruckGameMode`, set as the global default) generates the world in about 1–3 seconds,
   starts the traffic and drops the truck on the valley section of the mountain pass road.

You can use any level. If you place an `AWorldGenerator` actor in your own level, you can tweak it in the
Details panel and press **Generate** to preview the terrain and roads in the editor. Those components are
transient, so they are not saved into the map.

## Controls

| Key (gamepad)                    | Action |
|----------------------------------|--------|
| W / S  (RT / LT)                 | accelerate / brake. Hold S while stopped to select reverse |
| A / D  (left stick)              | steer |
| Space  (A)                       | handbrake (rear tandem axles) |
| T                                | automatic / manual gearbox |
| E / Q  (RB / LB)                 | shift up / down (manual mode; N/R selection in auto) |
| L  (X)                           | transfer case high / low range (below ~9 km/h). Use it on the hairpins |
| C  (Y)                           | chase camera / cab camera |
| Mouse  (right stick)             | look around |
| F  (B)                           | headlights |
| R                                | recover the truck onto the nearest road |
| M / H                            | toggle overview map / help |

## The world

```
                +Y ("north")
   ┌───────────────────────────────────┐   5 km
   │     ring road (~12.4 km, 70 km/h)  │
   │   ╭──────────── N junction ──────╮ │
   │   │        2 hairpins (north face)│ │
   │   │   ====== RIDGE / SADDLE ===== │ │   ridge ~380 m, saddle ~230 m
   │   │     8 hairpin switchbacks     │ │
   │   │  (south face, 18 m radius)    │ │
 W ┼───┼──── valley road ──╋───────────┼─┼ E     4 way signalised crossing
   │   │                   ┃ pass road │ │
   │   ╰──────────── S junction ──────╯ │   <- player starts here, heading north
   └───────────────────────────────────┘
```

* **Terrain**: a 501 x 501 vertex heightfield (10 m cells) split into 10 x 10 mesh tiles. It is built from
  fractal Perlin hills, an east-west mountain ridge with a saddle, and a raised rim at the map edge.
  Steep triangles are rendered as rock.
* **Roads**: designed with a turtle-style path builder (`FRoadPathBuilder`): straights, arcs, Hermite
  curves and Catmull-Rom loops.
  * *Ring road*: a closed loop with random sweeping curves, grade limited to 7 %.
  * *Mountain pass*: S-curves through the valley, then **8 hairpins** traversing the south face. Each
    hairpin is a 150° turn with an 18 m centre-line radius, close to the turning circle of a 5 ton truck.
    The road crosses the saddle and goes down **2 more hairpins** on the north face. Grade is limited to 9.5 %.
  * *Valley road*: west-east, crossing the pass road at a signalised 4 way junction.
  * Elevation profiles come from smoothing the terrain along the road, then relaxing them iteratively to
    the grade limit. Junction ends are pinned so connected roads meet at the same height.
  * The terrain is then **cut and filled** around every road (flat corridor plus a 30 m embankment blend).
  * Road meshes have asphalt, gravel shoulders and skirts. Lane markings are white edge lines, a dashed
    centre line, and a double solid yellow line on curves.
* **Props**: guard rails on the outside of tight curves and wherever the terrain drops beside the road,
  chevron boards on hairpins, about 6000 trees in clustered forests below the tree line, and rocks on slopes.
* **Environment**: if the level has none, the generator spawns a sun, sky atmosphere, real-time sky light
  and height fog.

## Traffic system

* **Lane graph**: every road has one lane per direction (right-hand traffic by default; set
  `bLeftHandTraffic` on the generator for left-hand traffic). Lanes connect through exits at junctions:
  ring ↔ pass road, ring ↔ valley road, and turns at the crossing.
* **Traffic lights**: two-phase controllers (`ATrafficSignal`) at all 5 junctions, with poles, housings
  and red/yellow/green lamps. Cycle: main green 22 s, yellow 3 s, all-red 2 s, side green 14 s.
* **AI vehicles** (`ATrafficVehicle`, managed by `ATrafficManager`): 45 by default, a mix of cars, vans,
  buses and trucks. Each follows the **Intelligent Driver Model**. Its "leader" is the nearest of:
  * the next vehicle on its lane,
  * a vehicle on the lane it is turning into (this gives merge yielding),
  * a red or yellow stop line,
  * your truck, if it overlaps the lane ahead. Traffic waits for you when you swing wide through a hairpin.
* **Speed**: desired speed is capped by the speed limit and by the curvature ahead, so traffic brakes
  before curves and crawls around hairpins.
* **Recovery**: vehicles stuck for over 60 s far from the player respawn elsewhere.

## The truck

`AArmyTruckPawn` is a self-contained raycast vehicle on one rigid body, so it needs no skeletal mesh or
Chaos Vehicle asset:

* 9.8 t curb mass + 4.5 t (5 short ton) payload, with a lowered centre of mass.
* 6 wheels on 3 axles (6x6 drive). The front axle steers, with speed-sensitive steering.
* Spring/damper struts tuned by natural frequency and damping ratio, with bump stops and anti-roll bars.
* Slip-angle lateral tyre forces, a friction circle and rolling resistance.
* Turbo diesel torque curve (1100 N·m), torque converter, 5-speed automatic with kick-down or manual
  gearbox, final drive 6.44, **2-speed transfer case** (low range ×2), engine braking.
* Service brakes, a rear handbrake with hill hold, and aerodynamic drag.
* HUD with speed, gear, RPM, grade %, mass, wheel slip, a **hairpin / sharp curve warning** read from
  the road geometry ahead, an off-road warning and an overview map with live traffic.

All parameters (mass, payload, suspension, engine, gear ratios, brakes, steering) are editable
`UPROPERTY`s, so you can subclass the pawn in Blueprint to tune it.

## Source layout

```
Source/ArmyTruckSim/
  Public/ & Private/
    ArmyTruckGameMode   generates the world, starts traffic, spawns the truck
    WorldGenerator      terrain, roads, meshes, props, lanes, signals, environment
    RoadPathBuilder     turtle/Hermite/Catmull-Rom road design helper
    TruckSimTypes       polylines, lanes, roads, heightfield, material helpers
    ArmyTruckPawn       6x6 truck physics, drivetrain, cameras, input
    ArmyTruckHUD        canvas HUD and map
    TrafficManager      AI traffic simulation (IDM, junctions, signals)
    TrafficVehicle      AI vehicle visuals and state
    TrafficSignal       traffic light controller and heads
Config/                 input mappings, default map / game mode, renderer settings
```

## Customising

* **Look**: assign your own `TerrainMaterial` / `RoadMaterial` on the generator. Vertex colours carry the
  grass/rock tint. Without overrides, tinted instances of the engine's `BasicShapeMaterial` are used.
* **Layout**: the generator exposes seed, map size, hill and ridge shape, hairpin radius, switchback count
  and leg length, max grades, speed limits, tree count and more.
* **Traffic**: change the vehicle count, seed and driver-model parameters on `ATrafficManager`, or set
  `VehicleClass` to a Blueprint subclass of `ATrafficVehicle` with real meshes.
