#include "TrafficVehicle.h"
#include "TruckSimTypes.h"
#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/CollisionProfile.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "UObject/ConstructorHelpers.h"

ATrafficVehicle::ATrafficVehicle()
{
	PrimaryActorTick.bCanEverTick = false;

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeFinder(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderFinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;

	Collision = CreateDefaultSubobject<UBoxComponent>(TEXT("Collision"));
	Collision->SetupAttachment(Root);
	Collision->SetCollisionProfileName(UCollisionProfile::Vehicle_ProfileName);
	Collision->SetCollisionResponseToChannel(ECC_Camera, ECR_Ignore);
	Collision->SetGenerateOverlapEvents(false);
	Collision->SetCanEverAffectNavigation(false);

	Body = MakePart(TEXT("Body"), CubeFinder.Object);
	Upper = MakePart(TEXT("Upper"), CubeFinder.Object);
	Glass = MakePart(TEXT("Glass"), CubeFinder.Object);
	LightsFront = MakePart(TEXT("LightsFront"), CubeFinder.Object);
	LightsRear = MakePart(TEXT("LightsRear"), CubeFinder.Object);
	for (int32 i = 0; i < 4; ++i)
	{
		Wheels.Add(MakePart(*FString::Printf(TEXT("Wheel%d"), i), CylinderFinder.Object));
	}
}

UStaticMeshComponent* ATrafficVehicle::MakePart(const TCHAR* Name, UStaticMesh* Mesh)
{
	UStaticMeshComponent* Part = CreateDefaultSubobject<UStaticMeshComponent>(Name);
	Part->SetupAttachment(Root);
	Part->SetStaticMesh(Mesh);
	Part->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Part->SetGenerateOverlapEvents(false);
	Part->SetCanEverAffectNavigation(false);
	return Part;
}

void ATrafficVehicle::SetPart(UStaticMeshComponent* Part, const FVector& LocM, const FVector& SizeM, const FLinearColor& Color)
{
	Part->SetRelativeLocation(LocM * 100.0);
	Part->SetRelativeScale3D(SizeM);
	Part->SetVisibility(!SizeM.IsNearlyZero());
	Part->SetMaterial(0, TruckSim::MakeColorMaterial(this, Color));
}

void ATrafficVehicle::Configure(ETrafficVehicleKind InKind, const FLinearColor& Paint)
{
	Kind = InKind;
	const FLinearColor Dark(0.02f, 0.02f, 0.025f);
	const FLinearColor GlassColor(0.04f, 0.06f, 0.08f);
	const FLinearColor Head(0.9f, 0.88f, 0.75f);
	const FLinearColor Tail(0.6f, 0.02f, 0.02f);

	double L = 4.4, W = 1.8, H = 1.45, WheelR = 0.33, AxleF = 1.35, AxleR = -1.35;
	switch (Kind)
	{
	case ETrafficVehicleKind::Car:
		L = 4.4; W = 1.8; H = 1.45; WheelR = 0.33; AxleF = 1.35; AxleR = -1.35;
		SetPart(Body, FVector(0.0, 0.0, 0.62), FVector(L, W, 0.65), Paint);
		SetPart(Upper, FVector(-0.25, 0.0, 1.17), FVector(2.2, W * 0.88, 0.5), Paint * 0.85f);
		SetPart(Glass, FVector(-0.25, 0.0, 1.15), FVector(2.25, W * 0.9, 0.38), GlassColor);
		MaxAccel = 220.0;
		TimeHeadway = 1.2;
		break;
	case ETrafficVehicleKind::Van:
		L = 5.1; W = 2.0; H = 2.3; WheelR = 0.36; AxleF = 1.75; AxleR = -1.6;
		SetPart(Body, FVector(-0.35, 0.0, 1.3), FVector(4.4, W, 1.9), Paint);
		SetPart(Upper, FVector(2.1, 0.0, 0.85), FVector(0.9, W * 0.98, 1.0), Paint * 0.9f);
		SetPart(Glass, FVector(1.88, 0.0, 1.65), FVector(0.06, W * 0.9, 0.6), GlassColor);
		MaxAccel = 170.0;
		TimeHeadway = 1.4;
		break;
	case ETrafficVehicleKind::Bus:
		L = 11.0; W = 2.5; H = 3.1; WheelR = 0.5; AxleF = 3.6; AxleR = -3.2;
		SetPart(Body, FVector(0.0, 0.0, 1.75), FVector(L, W, 2.7), Paint);
		SetPart(Upper, FVector(0.0, 0.0, 3.15), FVector(L * 0.92, W * 0.9, 0.1), FLinearColor(0.6f, 0.6f, 0.6f));
		SetPart(Glass, FVector(0.1, 0.0, 2.3), FVector(L * 0.97, W + 0.04, 0.85), GlassColor);
		MaxAccel = 110.0;
		TimeHeadway = 1.7;
		break;
	case ETrafficVehicleKind::Truck:
		L = 8.4; W = 2.45; H = 3.4; WheelR = 0.5; AxleF = 3.0; AxleR = -2.6;
		SetPart(Body, FVector(-1.1, 0.0, 2.0), FVector(6.0, W, 2.7), FLinearColor(0.7f, 0.7f, 0.68f));
		SetPart(Upper, FVector(3.0, 0.0, 1.55), FVector(2.2, W, 2.1), Paint);
		SetPart(Glass, FVector(4.11, 0.0, 2.0), FVector(0.04, W * 0.85, 0.8), GlassColor);
		MaxAccel = 100.0;
		TimeHeadway = 1.8;
		break;
	}

	SetPart(LightsFront, FVector(L * 0.5 - 0.02, 0.0, WheelR + 0.35), FVector(0.06, W * 0.8, 0.14), Head);
	SetPart(LightsRear, FVector(-L * 0.5 + 0.02, 0.0, WheelR + 0.4), FVector(0.06, W * 0.8, 0.12), Tail);

	const double Axles[2] = { AxleF, AxleR };
	for (int32 i = 0; i < 4; ++i)
	{
		const double X = Axles[i / 2];
		const double Y = (i % 2 == 0 ? -1.0 : 1.0) * (W * 0.5 - 0.12);
		SetPart(Wheels[i], FVector(X, Y, WheelR), FVector(WheelR * 2.0, WheelR * 2.0, 0.25), Dark);
		Wheels[i]->SetRelativeRotation(FRotator(0.0, 0.0, 90.0));
	}

	HalfLength = L * 50.0;
	HalfWidth = W * 50.0;
	Collision->SetBoxExtent(FVector(L * 50.0, W * 50.0, H * 50.0));
	Collision->SetRelativeLocation(FVector(0.0, 0.0, H * 50.0));
}
