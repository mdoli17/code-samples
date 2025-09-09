#include "Enemy/AI/Perception/Vision/VisionModelComponent.h"
#include "GameFramework/Actor.h"
#include "Components/SkeletalMeshComponent.h"
#include "DrawDebugHelpers.h"

#include "Perception/AIPerceptionComponent.h"
#include "Perception/AISense_Sight.h"

UVisionModelComponent::UVisionModelComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.TickGroup = TG_PrePhysics;

    DebugTargetThickness = 1.5f;
    DebugTargetColor = FColor::Yellow;
    DebugTargetRadius = 16.f;
}

#if WITH_EDITOR
void UVisionModelComponent::PostEditChangeProperty(struct FPropertyChangedEvent &PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);

    /* ---------- Range Limiters ---------- */

    if (ClueUp > DetectUp)
    {
        ClueUp = DetectUp;
    }

    if (ClueDown < DetectDown)
    {
        ClueDown = DetectDown;
    }
}
#endif

void UVisionModelComponent::BeginPlay()
{
    Super::BeginPlay();

    // Initialize smoothed pose
    const FTransform Raw = GetHeadSocketWorld();
    SmoothedPos = Raw.GetLocation();
    SmoothedRot = Raw.GetRotation();
}

void UVisionModelComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction *ThisTick)
{
    Super::TickComponent(DeltaTime, TickType, ThisTick);
    
    if (DeltaTime <= 0.f)
        return;

    UpdateSmoothedEyes(DeltaTime);

    // Actors which are currently perceived by sight perception
    TArray<AActor *> CurrentActors;
    if (Perception.IsValid())
    {
        Perception->GetCurrentlyPerceivedActors(UAISense_Sight::StaticClass(), CurrentActors);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Perception Isn't Valid"));
        return;
    }

    // Every actor which needs to be ticked (Including the non-percieved ones for decay)
    TSet<AActor *> Candidates;
    for (AActor *Actor : CurrentActors)
        if (IsValid(Actor))
            Candidates.Add(Actor);
    for (const auto &KV : VisionScorePerTarget)
    {
        if (AActor *Actor = KV.Key.Get())
            if (IsValid(Actor))
                Candidates.Add(Actor);
    }

    // Tick all current candidates
    for (AActor *Target : Candidates)
    {
        if (CurrentActors.Contains(Target))
        {
            TickTarget(Target, DeltaTime);
        }
        else // These actors aren't perceieved anymore, just decay
        {
            float &Score = VisionScorePerTarget.FindOrAdd(Target);
            Score = FMath::Clamp(Score - DecayRate * DeltaTime, 0.f, 1.f);
            HandleThresholds(Target, Score);

            if (bDebugDraw)
                DebugDrawTarget(Target, 0.f, Score);
        }
    }

    UpdateBestTarget();

    if (bDebugDraw)
    {
        DebugDrawFOV();
        if (AActor *Best = BestTarget.Get())
        {
            DebugDrawTarget(Best, ComputeVisibilityWeight(Best), BestScoreCached);
        }
    }
}

static float AlphaFromCutoff(const float Fc, const float DeltaTime)
{
    // EMA coefficient from cutoff frequency (Hz)
    return (Fc <= 0.f || DeltaTime <= 0.f) ? 1.f : 1.f - FMath::Exp(-2.f * PI * Fc * DeltaTime);
}

void UVisionModelComponent::UpdateSmoothedEyes(const float DeltaTime)
{
    const FTransform Raw = GetHeadSocketWorld();
    const float A = AlphaFromCutoff(CutoffHz, DeltaTime);

    SmoothedPos = FMath::Lerp(SmoothedPos, Raw.GetLocation(), A);
    SmoothedRot = FQuat::Slerp(SmoothedRot, Raw.GetRotation(), A).GetNormalized();
}

FTransform UVisionModelComponent::GetHeadSocketWorld() const
{
    if (const AActor *Owner = GetOwner())
    {
        if (const USkeletalMeshComponent *SkeletalMeshComponent = Owner->FindComponentByClass<USkeletalMeshComponent>())
        {
            if (HeadSocketName != NAME_None && SkeletalMeshComponent->DoesSocketExist(HeadSocketName))
            {
                return SkeletalMeshComponent->GetSocketTransform(HeadSocketName, RTS_World);
            }
        }

        // Fallback: actor eyes (location) + rotation
        FRotator EyesRotation;
        FVector EyesLocation;
        Owner->GetActorEyesViewPoint(EyesLocation, EyesRotation);
        return FTransform(EyesRotation, EyesLocation);
    }

    return FTransform::Identity;
}

FVector UVisionModelComponent::GetForwardFromQuat(const FQuat &Q) const
{
    switch (ForwardAxis)
    {
    case EVisionForwardAxis::Y:
        return Q.GetRightVector().GetSafeNormal();
    case EVisionForwardAxis::Z:
        return Q.GetUpVector().GetSafeNormal();
    default:
        return Q.GetForwardVector().GetSafeNormal();
    }
}

float UVisionModelComponent::GetVisionScoreFor(AActor *Target) const
{
    if (!Target)
        return 0.f;
    if (const float *Found = VisionScorePerTarget.Find(Target))
    {
        return *Found;
    }
    return 0.f;
}

float UVisionModelComponent::SampleOcclusion(const FVector &EyePos, const FVector &TgtPos) const
{
    if (OcclusionRays <= 0)
        return 1.f;
    UWorld *W = GetWorld();
    if (!W)
        return 1.f;

    int32 Hits = 0;
    for (int i = 0; i < OcclusionRays; ++i)
    {
        // Make a ray towards a random point around the target
        const FVector Rand = FMath::VRand(); // TODO: Maybe not generate random every frame?
        const FVector End = TgtPos + Rand * OcclusionJitterRadius;

        FHitResult HR;
        const bool bHit = W->LineTraceSingleByChannel(HR, EyePos, End, OcclusionTraceChannel);
        if (bHit && HR.bBlockingHit)
            ++Hits;

        if (bDebugDraw)
        {
            const FColor C = (bHit && HR.bBlockingHit) ? FColor::Red : FColor::Green;
            DrawDebugLine(W, EyePos, End, C, false, 0.f, 0, 1.f);
        }
    }

    const float ClearFrac = 1.f - (float)Hits / (float)OcclusionRays; // 1 = fully visible, 0 = fully blocked
    return ClearFrac;
}

float UVisionModelComponent::ComputeVisibilityWeight(AActor *Target) const
{
    if (!Target)
        return 0.f;

    const FVector EyePos = SmoothedPos;
    const FQuat EyeRot = SmoothedRot;

    // Build basis
    const FVector Fwd = GetForwardFromQuat(EyeRot);
    const FVector FwdH = FVector(Fwd.X, Fwd.Y, 0.f).GetSafeNormal();

    // Direction to target
    const FVector TgtPos = Target->GetActorLocation();
    const FVector D = (TgtPos - EyePos);
    const float Dist = D.Length();
    if (Dist <= KINDA_SMALL_NUMBER)
        return 1.f;

    const FVector Dir = D / Dist;
    const FVector DirH = FVector(Dir.X, Dir.Y, 0.f).GetSafeNormal();

    /* ---------- Hard Gates ---------- */
    if (Dist > LoseSightRadius)
        return 0.f;

    const float CosH = FVector::DotProduct(FwdH, DirH);
    const float CosHorizHalf = FMath::Cos(FMath::DegreesToRadians(HorizontalHalfAngleDeg));
    if (CosH < CosHorizHalf)
        return 0.f;

    /* ---------- Soft Falloffs ---------- */
    
    // Angle
    const float CosInner = FMath::Cos(FMath::DegreesToRadians(AngleInnerDeg));
    const float CosOuter = FMath::Cos(FMath::DegreesToRadians(AngleOuterDeg));
    float AngleW = FMath::SmoothStep(CosOuter, CosInner, CosH);

    if (AngleCurve)
    {
        // Optional: curve in angle-degrees (0..180). acos only for visualization override.
        const float AngleDeg = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(CosH, -1.f, 1.f)));
        AngleW = FMath::Max(AngleW, AngleCurve->GetFloatValue(AngleDeg));
    }

    // Distance
    float DistW = 1.f;
    if (DistanceMax > DistanceStart + KINDA_SMALL_NUMBER)
    {
        DistW = 1.f - FMath::Clamp((Dist - DistanceStart) / (DistanceMax - DistanceStart), 0.f, 1.f);
    }
    if (DistanceCurve)
    {
        DistW = FMath::Max(DistW, DistanceCurve->GetFloatValue(Dist));
    }

    // Occlusion
    const float OcclW = SampleOcclusion(EyePos, TgtPos);

    // Combine with exponents
    float W = FMath::Pow(AngleW, AngleExp) * FMath::Pow(DistW, DistExp) * FMath::Pow(OcclW,  OcclExp);

    if (!FMath::IsFinite(W))
        W = 0.f;
    return FMath::Clamp(W, 0.f, 1.f);
}

void UVisionModelComponent::HandleThresholds(AActor *Target, float SNow)
{
    const bool bWasDetected = DetectedSet.Contains(Target);
    const bool bWasClue = ClueSet.Contains(Target);

    // Handle detected state
    if (!bWasDetected && SNow >= DetectUp)
    {
        ClueSet.Remove(Target);
        DetectedSet.Add(Target);
        OnDetected.Broadcast(Target);
    }

    // Only manage clue state if not fully detected
    if (!DetectedSet.Contains(Target))
    {
        if (!bWasClue && SNow >= ClueUp)
        {
            ClueSet.Add(Target);
            LastKnownPosPerTarget.Add(Target, Target->GetActorLocation());
            OnClue.Broadcast(Target, LastKnownPosPerTarget[Target]);
        }
        else if (bWasClue && SNow < ClueDown)
        {
            ClueSet.Remove(Target);
            OnClueLost.Broadcast(Target);
        }
    }

    // Handle lost state (from detected)
    if (bWasDetected && SNow < DetectDown)
    {
        DetectedSet.Remove(Target);
        OnLost.Broadcast(Target);
    }
}

void UVisionModelComponent::TickTarget(AActor *Target, float DeltaTime)
{
    float &Score = VisionScorePerTarget.FindOrAdd(Target);

    const float W = ComputeVisibilityWeight(Target);

    // Deadzone: if weight is very low, just decay
    if (W < MinWeightDeadzone)
    {
        Score = FMath::Clamp(Score - DecayRate * DeltaTime, 0.f, 1.f);
        HandleThresholds(Target, Score);
        if (bDebugDraw)
            DebugDrawTarget(Target, W, Score);
        return;
    }

    // Weight Multiplier curve influence
    float gainMul = 1.0f;
    if (WeightToGainMul)
        gainMul = FMath::Clamp(WeightToGainMul->GetFloatValue(W), 0.f, 10.f);
    
    // Weight to Score Cap curve influence
    float cap = 1.0f;
    if (WeightToScoreCap)
        cap = FMath::Clamp(WeightToScoreCap->GetFloatValue(W), 0.f, 1.f);

    const float DeltaScore = W * GainRate * gainMul * DeltaTime;

    if (DeltaScore > 0.f)
    {
        if (Score < cap)
            Score = FMath::Min(Score + DeltaScore, cap);
        else
        {
        }

        LastKnownPosPerTarget.FindOrAdd(Target) = Target->GetActorLocation();
    }
    else
    {
        Score = FMath::Clamp(Score - DecayRate * DeltaTime, 0.f, 1.f);
    }

    HandleThresholds(Target, Score);

    if (bDebugDraw)
    {
        DebugDrawTarget(Target, W, Score);
    }
}

void UVisionModelComponent::UpdateBestTarget()
{
    BestTarget = nullptr;
    BestScoreCached = 0.f;

    for (const auto &KV : VisionScorePerTarget)
    {
        AActor *Target = KV.Key.Get();
        if (!IsValid(Target))
            continue;

        const float Score = KV.Value;
        if (Score > BestScoreCached)
        {
            BestScoreCached = Score;
            BestTarget = Target;
        }
    }
}

void UVisionModelComponent::DebugDrawFOV() const
{
    if (!bDebugDraw)
        return;
    if (UWorld *W = GetWorld())
    {
        const FVector EyePos = SmoothedPos;
        const FVector Fwd = GetForwardFromQuat(SmoothedRot);

        // Draw inner/outer horizontal FOV as boundary lines
        auto DrawRayAtAngle = [&](float Deg, const FColor &C)
        {
            const float Rad = FMath::DegreesToRadians(Deg);
            const FVector Dir = Fwd.RotateAngleAxis(Deg, FVector::UpVector);
            DrawDebugLine(W, EyePos, EyePos + Dir * DistanceMax, C, false, 0.f, 0, 1.5f);
        };

        DrawRayAtAngle(AngleInnerDeg, FColor::Cyan);
        DrawRayAtAngle(-AngleInnerDeg, FColor::Cyan);
        DrawRayAtAngle(AngleOuterDeg, FColor::Blue);
        DrawRayAtAngle(-AngleOuterDeg, FColor::Blue);

        // Sight Detect/Lose 
        DrawDebugSphere(W, EyePos, SightRadius, 24, FColor::Green, false, 0.f, 0, 0.5f);
        DrawDebugSphere(W, EyePos, LoseSightRadius, 24, FColor::Red, false, 0.f, 0, 0.5f);
    }
}

void UVisionModelComponent::DebugDrawTarget(AActor *Target, float Weight, float Score) const
{
    if (!bDebugDraw || !Target)
        return;
    if (UWorld *W = GetWorld())
    {
        const FVector P = Target->GetActorLocation();
        DrawDebugSphere(W, P, DebugTargetRadius, 8, DebugTargetColor, false, 0.f, 0, DebugTargetThickness);

        const FString Txt = FString::Printf(TEXT("w=%.2f  Score=%.2f"), Weight, Score);
        DrawDebugString(W, P + FVector(0, 0, 60), Txt, nullptr, FColor::White, 0.f, false);

        DrawDebugLine(W, SmoothedPos, P, DebugTargetColor, false, 0.f, 0, DebugTargetThickness);

        GEngine->AddOnScreenDebugMessage(1, 0.0f, DebugTargetColor, Txt);
    }
}