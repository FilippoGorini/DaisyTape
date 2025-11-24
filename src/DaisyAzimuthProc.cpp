#include "DaisyAzimuthProc.h"

// Constants
namespace {
    constexpr float inches2meters(float inches) {
        return inches / 39.370078740157f;
    }

    // Renamed to avoid conflict with libDaisy macro 'deg2rad'
    constexpr float degreesToRadians(float deg) {
        return deg * M_PI / 180.0f;
    }

    // 0.25 inches width converted to meters
    constexpr float tapeWidth = inches2meters(0.25f);
}

void AzimuthProc::prepare(float sampleRate)
{
    fs = sampleRate;

    for (int ch = 0; ch < 2; ++ch)
    {
        // Initialize delay lines if pointers are set
        if(delays[ch] != nullptr) {
            delays[ch]->Init();
            // Pre-fill delay with 0s? Init() does this.
        }
        
        // Initialize smoother (approx 50ms smoothing time)
        delaySampSmooth[ch].Init(sampleRate, 0.05f);
        
        // --- CRITICAL FIX ---
        // Force the smoother to start at 1.0f immediately.
        // This prevents the "sweep from 0" (5.5s delay) artifact on startup.
        delaySampSmooth[ch].SetCurrent(1.0f);
    }
}

void AzimuthProc::setDelayLinePointers(AzimuthDelayLine* delayL, AzimuthDelayLine* delayR)
{
    delays[0] = delayL;
    delays[1] = delayR;
}

void AzimuthProc::setAzimuthAngle(float angleDeg, float tapeSpeedIps)
{
    // If angle < 0, delay Left (idx 0). If angle > 0, delay Right (idx 1).
    const size_t delayIdx = (angleDeg < 0.0f) ? 0 : 1;
    
    const float tapeSpeed = inches2meters(tapeSpeedIps);
    const float azimuthAngle = degreesToRadians(std::abs(angleDeg));

    // Calculate physical distance offset
    float delayDist = tapeWidth * std::sin(azimuthAngle);
    
    // Calculate raw samples delay
    auto delaySamp = (delayDist * tapeSpeed) * fs; 

    // --- CRITICAL FIX ---
    // Daisysp::DelayLine::Read(1.0f) = Current Sample (0 latency).
    // Daisysp::DelayLine::Read(0.0f) = Buffer Tail (Max latency).
    // We MUST offset the target by +1.0f.
    
    delaySampSmooth[delayIdx].SetTarget(delaySamp + 1.0f);
    delaySampSmooth[1 - delayIdx].SetTarget(1.0f);
}

void AzimuthProc::processBlock(float* inL, float* inR, float* outL, float* outR, int32_t blockSize)
{
    float* inputs[2] = {inL, inR};
    float* outputs[2] = {outL, outR};

    for (int ch = 0; ch < 2; ++ch)
    {
        // Safety check
        if (delays[ch] == nullptr) {
            for(int i=0; i<blockSize; i++) outputs[ch][i] = inputs[ch][i];
            continue;
        }

        for (int n = 0; n < blockSize; ++n)
        {
            // Update smoothed delay time
            float currentDelay = delaySampSmooth[ch].Process();
            
            // Write input sample
            delays[ch]->Write(inputs[ch][n]);
            
            // Read with Hermite (Cubic) interpolation
            outputs[ch][n] = delays[ch]->ReadHermite(currentDelay);
        }
    }
}