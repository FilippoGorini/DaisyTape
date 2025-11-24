#pragma once
#ifndef DAISY_AZIMUTHPROC_H
#define DAISY_AZIMUTHPROC_H

#include "daisy_seed.h"
#include "daisysp.h"
#include <cmath>
#include <algorithm>

// 2^18 = 262,144 samples (~1MB per channel)
#define AZIMUTH_DELAY_SIZE 262144

// Define the type for the SDRAM object so main.cpp can use it
using AzimuthDelayLine = daisysp::DelayLine<float, AZIMUTH_DELAY_SIZE>;

/**
 * Simple One-Pole Smoother for delay time transitions.
 */
class AzimuthSmoother {
public:
    void Init(float sample_rate, float time_sec) {
        coeff_ = 1.0f / (time_sec * sample_rate);
        if (coeff_ > 1.0f) coeff_ = 1.0f;
        
        current_ = 0.0f;
        target_ = 0.0f;
    }

    void SetTarget(float target) {
        target_ = target;
    }

    // --- FIX: Allow forcing the current value to avoid startup sweeps ---
    void SetCurrent(float val) {
        current_ = val;
        target_ = val;
    }

    inline float Process() {
        float diff = target_ - current_;
        if (std::abs(diff) < 1e-4f) {
            current_ = target_;
        } else {
            current_ += diff * coeff_;
        }
        return current_;
    }

    float GetCurrent() const { return current_; }

private:
    float current_;
    float target_;
    float coeff_;
};


class AzimuthProc
{
public:
    AzimuthProc() {}
    ~AzimuthProc() {}

    void prepare(float sampleRate);
    
    // Link SDRAM objects
    void setDelayLinePointers(AzimuthDelayLine* delayL, AzimuthDelayLine* delayR);

    void setAzimuthAngle(float angleDeg, float tapeSpeedIps);
    
    void processBlock(float* inL, float* inR, float* outL, float* outR, int32_t blockSize);

private:
    float fs;
    
    // Pointers to SDRAM delay lines
    AzimuthDelayLine* delays[2];
    
    // Smoothers
    AzimuthSmoother delaySampSmooth[2];
};

#endif // DAISY_AZIMUTHPROC_H