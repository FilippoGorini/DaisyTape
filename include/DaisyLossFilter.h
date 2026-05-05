#pragma once
#ifndef DAISY_LOSSFILTER_H
#define DAISY_LOSSFILTER_H

#include "daisy_seed.h"
#include <cmath>
#include <algorithm>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Scaled Order: 64 * (48000/44100) ~= 70
// This ensures the frequency resolution matches the original plugin.
#define LOSS_FIR_ORDER 70

// Crossfade length in samples
#define LOSS_FADE_LEN 1024

/**
 * @brief A simple Stereo FIR Filter with settable coefficients.
 */
class StereoFIR
{
public:
    StereoFIR() { reset(); }

    void reset() {
        for(int i=0; i<LOSS_FIR_ORDER; i++) {
            coeffs[i] = 0.0f;
            stateL[i] = 0.0f;
            stateR[i] = 0.0f;
        }
        head = 0;
    }

    void copyStateFrom(const StereoFIR& other) {
        head = other.head;
        for(int i=0; i<LOSS_FIR_ORDER; i++) {
            stateL[i] = other.stateL[i];
            stateR[i] = other.stateR[i];
        }
    }

    void setCoefficients(const float* newCoeffs) {
        for(int i=0; i<LOSS_FIR_ORDER; i++) {
            coeffs[i] = newCoeffs[i];
        }
    }

    inline void process(float inL, float inR, float& outL, float& outR) {
        // Write input to ring buffer
        stateL[head] = inL;
        stateR[head] = inR;

        // Convolution
        float sumL = 0.0f;
        float sumR = 0.0f;
        int idx = head;

        for(int i=0; i<LOSS_FIR_ORDER; i++) {
            sumL += coeffs[i] * stateL[idx];
            sumR += coeffs[i] * stateR[idx];
            
            idx--;
            if(idx < 0) idx = LOSS_FIR_ORDER - 1;
        }

        outL = sumL;
        outR = sumR;

        // Advance head
        head++;
        if(head >= LOSS_FIR_ORDER) head = 0;
    }

private:
    float coeffs[LOSS_FIR_ORDER];
    float stateL[LOSS_FIR_ORDER];
    float stateR[LOSS_FIR_ORDER];
    int head;
};

/**
 * @brief Stereo Biquad for the Head Bump effect.
 */
struct StereoBiquad {
    float b0, b1, b2, a1, a2;
    float xL[2], yL[2];
    float xR[2], yR[2];

    void reset() {
        xL[0]=xL[1]=yL[0]=yL[1]=0.0f;
        xR[0]=xR[1]=yR[0]=yR[1]=0.0f;
        b0=b1=b2=a1=a2=0.0f;
    }

    void copyStateFrom(const StereoBiquad& other) {
        xL[0] = other.xL[0]; xL[1] = other.xL[1];
        yL[0] = other.yL[0]; yL[1] = other.yL[1];
        xR[0] = other.xR[0]; xR[1] = other.xR[1];
        yR[0] = other.yR[0]; yR[1] = other.yR[1];
    }

    void setCoeffs(float _b0, float _b1, float _b2, float _a1, float _a2) {
        b0=_b0; b1=_b1; b2=_b2; a1=_a1; a2=_a2;
    }

    inline void process(float inL, float inR, float& outL, float& outR) {
        // Direct Form I
        float ln = b0*inL + b1*xL[0] + b2*xL[1] - a1*yL[0] - a2*yL[1];
        xL[1] = xL[0]; xL[0] = inL;
        yL[1] = yL[0]; yL[0] = ln;
        outL = ln;

        float rn = b0*inR + b1*xR[0] + b2*xR[1] - a1*yR[0] - a2*yR[1];
        xR[1] = xR[0]; xR[0] = inR;
        yR[1] = yR[0]; yR[0] = rn;
        outR = rn;
    }
};

class LossFilter
{
public:
    LossFilter();
    ~LossFilter() {}

    void prepare(float sampleRate);

    // Called from main thread: compute coefficients into staging buffers
    void prepareParams(float speed, float spacing, float thickness, float gap);
    // Called from interrupt: atomically swap staged coefficients into back buffer and arm fade
    void applyParams();

    // Called from interrupt: apply filter and handle crossfade
    void processBlock(float* inL, float* inR, float* outL, float* outR, int32_t blockSize);

    float getLatencySamples() const;

private:
    // Math helpers
    void calcHeadBumpCoeffs(float speedIps, float gapMeters, StereoBiquad& filter);
    void calcFirCoeffs(float speed, float spacing, float thickness, float gap);

    float fs;
    bool onOff;

    // Double-buffered filters (active and inactive/fading)
    StereoFIR firFilters[2];
    StereoBiquad bumpFilters[2];

    volatile int activeFilterIdx;
    volatile int fadeCounter;
    volatile bool triggerFade;

    // Staging — written from main (prepareParams), read from interrupt (applyParams)
    volatile bool stageReady;
    float computedFir[LOSS_FIR_ORDER]; // staged FIR coefficients
    StereoBiquad stagedBump;           // staged bump filter coefficients

    // Parameters — stored to suppress redundant recomputes
    float p_speed, p_spacing, p_thickness, p_gap;

    // Temporary frequency-domain buffer used during FIR calculation
    float Hcoefs[LOSS_FIR_ORDER];
};

#endif // DAISY_LOSSFILTER_H