#pragma once
#ifndef DAISY_DEGRADE_H
#define DAISY_DEGRADE_H

#include "daisy_seed.h"
#include <cmath>
#include <vector>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Block size for parameter updates (internal logic)
#define DEG_BLOCK_SIZE 2048

/**
 * @brief Simple Pseudo-Random Number Generator
 */
struct RandomGen {
    float nextFloat() {
        // Standard LCG or use rand()
        return (float)rand() / (float)RAND_MAX;
    }
};

/**
 * @brief Noise Generator with linear gain smoothing
 */
class DegradeNoise {
public:
    void prepare() {
        prevGain = curGain = 0.0f;
    }

    void setGain(float newGain) {
        curGain = newGain;
    }

    void processBlock(float* buffer, int numSamples) {
        if (std::abs(curGain - prevGain) < 1e-5f) {
            // Constant gain
            for (int n = 0; n < numSamples; ++n) {
                buffer[n] = (gen.nextFloat() - 0.5f) * curGain;
            }
        } else {
            // Ramped gain
            for (int n = 0; n < numSamples; ++n) {
                float g = prevGain + (curGain - prevGain) * ((float)n / (float)numSamples);
                buffer[n] = (gen.nextFloat() - 0.5f) * g;
            }
            prevGain = curGain;
        }
    }

private:
    float curGain = 0.0f;
    float prevGain = 0.0f;
    RandomGen gen;
};

/**
 * @brief 1-Pole Lowpass Filter with frequency smoothing
 */
class DegradeFilter {
public:
    DegradeFilter() { reset(48000.0f); }

    void reset(float sampleRate) {
        fs = sampleRate;
        z[0] = z[1] = 0.0f;
        targetFreq = 20000.0f;
        currentFreq = 20000.0f;
        calcCoefs(currentFreq);
    }

    void setFreq(float newFreq) {
        targetFreq = newFreq;
    }

    inline void calcCoefs(float fc) {
        // Bilinear Transform 1-pole LPF math
        // tan(pi * fc / fs) approximation
        float wc = 2.0f * M_PI * fc / fs;
        // Simple tan approximation or std::tan
        float tan_val = std::tan(wc * 0.5f); 
        float c = 1.0f / tan_val;
        float a0 = c + 1.0f;

        b[0] = 1.0f / a0;
        b[1] = b[0];
        a[1] = (1.0f - c) / a0;
    }

    void process(float* buffer, int numSamples) {
        // Simple per-block smoothing for efficiency
        // (Original used sample-accurate smoothing, block is fine here)
        if (std::abs(targetFreq - currentFreq) > 1.0f) {
            currentFreq += (targetFreq - currentFreq) * 0.1f; // Slew
            calcCoefs(currentFreq);
        }

        for (int n = 0; n < numSamples; ++n) {
            float x = buffer[n];
            float y = z[1] + x * b[0];
            z[1] = x * b[1] - y * a[1];
            buffer[n] = y;
        }
    }

private:
    float fs;
    float currentFreq, targetFreq;
    float a[2], b[2], z[2];
};

/**
 * @brief Envelope Follower / Level Detector
 */
class SimpleLevelDetector {
public:
    void prepare(float sampleRate) {
        fs = sampleRate;
        envelope = 0.0f;
        setParameters(10.0f, 200.0f); // Default Attack/Release
    }

    void setParameters(float attackMs, float releaseMs) {
        attackCoeff = std::exp(-1000.0f / (attackMs * fs));
        releaseCoeff = std::exp(-1000.0f / (releaseMs * fs));
    }

    void process(const float* inL, const float* inR, float* levelOut, int numSamples) {
        for (int i = 0; i < numSamples; ++i) {
            // Mono-sum abs
            float input = (std::abs(inL[i]) + std::abs(inR[i])) * 0.5f;
            
            if (input > envelope)
                envelope = attackCoeff * (envelope - input) + input;
            else
                envelope = releaseCoeff * (envelope - input) + input;
            
            levelOut[i] = envelope;
        }
    }

private:
    float fs;
    float envelope;
    float attackCoeff, releaseCoeff;
};

/**
 * @brief Main Degrade Processor Class
 */
class DegradeProcessor {
public:
    DegradeProcessor();
    ~DegradeProcessor() {}

    void prepare(float sampleRate);
    
    // Parameters
    void setParameters(float depth, float amount, float variance, float envelope, bool enabled);

    void processBlock(float* inL, float* inR, int blockSize);

private:
    void cookParams();
    void processShortBlock(float* inL, float* inR, int numSamples);

    float fs;
    bool onOff;

    // Parameters
    float p_depth, p_amount, p_variance, p_envelope;

    // Sub-Processors
    DegradeFilter filters[2];
    DegradeNoise noises[2];
    SimpleLevelDetector levelDetector;
    
    // Internal Scratch Buffers (SRAM safe size)
    float noiseBuf[DEG_BLOCK_SIZE];
    float levelBuf[DEG_BLOCK_SIZE];

    RandomGen random;
    int sampleCounter;
    
    // Output Gain Smoother
    float targetGain, currentGain;
};

#endif // DAISY_DEGRADE_H