#pragma once
#ifndef TAPEPROCESSOR_H
#define TAPEPROCESSOR_H

#include <stdint.h> 
#include "Config.h" 
#include "DaisyInputFilters.h" 
#include "DaisyLossFilter.h" 
#include "DaisyDegrade.h"
#include "daisysp.h" 

// The Dry Delay uses the same massive size needed for latency compensation.
using DryDelayLine = daisysp::DelayLine<float, MAKEUP_DELAY_SIZE>; 

/**
 * @brief Structure holding all the exposed control parameters for the tape model.
 */
struct TapeParams
{
    // Input Filters
    float lowCutFreq;
    float highCutFreq;
    bool filtersEnabled;
    bool makeupEnabled;

    // Tape Physics (Loss Filter)
    float speed;     // Inches per second (e.g., 7.5, 15, 30)
    float gap;       // Microns
    float spacing;   // Microns
    float thickness; // Microns
    float loss;      // Not really needed, added just to ease serial logging

    // Degradation (Added)
    float deg_depth;
    float deg_amount;
    float deg_variance;
    float deg_envelope;
    bool deg_enabled;
    bool usePoint1x;

    // Global
    float dryWet;
};

/**
 * @brief Main Tape Emulation Processor
 */
class TapeProcessor
{
public:
    TapeProcessor() : dryWet(1.0f) {}
    ~TapeProcessor() {}

    void Init(float sampleRate, const TapeParams& params);

    /**
     * @brief CRITICAL: Sets the pointers to the globally allocated SDRAM delay lines.
     * Includes new pointers for the Azimuth/Loss section.
     */
    void setDelayLinePointers(MakeupDelayLine* makeL, MakeupDelayLine* makeR,
                              DryDelayLine* dryL, DryDelayLine* dryR);

    /**
     * @brief Updates all control parameters from the given structure.
     */
    void updateParams(const TapeParams& params);


    void processBlock(const float* inL,
                      const float* inR,
                      float* outL,
                      float* outR,
                      int32_t blockSize);

    void latencyCompensation(int32_t blockSize);
    void dryWetMix(float* outL, float* outR, int32_t blockSize);

private:
    // --- Processing Modules ---
    InputFilters inputFilters;
    LossFilter lossFilter;
    DegradeProcessor degradeProcessor; // <--- CRITICAL INSTANCE
    
    // ... Other modules (Hysteresis, etc.) will go here ...

    // --- Internal Buffers ---
    static constexpr int kMaxBlockSize = SAFE_MAX_BLOCK_SIZE;

    float bufferL[kMaxBlockSize];
    float bufferR[kMaxBlockSize];
    float dryBufferL[kMaxBlockSize];
    float dryBufferR[kMaxBlockSize];

    // --- Dry Path Delay (Pointers to SDRAM) ---
    DryDelayLine* dryDelayL = nullptr;
    DryDelayLine* dryDelayR = nullptr;

    // --- Parameters ---
    float dryWet;
};

#endif // TAPEPROCESSOR_H