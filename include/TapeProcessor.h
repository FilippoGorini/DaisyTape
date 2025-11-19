#pragma once
#ifndef TAPEPROCESSOR_H
#define TAPEPROCESSOR_H

#include <stdint.h> 
#include "Config.h" 
#include "DaisyInputFilters.h" 
#include "daisysp.h" 

// The Dry Delay uses the same massive size needed for latency compensation.
using DryDelayLine = daisysp::DelayLine<float, MAKEUP_DELAY_SIZE>; 

/**
 * @brief Structure holding all the exposed control parameters for the tape model.
 */
struct TapeParams
{
    float lowCutFreq;
    float highCutFreq;
    float dryWet;
    bool filtersEnabled;
    bool makeupEnabled;
    // Add other parameters (drive, rate, depth, etc.) here as they are ported
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
    
    // ... Other modules (Hysteresis, LossFilter, etc.) will go here ...

    // --- Internal Buffers (Can be up to SAFE_MAX_BLOCK_SIZE) ---
    static constexpr int kMaxBlockSize = SAFE_MAX_BLOCK_SIZE;

    float bufferL[kMaxBlockSize];
    float bufferR[kMaxBlockSize];
    float dryBufferL[kMaxBlockSize];
    float dryBufferR[kMaxBlockSize];

    // --- Dry Path Delay (Pointers to SDRAM) ---
    DryDelayLine* dryDelayL = nullptr;
    DryDelayLine* dryDelayR = nullptr;

    // --- Parameters (Internal state is kept in sub-processors) ---
    float dryWet;
};

#endif // TAPEPROCESSOR_H