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
 * @brief Main Tape Emulation Processor
 *
 * This class holds all the individual processing modules
 * (InputFilters, Hysteresis, etc.) and runs them in sequence,
 * managing the signal flow and latency compensation.
 *
 * It operates on a block-based architecture.
 */
class TapeProcessor
{
public:
    TapeProcessor() : dryWet(1.0f) {}
    ~TapeProcessor() {}

    void Init(float sampleRate);

    /**
     * @brief CRITICAL: Sets the pointers to the globally allocated SDRAM delay lines.
     * @param makeL/R Pointers for the Input Filter makeup delay.
     * @param dryL/R  Pointers for the main dry signal delay.
     */
    void setDelayLinePointers(MakeupDelayLine* makeL, MakeupDelayLine* makeR,
                              DryDelayLine* dryL, DryDelayLine* dryR);

    void processBlock(const float* inL,
                      const float* inR,
                      float* outL,
                      float* outR,
                      int32_t blockSize);

    void latencyCompensation(int32_t blockSize);
    void dryWetMix(float* outL, float* outR, int32_t blockSize);

    // --- Parameter Setters ---
    void SetLowCutFreq(float freqHz) { inputFilters.setLowCut(freqHz); }
    void SetHighCutFreq(float freqHz) { inputFilters.setHighCut(freqHz); }
    void SetFiltersEnabled(bool enabled) { inputFilters.setEnabled(enabled); }
    void SetMakeupEnabled(bool enabled) { inputFilters.setMakeupEnabled(enabled); }
    void SetDryWet(float normVal) { dryWet = normVal; }

private:
    // --- Processing Modules ---
    InputFilters inputFilters;
    
    // ... Other modules (Hysteresis, LossFilter, etc.) will go here ...

    // --- Internal Buffers (Can be up to SAFE_MAX_BLOCK_SIZE) ---
    static constexpr int kMaxBlockSize = SAFE_MAX_BLOCK_SIZE;

    // These hold the "wet" signal as it's being processed
    float bufferL[kMaxBlockSize];
    float bufferR[kMaxBlockSize];

    // These hold the clean "dry" signal
    float dryBufferL[kMaxBlockSize];
    float dryBufferR[kMaxBlockSize];

    // --- Dry Path Delay (Pointers to SDRAM) ---
    DryDelayLine* dryDelayL = nullptr;
    DryDelayLine* dryDelayR = nullptr;

    // --- Parameters ---
    float dryWet;
};

#endif // TAPEPROCESSOR_H
