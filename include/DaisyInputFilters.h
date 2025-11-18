#pragma once
#ifndef DAISY_INPUTFILTERS_H
#define DAISY_INPUTFILTERS_H

#include "DaisyLinkwitzRiley.h"
#include "daisysp.h" // For daisysp::DelayLine
#include <algorithm>
#include <cmath>
#include <cassert>

// Define the size needed for the delay line (2^21 samples)
#define MAKEUP_DELAY_SIZE 2097152 

// Define a safe max block size for internal buffers (e.g., 256 samples)
// This avoids relying on kMaxBlockSize from Config.h if it's too small.
#define SAFE_MAX_BLOCK_SIZE 256

/**
 * The DelayLine type must be defined globally in the main program using DSY_SDRAM_BSS.
 * The InputFilters class holds pointers to the globally allocated DelayLines.
 */
using MakeupDelayLine = daisysp::DelayLine<float, MAKEUP_DELAY_SIZE>;

class InputFilters
{
public:
    InputFilters(); 
    ~InputFilters() {}

    void prepare(float sampleRate, int numCh);
    
    /**
     * @brief CRITICAL: Sets the pointers to the globally allocated SDRAM delay lines.
     * @param delayL Pointer to the Left Channel delay line (SDRAM).
     * @param delayR Pointer to the Right Channel delay line (SDRAM).
     */
    void setDelayLinePointers(MakeupDelayLine* delayL, MakeupDelayLine* delayR);

    void processBlock(float* bufferL, float* bufferR, int32_t blockSize);
    void processBlockMakeup(float* bufferL, float* bufferR, int32_t blockSize);

    void setMakeupDelay(float delaySamples);
    
    // --- Parameter Setters ---
    void setLowCut(float freqHz);
    void setHighCut(float freqHz);
    void setEnabled(bool enabled) { onOff = enabled; }
    void setMakeupEnabled(bool enabled) { makeup = enabled; }
    bool isMakeupEnabled() const { return makeup; }
    bool isEnabled() const { return onOff; }

private:
    bool onOff;
    bool makeup;
    float fs;
    int numChannels;
    float lowCutFreq;
    float highCutFreq;

    LinkwitzRileyFilter<float> lowCutFilter[2];
    LinkwitzRileyFilter<float> highCutFilter[2];

    // *** FIX 1: Change objects to pointers ***
    MakeupDelayLine* makeupDelay[2];

    // *** FIX 2: Use a safe max size for internal scratch buffers ***
    float makeupLowBuffer[2][SAFE_MAX_BLOCK_SIZE];
    float makeupHighBuffer[2][SAFE_MAX_BLOCK_SIZE];
};

#endif // DAISY_INPUTFILTERS_H