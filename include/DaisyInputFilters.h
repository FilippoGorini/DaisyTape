#pragma once
#ifndef DAISY_INPUTFILTERS_H
#define DAISY_INPUTFILTERS_H

#include "Config.h"
#include "daisy_seed.h"
#include "DaisyLinkwitzRiley.h"
#include "daisysp.h" // For daisysp::DelayLine 
#include <algorithm>
#include <cmath>
#include <cassert>

// Define the size needed for the delay line (2^21 samples)
#define MAKEUP_DELAY_SIZE 2097152

// The DelayLine type must be defined globally in the main program using DSY_SDRAM_BSS.
// The InputFilters class holds pointers to the globally allocated DelayLines.
using MakeupDelayLine = daisysp::DelayLine<float, MAKEUP_DELAY_SIZE>;

class InputFilters
{
public:
    InputFilters();
    ~InputFilters() {}

    // This function sets the pointers to the globally allocated SDRAM delay lines
    void prepare(float sampleRate, int numCh);
    void setDelayLinePointers(MakeupDelayLine* delayL, MakeupDelayLine* delayR);

    void processBlock(float* bufferL, float* bufferR, int32_t blockSize);
    void processBlockMakeup(float* bufferL, float* bufferR, int32_t blockSize);
    void setMakeupDelay(float delaySamples);

    // Called from main thread: stage new parameters
    void prepareParams(float lowCut, float highCut, bool enabled, bool makeupEnabled);
    // Called from interrupt: apply staged parameters
    void applyParams();

private:
    // Staged values — written from main, consumed by applyParams() in interrupt
    float pendingLowCut;
    float pendingHighCut;
    bool pendingOnOff;
    bool pendingMakeup;
    volatile bool paramsDirty;

    // Volatile to avoid register caching during compiling optimization 
    volatile bool onOff;
    volatile bool makeup;
    float fs;
    int numChannels;
    float lowCutFreq;
    float highCutFreq;

    LinkwitzRileyFilter<float> lowCutFilter[2];
    LinkwitzRileyFilter<float> highCutFilter[2];

    // Change objects to pointers
    MakeupDelayLine* makeupDelay[2];

    // Use a safe max size for internarl scratch buffers
    float makeupLowBuffer[2][SAFE_MAX_BLOCK_SIZE];
    float makeupHighBuffer[2][SAFE_MAX_BLOCK_SIZE];
};

#endif // DAISY_INPUTFILTERS_H