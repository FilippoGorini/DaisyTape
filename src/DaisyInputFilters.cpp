#include "DaisyInputFilters.h"

InputFilters::InputFilters()
    : pendingLowCut(20.0f), pendingHighCut(22000.0f),
      pendingOnOff(false), pendingMakeup(false), paramsDirty(false),
      onOff(false), makeup(false),
      fs(48000.0f), numChannels(0),
      lowCutFreq(20.0f), highCutFreq(22000.0f),
      makeupDelay{ nullptr, nullptr }
{
}

/** CRITICAL: Function to link the SDRAM-allocated objects */
void InputFilters::setDelayLinePointers(MakeupDelayLine* delayL, MakeupDelayLine* delayR)
{
    makeupDelay[0] = delayL;
    makeupDelay[1] = delayR;
}

void InputFilters::prepare(float sampleRate, int numCh)
{
    fs          = sampleRate;
    numChannels = std::min(numCh, 2); // Max 2 channels

    for(int i = 0; i < numChannels; ++i)
    {
        lowCutFilter[i].prepare(fs, 1); // Only 1 channel per filter instance
        lowCutFilter[i].setCutoff(lowCutFreq);

        highCutFilter[i].prepare(fs, 1);
        highCutFilter[i].setCutoff(highCutFreq);
        
        // Init the SDRAM delay lines via the pointers
        if (makeupDelay[i] != nullptr)
        {
            makeupDelay[i]->Init();
            makeupDelay[i]->SetDelay(0.0f);
        }
    }
}

void InputFilters::processBlock(float* bufferL, float* bufferR, int32_t blockSize)
{
    assert(blockSize <= SAFE_MAX_BLOCK_SIZE);
    if(!onOff)
        return;

    float* buffers[2] = {bufferL, bufferR};

    for(int ch = 0; ch < numChannels; ++ch)
    {
        for(int n = 0; n < blockSize; ++n)
        {
            float inputSample = buffers[ch][n];
            float highPassSample = 0.0f;
            float bandPassSample = 0.0f;

            // 1. Low-cut filter (Separates Low-pass (Trash) from High-pass (Mid/High))
            lowCutFilter[ch].processSample(0,
                                           inputSample,
                                           makeupLowBuffer[ch][n],  // low-pass output (The part we're cutting)
                                           highPassSample);         // high-pass output (The part going forward)

            // 2. High-cut filter (Separates High-pass (Trash) from Low-pass (Mid/Low))
            highCutFilter[ch].processSample(0,
                                            highPassSample,
                                            bandPassSample,          // low-pass output (Our final clean signal for tape)
                                            makeupHighBuffer[ch][n]); // high-pass output (The part we're cutting)

            // 3. Write main signal back to buffer (This is the filtered signal going to Hysteresis)
            buffers[ch][n] = bandPassSample;
        }

        lowCutFilter[ch].snapToZero();
        highCutFilter[ch].snapToZero();
    }
}

void InputFilters::processBlockMakeup(float* bufferL, float* bufferR, int32_t blockSize)
{
    assert(blockSize <= SAFE_MAX_BLOCK_SIZE);
    
    // We only need to check for makeup if the overall filters are ON and makeup is ON
    if(!onOff || !makeup)
        return;

    float* buffers[2] = {bufferL, bufferR};

    for(int ch = 0; ch < numChannels; ++ch)
    {
        if (makeupDelay[ch] == nullptr) continue;

        for(int n = 0; n < blockSize; ++n)
        {
            // 1. Combine the makeup signals from the main processBlock
            float makeupSignal = makeupLowBuffer[ch][n] + makeupHighBuffer[ch][n];

            // 2. Write to delay line and read the delayed sample
            makeupDelay[ch]->Write(makeupSignal);
            float delayedMakeup = makeupDelay[ch]->Read();

            // 3. Add the delayed makeup back to the main buffer
            buffers[ch][n] += delayedMakeup;
        }
    }
}

void InputFilters::setMakeupDelay(float delaySamples)
{
    for(int i = 0; i < numChannels; ++i)
    {
        if (makeupDelay[i] != nullptr)
            makeupDelay[i]->SetDelay(delaySamples);
    }
}

void InputFilters::prepareParams(float lowCut, float highCut, bool enabled, bool makeupEnabled)
{
    pendingLowCut  = lowCut;
    pendingHighCut = highCut;
    pendingOnOff   = enabled;
    pendingMakeup  = makeupEnabled;
    __DMB();    // Data Memory Barrier: ARM Cortex CPU instruction that forces all pending memory write operations to complete before execution continues
    paramsDirty = true;
}

void InputFilters::applyParams()
{
    // Don't apply params unless they all have been prepared (paramsDirty flag is set to true)
    if (!paramsDirty) return;
    paramsDirty = false;

    onOff  = pendingOnOff;
    makeup = pendingMakeup;

    lowCutFreq = pendingLowCut;
    for (int i = 0; i < numChannels; ++i)
        lowCutFilter[i].setCutoff(lowCutFreq);

    highCutFreq = std::fmin(pendingHighCut, fs * 0.48f);
    for (int i = 0; i < numChannels; ++i)
        highCutFilter[i].setCutoff(highCutFreq);
}
