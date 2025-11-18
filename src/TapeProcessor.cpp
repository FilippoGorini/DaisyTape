#include "TapeProcessor.h"
#include <cstring>

// Implementation for linking the SDRAM objects
void TapeProcessor::setDelayLinePointers(MakeupDelayLine* makeL, MakeupDelayLine* makeR,
                                         DryDelayLine* dryL, DryDelayLine* dryR)
{
    // Pass Makeup Delay pointers to the InputFilters object
    inputFilters.setDelayLinePointers(makeL, makeR);
    
    // Store Dry Delay pointers internally
    dryDelayL = dryL;
    dryDelayR = dryR;
}

void TapeProcessor::Init(float sampleRate)
{
    const int numChannels = 2;
    inputFilters.prepare(sampleRate, numChannels);
    
    // Init the Dry Delay objects via the pointers
    if (dryDelayL != nullptr)
    {
        dryDelayL->Init();
        dryDelayL->SetDelay(0.0f);
    }
    if (dryDelayR != nullptr)
    {
        dryDelayR->Init();
        dryDelayR->SetDelay(0.0f);
    }
}

void TapeProcessor::processBlock(const float* inL,
                                 const float* inR,
                                 float* outL,
                                 float* outR,
                                 int32_t blockSize)
{
    // 1. Store dry signal and copy input to our internal wet buffer
    for (int32_t i = 0; i < blockSize; i++)
    {
        dryBufferL[i] = inL[i];
        dryBufferR[i] = inR[i];
        bufferL[i]    = inL[i];
        bufferR[i]    = inR[i];
    }

    // --- 2. WET SIGNAL PATH ---
    inputFilters.processBlock(bufferL, bufferR, blockSize);

    // ... Other modules will go here ...

    // --- 3. LATENCY COMPENSATION ---
    latencyCompensation(blockSize);

    // --- 4. MAKEUP GAIN PATH ---
    inputFilters.processBlockMakeup(bufferL, bufferR, blockSize);

    // ... Other makeup modules will go here ...

    // --- 5. FINAL MIX ---
    dryWetMix(outL, outR, blockSize);
}

void TapeProcessor::latencyCompensation(int32_t blockSize)
{
    // 1. Calculate total latency from all wet path modules
    float totalLatency = 0.0f;
    // totalLatency += lossFilter.getLatencySamples();
    // totalLatency += compression.getLatencySamples();

    // Placeholder: Use the fixed latency until we implement the other modules
    totalLatency = 300.0f;

    // 2. Set delay for InputFilters' makeup path
    inputFilters.setMakeupDelay(totalLatency);

    // 3. Set delay for the main Dry Path
    if (dryDelayL != nullptr) dryDelayL->SetDelay(totalLatency);
    if (dryDelayR != nullptr) dryDelayR->SetDelay(totalLatency);

    // 4. Process the dry buffer through the delay
    if (dryDelayL != nullptr && dryDelayR != nullptr)
    {
        for (int32_t i = 0; i < blockSize; i++)
        {
            dryDelayL->Write(dryBufferL[i]);
            dryDelayR->Write(dryBufferR[i]);
            dryBufferL[i] = dryDelayL->Read();
            dryBufferR[i] = dryDelayR->Read();
        }
    }
}

void TapeProcessor::dryWetMix(float* outL, float* outR, int32_t blockSize)
{
    for (int32_t i = 0; i < blockSize; i++)
    {
        float wetL = bufferL[i];
        float wetR = bufferR[i];
        
        float dryL = dryBufferL[i];
        float dryR = dryBufferR[i];

        // This is the classic linear crossfade mix
        outL[i] = (dryL * (1.0f - dryWet)) + (wetL * dryWet);
        outR[i] = (dryR * (1.0f - dryWet)) + (wetR * dryWet);
    }
}
