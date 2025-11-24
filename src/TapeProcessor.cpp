#include "TapeProcessor.h"
#include <cstring> 

void TapeProcessor::setDelayLinePointers(MakeupDelayLine* makeL, MakeupDelayLine* makeR,
                                         DryDelayLine* dryL, DryDelayLine* dryR)
{
    // Pass pointers to sub-modules
    inputFilters.setDelayLinePointers(makeL, makeR);
    
    // Store Dry Delay pointers internally
    dryDelayL = dryL;
    dryDelayR = dryR;
}

void TapeProcessor::Init(float sampleRate, const TapeParams& params)
{
    const int numChannels = 2;
    inputFilters.prepare(sampleRate, numChannels);
    lossFilter.prepare(sampleRate);
    degradeProcessor.prepare(sampleRate); // <--- ADDED PREPARE
    
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

    updateParams(params);
}

void TapeProcessor::updateParams(const TapeParams& params)
{
    // --- Input Filters ---
    inputFilters.setLowCut(params.lowCutFreq);
    inputFilters.setHighCut(params.highCutFreq);
    inputFilters.setEnabled(params.filtersEnabled);
    inputFilters.setMakeupEnabled(params.makeupEnabled);

    // --- Loss Filter ---
    lossFilter.setParameters(params.speed, 
                             params.spacing, 
                             params.thickness, 
                             params.gap);

    // --- Degrade Processor --- <--- ADDED PARAMS UPDATE
    degradeProcessor.setParameters(params.deg_depth,
                                   params.deg_amount,
                                   params.deg_variance,
                                   params.deg_envelope,
                                   params.deg_enabled);

    // --- Top-Level Parameters ---
    dryWet = params.dryWet;
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
    
    // A. Input Filters
    inputFilters.processBlock(bufferL, bufferR, blockSize);

    // B. Degrade Processor (Add Noise/Wow/Flutter before the tape head loss)
    // <--- ADDED DEGRADE PROCESSING HERE
    degradeProcessor.processBlock(bufferL, bufferR, blockSize);

    // C. Loss Filter (Head simulation)
    // Note: In original structure, this was last, but without Hysteresis/Compression,
    // we place it here. It modifies bufferL/bufferR in place.
    lossFilter.processBlock(bufferL, bufferR, bufferL, bufferR, blockSize);

    // --- 3. LATENCY COMPENSATION ---
    latencyCompensation(blockSize);

    // --- 4. MAKEUP GAIN PATH ---
    // Must happen AFTER latency compensation to align with the delayed wet signal
    inputFilters.processBlockMakeup(bufferL, bufferR, blockSize);

    // --- 5. FINAL MIX ---
    dryWetMix(outL, outR, blockSize);
}

void TapeProcessor::latencyCompensation(int32_t blockSize)
{
    // 1. Calculate total latency from all wet path modules
    float totalLatency = 0.0f;
    
    // The Loss Filter (FIR) introduces significant latency (Order/2)
    totalLatency += lossFilter.getLatencySamples();
    
    // Future: Add latency from Oversampling/Compression here
    // totalLatency += compression.getLatencySamples();

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

        outL[i] = (dryL * (1.0f - dryWet)) + (wetL * dryWet);
        outR[i] = (dryR * (1.0f - dryWet)) + (wetR * dryWet);
    }
}