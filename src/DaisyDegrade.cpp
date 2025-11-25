#include "DaisyDegrade.h"
#include <cmath>
#include <cassert>

DegradeProcessor::DegradeProcessor()
    : fs(48000.0f), onOff(true), usePoint1xFlag(false),
      p_depth(0.0f), p_amount(0.0f), p_variance(0.0f), p_envelope(0.0f),
      sampleCounter(0)
{
    paramRng.setSeed(0x12345678abcdefULL);
    gainSmoother.setCurrentAndTargetValue(1.0f);
}

void DegradeProcessor::prepare(float sampleRate)
{
    fs = sampleRate;
    sampleCounter = 0;

    for (int i = 0; i < 2; ++i)
    {
        filters[i].reset(fs);
        noises[i].prepare( (uint64_t)(0x1000 + i) );
    }

    levelDetector.prepare(fs, DEG_BLOCK_SIZE);
    
    gainSmoother.setCurrentAndTargetValue(1.0f);

    cookParams();
}

void DegradeProcessor::setParameters(float depth, float amount, float variance, float envelope, bool enabled, bool usePoint1x)
{
    p_depth = depth;
    p_amount = amount;
    p_variance = variance;
    p_envelope = envelope;
    onOff = enabled;
    usePoint1xFlag = usePoint1x;
}

void DegradeProcessor::cookParams()
{
    float depthValue = usePoint1xFlag ? p_depth * 0.1f : p_depth;

    float freqHz = 200.0f * std::pow(20000.0f / 200.0f, 1.0f - p_amount);
    float gainDB = -24.0f * depthValue;

    float noiseGain = 0.5f * depthValue * p_amount;
    for (int ch = 0; ch < 2; ++ch)
        noises[ch].setGain(noiseGain);

    for (int ch = 0; ch < 2; ++ch)
    {
        float rv = paramRng.nextFloat() - 0.5f; 
        float varFreq = p_variance * (freqHz / 0.6f) * rv;
        float finalFreq = freqHz + varFreq;

        if (finalFreq > fs * 0.49f) finalFreq = fs * 0.49f;
        if (finalFreq < 20.0f) finalFreq = 20.0f;

        filters[ch].setFreq(finalFreq);
    }

    float envSkew = 1.0f - std::pow(p_envelope, 0.8f);
    float attackMs = 10.0f;
    float releaseMs = 20.0f * std::pow(5000.0f / 20.0f, envSkew);
    levelDetector.setParameters(attackMs, releaseMs);

    float gainVar = p_variance * 36.0f * (paramRng.nextFloat() - 0.5f);
    float finalGainDB = gainDB + gainVar;
    if (finalGainDB > 3.0f) finalGainDB = 3.0f;

    // --- CRITICAL FIX: Set target for Smoother ---
    // Ramp to the new gain over the NEXT 2048 samples.
    // This guarantees the ramp takes ~42ms, not 1ms.
    float nextGain = std::pow(10.0f, finalGainDB / 20.0f);
    gainSmoother.setSteps(DEG_BLOCK_SIZE); 
    gainSmoother.setTargetValue(nextGain);
}

void DegradeProcessor::processBlock(float* inL, float* inR, int blockSize)
{
    if (!onOff)
        return;

    int processed = 0;
    while (processed < blockSize)
    {
        int remaining = blockSize - processed;
        int remainingUntilUpdate = DEG_BLOCK_SIZE - sampleCounter;
        int chunk = std::min(remaining, remainingUntilUpdate);

        float* chunkL = inL + processed;
        float* chunkR = inR + processed;

        processShortBlock(chunkL, chunkR, chunk);

        processed += chunk;
        sampleCounter += chunk;

        if (sampleCounter >= DEG_BLOCK_SIZE)
        {
            cookParams();
            sampleCounter = 0;
        }
    }
}

void DegradeProcessor::processShortBlock(float* chunkL, float* chunkR, int numSamples)
{
    // 1) Level detection
    levelDetector.process(chunkL, chunkR, levelBuf, numSamples, 2);

    bool applyEnvelope = (p_envelope > 0.0f);

    // 2) Left noise + filter
    std::memset(noiseBufL, 0, sizeof(float) * numSamples);
    noises[0].processBlock(noiseBufL, numSamples);
    for (int i = 0; i < numSamples; ++i)
    {
        float n = noiseBufL[i];
        if (applyEnvelope) n *= levelBuf[i];
        chunkL[i] += n;
    }
    filters[0].process(chunkL, numSamples);

    // 3) Right noise + filter
    std::memset(noiseBufR, 0, sizeof(float) * numSamples);
    noises[1].processBlock(noiseBufR, numSamples);
    for (int i = 0; i < numSamples; ++i)
    {
        float n = noiseBufR[i];
        if (applyEnvelope) n *= levelBuf[i];
        chunkR[i] += n;
    }
    filters[1].process(chunkR, numSamples);

    // 4) Apply Output Gain (Smoothed)
    for (int i = 0; i < numSamples; ++i)
    {
        // Get next value from the smoother.
        // This handles the interpolation over 2048 steps automatically.
        float g = gainSmoother.getNextValue();
        chunkL[i] *= g;
        chunkR[i] *= g;
    }
}