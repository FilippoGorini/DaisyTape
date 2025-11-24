#include "DaisyDegrade.h"

DegradeProcessor::DegradeProcessor() : 
    fs(48000.0f), onOff(true), 
    p_depth(0.0f), p_amount(0.0f), p_variance(0.0f), p_envelope(0.0f),
    sampleCounter(0), targetGain(1.0f), currentGain(1.0f)
{
}

void DegradeProcessor::prepare(float sampleRate)
{
    fs = sampleRate;
    sampleCounter = 0;

    for(int i=0; i<2; i++) {
        filters[i].reset(sampleRate);
        noises[i].prepare();
    }
    levelDetector.prepare(sampleRate);
}

void DegradeProcessor::setParameters(float depth, float amount, float variance, float envelope, bool enabled)
{
    p_depth = depth;
    p_amount = amount;
    p_variance = variance;
    p_envelope = envelope;
    onOff = enabled;
}

void DegradeProcessor::cookParams()
{
    // Logic ported from JUCE cookParams()
    
    // --- USER CHANGE: Force "0.1x" mode for finer control ---
    // This scales the raw depth input (0.0 - 1.0) down to (0.0 - 0.1)
    float depthValue = p_depth * 0.1f; 

    // Frequency Logic
    // 200Hz base, shifting up to 20kHz based on Amount
    // Inverse logic: High amount = Less filtering (higher freq)? Or more degrade?
    // Original: freqHz = 200 * (20000/200)^(1 - amt)
    // If Amt=0 -> Freq=20k (Clean). If Amt=1 -> Freq=200 (Dirty).
    float freqHz = 200.0f * std::pow(20000.0f / 200.0f, 1.0f - p_amount);
    
    // Gain Logic: -24dB * depth
    float gainDB = -24.0f * depthValue;

    // Update Noises gain
    // Gain = 0.5 * depth * amt
    float noiseGain = 0.5f * depthValue * p_amount;
    for(int i=0; i<2; i++) noises[i].setGain(noiseGain);

    // Update Filters with Random Variance
    // Var adds randomness to the cutoff frequency
    for(int i=0; i<2; i++) {
        float randVal = random.nextFloat() - 0.5f; // -0.5 to 0.5
        float freqVar = p_variance * (freqHz / 0.6f) * randVal;
        float finalFreq = freqHz + freqVar;
        
        // Clamp to Nyquist
        if (finalFreq > fs * 0.49f) finalFreq = fs * 0.49f;
        if (finalFreq < 20.0f) finalFreq = 20.0f;
        
        filters[i].setFreq(finalFreq);
    }

    // Level Detector Attack/Release skew
    float envSkew = 1.0f - std::pow(p_envelope, 0.8f);
    float attack = 10.0f;
    float release = 20.0f * std::pow(5000.0f / 20.0f, envSkew);
    levelDetector.setParameters(attack, release);

    // Output Gain with Variance
    float gainVar = p_variance * 36.0f * (random.nextFloat() - 0.5f);
    float finalGainDB = gainDB + gainVar;
    if (finalGainDB > 3.0f) finalGainDB = 3.0f; // Clip max gain
    
    targetGain = std::pow(10.0f, finalGainDB / 20.0f);
}

void DegradeProcessor::processBlock(float* inL, float* inR, int blockSize)
{
    if (!onOff) return;

    int samplesProcessed = 0;
    while (samplesProcessed < blockSize)
    {
        // Calculate how many samples to process in this chunk
        // We stop at the boundary of DEG_BLOCK_SIZE to update params (cookParams)
        int remainingInBlock = blockSize - samplesProcessed;
        int remainingUntilUpdate = DEG_BLOCK_SIZE - sampleCounter;
        
        int chunkSize = std::min(remainingInBlock, remainingUntilUpdate);

        // Pointers to the current chunk
        float* chunkL = inL + samplesProcessed;
        float* chunkR = inR + samplesProcessed;

        // Process this chunk
        processShortBlock(chunkL, chunkR, chunkSize);

        samplesProcessed += chunkSize;
        sampleCounter += chunkSize;

        // If we filled the counter, reset and cook params for the next chunk
        if (sampleCounter >= DEG_BLOCK_SIZE) {
            cookParams();
            sampleCounter = 0;
        }
    }
}

void DegradeProcessor::processShortBlock(float* chunkL, float* chunkR, int numSamples)
{
    // 1. Generate Noise
    // We reuse the same noise buffer for L and R to save memory? 
    // No, stereo noise is better. But we have 1 scratch buffer.
    // Let's generate for L, then R. Or process sample by sample.
    // The class has noises[2], but only one noiseBuf.
    // We'll generate noise into noiseBuf for Left, process, then reuse for Right.
    
    // 2. Level Detection (Envelope Follower)
    // Needs both channels.
    levelDetector.process(chunkL, chunkR, levelBuf, numSamples);

    bool applyEnvelope = p_envelope > 0.0f;

    // --- Process Left ---
    noises[0].processBlock(noiseBuf, numSamples);
    for(int i=0; i<numSamples; i++) {
        float n = noiseBuf[i];
        // Modulate noise by input envelope if enabled
        if (applyEnvelope) n *= levelBuf[i];
        // Add noise to input
        chunkL[i] += n;
    }
    // Filter Left
    filters[0].process(chunkL, numSamples);

    // --- Process Right ---
    noises[1].processBlock(noiseBuf, numSamples);
    for(int i=0; i<numSamples; i++) {
        float n = noiseBuf[i];
        if (applyEnvelope) n *= levelBuf[i];
        chunkR[i] += n;
    }
    // Filter Right
    filters[1].process(chunkR, numSamples);

    // --- Apply Output Gain ---
    // Simple smoothing
    if (std::abs(targetGain - currentGain) > 1e-4f) {
        for(int i=0; i<numSamples; i++) {
            currentGain += (targetGain - currentGain) * 0.01f; // Soft slew
            chunkL[i] *= currentGain;
            chunkR[i] *= currentGain;
        }
    } else {
        currentGain = targetGain;
        for(int i=0; i<numSamples; i++) {
            chunkL[i] *= currentGain;
            chunkR[i] *= currentGain;
        }
    }
}