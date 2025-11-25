#pragma once
#ifndef DAISY_DEGRADE_H
#define DAISY_DEGRADE_H

#include "daisy_seed.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdint>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Internal Control Block Size (Modulation Rate)
#define DEG_BLOCK_SIZE 2048

// -------------------------------
// JUCE-compatible 48-bit LCG Random
// -------------------------------
class JuceRandom
{
public:
    JuceRandom(uint64_t seed = 1ull) noexcept
    {
        setSeed(seed);
    }

    void setSeed(uint64_t newSeed) noexcept
    {
        seed = (int64_t)(newSeed & 0xFFFFFFFFFFFFLL);
    }

    int nextInt() noexcept
    {
        uint64_t s = (uint64_t)seed;
        s = (s * 0x5deece66dULL + 11ULL) & 0xFFFFFFFFFFFFULL;
        seed = (int64_t)s;
        return (int)(seed >> 16);
    }

    float nextFloat() noexcept
    {
        std::uint32_t v = static_cast<std::uint32_t>(nextInt());        // For some reason had to add std:: to avoid vscode warnings? It built succesfully even before nevertheless, so it's not really needed
        const float denom = static_cast<float>(std::numeric_limits<std::uint32_t>::max()) + 1.0f;
        float res = static_cast<float>(v) / denom;
        if (res == 1.0f) res = 1.0f - std::numeric_limits<float>::epsilon();
        return res;
    }

private:
    int64_t seed;
};

// -------------------------------
// Linear smoothed value (For Gain)
// Matches JUCE SmoothedValue<Linear> behavior
// -------------------------------
class LinSmoothed
{
public:
    LinSmoothed(float initial = 0.0f) noexcept
    {
        current = target = initial;
        stepsToTarget = 0;
        step = 0.0f;
    }

    void reset(int numSteps)
    {
        stepsToTarget = numSteps;
        setCurrentAndTargetValue(target);
    }

    void setCurrentAndTargetValue(float v)
    {
        target = current = v;
        stepsToTarget = 0;
        step = 0.0f;
    }

    void setTargetValue(float newValue)
    {
        if (newValue == target) return;
        if (stepsToTarget <= 0)
        {
            setCurrentAndTargetValue(newValue);
            return;
        }
        target = newValue;
        countdown = stepsToTarget;
        // Linear step calculation
        step = (target - current) / (float)countdown;
    }

    bool isSmoothing() const noexcept { return countdown > 0; }

    float getCurrentValue() const noexcept { return current; }

    float getNextValue() noexcept
    {
        if (!isSmoothing()) return target;
        --countdown;
        if (countdown > 0)
            current += step;
        else
            current = target;
        return current;
    }

    void setSteps(int s) { stepsToTarget = s; countdown = 0; }

private:
    float current, target, step;
    int stepsToTarget, countdown;
};

// -------------------------------
// Multiplicative smoothed value (For Freq)
// Matches JUCE SmoothedValue<Multiplicative> behavior
// -------------------------------
class MulSmoothed
{
public:
    MulSmoothed(float initial = 1.0f) noexcept
    {
        current = target = initial;
        stepsToTarget = 0;
        step = 1.0f;
    }

    void reset(int numSteps)
    {
        stepsToTarget = numSteps;
        setCurrentAndTargetValue(target);
    }

    void setCurrentAndTargetValue(float v)
    {
        target = current = v;
        stepsToTarget = 0;
        step = 1.0f;
    }

    void setTargetValue(float newValue)
    {
        if (newValue == target) return;
        if (stepsToTarget <= 0)
        {
            setCurrentAndTargetValue(newValue);
            return;
        }
        target = newValue;
        countdown = stepsToTarget;
        step = std::exp((std::log(std::abs(target)) - std::log(std::abs(current))) / (float)countdown);
    }

    bool isSmoothing() const noexcept { return countdown > 0; }

    float getCurrentValue() const noexcept { return current; }

    float getNextValue() noexcept
    {
        if (!isSmoothing()) return target;
        --countdown;
        if (countdown > 0)
            current *= step;
        else
            current = target;
        return current;
    }

    void setSteps(int s) { stepsToTarget = s; countdown = 0; }

private:
    float current, target, step;
    int stepsToTarget, countdown;
};

// -------------------------------
// Chow Level Detector
// -------------------------------
class ChowLevelDetector
{
public:
    void prepare(float sampleRate, int maxBlockSize = 1024)
    {
        fs = sampleRate;
        expFactor = -1000.0f / fs;
        yOld = 0.0f;
        increasing = true;
    }

    void setParameters(float attackMs, float releaseMs)
    {
        tauAtt = calcTimeConstant(attackMs, expFactor);
        tauRel = calcTimeConstant(releaseMs, expFactor);
    }

    void process(const float* inL, const float* inR, float* outLevel, int numSamples, int numChannels)
    {
        if (numChannels == 1)
        {
            for (int i = 0; i < numSamples; ++i)
                absBuf[i] = std::fabs(inL[i]);
        }
        else
        {
            for (int i = 0; i < numSamples; ++i)
            {
                float a0 = std::fabs(inL[i]);
                float a1 = std::fabs(inR[i]);
                absBuf[i] = (a0 + a1) * 0.5f; 
            }
        }

        for (int n = 0; n < numSamples; ++n)
            outLevel[n] = processSample(absBuf[n]);
    }

    inline float processSample(float x) noexcept
    {
        float tau = increasing ? tauAtt : tauRel;
        float out = yOld + tau * (x - yOld);
        increasing = out > yOld;
        yOld = out;
        return out;
    }

private:
    inline float calcTimeConstant(float timeMs, float expFactorLocal)
    {
        return timeMs < 1.0e-3f ? 0.0f : 1.0f - std::exp(expFactorLocal / timeMs);
    }

    float fs = 48000.0f;
    float expFactor = -1000.0f;
    float tauAtt = 1.0f, tauRel = 1.0f;
    float yOld = 0.0f;
    bool increasing = true;
    float absBuf[DEG_BLOCK_SIZE];
};

// -------------------------------
// DegradeNoise
// -------------------------------
class DegradeNoise
{
public:
    DegradeNoise() : rng(1) {}

    void prepare(uint64_t seed = 1)
    {
        rng.setSeed(seed);
        prevGain = curGain;
    }

    void setGain(float newGain) { curGain = newGain; }

    void processBlock(float* buffer, int numSamples)
    {
        if (curGain == prevGain)
        {
            for (int n = 0; n < numSamples; ++n)
                buffer[n] += (rng.nextFloat() - 0.5f) * curGain;
        }
        else
        {
            for (int n = 0; n < numSamples; ++n)
            {
                float alpha = (numSamples > 1) ? (float)n / (float)numSamples : 1.0f;
                float g = curGain * alpha + prevGain * (1.0f - alpha);
                buffer[n] += (rng.nextFloat() - 0.5f) * g;
            }
            prevGain = curGain;
        }
    }

    void seed(uint64_t s) { rng.setSeed(s); }

private:
    JuceRandom rng;
    float curGain = 0.0f;
    float prevGain = 0.0f;
};

// -------------------------------
// DegradeFilter
// -------------------------------
class DegradeFilter
{
public:
    DegradeFilter() : fs(48000.0f)
    {
        reset(fs);
    }

    void reset(float sampleRate)
    {
        fs = sampleRate;
        z[0] = z[1] = 0.0f;
        freqSm.setCurrentAndTargetValue(20000.0f);
        freqSm.setSteps(200); 
        calcCoefs(freqSm.getCurrentValue());
    }

    void setFreq(float newFreq)
    {
        if (newFreq <= 0.0f) newFreq = 20.0f;
        freqSm.setTargetValue(newFreq);
    }

    inline void calcCoefs(float fc)
    {
        float wc = 2.0f * M_PI * fc / fs;
        float tanv = std::tan(wc * 0.5f);
        float c = 1.0f / tanv;
        float a0 = c + 1.0f;

        b[0] = 1.0f / a0;
        b[1] = b[0];
        a[1] = (1.0f - c) / a0;
    }

    inline void process(float* buffer, int numSamples)
    {
        for (int n = 0; n < numSamples; ++n)
        {
            if (freqSm.isSmoothing())
                calcCoefs(freqSm.getNextValue());

            float x = buffer[n];
            float y = z[1] + x * b[0];
            z[1] = x * b[1] - y * a[1];
            buffer[n] = y;
        }
    }

private:
    float fs;
    MulSmoothed freqSm{20000.0f};
    float a[2]{1.0f, 0.0f}, b[2]{1.0f, 0.0f}, z[2]{0.0f, 0.0f};
};

// -------------------------------
// Main DegradeProcessor (Daisy port)
// -------------------------------
class DegradeProcessor
{
public:
    DegradeProcessor();
    ~DegradeProcessor() {}

    void prepare(float sampleRate);

    void setParameters(float depth, float amount, float variance, float envelope, bool enabled, bool usePoint1x = false);

    void processBlock(float* inL, float* inR, int blockSize);

private:
    void cookParams();
    void processShortBlock(float* chunkL, float* chunkR, int numSamples);

    float fs;
    bool onOff;
    bool usePoint1xFlag;

    // Parameters
    float p_depth, p_amount, p_variance, p_envelope;

    // Sub-Processors
    DegradeFilter filters[2];
    DegradeNoise noises[2];
    ChowLevelDetector levelDetector;

    // Internal Scratch Buffers
    float noiseBufL[DEG_BLOCK_SIZE];
    float noiseBufR[DEG_BLOCK_SIZE];
    float levelBuf[DEG_BLOCK_SIZE];

    JuceRandom paramRng;
    int sampleCounter;

    // --- CRITICAL FIX: Use Linear Smoother for Gain ---
    // This ensures gain changes ramp over a fixed duration (2048 samples)
    // regardless of the audio block size.
    LinSmoothed gainSmoother;
};

#endif // DAISY_DEGRADE_H