#pragma once
#ifndef DAISY_LINKWITZRILEYFILTER_H
#define DAISY_LINKWITZRILEYFILTER_H

#include <cmath>
#include <array>
#include <algorithm>
#include <cassert>

/** 4th-order Linkwitz-Riley Filter (2-channel version)
 * Ported from the JUCE implementation, adapted for Daisy Seed.
 */
template <typename SampleType>
class LinkwitzRileyFilter
{
public:
    /** Constructor. */
    LinkwitzRileyFilter()
        : numChannels_(0),
          g_(static_cast<SampleType>(0)),
          h_(static_cast<SampleType>(0)),
          sampleRate_(48000.0),
          cutoffFrequency_(static_cast<SampleType>(2000.0))
    {
        update();
        reset();        // ensure state starts zeroed
    }

    /** Sets the cutoff frequency of the filter in Hz. */
    void setCutoff(SampleType newCutoffFrequencyHz)
    {
        // assert(newCutoffFrequencyHz > static_cast<SampleType>(0));       // Probably not needed
        assert(newCutoffFrequencyHz < static_cast<SampleType>(0.5 * sampleRate_));

        cutoffFrequency_ = newCutoffFrequencyHz;
        update();
    }

    /** Initializes the filter. */
    void prepare(double newSampleRate, int newNumChannels)
    {
        assert(newSampleRate > 0);
        assert(newNumChannels > 0 && newNumChannels <= 2);

        numChannels_ = newNumChannels;
        sampleRate_  = newSampleRate;

        update();
        reset();
    }

    /** Resets the internal state variables of the filter. */
    void reset()
    {
        for (int ch = 0; ch < numChannels_; ++ch)
            std::fill(state_[ch].begin(), state_[ch].end(), static_cast<SampleType>(0));
    }

    /** Processes one sample, returning low-pass and high-pass outputs. */
    inline void processSample(size_t ch, SampleType x, SampleType& outputLow, SampleType& outputHigh) noexcept
    {
        // assumes ch < numChannels (validated by caller)
        auto yH = (x - (R2_ + g_) * state_[ch][0] - state_[ch][1]) * h_;

        auto tB = g_ * yH;
        auto yB = tB + state_[ch][0];
        state_[ch][0] = tB + yB;

        auto tL = g_ * yB;
        auto yL = tL + state_[ch][1];
        state_[ch][1] = tL + yL;

        auto yH2 = (yL - (R2_ + g_) * state_[ch][2] - state_[ch][3]) * h_;

        auto tB2 = g_ * yH2;
        auto yB2 = tB2 + state_[ch][2];
        state_[ch][2] = tB2 + yB2;

        auto tL2 = g_ * yB2;
        auto yL2 = tL2 + state_[ch][3];
        state_[ch][3] = tL2 + yL2;

        outputLow  = yL2;
        outputHigh = yL - R2_ * yB + yH - yL2;
    }

    /** Manually clears denormals (values near zero). */
    inline void snapToZero() noexcept
    {
        // Hardcoded threshold
        const SampleType zeroThreshold = static_cast<SampleType>(1.0e-9);       // TODO: test which value works best

        for (int ch = 0; ch < numChannels_; ++ch)
        {
            for (auto& element : state_[ch])
            {
                if (std::fabs(element) < zeroThreshold)
                    element = static_cast<SampleType>(0);
            }
        }
    }

private:
    void update()
    {
        g_ = static_cast<SampleType>(std::tan(M_PI * cutoffFrequency_ / sampleRate_));
        h_ = static_cast<SampleType>(1.0 / (1.0 + R2_ * g_ + g_ * g_));
    }

    int numChannels_;
    SampleType g_, h_;
    static constexpr SampleType R2_ = static_cast<SampleType>(1.41421356237);
    std::array<std::array<SampleType, 4>, 2> state_; // 2 channels max

    double sampleRate_;
    SampleType cutoffFrequency_;
};

#endif // DAISY_LINKWITZRILEYFILTER_H