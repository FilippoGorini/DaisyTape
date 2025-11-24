#include "DaisyLossFilter.h"

LossFilter::LossFilter() : 
    fs(48000.0f), onOff(true), activeFilterIdx(0), 
    fadeCounter(0), triggerFade(false),
    p_speed(0), p_spacing(0), p_thickness(0), p_gap(0)
{
}

void LossFilter::prepare(float sampleRate)
{
    fs = sampleRate;
    
    for(int i=0; i<2; i++) {
        firFilters[i].reset();
        bumpFilters[i].reset();
    }
    
    setParameters(15.0f, 0.5f, 0.5f, 0.5f);
    
    // Force active filter to have these coeffs immediately
    firFilters[activeFilterIdx].setCoefficients(computedFir);
    calcHeadBumpCoeffs(15.0f, 0.5f * 1.0e-6f, bumpFilters[activeFilterIdx]);
    triggerFade = false; // No fade on startup
}

float LossFilter::getLatencySamples() const
{
    // FIR Latency is generally Order / 2
    return onOff ? (float)LOSS_FIR_ORDER / 2.0f : 0.0f;
}

// --- HEAVY MATH (Run in Main Thread) ---
void LossFilter::setParameters(float speed, float spacing, float thickness, float gap)
{
    // Check if anything changed significantly
    if (std::abs(speed - p_speed) < 0.01f && 
        std::abs(spacing - p_spacing) < 0.01f &&
        std::abs(thickness - p_thickness) < 0.01f &&
        std::abs(gap - p_gap) < 0.01f) 
    {
        return; // No change
    }

    // Update stored params
    p_speed = speed;
    p_spacing = spacing;
    p_thickness = thickness;
    p_gap = gap;

    // If we are already fading, we skip this update to prevent collision
    // (Or we could force a hard update, but skipping is safer for now)
    if (fadeCounter > 0 || triggerFade) return;

    // 1. Determine the "Back" filter (the one not currently playing)
    int backIdx = (activeFilterIdx == 0) ? 1 : 0;

    // 2. Calculate FIR Coeffs into the Back Filter
    calcFirCoeffs(backIdx, speed, spacing, thickness, gap);

    // 3. Calculate Head Bump Coeffs into the Back Filter
    calcHeadBumpCoeffs(speed, gap * 1.0e-6f, bumpFilters[backIdx]);

    // 4. Signal Audio Thread to crossfade to this new filter
    triggerFade = true;
}

void LossFilter::calcHeadBumpCoeffs(float speedIps, float gapMeters, StereoBiquad& filter)
{
    // Constants matching JUCE code
    double speed = (double)speedIps;
    double gap = (double)gapMeters;
    double sampleRate = (double)fs;

    double bumpFreq = speed * 0.0254 / (gap * 500.0);
    double gain = std::max(1.5 * (1000.0 - std::abs(bumpFreq - 100.0)) / 1000.0, 1.0);
    
    double phi = 2.0 * M_PI * bumpFreq / sampleRate;
    
    // HACK: Using simple Biquad Peak formulas
    // A = sqrt(gain)
    double A_val = std::sqrt(gain);
    double alpha_val = std::sin(phi) / (2.0 * 2.0); // Q=2.0
    double cos_phi = std::cos(phi);

    double b0 = 1.0 + alpha_val * A_val;
    double b1 = -2.0 * cos_phi;
    double b2 = 1.0 - alpha_val * A_val;
    double a0 = 1.0 + alpha_val / A_val;
    double a1 = -2.0 * cos_phi;
    double a2 = 1.0 - alpha_val / A_val;

    // Normalize by a0
    filter.setCoeffs((float)(b0/a0), (float)(b1/a0), (float)(b2/a0), (float)(a1/a0), (float)(a2/a0));
}

void LossFilter::calcFirCoeffs(int targetFilterIdx, float speed, float spacing, float thickness, float gap)
{
    float binWidth = fs / (float)LOSS_FIR_ORDER;
    
    // 1. Frequency Domain Calculation
    for (int k = 0; k < LOSS_FIR_ORDER / 2; k++)
    {
        float freq = (float)k * binWidth;
        float waveNumber = 2.0f * M_PI * std::max(freq, 20.0f) / (speed * 0.0254f);
        float thickTimesK = waveNumber * (thickness * 1.0e-6f);
        float kGapOverTwo = waveNumber * (gap * 1.0e-6f) / 2.0f;

        float val = std::exp(-waveNumber * (spacing * 1.0e-6f)); // Spacing
        
        if (std::abs(thickTimesK) > 1e-5f)
            val *= (1.0f - std::exp(-thickTimesK)) / thickTimesK; // Thickness
        
        if (std::abs(kGapOverTwo) > 1e-5f)
            val *= std::sin(kGapOverTwo) / kGapOverTwo; // Gap

        Hcoefs[k] = val;
        Hcoefs[LOSS_FIR_ORDER - k - 1] = val; // Symmetric (Real FFT)
    }

    // 2. Inverse DFT (Naive implementation, slow but fine for Main Thread)
    for (int n = 0; n < LOSS_FIR_ORDER / 2; n++)
    {
        float sum = 0.0f;
        for (int k = 0; k < LOSS_FIR_ORDER; k++)
        {
            // Real part only because H is symmetric
            float angle = 2.0f * M_PI * (float)k * (float)n / (float)LOSS_FIR_ORDER;
            sum += Hcoefs[k] * std::cos(angle);
        }
        float val = sum / (float)LOSS_FIR_ORDER;
        
        computedFir[LOSS_FIR_ORDER / 2 + n] = val;
        computedFir[LOSS_FIR_ORDER / 2 - n] = val; // Symmetric Impulse Response (Linear Phase)
    }

    // 3. Push to filter
    firFilters[targetFilterIdx].setCoefficients(computedFir);
}

// --- AUDIO THREAD ---
void LossFilter::processBlock(float* inL, float* inR, float* outL, float* outR, int32_t blockSize)
{
    if (!onOff) return; 
    
    // 1. Handle Trigger from Main Thread
    if (triggerFade && fadeCounter == 0) {
        triggerFade = false;
        fadeCounter = LOSS_FADE_LEN;
        
        // --- CRITICAL FIX for CLICKS/POPS ---
        // When starting to fade to the "Back" filter, it currently has stale history 
        // (old audio data from seconds ago). We must synchronize its state with the 
        // Active filter so the only difference is the coefficients.
        
        int backIdx = (activeFilterIdx == 0) ? 1 : 0;
        
        // 1. Copy FIR Delay Line history
        firFilters[backIdx].copyStateFrom(firFilters[activeFilterIdx]);
        
        // 2. Copy IIR (Biquad) Feedback history
        bumpFilters[backIdx].copyStateFrom(bumpFilters[activeFilterIdx]);
    }

    // 2. Processing Loop
    for (int i = 0; i < blockSize; i++)
    {
        float l = inL[i];
        float r = inR[i];
        
        float firL, firR;
        float finalL, finalR;

        // A. Process ACTIVE filter
        firFilters[activeFilterIdx].process(l, r, firL, firR);
        bumpFilters[activeFilterIdx].process(firL, firR, finalL, finalR);

        // B. Handle Crossfade
        if (fadeCounter > 0)
        {
            int backIdx = (activeFilterIdx == 0) ? 1 : 0;
            float backL, backR, backFinalL, backFinalR;

            // Process BACK filter
            firFilters[backIdx].process(l, r, backL, backR);
            bumpFilters[backIdx].process(backL, backR, backFinalL, backFinalR);

            // Mix
            float gOld = (float)fadeCounter / (float)LOSS_FADE_LEN;
            float gNew = 1.0f - gOld;

            finalL = finalL * gOld + backFinalL * gNew;
            finalR = finalR * gOld + backFinalR * gNew;

            fadeCounter--;
            
            // End of fade? Swap!
            if (fadeCounter <= 0) {
                activeFilterIdx = backIdx; // The "Back" (New) filter is now Active
            }
        }

        outL[i] = finalL;
        outR[i] = finalR;
    }
}