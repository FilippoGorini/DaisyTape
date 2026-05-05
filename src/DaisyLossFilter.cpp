#include "DaisyLossFilter.h"

// Ensure the order is even, otherwise the symmetry logic breaks
static_assert(LOSS_FIR_ORDER % 2 == 0, "LOSS_FIR_ORDER must be even!");

LossFilter::LossFilter()
    : fs(48000.0f), onOff(true),
      activeFilterIdx(0), fadeCounter(0), triggerFade(false),
      stageReady(false),
      p_speed(-1.0f), p_spacing(-1.0f), p_thickness(-1.0f), p_gap(-1.0f)
{
}

void LossFilter::prepare(float sampleRate)
{
    fs = sampleRate;
    activeFilterIdx = 0;
    fadeCounter     = 0;
    triggerFade     = false;
    stageReady      = false;

    for (int i = 0; i < 2; i++) {
        firFilters[i].reset();
        bumpFilters[i].reset();
    }

    // Initialize active filter directly — no staging needed during prepare
    p_speed = 15.0f; p_spacing = 0.5f; p_thickness = 0.5f; p_gap = 0.5f;
    calcFirCoeffs(p_speed, p_spacing, p_thickness, p_gap);
    firFilters[activeFilterIdx].setCoefficients(computedFir);
    calcHeadBumpCoeffs(p_speed, p_gap * 1.0e-6f, bumpFilters[activeFilterIdx]);
}

float LossFilter::getLatencySamples() const
{
    // FIR Latency is generally Order / 2
    return onOff ? (float)LOSS_FIR_ORDER / 2.0f : 0.0f;
}

// --- HEAVY MATH (Main thread) ---
void LossFilter::prepareParams(float speed, float spacing, float thickness, float gap)
{
    if (speed < 0.1f) speed = 0.1f;
    if (gap < 0.1f) gap = 0.1f;

    // Check if anything changed significantly
    if (std::abs(speed - p_speed) < 0.01f &&
        std::abs(spacing - p_spacing) < 0.01f &&
        std::abs(thickness - p_thickness) < 0.01f &&
        std::abs(gap - p_gap) < 0.01f)
    {
        return;     // If not, return
    }

    p_speed = speed; 
    p_spacing = spacing; 
    p_thickness = thickness; 
    p_gap = gap;

    // Don't overwrite staging if a previous stage hasn't been consumed yet,
    // or if a crossfade is already running.
    if (stageReady || fadeCounter > 0 || triggerFade) return;

    // Compute into staging buffers — safe, interrupt never reads these until stageReady is set
    calcFirCoeffs(speed, spacing, thickness, gap);
    calcHeadBumpCoeffs(speed, gap * 1.0e-6f, stagedBump);

    __DMB(); // ensure all stores are visible before the flag
    stageReady = true;
}

// --- INTERRUPT THREAD ---
void LossFilter::applyParams()
{
    if (!stageReady) return;
    stageReady = false;

    // Determine back buffer index here — safe since we're in interrupt and activeFilterIdx is stable
    int backIdx = 1 - activeFilterIdx;

    firFilters[backIdx].setCoefficients(computedFir);
    bumpFilters[backIdx].setCoeffs(stagedBump.b0, stagedBump.b1, stagedBump.b2,
                                   stagedBump.a1, stagedBump.a2);
    triggerFade = true;
}

void LossFilter::calcHeadBumpCoeffs(float speedIps, float gapMeters, StereoBiquad& filter)
{
    // 1. Constants & Physics
    double speed = (double)speedIps;
    double gap = (double)gapMeters;
    double sampleRate = (double)fs;

    double bumpFreq = speed * 0.0254 / (gap * 500.0);
    double gainLinear = std::max(1.5 * (1000.0 - std::abs(bumpFreq - 100.0)) / 1000.0, 1.0);
    
    // 2. Convert to dB for RBJ formulas
    double gainDB = 20.0 * std::log10(gainLinear);
    double Q = 2.0;

    // 3. Exact RBJ Peaking EQ Formulas
    double w0 = 2.0 * M_PI * bumpFreq / sampleRate;
    double cos_w0 = std::cos(w0);
    double sin_w0 = std::sin(w0);
    double alpha = sin_w0 / (2.0 * Q);
    double A = std::pow(10.0, gainDB / 40.0);

    double b0r = 1.0 + alpha * A;
    double b1r = -2.0 * cos_w0;
    double b2r = 1.0 - alpha * A;
    double a0r = 1.0 + alpha / A;
    double a1r = -2.0 * cos_w0;
    double a2r = 1.0 - alpha / A;

    // 4. Normalize by a0
    filter.setCoeffs((float)(b0r / a0r), 
                     (float)(b1r / a0r), 
                     (float)(b2r / a0r), 
                     (float)(a1r / a0r), 
                     (float)(a2r / a0r));
}

void LossFilter::calcFirCoeffs(float speed, float spacing, float thickness, float gap)
{
    // --- CRITICAL FIX: Zero-fill the array first ---
    // The symmetric loop below misses index 0 (and overwrites center twice).
    // Without this, computedFir[0] contains garbage memory.
    std::fill(computedFir, computedFir + LOSS_FIR_ORDER, 0.0f);

    float binWidth = fs / (float)LOSS_FIR_ORDER;

    // Frequency domain calculation
    for (int k = 0; k < LOSS_FIR_ORDER / 2; k++)
    {
        float freq = (float)k * binWidth;
        float waveNumber = 2.0f * M_PI * std::max(freq, 20.0f) / (speed * 0.0254f);
        float thickTimesK = waveNumber * (thickness * 1.0e-6f);
        float kGapOverTwo = waveNumber * (gap * 1.0e-6f) / 2.0f;

        float val = std::exp(-waveNumber * (spacing * 1.0e-6f));

        if (std::abs(thickTimesK) > 1e-5f)
            val *= (1.0f - std::exp(-thickTimesK)) / thickTimesK;

        if (std::abs(kGapOverTwo) > 1e-5f)
            val *= std::sin(kGapOverTwo) / kGapOverTwo;

        Hcoefs[k] = val;
        Hcoefs[LOSS_FIR_ORDER - k - 1] = val;
    }

    // Inverse DFT
    for (int n = 0; n < LOSS_FIR_ORDER / 2; n++)
    {
        float sum = 0.0f;
        for (int k = 0; k < LOSS_FIR_ORDER; k++)
        {
            float angle = 2.0f * M_PI * (float)k * (float)n / (float)LOSS_FIR_ORDER;
            sum += Hcoefs[k] * std::cos(angle);
        }
        float val = sum / (float)LOSS_FIR_ORDER;

        computedFir[LOSS_FIR_ORDER / 2 + n] = val;
        computedFir[LOSS_FIR_ORDER / 2 - n] = val;
    }
    // Result sits in computedFir[] — caller (prepareParams or prepare) decides what to do with it
}

// --- AUDIO THREAD ---
void LossFilter::processBlock(float* inL, float* inR, float* outL, float* outR, int32_t blockSize)
{
    if (!onOff) return; 
    
    if (triggerFade && fadeCounter == 0) {
        triggerFade = false;
        fadeCounter = LOSS_FADE_LEN;
        
        // Sync state to avoid clicks
        int backIdx = (activeFilterIdx == 0) ? 1 : 0;
        firFilters[backIdx].copyStateFrom(firFilters[activeFilterIdx]);
        bumpFilters[backIdx].copyStateFrom(bumpFilters[activeFilterIdx]);
    }

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

            float gOld = (float)fadeCounter / (float)LOSS_FADE_LEN;
            float gNew = 1.0f - gOld;

            finalL = finalL * gOld + backFinalL * gNew;
            finalR = finalR * gOld + backFinalR * gNew;

            fadeCounter--;
            
            if (fadeCounter <= 0) {
                activeFilterIdx = backIdx; 
            }
        }

        outL[i] = finalL;
        outR[i] = finalR;
    }
}