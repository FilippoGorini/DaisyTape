#include "daisy_seed.h"
#include "daisysp.h"
#include "DaisyInputFilters.h"
#include "TapeProcessor.h"
#include <cmath>

using namespace daisy;
using namespace daisysp;

// Allocate makeup buffers in SDRAM
MakeupDelayLine DSY_SDRAM_BSS makeupDelayL;
MakeupDelayLine DSY_SDRAM_BSS makeupDelayR;

// Allocate dry buffers in SDRAM
DryDelayLine DSY_SDRAM_BSS dryDelayL;
DryDelayLine DSY_SDRAM_BSS dryDelayR;

// Declare global objects
DaisySeed hw;
TapeProcessor tapeProcessor;
CpuLoadMeter cpuLoadMeter;

// Struct to hold parameters
struct ControlData
{
    float lowCutFreq;
    float highCutFreq;
};
ControlData current_control_data;

// Variables for potentiometer readings
float pot_l = 0.0f;
float pot_h = 0.0f;

// Function declarations
void read_pots();
void update_params();


// Audio callback function
void AudioCallback(AudioHandle::InputBuffer in,
                   AudioHandle::OutputBuffer out,
                   size_t size)
{
    cpuLoadMeter.OnBlockStart();

    // Process audio block
    tapeProcessor.processBlock(
        in[0],
        in[1],
        out[0],
        out[1],
        size
    );

    cpuLoadMeter.OnBlockEnd();
}

void read_pots()
{
    pot_l = hw.adc.GetFloat(0);
    pot_h = hw.adc.GetFloat(1);
}

void update_params()
{
    // Map pot values exponentially for frequency control
    current_control_data.lowCutFreq =
        20.0f * powf(2000.0f / 20.0f, pot_l);
    current_control_data.highCutFreq =
        2000.0f * powf(22000.0f / 2000.0f, pot_h);

    // Update TapeProcessor parameters
    tapeProcessor.SetLowCutFreq(current_control_data.lowCutFreq);
    tapeProcessor.SetHighCutFreq(current_control_data.highCutFreq);
    tapeProcessor.SetDryWet(1.0f);
    // As these next two params are just on/off buttons we can probably take them out, no need to call them every time
    tapeProcessor.SetFiltersEnabled(true);
    tapeProcessor.SetMakeupEnabled(false);

    // Logging for debug
    hw.PrintLine("--- Status ---");
    hw.PrintLine("Lowcut freq: " FLT_FMT3,
                 FLT_VAR3(current_control_data.lowCutFreq));
    hw.PrintLine("Highcut freq: " FLT_FMT3,
                 FLT_VAR3(current_control_data.highCutFreq));
    hw.PrintLine("Avg CPU Load: " FLT_FMT3,
                 FLT_VAR3(cpuLoadMeter.GetAvgCpuLoad() * 100.0f));
    hw.PrintLine("Max CPU Load: " FLT_FMT3,
                 FLT_VAR3(cpuLoadMeter.GetMaxCpuLoad() * 100.0f));
    hw.PrintLine("Min CPU Load: " FLT_FMT3,
                 FLT_VAR3(cpuLoadMeter.GetMinCpuLoad() * 100.0f));
    hw.PrintLine("--------------");
}

int main(void)
{
    // Hardware initialization
    hw.Init();
    hw.SetAudioBlockSize(48);
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    float sample_rate = hw.AudioSampleRate();

    // Setup ADCs for pots
    AdcChannelConfig adcConfig[2];
    adcConfig[0].InitSingle(seed::A0);
    adcConfig[1].InitSingle(seed::A1);
    hw.adc.Init(adcConfig, 2);
    hw.adc.Start();

    // CPU Load Meter initialization for logging
    cpuLoadMeter.Init(sample_rate, hw.AudioBlockSize());

    // TapeProcessor initialization
    tapeProcessor.setDelayLinePointers(&makeupDelayL, &makeupDelayR, &dryDelayL, &dryDelayR);
    tapeProcessor.Init(sample_rate);

    // Start audio
    hw.StartLog();
    hw.StartAudio(AudioCallback);

    while(1)
    {
        // Read potentiometers
        read_pots();
        // Update parameters based on pot readings
        update_params();
        // Update at ~25Hz
        System::Delay(40);
    }
}