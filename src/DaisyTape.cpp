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
TapeParams params;
CpuLoadMeter cpuLoadMeter;


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


void read_map_params() 
{   
    // Read ADCs 
    const float pot_lowcut = hw.adc.GetFloat(0);
    const float pot_highcut = hw.adc.GetFloat(1);
    // const float pot_wetdry = hw.adc.GetFloat(2);     // To be added later

    // Map params if needed
    params.lowCutFreq = 20.0f * powf(2000.0f / 20.0f, pot_lowcut);          // Map Low Cut (20Hz to 2kHz, logarithmic scale)
    params.highCutFreq = 2000.0f * powf(22000.0f / 2000.0f, pot_highcut);   // Map High Cut (2kHz to 22kHz, logarithmic scale)
}


void log_status()
{ 
    hw.PrintLine("--- Status ---");
    hw.PrintLine("Lowcut freq: " FLT_FMT3,
                 FLT_VAR3(params.lowCutFreq));
    hw.PrintLine("Highcut freq: " FLT_FMT3,
                 FLT_VAR3(params.highCutFreq));
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

    // Setup audio configuration
    hw.SetAudioBlockSize(48);
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    float sample_rate = hw.AudioSampleRate();

    // Setup ADCs for potentiometers
    AdcChannelConfig adcConfig[2];
    adcConfig[0].InitSingle(seed::A0);
    adcConfig[1].InitSingle(seed::A1);
    hw.adc.Init(adcConfig, 2);

    // Setup TapeProcessor
    tapeProcessor.setDelayLinePointers(&makeupDelayL, &makeupDelayR, &dryDelayL, &dryDelayR);
    params.filtersEnabled = true;
    params.makeupEnabled  = false;
    params.dryWet        = 1.0f;
    params.lowCutFreq    = 20.0f;   
    params.highCutFreq   = 22000.0f;
    tapeProcessor.Init(sample_rate, params);

    // Setup CPU Load Meter
    cpuLoadMeter.Init(sample_rate, hw.AudioBlockSize());

    // Start adc, log and audio
    hw.adc.Start();
    hw.StartLog();
    hw.StartAudio(AudioCallback);

    while(1)
    {
        // Read potentiometers and map them to parameters
        read_map_params();
        // Update parameters
        tapeProcessor.updateParams(params);
        // Optional log
        log_status();
        // Update at ~50Hz
        System::Delay(20);      // Probably better to switch to a timer with a wait for next period for more regular execution of main?
    }
}