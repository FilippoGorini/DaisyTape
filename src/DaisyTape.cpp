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
CpuLoadMeter audioLoadMeter;
CpuLoadMeter mainLoadMeter;

// Potentiometer values
float pot_lowcut  = 0.0f;
float pot_highcut = 0.0f;
float pot_loss    = 0.0f;
float pot_speed   = 0.0f;


// Audio callback function
void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    audioLoadMeter.OnBlockStart();

    tapeProcessor.processBlock(
        in[0], in[1], out[0], out[1], size
    );

    audioLoadMeter.OnBlockEnd();
}


// Function to read and map potentiometer values to parameters
void read_map_params() 
{   
    // Read ADCs 
    const float pot_lowcut = hw.adc.GetFloat(0);
    const float pot_highcut = hw.adc.GetFloat(1);
    const float pot_loss    = hw.adc.GetFloat(2);
    const float pot_speed = hw.adc.GetFloat(3);

    // Input filters parameters mapping
    params.lowCutFreq = 20.0f * powf(2000.0f / 20.0f, pot_lowcut);          // Map Low Cut (20Hz to 2kHz, logarithmic scale)
    params.highCutFreq = 2000.0f * powf(22000.0f / 2000.0f, pot_highcut);   // Map High Cut (2kHz to 22kHz, logarithmic scale)
    // Loss filter parameters mapping
    params.gap = 1.0f + (pot_loss * 49.0f);                                 // Map Gap (1 to 50 microns)
    params.spacing = 0.1f + (pot_loss * 19.9f);                             // Map Spacing (0.1 to 20 microns)
    params.thickness = 0.1f + (pot_loss * 49.9f);                           // Map Thickness (0.1 to 50 microns)
    params.speed = 1.0f + (pot_speed * 49.0f); 
    params.loss = pot_loss;                                                 // Not needed, just for logging 
}


// Function to log current status
void log_status()
{ 
    hw.PrintLine("Lowcut freq: " FLT_FMT3, FLT_VAR3(params.lowCutFreq));
    hw.PrintLine("Highcut freq: " FLT_FMT3, FLT_VAR3(params.highCutFreq));
    hw.PrintLine("Loss Knob: " FLT_FMT3, FLT_VAR3(params.loss));
    hw.PrintLine("Speed (ips): " FLT_FMT3, FLT_VAR3(params.speed));
    hw.PrintLine("Avg CPU Load: " FLT_FMT3, FLT_VAR3(audioLoadMeter.GetAvgCpuLoad() * 100.0f));
    hw.PrintLine("Max CPU Load: " FLT_FMT3, FLT_VAR3(audioLoadMeter.GetMaxCpuLoad() * 100.0f));
    hw.PrintLine("Min CPU Load: " FLT_FMT3, FLT_VAR3(audioLoadMeter.GetMinCpuLoad() * 100.0f));
    hw.PrintLine("Avg Main Load: " FLT_FMT3, FLT_VAR3(mainLoadMeter.GetAvgCpuLoad() * 100.0f));
    hw.PrintLine("Max Main Load: " FLT_FMT3, FLT_VAR3(mainLoadMeter.GetMaxCpuLoad() * 100.0f));
    hw.PrintLine("Min Main Load: " FLT_FMT3, FLT_VAR3(mainLoadMeter.GetMinCpuLoad() * 100.0f));
    hw.PrintLine("------------");
}


int main(void)
{
    // Hardware initialization
    hw.Init();

    // Setup audio configuration
    hw.SetAudioBlockSize(4);
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    float sample_rate = hw.AudioSampleRate();

    // Setup ADCs for potentiometers
    AdcChannelConfig adcConfig[4];
    adcConfig[0].InitSingle(seed::A0);      // Lowcut
    adcConfig[1].InitSingle(seed::A1);      // Highcut
    adcConfig[2].InitSingle(seed::A2);      // Loss
    adcConfig[3].InitSingle(seed::A3);      // Tape speed
    hw.adc.Init(adcConfig, 4);

    // Setup TapeProcessor
    tapeProcessor.setDelayLinePointers(&makeupDelayL, &makeupDelayR, &dryDelayL, &dryDelayR);
    params.filtersEnabled = true;
    params.makeupEnabled  = false;
    params.dryWet        = 1.0f;
    params.lowCutFreq    = 20.0f;   
    params.highCutFreq   = 22000.0f;
    params.gap          = 1.0f;
    params.spacing      = 0.1f;
    params.thickness    = 0.1f;
    params.speed        = 15.0f;
    tapeProcessor.Init(sample_rate, params);

    // Setup CPU Load Meter
    audioLoadMeter.Init(sample_rate, hw.AudioBlockSize());
    mainLoadMeter.Init(100, 1);
    
    int log_counter = 0;

    // Start adc, log and audio
    hw.adc.Start();
    hw.StartLog();
    hw.StartAudio(AudioCallback);

    while(1)
    {   
        mainLoadMeter.OnBlockStart();

        // Read potentiometers and map them to parameters
        read_map_params();
        // Update parameters
        tapeProcessor.updateParams(params);
        // Optional log (50 times slower than the controls loop rate)
        if (log_counter++ > 50) {
            log_status();
            log_counter = 0;
        }

        mainLoadMeter.OnBlockEnd();

        // Update at ~100Hz
        System::Delay(10);      // Probably better to switch to a timer with a wait for next period for more regular execution of main?
    }
}