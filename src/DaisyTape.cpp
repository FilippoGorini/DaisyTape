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
    const float pot_lowcut = hw.adc.GetFloat(7);
    const float pot_highcut = hw.adc.GetFloat(6);
    const float pot_tape_loss    = hw.adc.GetFloat(5);
    const float pot_tape_speed = hw.adc.GetFloat(4);
    const float pot_deg_depth   = hw.adc.GetFloat(3); 
    const float pot_deg_amount  = hw.adc.GetFloat(2); 
    const float pot_deg_var     = hw.adc.GetFloat(1); 
    const float pot_deg_env     = hw.adc.GetFloat(0); 

    // Input filters parameters mapping
    params.lowCutFreq = 20.0f * powf(2000.0f / 20.0f, pot_lowcut);          // Map Low Cut (20Hz to 2kHz, logarithmic scale)
    params.highCutFreq = 2000.0f * powf(22000.0f / 2000.0f, pot_highcut);   // Map High Cut (2kHz to 22kHz, logarithmic scale)
    // Loss filter parameters mapping
    params.gap = 1.0f + (pot_tape_loss * 49.0f);                                 // Map Gap (1 to 50 microns)
    params.spacing = 0.1f + (pot_tape_loss * 19.9f);                             // Map Spacing (0.1 to 20 microns)
    params.thickness = 0.1f + (pot_tape_loss * 49.9f);                           // Map Thickness (0.1 to 50 microns)
    params.speed = 1.0f + (pot_tape_speed * 49.0f); 
    params.loss = pot_tape_loss;                                                 // Not needed, just for logging 
    // Degrade processor parameters mapping
    params.deg_depth    = pot_deg_depth;
    params.deg_amount   = pot_deg_amount;
    params.deg_variance = pot_deg_var;
    params.deg_envelope = pot_deg_env;
}


// Function to log current status
void log_status()
{ 
    hw.PrintLine("Deg depth: " FLT_FMT3, FLT_VAR3(params.deg_depth));
    hw.PrintLine("Deg amount: " FLT_FMT3, FLT_VAR3(params.deg_amount));
    hw.PrintLine("Deg variance: " FLT_FMT3, FLT_VAR3(params.deg_variance));
    hw.PrintLine("Deg envelope: " FLT_FMT3, FLT_VAR3(params.deg_envelope));
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
    AdcChannelConfig adcConfig[8];
    adcConfig[0].InitSingle(seed::A0);      // Lowcut
    adcConfig[1].InitSingle(seed::A1);      // Highcut
    adcConfig[2].InitSingle(seed::A2);      // Loss
    adcConfig[3].InitSingle(seed::A3);      // Tape speed
    adcConfig[4].InitSingle(seed::A4);      // Deg. depth
    adcConfig[5].InitSingle(seed::A5);      // Deg. amount
    adcConfig[6].InitSingle(seed::A6);      // Deg. variance
    adcConfig[7].InitSingle(seed::A7);      // Deg. envelope
    hw.adc.Init(adcConfig, 8);

    // Setup TapeProcessor
    tapeProcessor.setDelayLinePointers(&makeupDelayL, &makeupDelayR, &dryDelayL, &dryDelayR);
    params.filtersEnabled = true;
    params.makeupEnabled  = false;
    params.deg_enabled  = true;
    params.usePoint1x = true;
    params.dryWet        = 1.0f;
    params.lowCutFreq    = 20.0f;   
    params.highCutFreq   = 22000.0f;
    params.gap          = 1.0f;
    params.spacing      = 0.1f;
    params.thickness    = 0.1f;
    params.speed        = 15.0f;
    params.deg_depth    = 0.0f;
    params.deg_amount   = 0.0f;
    params.deg_variance = 0.0f;
    params.deg_envelope = 0.0f;
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