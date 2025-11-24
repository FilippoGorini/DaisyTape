% compare_lossfilter_coefs.m
% Compare JUCE-style FIR coeff computation vs Daisy variant
clear; clc; close all;

% Parameters (match your code)
ORDER = 64;         % base order in JUCE
fs_juce = 44100;    % target JUCE run
fs_daisy = 48000;   % Daisy run
speed = 30.0;       % example params (tape speed ips)
spacing = 10.0;     % um? (match units used in code)
thickness = 10.0;   % um
gap = 1.0;          % (the JUCE code multiplies gap by 1e-6 in calcHeadBumpFilter)

% Choose variant for Daisy:
% 'fixed' -> keep N = ORDER
% 'scale' -> curOrder = round(ORDER * fs_daisy / 44100)
daisyVariant = 'scale'; % 'fixed' or 'scale'

% --- Helper function: compute Hcoefs and time-domain symmetric h (same algorithm as JUCE) ---
function h = compute_fir_coefs(fs, order, speed, spacing, thickness, gap)
    curOrder = round(order * (fs / 44100.0)); % same scaling formula JUCE uses
    if curOrder < 4
        curOrder = 4;
    end
    if mod(curOrder,2) ~= 0
        curOrder = curOrder + 1; % keep even
    end

    binWidth = fs / curOrder;
    Hcoefs = zeros(curOrder,1);

    for k = 0:(curOrder/2 - 1)
        freq = max( (k)*binWidth, 20 ); % same as JUCE (freq >= 20)
        waveNumber = 2*pi * freq / (speed * 0.0254);
        thickTimesK = waveNumber * (thickness * 1.0e-6);
        kGapOverTwo = waveNumber * (gap * 1.0e-6) / 2.0;

        Hk = exp( -waveNumber * (spacing * 1.0e-6) );                 % spacing
        if abs(thickTimesK) > 0
            Hk = Hk * (1.0 - exp(-thickTimesK)) / thickTimesK;       % thickness
        end
        if abs(kGapOverTwo) > 0
            Hk = Hk * sin(kGapOverTwo) / kGapOverTwo;                % gap
        end

        Hcoefs(k+1) = Hk; % MATLAB 1-based
        Hcoefs(curOrder - k) = Hk; % symmetric
    end

    % Time domain impulse (inverse DFT via cosine sums as in JUCE)
    h = zeros(curOrder,1);
    half = curOrder/2;
    for n = 0:(half-1)
        idx = half + n;            % corresponds to JUCE: curOrder/2 + n (0-based)
        % build sum H[k] * cos(2*pi * k * n / curOrder)
        s = 0;
        for k = 0:(curOrder-1)
            s = s + Hcoefs(k+1) * cos(2*pi * k * n / curOrder);
        end
        val = s / curOrder;
        h(idx+1) = val;            % matlab index
        h(half - n + 1) = val;     % mirror
    end
end

% Compute JUCE FIR (design at JUCE fs scaling: since JUCE chooses curOrder = order*(fs/44100),
% evaluating at fs = actual plugin sample rate will give same binWidth. The reference is
% the running plugin — for comparison run JUCE at fs_juce.)
h_juce = compute_fir_coefs(fs_juce, ORDER, speed, spacing, thickness, gap);

% Daisy variants
if strcmp(daisyVariant,'fixed')
    % For 'fixed', we simply set curOrder = ORDER, but still compute Hcoefs using the same freq grid as JUCE:
    % To match JUCE's bin frequencies we must use binWidth = 44100/ORDER
    fs_for_bins = 44100;  % sample-rate used to define bin frequencies (to match JUCE)
    % compute with fs_for_bins but still produce N = ORDER
    % hack: temporarily compute using compute_fir_coefs with fs_for_bins, but that function scales curOrder internally
    % so we write a tiny inline version here to force N=ORDER
    curOrder = ORDER;
    binWidth = 44100 / ORDER;
    Hcoefs = zeros(curOrder,1);
    for k = 0:(curOrder/2 - 1)
        freq = max( (k) * binWidth, 20 );
        waveNumber = 2*pi * freq / (speed * 0.0254);
        thickTimesK = waveNumber * (thickness * 1.0e-6);
        kGapOverTwo = waveNumber * (gap * 1.0e-6) / 2.0;

        Hk = exp( -waveNumber * (spacing * 1.0e-6) );
        if abs(thickTimesK) > 0
            Hk = Hk * (1.0 - exp(-thickTimesK)) / thickTimesK;
        end
        if abs(kGapOverTwo) > 0
            Hk = Hk * sin(kGapOverTwo) / kGapOverTwo;
        end
        Hcoefs(k+1) = Hk;
        Hcoefs(curOrder - k) = Hk;
    end
    % inverse DFT
    h_daisy = zeros(curOrder,1);
    half = curOrder/2;
    for n = 0:(half-1)
        s = 0;
        for k = 0:(curOrder-1)
            s = s + Hcoefs(k+1)*cos(2*pi*k*n/curOrder);
        end
        val = s / curOrder;
        h_daisy(half + n + 1) = val;
        h_daisy(half - n + 1) = val;
    end

elseif strcmp(daisyVariant,'scale')
    % scaled order like JUCE: curOrder = round(ORDER * fs_daisy / 44100)
    h_daisy = compute_fir_coefs(fs_daisy, ORDER, speed, spacing, thickness, gap);
else
    error('Unknown daisyVariant');
end

% --- Compare results ---
fprintf('JUCE curOrder: %d, Daisy curOrder: %d\n', length(h_juce), length(h_daisy));

% If different lengths, zero-pad the shorter to the longer for FFT comparison
N = max(length(h_juce), length(h_daisy));
hj = [h_juce; zeros(N - length(h_juce), 1)];
hd = [h_daisy; zeros(N - length(h_daisy), 1)];

% Impulse responses already are h themselves (the filter impulse)
% Frequency responses
nfft = 65536;
HJ = fft(hj, nfft); HJ = HJ(1:nfft/2+1);
HD = fft(hd, nfft); HD = HD(1:nfft/2+1);
f = (0:(nfft/2))' * (fs_daisy / nfft);

figure;
semilogx(f, 20*log10(abs(HJ)), 'b', 'LineWidth', 1.2); hold on;
semilogx(f, 20*log10(abs(HD)), 'r--', 'LineWidth', 1.2);
grid on; xlabel('Frequency (Hz)'); ylabel('Magnitude (dB)');
legend('JUCE (design)', 'Daisy (computed)');
title('JUCE vs Daisy FIR frequency responses');

% Impulse difference
% Interpolate if lengths differ (or compute cross-correlation)
figure;
plot(0:(N-1), hj(1:N), 'b', 0:(N-1), hd(1:N), 'r--'); legend('JUCE h', 'Daisy h');
title('Time-domain FIR impulse (aligned by index)');

% compute max absolute error (if lengths differ we compare aligned first minLen)
minLen = min(length(h_juce), length(h_daisy));
maxAbsErr = max(abs(h_juce(1:minLen) - h_daisy(1:minLen)));
fprintf('Max abs coefficient difference (first %d samples): %.6e\n', minLen, maxAbsErr);

% Cross-correlation to measure sample offset
[c,lags] = xcorr(hj, hd);
[~, idxMax] = max(abs(c));
lagAtMax = lags(idxMax);
fprintf('Cross-correlation peak lag (positive means Daisy delayed by): %d samples\n', lagAtMax);
