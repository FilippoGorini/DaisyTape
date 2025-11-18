% This script analyzes the digital 4th-order Linkwitz-Riley filter
% from your C++ implementation.

clear;
clc;
close all;

% --- System Parameters ---
fs = 48000; % Sample rate (Hz)
R2 = sqrt(2); % The Q-factor control (1.414...) from the C++ code

% --- Define Cutoff Frequencies to Test ---
% These are based on the ranges in your InputFilters class
low_cutoffs  = [20, 250, 2000]; % For the High-Pass filter
high_cutoffs = [2000, 10000, 20000]; % For the Low-Pass filter


% === 1. Plot the High-Pass Filter Responses ===
figure;
hold on;

for fc = low_cutoffs
    % 1. Calculate digital filter coefficients for one 2nd-order stage
    % This matches the 'update()' function in the C++ code
    g = tan(pi * fc / fs);
    
    % Numerator for a 2nd-order HPF (High-Pass Filter)
    % H(z) = (1 - z^-1)^2 = 1 - 2z^-1 + z^-2
    b_hp = [1, -2, 1];
    
    % Denominator based on the TPT SVF structure
    a0 = 1 + g*R2 + g^2;
    a1 = -2 + 2*g^2;
    a2 = 1 - g*R2 + g^2;
    a = [a0, a1, a2];
    
    % 2. Calculate 4th-order coefficients
    % A 4th-order LR filter is two 2nd-order filters cascaded
    b_hp_4th = conv(b_hp, b_hp);
    a_4th = conv(a, a);
    
    % 3. Get frequency response
    % freqz() returns H (complex response), w (normalized freq)
    [H, w] = freqz(b_hp_4th, a_4th, 8192, fs);
    
    % 4. Plot
    plot(w, 20*log10(abs(H)), 'LineWidth', 2);
end

title('4th-Order Linkwitz-Riley High-Pass Filter (HPF)');
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
legend(strcat(string(low_cutoffs), ' Hz'));
set(gca, 'XScale', 'log');
grid on;
xlim([10, fs/2]);
ylim([-80, 5]);


% === 2. Plot the Low-Pass Filter Responses ===
figure;
hold on;

for fc = high_cutoffs
    % 1. Calculate digital filter coefficients for one 2nd-order stage
    g = tan(pi * fc / fs);
    
    % Numerator for a 2nd-order LPF (Low-Pass Filter)
    % H(z) = g^2 * (1 + z^-1)^2 = g^2 * [1, 2, 1]
    b_lp = g^2 * [1, 2, 1];
    
    % Denominator (same as the HPF)
    a0 = 1 + g*R2 + g^2;
    a1 = -2 + 2*g^2;
    a2 = 1 - g*R2 + g^2;
    a = [a0, a1, a2];
    
    % 2. Calculate 4th-order coefficients
    b_lp_4th = conv(b_lp, b_lp);
    a_4th = conv(a, a);
    
    % 3. Get frequency response
    [H, w] = freqz(b_lp_4th, a_4th, 8192, fs);
    
    % 4. Plot
    plot(w, 20*log10(abs(H)), 'LineWidth', 2);
end

title('4th-Order Linkwitz-Riley Low-Pass Filter (LPF)');
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
legend(strcat(string(high_cutoffs), ' Hz'));
set(gca, 'XScale', 'log');
grid on;
xlim([10, fs/2]);
ylim([-80, 5]);