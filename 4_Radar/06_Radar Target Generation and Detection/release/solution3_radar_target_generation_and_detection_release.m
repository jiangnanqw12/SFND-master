clear all
clc;
Velocity = -28; % m/s
InitialRange = 75; % m
c = 3e8; % m/s  (speed of light)
fprintf("Initial velocity = %d and range = %d\n", Velocity, InitialRange);
Bsweep = c / (2 * 1); % Bsweep in Hz
Tchirp = 5.5 * (2 * 200 / c); % Tchirp in seconds
slope = Bsweep / Tchirp;
fc = 77e9; % carrier freq (Hz)
Nd = 128; % #of doppler cells OR #of sent periods % number of chirps
Nr = 1024; %for length of time OR # of range cells
t = linspace(0, Nd * Tchirp, Nr * Nd); %total time for samples
Tx = zeros(1, length(t)); %transmitted signal
Rx = zeros(1, length(t)); %received signal
Mix = zeros(1, length(t)); %beat signal
r_t = zeros(1, length(t));
td = zeros(1, length(t));
for i = 1:length(t)
    r_t(i) = InitialRange + Velocity * t(i);
    td(i) = 2 * r_t(i) / c;
    Tx(i) = cos(2 * pi * (fc * t(i) + slope * (t(i)^2) / 2));
    delay = t(i) - td(i);
    Rx (i) = cos(2 * pi * (fc * delay + slope * (delay^2) / 2)); % + 2.0*randn();
    Mix(i) = Tx(i) * Rx(i);
end
Mix = reshape(Mix, [Nr, Nd]);
signal_fft = fft(Mix, Nr);
L = Tchirp * Bsweep;
signal_fft = abs(signal_fft / L);
signal_fft = signal_fft(1:L / 2 + 1);
[fBeat, fBeatInd] = max(signal_fft);
f = Bsweep * (0:(L / 2)) / L;
subplot(2, 1, 1)
plot(f, signal_fft)
title('Single-Sided Amplitude Spectrum')
xlabel('Frequency (Hz)')
fBeatTxt = ['Beat Frequency: ', num2str(fBeat), ' Hz'];
text(1.15 * f(fBeatInd), 0.75 * signal_fft(fBeatInd), fBeatTxt);
R = (c * Tchirp * f) / (2 * Bsweep);
subplot(2, 1, 2);
plot(R, signal_fft)
title('Range from Single-Sided Amplitude Spectrum')
xlabel('Range (m)')
axis ([0 200 0 1]);
RangeTxt = ['Estimated Range: ', num2str(R(fBeatInd)), ' m'];
text(.6 * R(fBeatInd), 1.25 * signal_fft(fBeatInd), RangeTxt);
Mix = reshape(Mix, [Nr, Nd]);
sig_fft2 = fft2(Mix, Nr, Nd);
sig_fft2 = sig_fft2(1:Nr / 2, 1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10 * log10(RDM);
doppler_axis = linspace(-100, 100, Nd);
range_axis = linspace(-200, 200, Nr / 2) * ((Nr / 2) / 400);
figure, surf(doppler_axis, range_axis, RDM);
Tr = 3;
Td = 12;
Gr = 2;
Gd = 3;
offset = 22;
noise_level = zeros(1, 1);
cellNum = (2 * Tr + 2 * Gr + 1) * (2 * Td + 2 * Gd + 1) - (2 * Gr + 1) * (2 * Gd + 1);
signalCFAR = zeros(Nr / 2, Nd);
for i = 1:(Nr / 2 - (2 * Gr + 2 * Tr))
    for j = 1:(Nd - (2 * Gd + 2 * Td))
        s1 = sum(db2pow(RDM(i:i + 2 * Tr + 2 * Gr, j:j + 2 * Td + 2 * Gd)), 'all');
        s2 = sum(db2pow(RDM(i + Tr:i + Tr + 2 * Gr, j + Td:j + Td + 2 * Gd)), 'all');
        noiseLevel = s1 - s2;
        threshold = noiseLevel / cellNum;
        threshold = db2pow(pow2db(threshold)) * offset;
        signal = db2pow(RDM(i + Tr + Gr, j + Td + Gd));
        if (signal <= threshold)
            signalCFAR(i + Tr + Gr, j + Td + Gd) = 0;
        else
            signalCFAR(i + Tr + Gr, j + Td + Gd) = 1;
        end
    end
end
figure('Name', '2d-CFAR');
surf(doppler_axis, range_axis, signalCFAR);
xlabel('Speed (m/s)')
ylabel('Range (m)')
zlabel('Detection')
colorbar;
