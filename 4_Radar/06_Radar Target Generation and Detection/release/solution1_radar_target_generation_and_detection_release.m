clear all;
clc;
R = 110; % target's initial position [m]
v = -20; % target's initial velocity [m/s]
c = 3e8; % speed of light [m/s]
d_res = 1; % range resolution [m]
B = c / (2 * d_res);
R_max = 200; % maximum range [m]
Tchirp = 5.5 * 2 * (R_max) / c;
slope = B / Tchirp;
fc = 77e9; % carrier freq
Nr = 1024; % for length of time OR # of range cells
Nd = 128; % #of doppler cells OR #of sent periods % number of chirps
t = linspace(0, Nd * Tchirp, Nr * Nd); %total time for samples
Tx = zeros(1, length(t)); % transmitted signal
Rx = zeros(1, length(t)); % received signal
Mix = zeros(1, length(t)); % beat signal
r_t = zeros(1, length(t));
td = zeros(1, length(t));
for i = 1:length(t)
    r_t(i) = R + v * t(i);
    td(i) = (2 * r_t(i)) / c;
    Tx(i) = cos(2 * pi * (fc * t(i) + (slope * t(i)^2) / 2));
    Rx(i) = cos(2 * pi * (fc * (t(i) - td(i)) + (slope * (t(i) - td(i))^2) / 2));
end
Mix = Tx .* Rx;
Mix = reshape(Mix, [Nr, Nd]);
Y = fft(Mix, [], 1); % Y = fft(Mix,Nr);
P = Y ./ Nr;
P2 = abs(P);
P1 = P2(1:Nr / 2 + 1);
figure ('Name', 'Range from First FFT'); plot(P1)
axis ([0 200 0 0.5]);
Mix = reshape(Mix, [Nr, Nd]);
sig_fft2 = fft2(Mix, Nr, Nd);
sig_fft2 = sig_fft2(1:Nr / 2, 1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10 * log10(RDM);
doppler_axis = linspace(-100, 100, Nd);
range_axis = linspace(-200, 200, Nr / 2) * ((Nr / 2) / 400);
figure('Name', '2D FFT output - Range Doppler Map'), surf(doppler_axis, range_axis, RDM);
Tr = 7; % number of training cells for range
Td = 7; % number of training cells for doppler
Gr = 2; % number of guard cells for range
Gd = 2; % number of guard cells for doppler
offset = 5; % offset the threshold by SNR value in [dB]
range = 2 * (Tr + Gr) + 1;
doppler = 2 * (Td + Gd) + 1;
noise_level = ones(doppler, range);
for i = (1 + Td):(1 + Td + Gd * 2)
    for j = (1 + Tr):(1 + Tr + Gr * 2)
        noise_level(i, j) = 0;
    end
end
noise_level = noise_level / sum(noise_level, 'all');
threshold_CFAR = pow2db(conv2(db2pow(RDM), noise_level, 'same')) + offset;
RDM = double(RDM >= threshold_CFAR);
RDM(union(1:(Tr + Gr), end - (Tr + Gr - 1):end), :) = 0; % truncated map range
RDM(:, union(1:(Td + Gd), end - (Td + Gd - 1):end)) = 0; % truncated map doppler
figure('Name', 'Output of the 2D CFAR process'), surf(doppler_axis, range_axis, RDM);
colorbar;
