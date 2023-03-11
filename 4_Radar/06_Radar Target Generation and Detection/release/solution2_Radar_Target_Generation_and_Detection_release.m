close all;
clear;
clc;
target_pos_initial = 40;
target_vel_initial = 50;
c = 3e8;
Rmax = 200;
Rres = 1;
Tchirp = 5.5 * 2 * Rmax / c;
B = c / (2 * Rres);
slope = B / Tchirp;
fc = 77e9; %carrier freq
Nd = 128; % #of doppler cells OR #of sent periods % number of chirps
Nr = 1024; %for length of time OR # of range cells
t = linspace(0, Nd * Tchirp, Nr * Nd); %total time for samples
Tx = zeros(1, length(t)); %transmitted signal
Rx = zeros(1, length(t)); %received signal
Mix = zeros(1, length(t)); %beat signal
r_t = zeros(1, length(t));
td = zeros(1, length(t));
for i = 1:length(t)
    r_t(i) = target_pos_initial + target_vel_initial * t(i);
    td(i) = 2 * r_t(i) / c;
    Tx(i) = cos(2 * pi * (fc * t(i) + (slope * (t(i)^2) / 2)));
    Rx(i) = cos(2 * pi * (fc * (t(i) - td(i)) + (slope * ((t(i) - td(i))^2) / 2)));
    Mix(i) = Tx(i) * Rx(i);
end
Mix_new = reshape(Mix, [Nr, Nd]);
signal_fft = fft(Mix_new, [], 1) / Nr;
signal_fft = abs(signal_fft);
signal_fft2 = signal_fft(1:((Nr - 1) / 2));
figure ('Name', 'Range from First FFT')
plot(signal_fft2)
xlabel('Range (ft)');
ylabel('FFT Magnitude');
axis ([0 200 0 1]);
Mix = reshape(Mix, [Nr, Nd]);
sig_fft2 = fft2(Mix, Nr, Nd);
sig_fft2 = sig_fft2(1:Nr / 2, 1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10 * log10(RDM);
doppler_axis = linspace(-100, 100, Nd);
range_axis = linspace(-200, 200, Nr / 2) * ((Nr / 2) / 400);
figure, surf(doppler_axis, range_axis, RDM);
Tr = 10;
Td = 10;
Gr = 5;
Gd = 5;
offset = 8;
noise_level = zeros(1, 1);
RDM_copy = RDM;
for i = Tr + Gr + 1:Nr / 2 - (Gr + Tr)
    for j = Td + Gd + 1:Nd - (Gd + Td)
        noise_level = 0;
        for p = i - (Tr + Gr):i + Tr + Gr
            for q = j - (Td + Gd):j + Td + Gd
                if abs(i - p) > Gr || abs(j - q) > Gd
                    noise_level = noise_level + db2pow(RDM_copy(p, q));
                end
            end
        end
        threshold = pow2db(noise_level / (((2 * Tr + 2 * Gr + 1) * (2 * Td + 2 * Gd + 1)) - ((2 * Gr + 1) * (2 * Gd + 1))));
        threshold = threshold + offset;
        if RDM(i, j) < threshold
            RDM(i, j) = 0;
        else
            RDM(i, j) = 1;
        end
    end
end
RDM(1:Tr + Gr, :) = 0;
RDM(Nr / 2 - (Gr + Tr) + 1:end, :) = 0;
RDM(:, 1:Td + Gd) = 0;
RDM(:, Nd - (Gd + Td) + 1:end) = 0;
figure, surf(doppler_axis, range_axis, RDM);
colorbar;
