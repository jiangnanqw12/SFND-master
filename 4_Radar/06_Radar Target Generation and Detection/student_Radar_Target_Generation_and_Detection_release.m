
target_start_position = 110; %target range =100m
target_velocity = -20; %target doppler velocity = -20m/s
fc = 77e9; %carrier freq
max_range = 200;
range_resolution = 1; % range resolution can determine bandwith
max_velocity = 70; % max velocity can determine Ts_x-axis
c = 3e8;
wavelength = c / fc; %wavelength
bandwidth = c / (2 * range_resolution); %d_res = c/2*Bsweep
Tchirp = 5.5 * 2 * max_range / c;
slope = bandwidth / Tchirp;
Nd = 128; % #of doppler cells OR #of sent periods % number of chirps
Nr = 1024; %for length of time OR # of range cells
t = linspace(0, Nd * Tchirp, Nr * Nd); %total time for samples
Tx = zeros(1, length(t)); %transmitted signal
Rx = zeros(1, length(t)); %received signal
Mix = zeros(1, length(t)); %beat signal
r_t = zeros(1, length(t));
td = zeros(1, length(t));
target_position = target_start_position;
for i = 1:length(t)
    target_position = target_start_position + target_velocity * t(i);
    time_delay = (2 * target_position) / c; %compute time_delay.
    Tx(i) = cos(2 * pi * (fc * t(i) + slope * t(i) * t(i) / 2));
    Rx(i) = cos(2 * pi * (fc * (t(i) - time_delay) + slope * (t(i) - time_delay) * (t(i) - time_delay) / 2));
    Mix(i) = Tx(i) * Rx(i);
end
Mix = reshape(Mix, [Nr, Nd]);
Y = fft(Mix, Nr, 1);
P2 = abs(Y / Nr);
P1 = P2(1:Nr / 2 + 1);
P1T = P1;
figure ('Name', 'Range from First FFT')
subplot(3, 1, 1)
plot(P1);
axis ([0 200 0 1]);
title("first")
P1M = P2(1:Nr / 2 + 1, :);
subplot(3, 1, 2)
plot(P1M(:, 1));
axis ([0 200 0 1]);
title("first")
subplot(3, 1, 3)
plot(P1M(:, 2));
axis ([0 200 0 1]);
title("second")
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
Td = 8;
Gr = 4;
Gd = 4;
offset = 6;
noise_level = zeros(1, 1);
RESULT = zeros(Nr / 2, Nd);
for i = Tr + Gr + 1:Nr / 2 - (Gr + Tr)
    for j = Td + Gd + 1:Nd - (Gd + Td)
        noise_level = 0;
        train_cell = 0;
        for p = i - (Tr + Gr):i + Tr + Gr
            for q = j - (Td + Gd):j + Td + Gd
                if (abs(i - p) > Gr || abs(j - q) > Gd)
                    noise_level = noise_level + db2pow(RDM(p, q));
                    train_cell = train_cell + 1;
                end
            end
        end
        noise_average = noise_level / train_cell;
        noise_threshold = pow2db(noise_average);
        noise_threshold = noise_threshold + offset;
        CUT = RDM(i, j);
        if (CUT < noise_threshold)
            RESULT(i, j) = 0;
        else
            RESULT(i, j) = 1;
        end
    end
end
figure, surf(doppler_axis, range_axis, RESULT);
colorbar;
