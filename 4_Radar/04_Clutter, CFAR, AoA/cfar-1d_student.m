% Implement 1D CFAR using lagging cells on the given noise and target scenario.

% Close and delete all currently open figures
close all;

% Data_points
%First, we pick the number of samples on which we want to run the CFAR.
Ns = 1000;

% Generate random noise
%Then, we generate the noise using the same number of samples and take
%the absolute value of it.
s = randn(Ns, 1);

%Targets location. Assigning bin 100, 200, 300 and 700 as mock Targets with the amplitudes of 8, 9, 4, 11.
s([100, 200, 300, 700]) = [8 9 4 11];

%plot the output
plot(s);

% TODO: Apply CFAR to detect the targets by filtering the noise.

% 1. Define the following:
% 1a. Training Cells
T = 12
% 1b. Guard Cells
G = 4

% Offset : Adding room above noise threshold for desired SNR
% since you are working with the linear values,you multiply the offset to the threshold
offset = 3;

% Vector to hold threshold values
threshold_cfar = [];

%Vector to hold final signal after thresholding
signal_cfar = [];

% 2. Slide window across the signal length
% So we run the first training cells starting from the first pin,
% and we keep stepping it through till the end of the vector.
% But make sure that you give the room for last set of guard and training cells,
% along with the cell in the test.
for i = 1:(Ns - (G + T))

    % 2. - 5. Determine the noise threshold by measuring it within the training cells
    % compute noise of training cell
    noise_level = sum(s(i:i + T - 1));
    threshold = (noise_level / T) * offset;
    threshold_cfar = [threshold_cfar, {threshold}];
    % 6. Measuring the signal within the CUT
    signal = s[i + T + G];
    % 8. Filter the signal above the threshold
    if (signal < threshold)
        signal = 0;
        end;
        signal_cfar = [signal_cfar, {signal}];
    end

    % plot the filtered signal
    plot (cell2mat(signal_cfar), 'g--');

    % plot original sig, threshold and filtered signal within the same figure.
    figure, plot(s);
    hold on, plot(cell2mat(circshift(threshold_cfar, G)), 'r--', 'LineWidth', 2)
    hold on, plot (cell2mat(circshift(signal_cfar, (T + G))), 'g--', 'LineWidth', 4);
    legend('Signal', 'CFAR Threshold', 'detection')
