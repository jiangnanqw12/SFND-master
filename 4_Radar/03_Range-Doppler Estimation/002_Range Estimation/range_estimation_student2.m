% TODO : Find the Bsweep of chirp for 1 m resolution
c = 3 * (10^8);
range_resolution = 1;
range_max = 300;
beat_freq = [0, 1.1, 13, 24] * (10^6);
B_sweep = c / (2 * range_resolution);
% TODO : Calculate the chirp time based on the Radar's Max Range
T_sweep = 2 * range_max / c;
% TODO : define the frequency shifts

% Display the calculated range
disp(calculated_range);
