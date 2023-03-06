% TODO : Find the Bsweep of chirp for 1 m resolution
c=3*10^8;
resolution=1;
Bsweep=c/(2*resolution);
% TODO : Calculate the chirp time based on the Radar's Max Range
range_max = 300;
Ts = 5.5 * (range_max * 2 /c);

% TODO : define the frequency shifts
fb=[0, 1.1e6 , 13e6 , 24e6 ];

% Display the calculated range
calculated_range=c*Ts*fb/2/Bsweep;
disp(calculated_range);