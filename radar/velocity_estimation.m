c = 3*10^8;         %speed of light
frequency = 77e9;   %frequency in Hz


lambda = c / frequency;

doppler_shifts = [3 -4.5 11 -3] * 1e3;

Vr = lambda * doppler_shifts / 2;


disp(Vr);