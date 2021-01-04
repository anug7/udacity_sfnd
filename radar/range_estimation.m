
c = 3 * 10^8;
max_r = 300;
delta_r = 1; % resolution

Bsweep = c / (2 * delta_r);

Ts = 5.5 * 2 * max_r / c;
    
beat_freq = [0.0 1.1 13.0 24.0] * 1e6;

calculated_range = c * Ts * beat_freq / (2 * Bsweep);

disp(calculated_range)