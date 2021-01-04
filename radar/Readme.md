Goal of the assigment
1. To simulate the FMCW signal for a target detection, range estimation and velocity estimation
2. Implement 2D FFT and then 2D CFAR for clutter removal using dynamic thresholding

**Steps**

 Step 1: Radar Specs
 ```
max_r = 200; % Max range that can be detected by Radar in m
delta_r = 1; % resolution of range measurement in m
max_v = 100; % maximum velocity in m/2
c = 3e8;     % velocity of light
% carrier signal frequency
fc= 77e9;
```

Step 2: FMCW specs
```
Bsweep =  c / (2 * delta_r); % Sweep frequency calculated based on range resolution
Tchrip = 5.5 * 2 * max_r / c; % Chrip time based on max range
slope = Bsweep / Tchrip;      % rate of frequency change
```

Step 3: Initial Target configuration
```
initial_pos = 23;
initial_vel= -54;
```

Step 4: Transmitter and Receiver signal simulation
```
for i=1:length(t)         
    % *%TODO* :
    %For each time stamp update the Range of the Target for constant velocity. 
    r_t(i) = initial_pos + (initial_vel * t(i));
    td(i) = 2 * r_t(i) / c;  % calculate delay in receiver signal based on current range of the object
    
    % *%TODO* :
    %For each time sample we need update the transmitted and
    %received signal.t
    tr = t(i);
    td = t(i) - td(i);
    % Use timestep to generate signal at t(i)
    Tx(i) = cos(2 * pi * (fc * tr + (slope * tr^2) / 2)); % compute transmitter signal based on fc and slope
    Rx(i) = cos(2 * pi * (fc * td + (slope * td^2) / 2)); % compute receiver signal based on fc, slope and delay due to object position
    
    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i).* Rx(i);
    
end
```

Step 5:

Compute 1d fft gives the range of the object.In our case, it's at 23m
