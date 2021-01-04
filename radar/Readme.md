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
![Range estimation](https://github.com/anug7/udacity_sfnd/blob/dev/radar/images/range.png)

Step 6:
2nd FFT gives Range Vs doppler shit (veocity). Which is also called as RDM - Range Doppler Map

![RDM](https://github.com/anug7/udacity_sfnd/blob/dev/radar/images/range_vel.png)

Step 7: Clutter Removal
As of Step 6, we could see more noise associated with the signal. This signal cannot be used for used for object detection, we need to remove noises called clutter. One way to remove that is using dynamic thresholding technique called Constan False Alarm Rate.

Parameters to 2D CFAR:
* No of Training cells are selected based on amount of noise spreaing and thresholding. I have selected this based on empirical results
* No of Guard cells are seleced based on CUT value leaking. I have selected the optimal value based on experiments.
```
%Select the number of Training Cells in both the dimensions.
Tr = 10;
Td = 8;

% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr = 4;
Gd = 4;
```

The implementation of the algorithm is as follows,
* Iterate over the entire RDM and compute the 2D grid for each of the CUT
* Sum up all the values of 2D grid present inside the Grid and ignore the Guard cells
* Average the summed up value with total no. of cells *((2*(Td+Gd+1)*2*(Tr+Gr+1)-(Gr*Gd)-1))*
* Use the computed threshold-ed value from above and use to threshold the CUT value
```
for i = Tr+Gr+1:(Nr/2)-(Tr+Gr)
    for j = Td+Gd+1:(Nd)-(Td+Gd)
        noise_level = 0;
        %Iterate over grid around CUT
        for p = i-(Tr+Gr) : i+(Tr+Gr)
            for q = j-(Td+Gd) : j+(Td+Gd)
                %ignore Guard cells
                if (abs(i-p) > Gr || abs(j-q) > Gd)
                    noise_level = noise_level + db2pow(RDM(p,q));
                end
            end
        end
        total_cells = (2*(Td+Gd+1)*2*(Tr+Gr+1)-(Gr*Gd)-1);
        %compute threshold average and convert to power from db
        threshold = pow2db(noise_level/total_cells);
        threshold = threshold + offset;
        %actual signal value of CUT
        cell_value = RDM(i,j);  
        if (cell_value < threshold)
            RDM(i,j) = 0;
        else
            RDM(i,j) = 1;
        end
        
    end
end

```
Step 8:
As the edge cells don't have complete 2D grid cells, they are set to zero. It's implemented as follows,

```
RDM(RDM~=0 & RDM~=1) = 0;
```

Output of CFAR algorithm

![CFAR](https://github.com/anug7/udacity_sfnd/blob/dev/radar/images/cfar.png)
