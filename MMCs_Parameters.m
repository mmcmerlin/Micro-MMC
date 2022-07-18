clear;
clc;

Vdc = 16;

Nsm = 4;
Csm = 3 * 6.8e-3;
Vsm = 4;
fpwm = 1e4;
Rsm = 1e3;
Trot = 1e-3;

Vout = 7;
w = 50*2*pi;
bias = 0;
Ts = 900e-6;

Iccref = 0;
ficcref = 3*w;

R = 20;
L = 5 * 1.2e-3;
R_L = R/L;
%% Double closed loop PI controller
Kp_U = 0.01; 

Ki_U = 0.5; 

Kr_U = 0;%0.00001; 

%%  

Kp_I = 0.1; 

Ki_I = 5; 

Kr_I = 0;%0.002; 

% %%  

Kp_cc = 0.1; 

Ki_cc = 10; 

Kr_cc = 0;%0.01; 

%  

Kp_E1 = 5; 

Ki_E1 = 10; 

%  

Kp_E2 = 5; 

Ki_E2 = 10; 