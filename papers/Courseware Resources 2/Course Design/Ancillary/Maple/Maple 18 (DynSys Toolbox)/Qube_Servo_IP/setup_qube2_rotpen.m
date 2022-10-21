%% Load Model
qube2_rotpen_param;
% Define center of mass
lp = Lp/2;
% Set open-loop state-space model of rotary single-inverted pendulum (SIP)
% QUBE_ROTPEN_ABCD_eqns;
% Set open-loop state-space model of rotary single-inverted pendulum (SIP)
qube2_rotpen_tf;
% Display matrices
P_ARM
P_PEN
% Save into numerator and denominator for Simulink diagram
[arm_num,arm_den]=tfdata(P_ARM,'v');
[pen_num,pen_den]=tfdata(P_PEN,'v');
% 
%% PID Control Design
% Simulate closed-loop inverted pendulum response by itself given an
% impulse disturbance of 1 V
Cp = pid(30,0,3);
Gp = feedback(P_PEN,Cp);
figure(1);
impulse(Gp);
