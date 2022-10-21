s=tf('s'); % TF: Voltage to Rotary Arm, Theta(s)/Vm(s)
P_ARM = ((-Mp*lp^2-Jp)*km*s^2+Mp*g*Lr*lp*km)/((-Jp*Lr^2*Mp*Rm-Jr*Mp*Rm*lp^2-Jp*Jr*Rm)*s^4+(-Mp*km^2*lp^2-Jp*km^2)*s^3+(Lr^3*Mp^2*Rm*g*lp+Jr*Lr*Mp*Rm*g*lp)*s^2+g*Lr*Mp*km^2*lp*s);
% TF: Voltage to Pendulum, Alpha(s)/Vm(s)
P_PEN = -lp*km*Mp*Lr*s/((-Jp*Lr^2*Mp*Rm-Jr*Mp*Rm*lp^2-Jp*Jr*Rm)*s^3+(-Mp*km^2*lp^2-Jp*km^2)*s^2+(Lr^3*Mp^2*Rm*g*lp+Jr*Lr*Mp*Rm*g*lp)*s+g*Lr*Mp*km^2*lp);
