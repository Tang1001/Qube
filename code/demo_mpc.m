% plant = tf(1,[1 0 0]); % 被控对象是一个双积分系统，定义其传递函数
%% 设计MPC控制器
Ts = 0.01;    %步长
p = 100;      %预测时域
m = 3;       %控制时域
mpcobj = mpc(sys_qube_upright, Ts, p, m); %创建mpc控制器
mpcobj.MV = struct('Min',-1,'Max',1);  %限制mpc控制器的输出