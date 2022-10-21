%% dataset
clear
% load("newdata/sweep_input_2hzt5hz_amp0.07.mat")
% load("newdata/sweep_input_2hzt5hz_amp0.06.mat")
load("newdata/sweep_input_1hzt6hz_amp0.035.mat")
% load("newdata/sweep_input_2hzt6hz_amp0.05.mat")
% load("newdata/sweep_input_2hzt10hz_amp0.04.mat")

Y=[alp,the];
U=voltage;
Ts=0.01;
%%
num = length(U);
t=0:Ts:Ts*(num-1);
% n_split = ceil(10001/3);
% num_train=10000;
num_train=ceil(num/10);
num_valid=num-num_train;

u_train = U(1:num_train);
y_train = Y(1:num_train,:);
t_train = t(1:num_train);
% 
% u_valid = U(num_train+1:end);
% y_valid = Y(num_train+1:end,:);
% t_valid = t(num_train+1:end);

% u_train = U(100:end);
% y_train = Y(100:end,:);
% t_train = t(100:end);

% u_train = U;
% y_train = Y;
% t_train = t;


figure(1)
subplot(2,1,1)
plot(t,the)
subplot(2,1,2)
plot(t,alp)

figure(2)
plot(t,voltage)



%% 
data = iddata(y_train, u_train, Ts, 'Name', 'Qube');
% data = iddata(Y, U, Ts, 'Name', 'Qube');
data.InputName = 'Voltage';
data.InputUnit = 'V';
data.OutputName = {'Theta Angular position', 'Alpha Angular position'};
data.OutputUnit = {'rad', 'rad'};
data.Tstart = 0;
data.TimeUnit = 's';


%initial guesses
load("jaco.mat")
A0=Ts*A_lin_v+eye(4);
B0=Ts*B_lin_v;
eig(A0)

theta = matrices2theta(A0,B0); 

%%
% T=Ts;
init_sys = idgrey('modelQube',theta,'d',0,Ts);


%%
opt = greyestOptions;
opt.InitialState = 'zero';  %x0=[0,0,0,0]
opt.Display = 'full';
opt.DisturbanceModel='none';
opt.EnforceStability= true ;
sys = greyest(data,init_sys,opt);

%%
opt = compareOptions('InitialCondition','zero');
compare(data,sys,Inf,opt)
sys_qube = ss(sys.A,sys.B,sys.C,sys.D,Ts);

%% get the upright model
A_id_con_v=(sys.A-eye(4))/Ts;
B_id_con_v=sys.B/Ts;

% convert to upright position
A_upright=A_id_con_v;
B_upright=B_id_con_v;
% A34=-A34;
% A42=-A42;
% A43=-A43;
% B4=-B4;

A_upright(4,1)=-A_upright(4,1);
% A_upright(3,1)=0;

A_upright(3,4)=-A_upright(3,4);
A_upright(4,2)=-A_upright(4,2);
A_upright(4,3)=-A_upright(4,3);
B_upright(4)=-B_upright(4);


eig(A_id_con_v)
eig(A_upright)


%upright model
A_upright_d = A_upright*Ts+eye(4);
B_upright_d = B_upright*Ts;
sys_qube_upright = ss(A_upright_d,B_upright_d,sys.C,sys.D,Ts);



%%
rank(obsv(sys_qube_upright.A,sys_qube_upright.C))
rank(ctrb(sys_qube_upright.A,sys_qube_upright.B))
Pol  = pole(sys_qube_upright)
% figure(1)
% step(sys_qube_upright)
% hold on;

%% Poles Placement
% % p = [0.9+0.01i,0.9-0.01i,0.7,0.6];
% % p = [0.9+0.2i,0.9-0.2i,0.7,0.6];
% damping_ratio=0.8;
% natural_frequency=4;
% p1=natural_frequency*(-damping_ratio+sqrt(damping_ratio^2-1));
% p2=natural_frequency*(-damping_ratio-sqrt(damping_ratio^2-1));
% [b,a] = zp2tf([],[p1,p2],1);
% [A,B,C,D]=tf2ss(b,a);
% Ad=A*Ts+eye(2);
% p_2order=eig(Ad);
% 
% p = [0.1,0.9,p_2order(1),p_2order(2)];
% K = place(sys_qube_upright.A,sys_qube_upright.B,p);
% K
% Acl = sys_qube_upright.A-sys_qube_upright.B*K;
% sys_qube_pp = ss(Acl,sys_qube_upright.B,sys_qube_upright.C,sys_qube_upright.D,Ts);
% Pcl = pole(sys_qube_pp)
% % figure(1)
% % step(sys_qube_pp);
% 
% k_dc_pp=dcgain(sys_qube_pp);
% k_r_pp=1/k_dc_pp

%% LQR
Q=[20,0,0,0;
    0,1,0,0;
    0,0,1,0;
    0,0,0,1];
R=1;
[K_lqr,S,P] = lqr(sys_qube_upright,Q,R);
K_lqr
Acl_lqr = sys_qube_upright.A-sys_qube_upright.B*K_lqr;
sys_qube_lqr = ss(Acl_lqr,sys_qube_upright.B,sys_qube_upright.C,sys_qube_upright.D,Ts);
Pcl = pole(sys_qube_lqr)
% figure(2)
% step(sys_qube_lqr)

k_dc_lqr=dcgain(sys_qube_lqr);
k_r_lqr=1/k_dc_lqr





%%
save('model_greyest','sys_qube','sys_qube_upright','sys_qube_pp','sys_qube_lqr','K','K_lqr')

%%
function theta = matrices2theta(Abar,Bbar) 
%%
% Function INPUT
% Abar      System matrix Abar (matrix of size n x n)
% Bbar      System matrix Bbar (matrix of size n x m)

% Function OUTPUT
% theta     Paramter vector (vector of size n*n+n*m+l*n+l*m+n*l+n)

th_size = 8+2;
theta = zeros(th_size,1);


theta(1)=Abar(3,1);
theta(2)=Abar(3,2);
theta(3)=Abar(3,3);
theta(4)=Abar(3,4);
theta(5)=Abar(4,1);
theta(6)=Abar(4,2);
theta(7)=Abar(4,3);
theta(8)=Abar(4,4);
theta(9)=Bbar(3,1);
theta(10)=Bbar(4,1);
% theta(9:12)=x0;

end