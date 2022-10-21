%% dataset
clear 
% load("newdata/sweep_input_2hzt5hz_amp0.07.mat")
% load("newdata/sweep_input_2hzt5hz_amp0.06.mat")
load("newdata/sweep_input_1hzt6hz_amp0.035.mat")
% load("newdata/sweep_input_2hzt6hz_amp0.05.mat")
% load("newdata/sweep_input_2hzt10hz_amp0.04.mat")

Y=[alp,the];
U=voltage;
%%
Ts=0.01;
num = length(U);
t=0:Ts:Ts*(num-1);
% n_split = ceil(10001/3);
% num_train=10000;
num_train=ceil(num/6);
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
load("jaco.mat")
A0=Ts*A_lin_v+eye(4)
B0=Ts*B_lin_v 
% eig(A_lin)
eig(A_lin_v)
eig(A0)

x00=[0;0;0;0];
C0=[1,0,0,0; 0,1,0,0];
D0=[0;0];



%% PEM
maxiter=500;
[Abar,Bbar,C,D,x0] = oem(A0,B0,C0,D0,x00,y_train,u_train,maxiter);
eig(Abar)

% init_sys=ss(A0,B0,C0,D0,Ts);
% sys = pem("data/sweep_input_1.5hz4hz_amp0.02.mat",init_sys)

%%
% A=[0,0,1,0;0,0,0,1;0,100.3,-0.2359,0.9629;0,-203.9,0.2014,-1.957]
% B=[0;0;1049;-895.1]
% Abar=Ts*A+eye(4);
% Bbar=Ts*B;


%%

sys=ss(Abar,Bbar,C,D,Ts);
y_pem=lsim(sys,u_train,t_train,x0);
VAF_pem = max(0,(1-norm(y_train-y_pem)^2/norm(y_train)^2))
RMSE_pem = sqrt(norm(y_train-y_pem)^2/num_train)


figure(3)
subplot(2,1,1)
plot(t_train,y_train(:,1),t_train,y_pem(:,1))
subplot(2,1,2)
plot(t_train,y_train(:,2),t_train,y_pem(:,2))

%%
% ytest_pem=lsim(sys,u_valid,t_valid,x0);
% 
% VAFtest_pem = max(0,(1-norm(y_valid-ytest_pem)^2/norm(y_valid)^2))
% RMSEtest_pem = sqrt(norm(y_valid-ytest_pem)^2/num_valid)
% 
% figure(4)
% subplot(2,1,1)
% plot(t_valid,y_valid(:,1),t_valid,ytest_pem(:,1))
% subplot(2,1,2)
% plot(t_valid,y_valid(:,2),t_valid,ytest_pem(:,2))

%%
function [Abar,Bbar,C,D,x0] = oem(A0,B0,C0,D0,x00,y,u,maxiter)

N = length(y);
theta = matrices2theta(A0, B0, C0, D0,x00);
p = length(theta);
x = zeros(4,N);
x(:,1) = x00;

dA = zeros(4,4,10);
for i=1:4
    dA(3,i,i) = 1;
end

for i=1:4
    dA(4,i,i+4) = 1;
end

dB = zeros(4,1,10);
dB(:,:,9) = [0;0;1;0];
dB(:,:,10) = [0;0;0;1];

dxk0 = zeros(4,1,p);


A = A0; B = B0; C = C0; D = D0;

for i=1:maxiter
    % Error vector
    En = zeros(N*2,1);
    for k=1:N
        x(:,k+1) = A*x(:,k) + B.*u(k);
        yk_hat = C*x(:,k) + D*u(k);
        En(k*2-1:k*2) = y(k,:)' - yk_hat;
    end

    % Jacobian of error vector
    psi_n = zeros(N*2,p);
    for j=1:p
        d_xk = dxk0(:,:,j);
        for k=1:N  
            d_xkp1 = A*d_xk + dA(:,:,j)*x(:,k) + dB(:,:,j).*u(k);
            dyk_hat = C*d_xk; % derivatives of C and D always zero
            psi_n(k*2-1:k*2,j) = -dyk_hat;
            d_xk = d_xkp1;
        end
    end

    dJn = 2/N*psi_n'*En;
    Hn = 2/N*(psi_n'*psi_n);
%     lambda = 8e-1;
    lambda = 0.9;
    theta = theta - inv(Hn+lambda*eye(length(Hn)))*dJn;
    [A,B,C,D,x(:,1)] = theta2matrices(theta,1,1,1,1,1);

end
[Abar,Bbar,C,D,x0] = theta2matrices(theta,1,1,1,1,1);
     
end

% function [Abar,Bbar,C,D,x0] = oem(A0,B0,C0,D0,x00,y,u,maxiter)
% % Instructions:
% % Implement your Output Error Model estimation method here.
% % Use the following function inputs and outputs.
% % Function INPUT
% % A0 Initial guess for system matrix A (matrix of size n x n)
% % B0 Initial guess for system matrix B (matrix of size n x m)
% % C0 Initial guess for system matrix C (matrix of size l x n)
% % D0 Initial guess for system matrix D (matrix of size l x m)
% % x00 Initial guess for initial state (vector of size n x one)
% % u System input (matrix of size N x m)
% % y System output (matrix of size N x l)
% % maxiter Maximum number of iterations (scalar)
% %
% % Function OUTPUT
% % Abar Estimate of system matrix A (matrix of size n x n)
% % Bbar Estimate of system matrix B (matrix of size n x m)
% % C Estimate of system matrix C (matrix of size l x n)
% % D Estimate of system matrix D (matrix of size l x m)
% % x0 Estimate of initial state (vector of size n x one)
% 
% Abar=A0;
% Bbar=B0;
% C=C0;
% D=D0;
% 
% 
% theta = matrices2theta(A0,B0,C0,D0,x00);
% 
% % N=length(y)
% I=size(y,2);
% N=size(y,1);
% P=length(theta);
% 
% psi=zeros(N*I,P);
% E=zeros(N*I,1);
% 
% 
% X=zeros(length(x00),N);
% X(:,1)=x00;
% % x0=x00;
% 
% dA=zeros(4,4,6);
% dA(3,2,1)=1;
% dA(3,3,2)=1;
% dA(3,4,3)=1;
% dA(4,2,4)=1;
% dA(4,3,5)=1;
% dA(4,4,6)=1;
% 
% dB3=[0;0;1;0];
% dB4=[0;0;0;1];
% 
% lamda=0.1;
% for num=1:maxiter
% 
%     %E 0~N-1
%     for i=1:N
%         X(:,i+1)=Abar*X(:,i)+Bbar*u(i);
%         ybar=C*X(:,i)+D*u(i);
%         E(I*i-1:I*i)=y(i,:)'-ybar;
%     end
% 
% 
%     %psi
%     %1~6
%     for phiindex=1:6
%         partial_xk=[0;0;0;0];
%     for i=1:N
% %         i
% %         partial_xk
%         partial_xk1 = Abar*partial_xk+dA(:,:,phiindex)*X(:,i);
%         psi(I*i-1:I*i,phiindex)=-C*partial_xk;
% % if phiindex==7&&num==2
% %         i
% %         Abar
% %         partial_xk
% %         dA(:,:,phiindex)
% %         X(:,i)
% %         psi(i,phiindex)
% % end
%         partial_xk=partial_xk1;
% 
%     end
%     end
% 
%     %7
%     partial_xk=[0;0;0;0];
%     for i=1:N
%         partial_xk1 = Abar*partial_xk+dB3*u(i);
%         psi(I*i-1:I*i,7)=-C*partial_xk;
%         partial_xk=partial_xk1;
%     end
%     %8
%     partial_xk=[0;0;0;0];
%     for i=1:N
%         partial_xk1 = Abar*partial_xk+dB4*u(i);
%         psi(I*i-1:I*i,8)=-C*partial_xk;
%         partial_xk=partial_xk1;
%     end
%     
% 
%     
%     %9
%     partial_xk=[1;0;0;0];
%     for i=1:N
%         partial_xk1 = Abar*partial_xk;
%         psi(I*i-1:I*i,9) = -C * partial_xk;
%         partial_xk = partial_xk1;
%     end
%     %10
%     partial_xk=[0;1;0;0];
%     for i=1:N
%         partial_xk1 = Abar*partial_xk;
%         psi(I*i-1:I*i,10)=-C*partial_xk;
%         partial_xk=partial_xk1;
%     end
%     %11
%     partial_xk=[0;0;1;0];
%     for i=1:N
%         partial_xk1 = Abar*partial_xk;
%         psi(I*i-1:I*i,11)=-C*partial_xk;
%         partial_xk=partial_xk1;
%     end
%     %12
%     partial_xk=[0;0;0;1];
%     for i=1:N
%         partial_xk1 = Abar*partial_xk;
%         psi(I*i-1:I*i,12)=-C*partial_xk;
%         partial_xk=partial_xk1;
%     end
% 
%     
%     H=2/N*psi'*psi;
%     Jdot=2/N*psi'*E;
%     theta=theta-inv(H+lamda*eye(P))*Jdot;
%     [Abar,Bbar,C,D,X(:,1)] = theta2matrices(theta,size(Abar),size(Bbar),size(C),size(D),size(X(:,1)));
% 
% end
% x0=X(:,1);
% end



function theta = matrices2theta(Abar,Bbar,C,D,x0) 
%%
% Function INPUT
% Abar      System matrix Abar (matrix of size n x n)
% Bbar      System matrix Bbar (matrix of size n x m)
% C         System matrix C (matrix of size l x n)
% D         System matrix D (matrix of size l x m)
% x0        Initial state (vector of size n x one)

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


function [Abar,Bbar,C,D,x0] = theta2matrices(theta,Asize,Bsize,Csize,Dsize,xsize)
%%
% Function INPUT
% theta     Paramter vector (vector of size n*n+n*m+l*n+l*m+n*l+n)
% Asize     Size of Abar 
% Bsize     Size of Bbar 
% Csize     Size of C 
% Dsize     Size of D 
% xsize     Size of x0

% Function OUTPUT
% Abar      System matrix A (matrix of size n x n)
% Bbar      System matrix B (matrix of size n x m)
% C         System matrix C (matrix of size l x n)
% D         System matrix D (matrix of size l x m)
% x0        Initial state (vector of size n x one)

Ts = 0.01;
Abar = [1 0 Ts 0; 0 1 0 Ts;theta(1:4)';theta(5:8)'];
Bbar = [0;0;theta(9);theta(10)];
C = [1 0 0 0;0 1 0 0];
D = 0;
x0 = [0;0;0;0];
end
