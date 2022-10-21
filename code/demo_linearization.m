%% accoridng the manual
clear 

syms tau 
syms theta0 dtheta0 ddtheta0
syms theta1 dtheta1 ddtheta1
syms L0 m1 l1
syms I0 J1 g
syms b0 b1

alpha = I0 + m1*L0^2 + m1*l1^2*sin(theta1)^2;
gamma = m1*L0*l1*cos(theta1);
beta = m1*l1^2*sin(2*theta1);
rau = m1*L0*l1*sin(theta1);

% jacobian(alpha,L0)

Q0 = tau-b0*dtheta0;
Q1 = -b1*dtheta1;

%% method1
f0 = alpha*ddtheta0 + beta*dtheta0*dtheta1 + gamma*ddtheta1 - rau*dtheta1^2 - Q0;
f1 = gamma*ddtheta0 + (m1*l1^2+J1)*ddtheta1 - 1/2*beta*dtheta0^2 + m1*g*l1*sin(theta1) - Q1;

% eqns = [alpha*ddtheta0 + beta*dtheta0*dtheta1 + gamma*ddtheta1 - rau*dtheta1^2 - Q0==0, gamma*ddtheta0 + (m1*l1^2+J1)*ddtheta1 - 1/2*beta*dtheta0^2 + m1*g*l1*sin(theta1) - Q1==0];
eqns = [f0==0,f1==0];
[f0_ddtheta0,f1_ddtheta1] = solve(eqns,[ddtheta0 ddtheta1]);

f0_ddtheta0=simplify(f0_ddtheta0)
f1_ddtheta1=simplify(f1_ddtheta1)


%% method2
% f0 = (m1*L0^2 + I0)*ddtheta0 + m1*l1*L0*ddtheta1 - Q0;
% f1 = m1*l1*L0*ddtheta0 + (J1 + m1*l1^2)*ddtheta1 + m1*l1*g*theta1 - Q1;
% 
% % eqns = [alpha*ddtheta0 + beta*dtheta0*dtheta1 + gamma*ddtheta1 - rau*dtheta1^2 - Q0==0, gamma*ddtheta0 + (m1*l1^2+J1)*ddtheta1 - 1/2*beta*dtheta0^2 + m1*g*l1*sin(theta1) - Q1==0];
% eqns = [f0==0,f1==0];
% [f0_ddtheta0,f1_ddtheta1] = solve(eqns,[ddtheta0 ddtheta1]);
% 
% f0_ddtheta0=simplify(f0_ddtheta0)
% f1_ddtheta1=simplify(f1_ddtheta1)



%%
A31=jacobian(f0_ddtheta0,theta0)
A32=jacobian(f0_ddtheta0,theta1)
A33=jacobian(f0_ddtheta0,dtheta0)
A34=jacobian(f0_ddtheta0,dtheta1)

A41=jacobian(f1_ddtheta1,theta0)
A42=jacobian(f1_ddtheta1,theta1)
A43=jacobian(f1_ddtheta1,dtheta0)
A44=jacobian(f1_ddtheta1,dtheta1)


B3=jacobian(f0_ddtheta0,tau)
B4=jacobian(f1_ddtheta1,tau)

%% linearization (0,0,0,0);
eqb=[0,0,0,0];
% eqb=[0,pi,0,0];
states=[theta0,theta1,dtheta0,dtheta1];
A32=simplify(linearization(A32,states,eqb));
A33=simplify(linearization(A33,states,eqb));
A34=simplify(linearization(A34,states,eqb));

A42=simplify(linearization(A42,states,eqb));
A43=simplify(linearization(A43,states,eqb));
A44=simplify(linearization(A44,states,eqb));

B3=simplify(linearization(B3,states,eqb));
B4=simplify(linearization(B4,states,eqb));


% A34=-A34;
% A42=-A42;
% A43=-A43;
% B4=-B4;



A=[0,0,1,0;
    0,0,0,1;
    0,A32,A33,A34;
    0,A42,A43,A44]

B=[0;0;B3;B4]


%% estimation of parameters
g=9.81;
L0=0.085;
lp=0.129;
l1=lp/2;
m0=0.095;
m1=0.024;

% I0=1/3*m0*L0^2;
% J1=1/3*m1*l1^2;
I0=1/2*m0*L0^2;
J1=1/2*m1*l1^2;

b0=0.00015;
b1=0.0005;

% b0=0;
% b1=0;


% A_lin=vpa(subs(A),5)
% B_lin=vpa(subs(B),5)

A_lin=double(subs(A))
B_lin=double(subs(B))

% vpa(subs(A32),5)
% vpa(subs(A33),5)
% vpa(subs(A34),5)
% 
% vpa(subs(A42),5)
% vpa(subs(A43),5)
% vpa(subs(A44),5)
% 
% vpa(subs(B3),5)
% vpa(subs(B4),5)



%% tau to voltage
km=0.042;
Rm=8.4;

B_V = km/Rm;
B_dtheta0 = -km*km/Rm;





A_lin_v=A_lin;
A_lin_v(3,3)=A_lin_v(3,3)+B_dtheta0*B_lin(3);
A_lin_v(4,3)=A_lin_v(4,3)+B_dtheta0*B_lin(4);
A_lin_v

B_lin_v=B_lin*B_V


eig(A_lin_v)
% eig(A_lin_v[[1,3],[1,3]])

% %%
% syms abc aaa y
% y=abc^2+aaa;
% abc=2;
% aaa=2;
% subs(y)
% 

save('jaco','A_lin_v','B_lin_v','A_lin','B_lin')



%%
function A_entry_linearized=linearization(A_entry,states,point)
% A_entry=subs(A_entry,theta0,point(1));
% A_entry=subs(A_entry,theta1,point(2));
% A_entry=subs(A_entry,dtheta0,point(3));
% A_entry=subs(A_entry,dtheta1,point(4));

A_entry=subs(A_entry,states(1),point(1));
A_entry=subs(A_entry,states(2),point(2));
A_entry=subs(A_entry,states(3),point(3));
A_entry=subs(A_entry,states(4),point(4));

A_entry_linearized = A_entry;

end


