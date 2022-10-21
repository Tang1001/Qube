par1 = 0.1; % Initial guess for A(1,1) 
par2 = -2;  % Initial guess for A(1,2) 
par3 = 1;   % Initial guess for C(1,1) 
par4 = 3;   % Initial guess for C(1,2) 
par5 = 0.2; % Initial guess for R1(1,1)
Pvec = [par1; par2; par3; par4; par5];
auxVal = 1; % R2=1

Minit = idgrey('mynoise',Pvec,'d',auxVal);