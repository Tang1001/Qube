function [A,B,C,D] = modelQube(par,Ts,aux)
A=[1 0 Ts 0;
    0 1 0 Ts;
    par(1),par(2),par(3),par(4);
    par(5),par(6),par(7),par(8)];

B=[0;0;par(9);par(10)];


C=[1,0,0,0; 0,1,0,0];
D=[0;0];

end