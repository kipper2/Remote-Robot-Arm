function [sys,x0,str,ts]=threeDOFdynamics(t,x,u,flag)
switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1,
    sys=mdlDerivatives(t,x,u);
case 3,
    sys=mdlOutputs(t,x,u);
case {2, 4, 9 }
    sys = [];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end


function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 16;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 16;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys=simsizes(sizes);
x0=[0 0 0 0 0 0 1.798e-3 0.864e-3 0.486e-3 2.766e-3 0.308e-3 2.526e-3 0.652e-3 164.458e-3 94.050e-3 117.294e-3];
str=[];
ts=[0 0];


function sys=mdlDerivatives(t,x,u)

q1 = x(1);
q2 = x(2);
q3 = x(3);
q1_dot = x(4);
q2_dot = x(5);
q3_dot = x(6);
k1 = x(7); % k1,k2,...,k10 are the dynamic parameters of the model
k2 = x(8);
k3 = x(9);
k4 = x(10);
k5 = x(11);
k6 = x(12);
k7 = x(13);
k8 = x(14);
k9 = x(15);
k10 = x(16);

Q_dot=[q1_dot;q2_dot;q3_dot];
Controlandexternal = [u(1)-u(4);u(2)-u(5);u(3)-u(6)]; % the control torque u(1) u(2) u(3) and disturbance u(4) u(5) u(6) 

%惯性矩阵    
M_actual=[k1+k2*cos(2.0*q2)+k3*cos(2.0*q3)+k4*cos(q2)*sin(q3) k5*sin(q2) 0;k5*sin(q2) k6 -0.5*k4*sin(q2-q3);0 -0.5*k4*sin(q2-q3) k7];
M_actual_inv=inv(M_actual);

%科氏力
C_actual=[-k2*q2_dot*sin(2.0*q2)-k3*q3_dot*sin(2.0*q3)-0.5*k4*q2_dot*sin(q2)*sin(q3)+0.5*k4*q3_dot*cos(q2)*cos(q3) -k2*q1_dot*sin(2.0*q2)+k5*q2_dot*cos(q2)-0.5*k4*q1_dot*sin(q2)*sin(q3) -k3*q1_dot*sin(2.0*q3)+0.5*k4*q1_dot*cos(q2)*cos(q3);
           k2*q1_dot*sin(2.0*q2)+0.5*k4*q1_dot*sin(q2)*sin(q3) 0 0.5*k4*q3_dot*cos(q2-q3);
           k3*q1_dot*sin(2.0*q3)+0.5*k4*q1_dot*cos(q2)*cos(q3) -0.5*k4*q2_dot*cos(q2-q3) 0];
C1_actual=-M_actual_inv*C_actual;

%重力
G_actual=[0;k8*cos(q2)+k10*(q2-0.5*pi);k9*sin(q3)];
G1_actual=-M_actual_inv*G_actual;

Q_dot_dot=C1_actual*Q_dot+G1_actual+M_actual\Controlandexternal;

sys(1)=q1_dot;
sys(2)=q2_dot;
sys(3)=q3_dot;

sys(4)=Q_dot_dot(1);
sys(5)=Q_dot_dot(2);
sys(6)=Q_dot_dot(3);

sys(7)= 0;
sys(8)= 0;
sys(9)= 0;
sys(10)= 0;
sys(11) = 0;
sys(12) = 0;
sys(13) = 0;
sys(14)= 0;
sys(15) = 0;
sys(16) = 0;

function sys=mdlOutputs(t,x,u)

sys(1)=x(1);
sys(2)=x(2);
sys(3)=x(3);

sys(4)=x(4);
sys(5)=x(5);
sys(6)=x(6);

sys(7)= x(7);
sys(8)= x(8);
sys(9)= x(9);
sys(10)= x(10);
sys(11) = x(11);
sys(12) = x(12);
sys(13) = x(13);
sys(14)= x(14);
sys(15) = x(15);
sys(16) = x(16);

