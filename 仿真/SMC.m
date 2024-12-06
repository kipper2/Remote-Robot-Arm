function [sys,x0,str,ts,simStateCompliance] = sfuntmpl(t,x,u,flag)

switch flag,
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes; 
  case 1,
    sys=mdlDerivatives(t,x,u);     
  case 3,
    sys=mdlOutputs(t,x,u);     
  case {2, 4, 9},
    sys=mdlTerminate(t,x,u);   
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));  
end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

sizes = simsizes;

sizes.NumContStates  = 6;  
sizes.NumDiscStates  = 0;  
sizes.NumOutputs     = 3;  
sizes.NumInputs      = 12;  
sizes.DirFeedthrough = 0;  
sizes.NumSampleTimes = 1;  

sys = simsizes(sizes);
x0  = [0.001 0.001 0.001 0.001 0.001 0.001]; 
str = []; 
ts  = [0 0];
simStateCompliance = 'UnknownSimState';


function sys=mdlDerivatives(t,x,u)
k1 = 1.798e-3; % k1,k2,...,k10 are the dynamic parameters of the model
k2 = 0.864e-3;
k3 = 0.486e-3;
k4 = 2.766e-3;
k5 = 0.308e-3;
k6 = 2.526e-3;
k7 = 0.652e-3;
k8 = 164.458e-3;
k9 = 94.050e-3;
k10 = 117.294e-3;

A = [-10, 0,  0;
      0, -10, 0;
      0,   0, -10];

M = [k1+k2*cos(2.0*u(2))+k3*cos(2.0*u(3))+k4*cos(u(2))*sin(u(3)), k5*sin(u(2)), 0;
     k5*sin(u(2)), k6, -0.5*k4*sin(u(2)-u(3));
     0, -0.5*k4*sin(u(2)-u(3)), k7];

F   = [1.1*u(2)+0.3*sin(3*u(1));
       1.2*u(5)+0.4*u(4);
       1.4*u(8)+0.3*sin(3*u(7))];

tau = [0.2*sin(u(2));
       0.1*sin(u(5));
       0.15*sin(u(8))];

phi = [10*u(1)^3+0.5*u(5)+10*u(7);
       1.24*u(4)+0.04*u(5)+0.6*sin(u(8));
       0.6*sin(u(1))+5*u(7)+3*sin(u(7))];

x_tilde = [x(1); x(2); x(3)];
S = [u(10); u(11); u(12)];
x_tilde_dot = A * x_tilde + M^(-1) *(- F - tau) + phi - S;
z_dot = -4 * sat(x_tilde);

sys = [x_tilde_dot(1), x_tilde_dot(2), x_tilde_dot(3), z_dot(1), z_dot(2), z_dot(3)];
%sys = [1, 1, 1, z_dot(1), z_dot(2), z_dot(3)];
 
function sys=mdlUpdate(t,x,u)

sys = [];


function sys=mdlOutputs(t,x,u)

x_tilde = [x(1); x(2); x(3)];
z       = [x(4); x(5); x(6)];

S = 4 * norm(x_tilde)^(0.5)* sat(x_tilde) - z;
controller_u = -satlin(S) - S + 4 * sat(x_tilde);

sys = [controller_u(1,1); controller_u(2,1); controller_u(3,1)];


function sys=mdlTerminate(t,x,u)
sys = [];

function y=sat(x)
    y = zeros(size(x,1), size(x,2));

    for i = 1:1:size(x,1)
        for ii = 1:1:size(x,2)
            if x(i,ii) > 0.05
                y(i,ii) = 1;
            elseif x(i,ii) < -0.05
                y(i,ii) = -1;
            else
                y(i,ii) = 20*x(i,ii);
            end
        end
    end