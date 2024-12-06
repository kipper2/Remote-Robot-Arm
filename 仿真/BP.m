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

sizes.NumContStates  = 9; 
sizes.NumDiscStates  = 0; 
sizes.NumOutputs     = 3; 
sizes.NumInputs      = 12;  
sizes.DirFeedthrough = 0; 
sizes.NumSampleTimes = 0; 

sys = simsizes(sizes);
x0  = [0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]; 
str = []; 
ts  = [];
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

x_overline_1 = [u(1);
                u(2);
                u(3)];
x_overline_2 = [u(4);
                u(5);
                u(6)];
x_overline_3 = [u(7);
                u(8);
                u(9)];

phi_1 = [10*u(1)^3     + 0.5*u(5)  + 10*u(7);
         1.24*u(4)     + 0.04*u(5) + 0.6*sin(u(8))
         0.6*sin(u(1)) + 5*u(7)    + 3*sin(u(7))];
phi_2 = [50*u(1)^3     + 0.5*u(5)  + 20*u(7);
         7.44*u(4)     + 0.24*u(5) + 3.6*sin(u(8))
         56*sin(u(1))  + 70*u(7)^2 + 1.05*sin(u(7))];

mu = M * (- F - tau);

W_hat_1_dot = - 5 * (x(3).' * A^(-1)).' * (sigmoidM(x(2) * x_overline_1)).' - 0.02*norm(x(3))*x(1);
V_hat_1_dot = - 5 * (x(3).' * A^(-1) * x(1) * (I - diag(sigmoidM(x(2)*x(3)^2)))).' * x_overline_1.' - 0.02 * norm(x(3))*x(2);
W_hat_2_dot = - 5 * (x(4).' * A^(-1)).' * (sigmoidM(x(7) * x_overline_2)).' - 0.02*norm(x(4))*x(6);
V_hat_2_dot = - 5 * (x(4).' * A^(-1) * x(6) * (I - diag(sigmoidM(x(7)*x(4)^2)))).' * x_overline_2.' - 0.02 * norm(x(4))*x(7);
W_hat_3_dot = - 5 * (x(5).' * A^(-1)).' * (sigmoidM(x(9) * x_overline_3)).' - 0.02*norm(x(5))*x(8);
V_hat_3_dot = - 5 * (x(5).' * A^(-1) * x(8) * (I - diag(sigmoidM(x(9)*x(5)^2)))).' * x_overline_3.' - 0.02 * norm(x(5))*x(9);
5
x_tilde_1_dot  = A * x(3) + mu + phi_1 + phi_2 - u(10); 
x_tilde_2_dot  = A * x(4) + mu + phi_1 + phi_2 - u(11); 
x_tilde_3_dot  = A * x(5) + mu + phi_1 + phi_2 - u(12); 

sys = [W_hat_1_dot(), V_hat_1_dot, x_tilde_1_dot, x_tilde_2_dot, x_tilde_3_dot, W_hat_2_dot, V_hat_2_dot, W_hat_3_dot, V_hat_3_dot];

 
function sys=mdlUpdate(t,x,u)

sys = [];


function sys=mdlOutputs(t,x,u)
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

x_overline_1 = [u(1);
                u(2);
                u(3)];
x_overline_2 = [u(4);
                u(5);
                u(6)];
x_overline_3 = [u(7);
                u(8);
                u(9)];

phi_1 = [10*u(1)^3     + 0.5*u(5)  + 10*u(7);
         1.24*u(4)     + 0.04*u(5) + 0.6*sin(u(8))
         0.6*sin(u(1)) + 5*u(7)    + 3*sin(u(7))];
phi_2 = [50*u(1)^3     + 0.5*u(5)  + 20*u(7);
         7.44*u(4)     + 0.24*u(5) + 3.6*sin(u(8))
         56*sin(u(1))  + 70*u(7)^2 + 1.05*sin(u(7))];

mu = M * (- F - tau);

W_hat_1_dot = - 5 * (x(3).' * A^(-1)).' * (sigmoidM(x(2) * x_overline_1)).' - 0.02*norm(x(3))*x(1);
V_hat_1_dot = - 5 * (x(3).' * A^(-1) * x(1) * (I - diag(sigmoidM(x(2)*x(3)^2)))).' * x_overline_1.' - 0.02 * norm(x(3))*x(2);
W_hat_2_dot = - 5 * (x(4).' * A^(-1)).' * (sigmoidM(x(7) * x_overline_2)).' - 0.02*norm(x(4))*x(6);
V_hat_2_dot = - 5 * (x(4).' * A^(-1) * x(6) * (I - diag(sigmoidM(x(7)*x(4)^2)))).' * x_overline_2.' - 0.02 * norm(x(4))*x(7);
W_hat_3_dot = - 5 * (x(5).' * A^(-1)).' * (sigmoidM(x(9) * x_overline_3)).' - 0.02*norm(x(5))*x(8);
V_hat_3_dot = - 5 * (x(5).' * A^(-1) * x(8) * (I - diag(sigmoidM(x(9)*x(5)^2)))).' * x_overline_3.' - 0.02 * norm(x(5))*x(9);
5
x_tilde_1_dot  = A * x(3) + mu + phi_1 + phi_2 - u(10); 
x_tilde_2_dot  = A * x(4) + mu + phi_1 + phi_2 - u(11); 
x_tilde_3_dot  = A * x(5) + mu + phi_1 + phi_2 - u(12); 

%Layer_Output_1 = x(1) * sigmoidM(x(2) * x_overline_1);
Layer_Output_1 = x(1) * sigmoidM(x(2) * x_overline_1);
Layer_Output_2 = x(6) * sigmoidM(x(7) * x_overline_2);
Layer_Output_3 = x(8) * sigmoidM(x(9) * x_overline_3);

sys = [Layer_Output_1(1), 1, 1];


function sys=mdlTerminate(t,x,u)
sys = [];



function y = sigmoid(x)
    y = 1/(1+exp(-x));



function y = sigmoidM(x)
    y = zeros(size(x,1), size(x,2));
    for i = 1:1:size(x,1)
        for ii = 1:1:size(x,2)
            y(i,ii) = sigmoid(x(i, ii));
        end
    end
