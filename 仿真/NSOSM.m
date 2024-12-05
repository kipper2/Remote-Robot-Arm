function [sys,x0,str,ts,simStateCompliance] = sfuntmpl(t,x,u,flag)
A = [-10, 0, -10;
     0,  -10, 0;
     0,   0, -10];

M = [1, 1, 1;
     1, 1, 1;
     1, 1, 1];

F   = [1.1*u(2) + 0.3*sin(3*u(1));
       1.2*u(5) + 0.4*u(4);
       1.4*u(8) + 0.3*sin(3*u(7))];
tua = [0.2  * sin(u(2));
       0.1  * sin(u(5));
       0.15 * sin(u(8))];

phi_1 = [10*u(1)^3     + 0.5*u(5)  + 10*u(7);
         1.24*u(4)     + 0.04*u(5) + 0.6*sin(u(8))
         0.6*sin(u(1)) + 5*u(7)    + 3*sin(u(7))];
phi_2 = [50*u(1)^3     + 0.5*u(5)  + 20*u(7);
         7.44*u(4)     + 0.24*u(5) + 3.6*sin(u(8))
         56*sin(u(1))  + 70*u(7)^2 + 1.05*sin(u(7))];

mu = M * (- F - tau);

phi = phi_1 + phi_2;

x_overline_1 = [q1, q1_dot, torque1];
x_overline_2 = [q2, q2_dot, torque1];
x_overline_3 = [q3, q3_dot, torque1];

W_hat = ones(4,3);
V_hat = ones(3,4);

omega = 10;

switch flag,
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes; 
  case 1,
    sys=mdlDerivatives(t,x,u); 
  case 2,
    sys=mdlUpdate(t,x,u);     
  case 3,
    sys=mdlOutputs(t,x,u);     
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u); 
  case 9,
    sys=mdlTerminate(t,x,u);   
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));  
end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sizes = simsizes;

sizes.NumContStates  = 9;  
sizes.NumDiscStates  = 0;  
sizes.NumOutputs     = 9;  
sizes.NumInputs      = 9;  
sizes.DirFeedthrough = 0;  
sizes.NumSampleTimes = 1;  

sys = simsizes(sizes);
x0  = []; 
str = []; 
ts  = [0 0];
simStateCompliance = 'UnknownSimState';


function sys=mdlDerivatives(t,x,u)
x_tidle_dot_S = A * x_tidle + M^(-1) *(-1 * F - tau_d) + u - S;
z_dot = -k_2 * sign(x_tidle);
st = 0;
if norm(x_tidle) > omega
    st = 1;
else
    st = 0;
end
N = W_hat*sigmoidM(V_hat * x_overline_1);
x_tilde_dot = A*x_tilde + M.'*(-F-tau) + phi - S - st*N; 
sys = [x_tidle_dot_S, z_dot, x_tilde_dot];


function sys=mdlUpdate(t,x,u)

sys = [];


function sys=mdlOutputs(t,x,u)
S = k_1 * norm(x_tidle)^(0.5)* sign(x_tidle) - x(2);

sys = [x(3)];


function sys=mdlGetTimeOfNextVarHit(t,x,u)

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
