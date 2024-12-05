function [sys,x0,str,ts,simStateCompliance] = sfuntmpl(t,x,u,flag)

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

sizes.NumContStates  = 3;  
sizes.NumDiscStates  = 0;  
sizes.NumOutputs     = 1;  
sizes.NumInputs      = 3;  
sizes.DirFeedthrough = 1;  
sizes.NumSampleTimes = 1;  

sys = simsizes(sizes);
x0  = []; 
str = []; 
ts  = [0 0];
simStateCompliance = 'UnknownSimState';


function sys=mdlDerivatives(t,x,u)
x_tidle_dot = A * x_tidle + M^(-1) *(-1 * F - tau_d) + u - S;
z_dot = -k_2 * sign(x_tidle);
sys = [x_tidle_dot, z_dot];


function sys=mdlUpdate(t,x,u)

sys = [];


function sys=mdlOutputs(t,x,u)
S = k_1 * norm(x_tidle)^(0.5)* sign(x_tidle) - x(2);
controller_u = -sign(S) - S - k_2*sign(x(1)) - k_1*sign(x(1))/2*norm(x(1),2)^(0.5)*x(1);

sys = [controller_u];


function sys=mdlGetTimeOfNextVarHit(t,x,u)

function sys=mdlTerminate(t,x,u)

sys = [];