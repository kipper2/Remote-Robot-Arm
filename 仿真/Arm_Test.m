clear;
clc;
L1 = Link([0,0, 0,  -pi/2]);

L2 = Link([0, 0, 0.215, 0]);

L3 = Link([0, 0,  0.17,  0]);

Arm = SerialLink([L1, L2, L3]);

% T1=transl(0.5,0,0);%根据给定起始点，得到起始点位姿
% T2=transl(0,0.5,0);%根据给定终止点，得到终止点位姿
% q1=robot.ikine(T1);%根据起始点位姿，得到起始点关节角
% q2=robot.ikine(T2);%根据终止点位姿，得到终止点关节角
% [q ,qd, qdd]=jtraj(q1,q2,50); %五次多项式轨迹，得到关节角度，角速度，角加速度，50为采样点个数



load('matlab.mat')
q = out.simout;
grid on
T=Arm.fkine(q);%根据插值，得到末端执行器位姿
nT=T.T;  plot3(squeeze(nT(1,4,:)),squeeze(nT(2,4,:)),squeeze(nT(3,4,:)));
hold on
Arm.plot(q);%动画演示