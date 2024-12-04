
clear all;
close all;

ts=0.001;

a=25;b=133;
sys=tf(b,[1,a,0]);
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

A1=[0 1;0 -a];
B1=[0;b];
C1=[1 0];
D1=[0];
[A,B,C,D]=c2dm(A1,B1,C1,D1,ts,'z');

Q=1;              
R=1;               

P=B*Q*B';          
x=zeros(2,1);       

u_1=0;u_2=0;
y_1=0;y_2=0;
ei=0;
error_1=0;
for k=1:1:1000
time(k)=k*ts;

yd(k)=sin(0.01*k);
kp=80.0;ki=10;kd=0.2;

w(k)=0.002*rands(1);   
v(k)=0.002*rands(1);   
y(k)=-den(2)*y_1-den(3)*y_2+num(2)*u_1+num(3)*u_2;
yv(k)=y(k)+v(k);


Mn=P*C'/(C*P*C'+R);
P=A*P*A'+B*Q*B'; 
P=(eye(2)-Mn*C)*P;
    
x=A*x+Mn*(yv(k)-C*A*x);
ye(k)=C*x+D;     
    
M=1;
if M==1         
 	yout(k)=yv(k);
elseif M==2     
   yout(k)=ye(k);
end
error(k)=yd(k)-yout(k);
ei=ei+error(k)*ts;

u(k)=kp*error(k)+ki*ei+kd*(error(k)-error_1)/ts;  
u(k)=u(k)+w(k);

errcov(k)=C*P*C';     


x=A*x+B*u(k);

u_2=u_1;u_1=u(k);
y_2=y_1;y_1=yout(k);
error_1=error(k);
end
figure(1);
plot(time,yd,'r',time,yout,'k:','linewidth',2);
xlabel('time(s)');ylabel('yd,yout');
legend('Ideal position signal','Position tracking');