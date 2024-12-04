%% �ô���Ϊ����BP���������ʶ��
% [q1, q1_dot, torque1, q2, q2_dot, torque2, q3, q3_dot, torque3] -> [q1_ddot, q2_ddot, q3_ddot]
%9-10-3
%q_overline = [q, q_dot, torque].'
%% ��ջ�������
clc
clear

%% ѵ������Ԥ��������ȡ����һ��
A = ... 
[ 10 0  0;
  0  10 0;
  0  0  10];
I = ... 
[ 1 0 0;
  0 1 0;
  0 0 1];
Input_Data  = zeros(9:100);
Output_Data = zeros(3:100);


%�������ݹ�һ��
[inputn,inputps]=mapminmax(Input_Data);

%% ����ṹ��ʼ��
innum=9;
midnum=10;
outnum=3;

% W_hat * sigmoid(V_hat * x_overline(t)) = 0

%Ȩֵ��ʼ��
W_hat_Init = zeros(100:9);
V_hat_Init = 0;
Boundary_1_init = 10;
Boundary_2_init = 10;

%ѧϰ��
xite=0.1;
alfa=0.01;
loopNumber=10;

W_Hat = W_hat_Init;
V_Hat = V_hat_Init;
Boundary_1 = Boundary_1_init;
Boundary_2 = Boundary_2_init;

%% ����ѵ��
for ii=1:loopNumber
    E(ii)=0;
    for i=1:1:size(Input_Data,1)
       %% ����Ԥ����� 
        x=Input_Data(:,i);
        % ���������
        for j=1:1:midnum
            I(j)=Input_Data(:,i)'*W_Hat(j,:)'+ Boundary_1(j);
            Iout(j)=1/(1+exp(-I(j)));
        end
        % ��������
        yn=V_Hat'*Iout'+Boundary_2;
        
       %% Ȩֵ��ֵ����
        %�������
        e=output_train(:,i)-yn;     
        E(ii)=E(ii)+sum(abs(e));

        W_Hat = integral((-5*(x.'\A).')*((1/1+exp(V_Hat*x)).') - 0.021 * norm(x,2)* W_Hat);
        V_Hat = integral(-5*(x.' * inv(A)*W_Hat(I - diag((1/1+exp(V_Hat*x))^2))))
        %����Ȩֵ�仯��
        dw2=e*Iout;
        db2=e';
        
        for j=1:1:midnum
            S=1/(1+exp(-I(j)));
            FI(j)=S*(1-S);
        end      
        for k=1:1:innum
            for j=1:1:midnum
                dw1(k,j)=FI(j)*x(k)*(e(1)*w2(j,1)+e(2)*w2(j,2)+e(3)*w2(j,3)+e(4)*w2(j,4));
                db1(j)=FI(j)*(e(1)*w2(j,1)+e(2)*w2(j,2)+e(3)*w2(j,3)+e(4)*w2(j,4));
            end
        end
           
        w1=w1_1+xite*dw1';
        b1=b1_1+xite*db1';
        w2=w2_1+xite*dw2';
        b2=b2_1+xite*db2';
        
        w1_2=w1_1;w1_1=w1;
        w2_2=w2_1;w2_1=w2;
        b1_2=b1_1;b1_1=b1;
        b2_2=b2_1;b2_1=b2;
    end
end
 

%% ���������źŷ���
inputn_test=mapminmax('apply',input_test,inputps);

for ii=1:1
    for i=1:500%1500
        %���������
        for j=1:1:midnum
            I(j)=inputn_test(:,i)'*w1(j,:)'+b1(j);
            Iout(j)=1/(1+exp(-I(j)));
        end
        
        fore(:,i)=w2'*Iout'+b2;
    end
end



%% �������
%������������ҳ�������������
for i=1:500
    output_fore(i)=find(fore(:,i)==max(fore(:,i)));
end

%BP����Ԥ�����
error=output_fore-output1(n(1501:2000))';



%����Ԥ�����������ʵ����������ķ���ͼ
figure(1)
plot(output_fore,'r')
hold on
plot(output1(n(1501:2000))','b')
legend('Ԥ���������','ʵ���������')

%�������ͼ
figure(2)
plot(error)
title('BP����������','fontsize',12)
xlabel('�����ź�','fontsize',12)
ylabel('�������','fontsize',12)

%print -dtiff -r600 1-4

k=zeros(1,4);  
%�ҳ��жϴ���ķ���������һ��
for i=1:500
    if error(i)~=0
        [b,c]=max(output_test(:,i));
        switch c
            case 1 
                k(1)=k(1)+1;
            case 2 
                k(2)=k(2)+1;
            case 3 
                k(3)=k(3)+1;
            case 4 
                k(4)=k(4)+1;
        end
    end
end

%�ҳ�ÿ��ĸ����
kk=zeros(1,4);
for i=1:500
    [b,c]=max(output_test(:,i));
    switch c
        case 1
            kk(1)=kk(1)+1;
        case 2
            kk(2)=kk(2)+1;
        case 3
            kk(3)=kk(3)+1;
        case 4
            kk(4)=kk(4)+1;
    end
end

%��ȷ��
rightridio=(kk-k)./kk
%web browser http://www.ilovematlab.cn/thread-60056-1-1.html