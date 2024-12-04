%% �ô���Ϊ����BP���������ʶ��
% [q1, q1_dot, torque1, q2, q2_dot, torque2, q3, q3_dot, torque3] -> [q1_ddot, q2_ddot, q3_ddot]
%9-10-3
%% ��ջ�������
clc
clear

%% ѵ������Ԥ��������ȡ����һ��

Input_Data= zeros(9:100);


%�������ݹ�һ��
[inputn,inputps]=mapminmax(Input_Data);

%% ����ṹ��ʼ��
innum=9;
midnum=10;
outnum=3;

% W_hat * sigmoid(V_hat * x_overline(t)) = 0

%Ȩֵ��ʼ��
W_hat_Init = 0;
V_hat_Init = 0;
boundary_1 = 10;
BOundary_2 = 10;


%ѧϰ��
xite=0.1;
alfa=0.01;
loopNumber=10;

I=zeros(1,midnum);
Iout=zeros(1,midnum);
FI=zeros(1,midnum);
dw1=zeros(innum,midnum);
db1=zeros(1,midnum);

%% ����ѵ��
E=zeros(1,loopNumber);
for ii=1:loopNumber
    E(ii)=0;
    for i=1:1:1500
       %% ����Ԥ����� 
        x=inputn(:,i);
        % ���������
        for j=1:1:midnum
            I(j)=inputn(:,i)'*w1(j,:)'+b1(j);
            Iout(j)=1/(1+exp(-I(j)));
        end
        % ��������
        yn=w2'*Iout'+b2;
        
       %% Ȩֵ��ֵ����
        %�������
        e=output_train(:,i)-yn;     
        E(ii)=E(ii)+sum(abs(e));
        
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
fore=zeros(4,500);
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
output_fore=zeros(1,500);
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
rightridio=(kk-k)./kk;
disp('��ȷ��')
disp(rightridio);
web browser www.matlabsky.com