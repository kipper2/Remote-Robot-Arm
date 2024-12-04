%% 该代码为基于BP网络的语言识别
% [q1, q1_dot, torque1, q2, q2_dot, torque2, q3, q3_dot, torque3] -> [q1_ddot, q2_ddot, q3_ddot]
%9-10-3
%% 清空环境变量
clc
clear

%% 训练数据预测数据提取及归一化

Input_Data= zeros(9:100);


%输入数据归一化
[inputn,inputps]=mapminmax(Input_Data);

%% 网络结构初始化
innum=9;
midnum=10;
outnum=3;

% W_hat * sigmoid(V_hat * x_overline(t)) = 0

%权值初始化
W_hat_Init = 0;
V_hat_Init = 0;
boundary_1 = 10;
BOundary_2 = 10;


%学习率
xite=0.1;
alfa=0.01;
loopNumber=10;

I=zeros(1,midnum);
Iout=zeros(1,midnum);
FI=zeros(1,midnum);
dw1=zeros(innum,midnum);
db1=zeros(1,midnum);

%% 网络训练
E=zeros(1,loopNumber);
for ii=1:loopNumber
    E(ii)=0;
    for i=1:1:1500
       %% 网络预测输出 
        x=inputn(:,i);
        % 隐含层输出
        for j=1:1:midnum
            I(j)=inputn(:,i)'*w1(j,:)'+b1(j);
            Iout(j)=1/(1+exp(-I(j)));
        end
        % 输出层输出
        yn=w2'*Iout'+b2;
        
       %% 权值阀值修正
        %计算误差
        e=output_train(:,i)-yn;     
        E(ii)=E(ii)+sum(abs(e));
        
        %计算权值变化率
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
 

%% 语音特征信号分类
inputn_test=mapminmax('apply',input_test,inputps);
fore=zeros(4,500);
for ii=1:1
    for i=1:500%1500
        %隐含层输出
        for j=1:1:midnum
            I(j)=inputn_test(:,i)'*w1(j,:)'+b1(j);
            Iout(j)=1/(1+exp(-I(j)));
        end
        
        fore(:,i)=w2'*Iout'+b2;
    end
end

%% 结果分析
%根据网络输出找出数据属于哪类
output_fore=zeros(1,500);
for i=1:500
    output_fore(i)=find(fore(:,i)==max(fore(:,i)));
end

%BP网络预测误差
error=output_fore-output1(n(1501:2000))';

%画出预测语音种类和实际语音种类的分类图
figure(1)
plot(output_fore,'r')
hold on
plot(output1(n(1501:2000))','b')
legend('预测语音类别','实际语音类别')

%画出误差图
figure(2)
plot(error)
title('BP网络分类误差','fontsize',12)
xlabel('语音信号','fontsize',12)
ylabel('分类误差','fontsize',12)

%print -dtiff -r600 1-4

k=zeros(1,4);  
%找出判断错误的分类属于哪一类
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

%找出每类的个体和
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

%正确率
rightridio=(kk-k)./kk;
disp('正确率')
disp(rightridio);
web browser www.matlabsky.com