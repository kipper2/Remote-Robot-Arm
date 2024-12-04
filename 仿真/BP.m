%% �ô���Ϊ����BP���������ʶ��
% [q1, q1_dot, torque1, q2, q2_dot, torque2, q3, q3_dot, torque3] -> [q1_ddot, q2_ddot, q3_ddot]
%9-10-3
%q_overline = [q, q_dot, torque].'
%% ��ջ�������
clc
clear

%% ѵ������Ԥ��������ȡ����һ��

mu = inv(M) \ (-F - tau);

%x_tilde_dot = A * x_tilde + mu + phi - W_hat*1/(1+exp(-1*(V_hat * x_overline)))
x_tilde_dot = A * x_tilde + mu + phi;

W_hat_dot = -alpha_1 * (x_hat.' * A^(-1)).' * (1\(1+exp(V_hat * x_overline))).' - rho_1 * norm(x_tilde, 2) * W_hat;
V_hat_dot = -alpha_2 * (x_hat.' * A^(-1)*W_hat*(I - diag(1/1+exp(V_hat*x_overline)))).' * (x_overline).' - rho_2 * norm(x_tilde, 2) * V_hat;

Input_Train
Median_Train
Output_Train
Input_Test
median_Test
Output_Test

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

%ѧϰ��
xite=0.1;
alfa=0.01;
loopNumber=10;

W_Hat = W_hat_Init;
V_Hat = V_hat_Init;


%% ����ѵ��
for ii=1:loopNumber
    E(ii)=0;
    for i=1:1:size(Input_Data,1)
       %% ����Ԥ����� 
        x=Input_Data(:,i);
        % ���������
        for j=1:1:midnum
            I(j)=Input_Train(:,i)'*V_hat(j,:)';
            Median_Train(j)=1/(1+exp(-I(j)));
        end
        % ��������
        Output_Train=W_hat'*Median_Train';

        W_hat = W_hat -alpha_1 * (x_hat.' * A^(-1)).' * (1\(1+exp(V_hat * x_overline))).' - rho_1 * norm(x_tilde, 2) * W_hat;
        V_hat = V_hat -alpha_2 * (x_hat.' * A^(-1)*W_hat*(I - diag(1/1+exp(V_hat*x_overline)))).' * (x_overline).' - rho_2 * norm(x_tilde, 2) * V_hat;
       
    end
end
 

%% ���������źŷ���
inputn_test=mapminmax('apply',input_test,inputps);

for ii=1:1
    for i=1:loopNumber
        %���������
        for j=1:1:midnum
            I(j)=Input_Test(:,i)'*V_hat';
            Median_Test=1/(1+exp(-I(j)));
        end
        
        Output_Test = W_hat'*Median_Test' ;
    end
end



%% �������
%������������ҳ�������������
for i=1:loopNumber
    Output(i)=find(outout_Test(:,i)==max(outout_Test(:,i)));
end

%BP����Ԥ�����
error=output_fore-output1(n(1501:2000))';