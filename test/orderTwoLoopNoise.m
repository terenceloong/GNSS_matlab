% 计算二阶环输出噪声
% 2Hz带宽，输出噪声比例为0.07
% 25Hz带宽，输出噪声比例为0.22

[K1, K2] = orderTwoLoopCoef(25, 0.707, 1);

n = 10000; %总点数
dt = 0.001; %时间间隔
X = randn(n,1); %输入
Y = zeros(n,1); %输出

x1 = 0; %控制器积分输出
x2 = 0; %总积分输出
for k=1:n
    e = X(k) - x2;
    x1 = x1 + K2*e*dt;
    x2 = x2 + (K1*e+x1)*dt;
    Y(k) = x2;
end

figure
plot((1:n)*dt, X)
hold on
plot((1:n)*dt, Y)

disp(std(Y)) %输出噪声标准差