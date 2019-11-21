% 测试载噪比计算方法
% M的值小时，CN0i噪声大，但M增大时，CN0i噪声下降不明显
% 平均点数越大，CN0m噪声越小，当平均点数为1000往上时，CN0m非常接近理论值

A = 10;
sigma = 0.2;

n = 200*1000; %数据点数

I = randn(n,1)*sigma + A;
Q = randn(n,1)*sigma;

% figure %画散点分布
% plot(I,Q, 'LineStyle','none', 'Marker','.')
% grid on
% axis equal
% set(gca, 'Xlim', [-5*sigma-A, 5*sigma+A])
% set(gca, 'Ylim', [-5*sigma-A, 5*sigma+A])

M = 20; %宽带窄带计算数据段点数
N = n/M; %数据段个数

NBP_WBP = zeros(N,1);

for k=1:N
    Id = I((k-1)*M+(1:M));
    Qd = Q((k-1)*M+(1:M));
    WBP = sum(Id.^2 + Qd.^2); %先平方再求和
    NBP = sum(Id)^2 + sum(Qd)^2; %先求和再平方
    NBP_WBP(k) = NBP / WBP;
end

figure
CN0i = sqrt(2*(NBP_WBP-1)./(M-NBP_WBP)); %用一次NBP/WBP计算载噪比
plot(CN0i)
hold on

Z = movmean(NBP_WBP,25); %使用NBP/WBP的均值计算载噪比，这个是载噪比计算公式所表达的
CN0m = sqrt(2*(Z-1)./(M-Z)); %算出来的值应该等于A/sigma
plot(CN0m)