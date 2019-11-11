% 画伪随机码自相关函数

n = 8; %一个子载波内取几个点
N = 20460*n;

B1Ccode = BDS_B1C_data_generate(19); %伪随机码
B1Ccode = reshape([B1Ccode;-B1Ccode],10230*2,1)'; %加子载波
code = B1Ccode(floor((0:N-1)/n) + 1)'; %采样，列向量
codes = [code; code]'; %行向量
codes = circshift(codes,4*n); %移两个码片

m = 8*n + 1; %总计算点数
R = zeros(m,1); %存结果

% 算自相关
for k=1:m
    R(k) = codes(k:k+N-1) * code;
end

% 画图
figure
plot(((1:m)-(m+1)/2)/2/n, R/20460/n, 'Marker','.', 'MarkerSize',10)
grid on
xlabel('码片')
ylabel('归一化自相关值')