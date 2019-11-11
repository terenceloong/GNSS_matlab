% 指定PRN，连续对一颗卫星进行捕获操作，看峰值是否变化

sample_offset = 0*4e6;
file_path = 'E:\GNSS data\B210_20190726_205109_ch1.dat';
PRN = 20;

N = 40000; %有效长度，10ms
Ns = 2*N; %采样点数，20ms
fs = 4e6; %采样频率，Hz
fc = 1.023e6; %码频率，Hz

carrFreq = -5e3:(fs/N/2):5e3; %频率搜索范围，频率步进50ms
M = length(carrFreq); %频率搜索个数
result = zeros(M,N); %存搜索结果，行是载波频率，列是码相位
corrValue = zeros(M,1); %每个搜索频率相关最大值
corrIndex = zeros(M,1); %最大值对应的码相位索引

% 画图
YLim = [0,2e6];
figure
subplot(2,1,1)
h1 = plot(result(1,:));
set(gca, 'YLim', YLim)
grid on
title(['PRN = ',num2str(PRN)])
subplot(2,1,2)
h2 = plot(carrFreq, result(:,1)');
set(gca, 'YLim', YLim)
grid on

% 伪码
B1Ccode = BDS_B1C_pilot_generate(PRN); %捕获导频分量性能更好，基底噪声一样，相关峰更高
B1Ccode = reshape([B1Ccode;-B1Ccode],10230*2,1)'; %把子载波算进来，行向量
codes = B1Ccode(floor((0:N-1)*fc*2/fs) + 1); %采样
code = [zeros(1,N), codes]; %补零
CODE = fft(code); %码FFT

% 打开文件
fclose('all'); %关闭之前打开的所有文件
fileID = fopen(file_path, 'r');
fseek(fileID, round(sample_offset*4), 'bof');
if int32(ftell(fileID))~=int32(sample_offset*4)
    error('Sample offset error!');
end

for n=1:100
    baseband = double(fread(fileID, [2,Ns], 'int16')); %取20ms数据
    baseband = baseband(1,:) + baseband(2,:)*1i; %行向量
    for k=1:M
        carrier = exp(-2*pi * carrFreq(k) * (0:Ns-1)/fs * 1i); %本地复载波，负频率
        x = baseband .* carrier;
        X = fft(x);
        Y = conj(X).*CODE;
        y = abs(ifft(Y));
        result(k,:) = y(1:N); %只取前N个
        [corrValue(k), corrIndex(k)] = max(result(k,:)); %寻找一次相关的最大值及其索引
    end
    % 寻找相关峰
    [~, index] = max(corrValue); %最大峰大小和对应的频率索引
    % 画图
    set(h1, 'Ydata', result(index,:));
    set(h2, 'Ydata', result(:,corrIndex(index))');
    drawnow
end

fclose(fileID);