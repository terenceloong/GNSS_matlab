function acqResults = BDS_B1C_acq(file_path, sample_offset)
% 北斗B1C信号捕获，只搜索北斗三号卫星，20ms数据长度，结果存在变量acqResults中
% B1C信号只在北斗三号MEO、IGSO上播发
% sample_offset：抛弃前多少个采样点处开始处理

%%
N = 40000; %有效长度，10ms
Ns = 2*N; %采样点数，20ms
fs = 4e6; %采样频率，Hz
fc = 1.023e6; %码频率，Hz

carrFreq = -5e3:(fs/N/2):5e3; %频率搜索范围，频率步进50ms
M = length(carrFreq); %频率搜索个数

acqThreshold = 1.4; %搜索阈值，最大峰比第二大峰大多少倍

%%
% 取20ms数据
fileID = fopen(file_path, 'r');
    fseek(fileID, round(sample_offset*4), 'bof');
    if int32(ftell(fileID))~=int32(sample_offset*4)
        error('Sample offset error!');
    end
    baseband = double(fread(fileID, [2,Ns], 'int16'));
    baseband = baseband(1,:) + baseband(2,:)*1i; %行向量
fclose(fileID);

result = zeros(M,N); %存搜索结果，行是载波频率，列是码相位
corrValue = zeros(M,1); %每个搜索频率相关最大值
corrIndex = zeros(M,1); %最大值对应的码相位索引

% 卫星列表
svList = [19:30,32:38]; %当前可用的北斗三号卫星，截止到2019年10月29号19颗，38为IGSO，其他为MEO

% 存捕获结果，第一列码相位，第二列载波频率
acqResults = NaN(length(svList),2);

%% 捕获算法
for PRN=svList
%     B1Ccode = BDS_B1C_code_data(PRN);
    B1Ccode = BDS_B1C_code_pilot(PRN); %捕获导频分量性能更好，基底噪声一样，相关峰更高
    B1Ccode = reshape([B1Ccode;-B1Ccode],10230*2,1)'; %把子载波算进来，行向量
    codes = B1Ccode(floor((0:N-1)*fc*2/fs) + 1); %采样，算子载波后，码频率乘2
    code = [zeros(1,N), codes]; %补零
    CODE = fft(code); %码FFT
    
    %----搜索
    for k=1:M
        carrier = exp(-2*pi * carrFreq(k) * (0:Ns-1)/fs * 1i); %本地复载波，负频率
        x = baseband .* carrier;
        X = fft(x);
        Y = conj(X).*CODE;
        y = abs(ifft(Y));
        result(k,:) = y(1:N); %只取前N个
        [corrValue(k), corrIndex(k)] = max(result(k,:)); %寻找一次相关的最大值及其索引
    end
    
    %----寻找相关峰
    [peakSize, index] = max(corrValue); %最大峰大小和对应的频率索引
    corrValue(mod(index+(-5:5)-1,M)+1) = 0; %排除掉最大相关峰周围的点
    secondPeakSize = max(corrValue); %第二大峰
    
    %----捕获到信号
    if (peakSize/secondPeakSize)>acqThreshold
        % 画图
        figure
        subplot(2,1,1) %码相位方向
        plot(result(index,:)) %result的行
        grid on
        title(['PRN = ',num2str(PRN)])
        subplot(2,1,2) %频率方向
        plot(carrFreq, result(:,corrIndex(index))') %result的列
        grid on
        drawnow
        %存储捕获结果
        ki = find(svList==PRN,1);
        acqResults(ki,1) = corrIndex(index); %码相位
        acqResults(ki,2) = carrFreq(index); %载波频率
    end
end

acqResults = [svList', acqResults]; %第一列添加卫星编号

end