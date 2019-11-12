% 验证实际带宽有限情况下的自相关
% 先做捕获、跟踪，确定码相位和准确的载波频率
% 可以设置做相关用的采样点数，看不同相关时间下的自相关结果，时间越长，自相关曲线越平滑

% 文件路径
file_path = 'E:\GNSS data\B210_20190726_205109_ch1.dat';

PRN = 15;
acqCodePhase = 2341; %捕获码相位结果
carrFreq = -1227+0; %跟踪得到的载波频率，加个误差，看频率有误差时的相关结果

fs = 4e6; %采样频率，Hz
fc = 1.023e6; %码频率，Hz
Nc = 40000; %一个码周期的采样点数
N = 40000; %做相关用的采样点数，4000对应1ms，40000对应10ms

% 取最前面100ms的点
fclose('all');
fileID = fopen(file_path, 'r');
baseband = double(fread(fileID, [2,100*4e3], 'int16'));
baseband = baseband(1,:) + baseband(2,:)*1i; %行向量
fclose(fileID);

% 从码周期开始点连续取一整个码周期的数据
index = Nc - acqCodePhase + 2; %捕获得到的码周期开始点的索引
data = baseband(index:index+N-1);

carrier = exp(-2*pi * carrFreq * (0:N-1)/fs * 1i); %本地复载波，负频率

x = data .* carrier; %基带信号乘载波

CAcode = GPS_L1_CA_generate(PRN);

phase = -2:0.01:2; %主码相位区间
n = length(phase);
R = zeros(n,1); %存结果

for k=1:n
    code = CAcode(mod(floor((0:N-1)*fc/fs+phase(k)),1023) + 1); %码采样，加相位偏移
    R(k) = x * code'; %与码做相关，复数，有相位差
end

figure
Rm = abs(R); %R的模长
plot(phase+0.04, Rm/max(Rm))
grid on
% 除以max(Rm)用来归一化，乘相角符号用来使相关值有正有负，最后乘1/-1使主峰为正
% phase减个值使峰值在中央，看图调。因为捕获得到的相位值有误差，直接得到的峰值可能偏一点，偏得越大表示捕获确定的相位误差越大