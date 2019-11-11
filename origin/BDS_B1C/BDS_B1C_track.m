function [ch, I_Q, disc] = BDS_B1C_track(ch, sampleFreq, buffSize, rawSignal)

%% 提取通道信息
codeData       = ch.codeData;
codePolit      = ch.codePilot;
timeIntMs      = ch.timeIntMs;
blkSize        = ch.blkSize;
carrNco        = ch.carrNco;
codeNco        = ch.codeNco;
remCarrPhase   = ch.remCarrPhase;
remCodePhase   = ch.remCodePhase;
PLL            = ch.PLL;
DLL            = ch.DLL;

ch.dataIndex = ch.dataIndex + blkSize;

timeInt = timeIntMs * 0.001; %积分时间，s

%% 基本处理
% 时间序列
t = (0:blkSize-1) / sampleFreq;
te = blkSize / sampleFreq;

% 生成本地载波
theta = (remCarrPhase + carrNco*t) * 2; %乘2因为后面是以pi为单位求三角函数
carr_cos = cospi(theta); %本地载波
carr_sin = sinpi(theta);
theta_next = remCarrPhase + carrNco*te;
remCarrPhase = mod(theta_next, 1); %剩余载波相位，周

% 生成本地码
tcode = remCodePhase + codeNco*t + 2; %加2保证求滞后码时大于1
% earlyCodeI  = codeData(floor(tcode+0.3));  %超前码（数据分量）
promptCodeI = codeData(floor(tcode));      %即时码
% lateCodeI   = codeData(floor(tcode-0.3));  %滞后码
earlyCodeQ  = codePolit(floor(tcode+0.3)); %超前码（导频分量）
promptCodeQ = codePolit(floor(tcode));     %即时码
lateCodeQ   = codePolit(floor(tcode-0.3)); %滞后码
remCodePhase = mod(remCodePhase + codeNco*te, 20460); %剩余载波相位，周

% 原始数据乘载波
iBasebandSignal = rawSignal(1,:).*carr_cos + rawSignal(2,:).*carr_sin; %乘负载波
qBasebandSignal = rawSignal(2,:).*carr_cos - rawSignal(1,:).*carr_sin;

% 六路积分（使用数据分量跟踪）
% I_E = iBasebandSignal * earlyCodeI;
% Q_E = qBasebandSignal * earlyCodeI;
% I_P = iBasebandSignal * promptCodeI;
% Q_P = qBasebandSignal * promptCodeI;
% I_L = iBasebandSignal * lateCodeI;
% Q_L = qBasebandSignal * lateCodeI;
% I = I_P;
% Q = Q_P;
% 六路积分（使用导频分量跟踪）
% I_E = iBasebandSignal * earlyCodeQ;
% Q_E = qBasebandSignal * earlyCodeQ;
% I_P = iBasebandSignal * promptCodeQ;
% Q_P = qBasebandSignal * promptCodeQ;
% I_L = iBasebandSignal * lateCodeQ;
% Q_L = qBasebandSignal * lateCodeQ;
% I = I_P;
% Q = Q_P;
% 六路积分（双通道跟踪）
I_E = iBasebandSignal * earlyCodeQ;
Q_E = qBasebandSignal * earlyCodeQ;
I_P = -qBasebandSignal * promptCodeI; %数据分量，幅值比1:sqrt(29/11)，1:624
Q_P =  iBasebandSignal * promptCodeQ; %导频分量
I_L = iBasebandSignal * lateCodeQ;
Q_L = qBasebandSignal * lateCodeQ;
I = iBasebandSignal * promptCodeQ;
Q = qBasebandSignal * promptCodeQ;

% 码鉴相器
S_E = sqrt(I_E^2+Q_E^2);
S_L = sqrt(I_L^2+Q_L^2);
codeError = (11/30) * (S_E-S_L)/(S_E+S_L); %单位：码片

% 载波鉴相器
carrError = atan(Q/I) / (2*pi); %单位：周

%% 跟踪算法
%----PLL
PLL.Int = PLL.Int + PLL.K2*carrError*timeInt; %锁相环积分器
carrNco = PLL.Int + PLL.K1*carrError;
carrFreq = PLL.Int;
%----DLL
DLL.Int = DLL.Int + DLL.K2*codeError*timeInt; %延迟锁定环积分器
codeNco = DLL.Int + DLL.K1*codeError;
codeFreq = DLL.Int;

%% 更新下一数据块位置
trackDataTail = ch.trackDataHead + 1;
if trackDataTail>buffSize
    trackDataTail = 1;
end
if ch.codeTarget==20460
    ch.codeTarget = 2046;
else
    ch.codeTarget = ch.codeTarget + 2046; %跟踪目标码相位，整秒跳
end
blkSize = ceil((ch.codeTarget-remCodePhase)/codeNco*sampleFreq);
trackDataHead = trackDataTail + blkSize - 1;
if trackDataHead>buffSize
    trackDataHead = trackDataHead - buffSize;
end
ch.trackDataTail = trackDataTail;
ch.blkSize       = blkSize;
ch.trackDataHead = trackDataHead;

%% 更新通道信息
ch.carrNco        = carrNco;
ch.codeNco        = codeNco;
ch.carrFreq       = carrFreq;
ch.codeFreq       = codeFreq;
ch.remCarrPhase   = remCarrPhase;
ch.remCodePhase   = remCodePhase;
ch.PLL            = PLL;
ch.DLL            = DLL;

%% 输出
I_Q = [I_P, I_E, I_L, Q_P, Q_E, Q_L];
disc = [codeError, carrError];

end