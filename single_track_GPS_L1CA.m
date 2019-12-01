% GPS L1CA跟踪
% 一个卫星跟踪20s时间花费8.5s

% 处理数据例子：F:\GNSS data\20190726\B210_20190726_205109_ch1.dat，15号卫星

clear %清变量
clc %清屏幕
fclose('all'); %清文件

%% 数据文件
% data_file_A = 'F:\GNSS data\20190726\B210_20190726_205109_ch1.dat';
data_file_A = 'F:\GNSS data\20190826\B210_20190826_104744_ch1.dat';

%% 运行时间
msToProcess = 300*1*1000; %处理总时间
sample_offset = 0*4e6; %抛弃前多少个采样点
sampleFreq = 4e6; %接收机采样频率

%% 计时开始
tic

%% 初始化接收机
%--数据缓存
buffBlkNum = 40;                     %采样数据缓存块数量（要保证捕获时存储恰好从头开始）
buffBlkSize = 4000;                  %一个块的采样点数（1ms）
buffSize = buffBlkSize * buffBlkNum; %采样数据缓存大小
buff_A = zeros(2,buffSize);          %采样数据缓存，第一行I，第二行Q
buffBlkPoint = 0;                    %数据该往第几块存，从0开始
buffHead = 0;                        %最新数据的序号，buffBlkSize的倍数

%% 创建GPS处理通道
%--创建日志文件
logFile_GPS_A = '.\temp\log_GPS_A.txt'; %日志文件
logID_GPS_A = fopen(logFile_GPS_A, 'w');

%--卫星列表
svList_GPS = 6;
% svList_GPS = 15;
% svList_GPS = [10;13;15;20;21;24];
svN_GPS = length(svList_GPS);

%--创建通道
channels_GPS_A = cell(svN_GPS,1); %创建cell
for k=1:svN_GPS %创建跟踪通道对象
    channels_GPS_A{k} = GPS_L1CA_channel(sampleFreq, buffSize, svList_GPS(k), logID_GPS_A);
end

%--初始化通道
% channels_GPS_A{1}.init([2947, 3250], 0);
% channels_GPS_A{2}.init([2704,-2750], 0);
% channels_GPS_A{3}.init([2341,-1250], 0);
% channels_GPS_A{4}.init([2772, 2250], 0);
% channels_GPS_A{5}.init([2621, -750], 0);
% channels_GPS_A{6}.init([1384, 2000], 0);

% channels_GPS_A{1}.init([2947, 3250], 0);
% channels_GPS_A{1}.init([2704,-2750], 0);
% channels_GPS_A{1}.init([2341,-1250], 0);
% channels_GPS_A{1}.init([2772, 2250], 0);
% channels_GPS_A{1}.init([2621, -750], 0);
% channels_GPS_A{1}.init([1384, 2000], 0);

channels_GPS_A{1}.init([ 301, 1750], 0);

%--通道输出存储空间
m = msToProcess + 10;
trackResults_GPS = struct('PRN',0, 'n',1, ...
'dataIndex',    zeros(m,1), ...
...'remCodePhase', zeros(m,1), ... %可以不存，注释掉在前面加...
'codeFreq',     zeros(m,1), ...
...'remCarrPhase', zeros(m,1), ... %可以不存
'carrFreq',     zeros(m,1), ...
'I_Q',          zeros(m,6), ...
'disc',         zeros(m,3), ...
'std',          zeros(m,2));
trackResults_GPS_A = repmat(trackResults_GPS, svN_GPS,1);
clearvars trackResults_GPS
for k=1:svN_GPS
    trackResults_GPS_A(k).PRN = svList_GPS(k);
end

%% 打开文件，创建进度条
fileID_A = fopen(data_file_A, 'r');
fseek(fileID_A, round(sample_offset*4), 'bof'); %不取整可能出现文件指针移不过去
if int64(ftell(fileID_A))~=int64(sample_offset*4)
    error('Sample offset error!');
end
f = waitbar(0, ['0s/',num2str(msToProcess/1000),'s']);

%% 信号处理
for t=1:msToProcess
    %% 更新进度条
    if mod(t,1000)==0 %1s步进
        waitbar(t/msToProcess, f, [num2str(t/1000),'s/',num2str(msToProcess/1000),'s']);
    end
    
    %% 读数据
    buff_A(:,buffBlkPoint*buffBlkSize+(1:buffBlkSize)) = double(fread(fileID_A, [2,buffBlkSize], 'int16')); %取数据，行向量，往缓存里放
    buffBlkPoint = buffBlkPoint + 1;
    buffHead = buffBlkPoint * buffBlkSize;
    if buffBlkPoint==buffBlkNum
        buffBlkPoint = 0; %缓存从头开始
    end
    
    %% GPS跟踪
    for k=1:svN_GPS
        if channels_GPS_A{k}.state~=0
            while 1
                % 判断是否有完整的跟踪数据
                if mod(buffHead-channels_GPS_A{k}.trackDataHead,buffSize)>(buffSize/2)
                    break
                end
                % 存跟踪结果（通道参数）
                n = trackResults_GPS_A(k).n;
                trackResults_GPS_A(k).dataIndex(n,:)    = channels_GPS_A{k}.dataIndex;
                % trackResults_GPS_A(k).remCodePhase(n,:) = channels_GPS_A{k}.remCodePhase;
                trackResults_GPS_A(k).codeFreq(n,:)     = channels_GPS_A{k}.codeFreq;
                % trackResults_GPS_A(k).remCarrPhase(n,:) = channels_GPS_A{k}.remCarrPhase;
                trackResults_GPS_A(k).carrFreq(n,:)     = channels_GPS_A{k}.carrFreq;
                % 基带处理
                trackDataHead = channels_GPS_A{k}.trackDataHead;
                trackDataTail = channels_GPS_A{k}.trackDataTail;
                if trackDataHead>trackDataTail
                    [I_Q, disc] = channels_GPS_A{k}.track(buff_A(:,trackDataTail:trackDataHead));
                else
                    [I_Q, disc] = channels_GPS_A{k}.track([buff_A(:,trackDataTail:end),buff_A(:,1:trackDataHead)]);
                end
                channels_GPS_A{k}.parse; %可以注释掉，只跟踪不解析
                % 存跟踪结果（跟踪结果）
                trackResults_GPS_A(k).I_Q(n,:)          = I_Q;
                trackResults_GPS_A(k).disc(n,:)         = disc;
                trackResults_GPS_A(k).std(n,:)          = sqrt([channels_GPS_A{k}.varCode.D,channels_GPS_A{k}.varPhase.D]);
                trackResults_GPS_A(k).n                 = n + 1;
            end
        end
    end
    
end

%% 关闭文件，关闭进度条
fclose(fileID_A);
fclose(logID_GPS_A);
close(f);

%% 删除空白数据
trackResults_GPS_A = clean_trackResults(trackResults_GPS_A);

%% 打印通道日志
clc
disp('<--------GPS A-------->')
print_log(logFile_GPS_A, svList_GPS);

%% 画图
plot_track_GPS_A

%% 清除变量
clearvars -except sampleFreq msToProcess tf ...
                  svList_GPS channels_GPS_A trackResults_GPS_A ...

%% 计时结束
toc