% 同时处理GPS和北斗信号
clear %清变量
clc %清屏幕
fclose('all'); %清文件

%% 数据文件
data_file = 'E:\GNSS data\B210_20190726_205109_ch1.dat';

%% 运行时间
msToProcess = 60*1*1000; %处理总时间
sample_offset = 0*4e6; %抛弃前多少个采样点
sampleFreq = 4e6; %接收机采样频率

%% 当前文件夹
curr_full_path = mfilename('fullpath'); %当前代码完整路径，包含文件名
[curr_path, ~] = fileparts(curr_full_path); %提取当前代码所在路径

%% 计时开始
tic

%% 初始化接收机
%--数据缓存
buffBlkNum = 40;                     %采样数据缓存块数量（要保证捕获时存储恰好从头开始）
buffBlkSize = 4000;                  %一个块的采样点数（1ms）
buffSize = buffBlkSize * buffBlkNum; %采样数据缓存大小
buff = zeros(2,buffSize);            %采样数据缓存，第一行I，第二行Q
buffBlkPoint = 0;                    %数据该往第几块存，从0开始
buffHead = 0;                        %最新数据的序号，buffBlkSize的倍数

%--接收机时间 (GPS时间)
tf = sscanf(data_file((end-22):(end-8)), '%4d%02d%02d_%02d%02d%02d')'; %数据文件开始采样时间（日期时间数组）
[tw, ts] = GPS_time(tf); %tw：GPS周数，ts：GPS周内秒数
ta = [ts,0,0] + sample2dt(sample_offset, sampleFreq); %初始化接收机时间（周内秒数），[s,ms,us]
ta = time_carry(round(ta,2)); %取整

%--接收机状态
receiverState = 0; %接收机状态，0表示未初始化，时间还不对，1表示时间已经校正
deltaFreq = 0; %时钟差，理解为百分比，如果差1e-9，生成1500e6Hz的波会差1.5Hz
dtpos = 10; %定位时间间隔，ms
tp = [ta(1),0,0]; %tp为下次定位时间
tp(2) = (floor(ta(2)/dtpos)+1) * dtpos; %加到下个整目标时间
tp = time_carry(tp); %进位

%% 创建GPS处理通道
%--创建日志文件
logFile_GPS_A = [curr_path,'\log_GPS_A.txt']; %日志文件
logID_GPS_A = fopen(logFile_GPS_A, 'w');

%--卫星列表
svList_GPS = [10;13;15;20;21;24];
svN_GPS = length(svList_GPS);

%--创建通道
channels_GPS_A = cell(svN_GPS,1); %创建cell
for k=1:svN_GPS %创建跟踪通道对象
    channels_GPS_A{k} = GPS_L1CA_channel(sampleFreq, buffSize, svList_GPS(k), logID_GPS_A);
end

%--初始化通道
channels_GPS_A{1}.init([2947, 3250], 0);
channels_GPS_A{2}.init([2704,-2750], 0);
channels_GPS_A{3}.init([2341,-1250], 0);
channels_GPS_A{4}.init([2772, 2250], 0);
channels_GPS_A{5}.init([2621, -750], 0);
channels_GPS_A{6}.init([1384, 2000], 0);

%--通道输出存储空间
m = msToProcess + 10;
trackResults_GPS_A = struct('PRN',0, 'n',1, ...
'dataIndex',    zeros(m,1), ...
...'remCodePhase', zeros(m,1), ... %可以不存，注释掉在前面加...
'codeFreq',     zeros(m,1), ...
...'remCarrPhase', zeros(m,1), ... %可以不存
'carrFreq',     zeros(m,1), ...
'I_Q',          zeros(m,6), ...
'disc',         zeros(m,3), ...
'std',          zeros(m,2));
trackResults_GPS_A = repmat(trackResults_GPS_A, svN_GPS,1);
for k=1:svN_GPS
    trackResults_GPS_A(k).PRN = svList_GPS(k);
end

%% 创建北斗处理通道
%--创建日志文件
logFile_BDS_A = [curr_path,'\log_BDS_A.txt']; %日志文件
logID_BDS_A = fopen(logFile_BDS_A, 'w');

%--卫星列表
svList_BDS = [19;20;22;36;37;38];
svN_BDS = length(svList_BDS);

%--创建通道
channels_BDS_A = cell(svN_BDS,1); %创建cell
for k=1:svN_BDS %创建跟踪通道对象
    channels_BDS_A{k} = BDS_B1C_channel(sampleFreq, buffSize, svList_BDS(k), logID_BDS_A);
end

%--初始化通道
channels_BDS_A{1}.init([14319, -200], 0);
channels_BDS_A{2}.init([19294,-2450], 0);
channels_BDS_A{3}.init([29616, 2300], 0);
channels_BDS_A{4}.init([13406, 3300], 0);
channels_BDS_A{5}.init([15648,-2100], 0);
channels_BDS_A{6}.init([27633,-1900], 0);

%--通道输出存储空间
m = msToProcess + 10;
trackResults_BDS_A = struct('PRN',0, 'n',1, ...
'dataIndex',    zeros(m,1), ...
...'remCodePhase', zeros(m,1), ... %可以不存，注释掉在前面加...
'codeFreq',     zeros(m,1), ...
...'remCarrPhase', zeros(m,1), ... %可以不存
'carrFreq',     zeros(m,1), ...
'I_Q',          zeros(m,8), ...
'disc',         zeros(m,2), ...
'std',          zeros(m,2));
trackResults_BDS_A = repmat(trackResults_BDS_A, svN_BDS,1);
for k=1:svN_BDS
    trackResults_BDS_A(k).PRN = svList_BDS(k);
end

%% 打开文件，创建进度条
fileID = fopen(data_file, 'r');
fseek(fileID, round(sample_offset*4), 'bof'); %不取整可能出现文件指针移不过去
if int64(ftell(fileID))~=int64(sample_offset*4)
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
    buff(:,buffBlkPoint*buffBlkSize+(1:buffBlkSize)) = double(fread(fileID, [2,buffBlkSize], 'int16')); %取数据，行向量，往缓存里放
    buffBlkPoint = buffBlkPoint + 1;
    buffHead = buffBlkPoint * buffBlkSize;
    if buffBlkPoint==buffBlkNum
        buffBlkPoint = 0; %缓存从头开始
    end
    
    %% 更新接收机时间
    sampleFreq0 = sampleFreq * (1+deltaFreq); %真实采样频率
    ta = time_carry(ta + sample2dt(buffBlkSize, sampleFreq0));
    
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
                channels_GPS_A{k}.set_deltaFreq(deltaFreq); %更新通道频率误差，保证采样频率是准的
                trackDataHead = channels_GPS_A{k}.trackDataHead;
                trackDataTail = channels_GPS_A{k}.trackDataTail;
                if trackDataHead>trackDataTail
                    [I_Q, disc] = channels_GPS_A{k}.track(buff(:,trackDataTail:trackDataHead));
                else
                    [I_Q, disc] = channels_GPS_A{k}.track([buff(:,trackDataTail:end),buff(:,1:trackDataHead)]);
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
    
    %% BDS跟踪
    for k=1:svN_BDS
        if channels_BDS_A{k}.state~=0
            while 1
                % 判断是否有完整的跟踪数据
                if mod(buffHead-channels_BDS_A{k}.trackDataHead,buffSize)>(buffSize/2)
                    break
                end
                % 存跟踪结果（通道参数）
                n = trackResults_BDS_A(k).n;
                trackResults_BDS_A(k).dataIndex(n,:)    = channels_BDS_A{k}.dataIndex;
                % trackResults_BDS_A(k).remCodePhase(n,:) = channels_BDS_A{k}.remCodePhase;
                trackResults_BDS_A(k).codeFreq(n,:)     = channels_BDS_A{k}.codeFreq;
                % trackResults_BDS_A(k).remCarrPhase(n,:) = channels_BDS_A{k}.remCarrPhase;
                trackResults_BDS_A(k).carrFreq(n,:)     = channels_BDS_A{k}.carrFreq;
                % 基带处理
                channels_BDS_A{k}.set_deltaFreq(deltaFreq); %更新通道频率误差，保证采样频率是准的
                trackDataHead = channels_BDS_A{k}.trackDataHead;
                trackDataTail = channels_BDS_A{k}.trackDataTail;
                if trackDataHead>trackDataTail
                    [I_Q, disc] = channels_BDS_A{k}.track(buff(:,trackDataTail:trackDataHead));
                else
                    [I_Q, disc] = channels_BDS_A{k}.track([buff(:,trackDataTail:end),buff(:,1:trackDataHead)]);
                end
                channels_BDS_A{k}.parse; %可以注释掉，只跟踪不解析
                % 存跟踪结果（跟踪结果）
                trackResults_BDS_A(k).I_Q(n,:)          = I_Q;
                trackResults_BDS_A(k).disc(n,:)         = disc;
                trackResults_BDS_A(k).std(n,:)          = sqrt([channels_BDS_A{k}.varCode.D,channels_BDS_A{k}.varPhase.D]);
                trackResults_BDS_A(k).n                 = n + 1;
            end
        end
    end
    
    %% 定位
    
end

%% 关闭文件，关闭进度条
fclose(fileID);
fclose(logID_GPS_A);
fclose(logID_BDS_A);
close(f);

%% 删除空白数据
trackResults_GPS_A = trackResults_clean(trackResults_GPS_A);
trackResults_BDS_A = trackResults_clean(trackResults_BDS_A);

%% 打印通道日志
clc
disp('<--------GPS A-------->')
print_log(logFile_GPS_A, svList_GPS);
disp('<--------BDS A-------->')
print_log(logFile_BDS_A, svList_BDS);

%% 画图
plot_trackResults_GPS(trackResults_GPS_A, msToProcess, sampleFreq)
plot_trackResults_BDS(trackResults_BDS_A, msToProcess, sampleFreq)

%% 计时结束
toc

%% 画图函数
function plot_trackResults_GPS(trackResults, msToProcess, sampleFreq)
    for k=1:size(trackResults,1)
        if trackResults(k).n==1 %不画没跟踪的通道
            continue
        end

        % 建立坐标轴
        screenSize = get(0,'ScreenSize'); %获取屏幕尺寸
        if screenSize(3)==1920 %根据屏幕尺寸设置画图范围
            figure('Position', [390, 280, 1140, 670]);
        elseif screenSize(3)==1368 %SURFACE
            figure('Position', [114, 100, 1140, 670]);
        elseif screenSize(3)==1440 %小屏幕
            figure('Position', [150, 100, 1140, 670]);
        elseif screenSize(3)==1600 %T430
            figure('Position', [230, 100, 1140, 670]);
        else
            error('Screen size error!')
        end
        ax1 = axes('Position', [0.08, 0.4, 0.38, 0.53]);
        hold(ax1,'on');
        axis(ax1, 'equal');
        title(['PRN = ',num2str(trackResults(k).PRN)])
        ax2 = axes('Position', [0.53, 0.7 , 0.42, 0.25]);
        hold(ax2,'on');
        ax3 = axes('Position', [0.53, 0.38, 0.42, 0.25]);
        hold(ax3,'on');
        grid(ax3,'on');
        ax4 = axes('Position', [0.53, 0.06, 0.42, 0.25]);
        hold(ax4,'on');
        grid(ax4,'on');
        ax5 = axes('Position', [0.05, 0.06, 0.42, 0.25]);
        hold(ax5,'on');
        grid(ax5,'on');

        % 画图
        plot(ax1, trackResults(k).I_Q(1001:end,1),trackResults(k).I_Q(1001:end,4), 'LineStyle','none', 'Marker','.') %I/Q图
        plot(ax2, trackResults(k).dataIndex/sampleFreq, trackResults(k).I_Q(:,1))
        plot(ax4, trackResults(k).dataIndex/sampleFreq, trackResults(k).carrFreq, 'LineWidth',1.5) %载波频率
        plot(ax5, trackResults(k).dataIndex/sampleFreq, trackResults(k).disc(:,1))
        plot(ax5, trackResults(k).dataIndex/sampleFreq, trackResults(k).std(:,1))

        % 调整坐标轴
        set(ax2, 'XLim',[0,msToProcess/1000])
        set(ax3, 'XLim',[0,msToProcess/1000])
        set(ax4, 'XLim',[0,msToProcess/1000])
        set(ax5, 'XLim',[0,msToProcess/1000])
    end
end

function plot_trackResults_BDS(trackResults, msToProcess, sampleFreq)
    for k=1:size(trackResults,1)
        if trackResults(k).n==1 %不画没跟踪的通道
            continue
        end

        % 建立坐标轴
        screenSize = get(0,'ScreenSize'); %获取屏幕尺寸
        if screenSize(3)==1920 %根据屏幕尺寸设置画图范围
            figure('Position', [390, 280, 1140, 670]);
        elseif screenSize(3)==1368 %SURFACE
            figure('Position', [114, 100, 1140, 670]);
        elseif screenSize(3)==1440 %小屏幕
            figure('Position', [150, 100, 1140, 670]);
        elseif screenSize(3)==1600 %T430
            figure('Position', [230, 100, 1140, 670]);
        else
            error('Screen size error!')
        end
        ax1 = axes('Position', [0.08, 0.4, 0.38, 0.53]);
        hold(ax1,'on');
        axis(ax1, 'equal');
        title(['PRN = ',num2str(trackResults(k).PRN)])
        ax2 = axes('Position', [0.53, 0.7 , 0.42, 0.25]);
        hold(ax2,'on');
        ax3 = axes('Position', [0.53, 0.38, 0.42, 0.25]);
        hold(ax3,'on');
        grid(ax3,'on');
        ax4 = axes('Position', [0.53, 0.06, 0.42, 0.25]);
        hold(ax4,'on');
        grid(ax4,'on');
        ax5 = axes('Position', [0.05, 0.06, 0.42, 0.25]);
        hold(ax5,'on');
        grid(ax5,'on');

        % 画图
        plot(ax1, trackResults(k).I_Q(1001:end,7),trackResults(k).I_Q(1001:end,8), 'LineStyle','none', 'Marker','.') %I/Q图
        plot(ax2, trackResults(k).dataIndex/sampleFreq, trackResults(k).I_Q(:,8)) %Q
        plot(ax2, trackResults(k).dataIndex/sampleFreq, trackResults(k).I_Q(:,7)) %I
        plot(ax4, trackResults(k).dataIndex/sampleFreq, trackResults(k).carrFreq, 'LineWidth',1.5) %载波频率
        plot(ax5, trackResults(k).dataIndex/sampleFreq, trackResults(k).disc(:,1))
        plot(ax5, trackResults(k).dataIndex/sampleFreq, trackResults(k).std(:,1))

        % 调整坐标轴
        set(ax2, 'XLim',[0,msToProcess/1000])
        set(ax3, 'XLim',[0,msToProcess/1000])
        set(ax4, 'XLim',[0,msToProcess/1000])
        set(ax5, 'XLim',[0,msToProcess/1000])
    end
end