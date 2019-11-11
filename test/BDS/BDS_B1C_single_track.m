% 提前给定捕获结果，验证指定文件北斗B1C信号跟踪性能
% 内存不够运行会变慢
% 正常一个卫星跟踪20s时间花费9s

clear
clc

%% 数据文件
data_file = 'E:\GNSS data\B210_20190726_205109_ch1.dat';

%% 计时开始
tic

%% 创建日志文件
curr_full_path = mfilename('fullpath'); %当前代码完整路径，包含文件名
[curr_path, ~] = fileparts(curr_full_path); %提取当前代码所在路径
fclose('all'); %关闭之前打开的所有文件
log_file = [curr_path,'\log.txt']; %日志文件
logID = fopen(log_file, 'w'); %在当前代码路径下创建日志文件（时间顺序的日志）

%% 运行时间
msToProcess = 20*1*1000; %处理总时间
sample_offset = 0*4e6; %抛弃前多少个采样点
sampleFreq = 4e6; %接收机采样频率

%% 数据缓存
buffBlkNum = 40;                     %采样数据缓存块数量（要保证捕获时存储恰好从头开始）
buffBlkSize = 4000;                  %一个块的采样点数（1ms）
buffSize = buffBlkSize * buffBlkNum; %采样数据缓存大小
buff = zeros(2,buffSize);            %采样数据缓存，第一行I，第二行Q
buffBlkPoint = 0;                    %数据该往第几块存，从0开始
buffHead = 0;                        %最新数据的序号，buffBlkSize的倍数

%% 获取文件时间
tf = sscanf(data_file((end-22):(end-8)), '%4d%02d%02d_%02d%02d%02d')'; %数据文件开始采样时间（日期时间数组）
[tw, ts] = BDS_time(tf); %tw：BDT周数，ts：BDT周内秒数
ta = [ts,0,0] + sample2dt(sample_offset, sampleFreq); %初始化接收机时间（周内秒数），[s,ms,us]
ta = time_carry(round(ta,2)); %取整

%% 卫星列表
svList = 19;
% svList = [19;20;22;36;37;38];
svN = length(svList);

%% 为每颗可能见到的卫星分配跟踪通道
channels = cell(svN,1); %创建cell
for k=1:svN %创建跟踪通道对象
    channels{k} = BDS_B1C_channel(sampleFreq, buffSize, svList(k), logID);
end
% 根据捕获结果初始化通道
% channels{1}.init([14319, -200], 0);
% channels{2}.init([19294,-2450], 0);
% channels{3}.init([29616, 2300], 0);
% channels{4}.init([13406, 3300], 0);
% channels{5}.init([15648,-2100], 0);
% channels{6}.init([27633,-1900], 0);

channels{1}.init([14319, -200], 0); %同相
% channels{1}.init([19294,-2450], 0); %反向
% channels{1}.init([29616, 2300], 0);
% channels{1}.init([13406, 3300], 0);
% channels{1}.init([15648,-2100], 0);
% channels{1}.init([27633,-1900], 0);

%% 创建跟踪结果存储空间
trackResults = repmat(trackResult_struct(msToProcess), svN,1);
for k=1:svN
    trackResults(k).PRN = svList(k);
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
    ta = time_carry(ta + sample2dt(buffBlkSize, sampleFreq));
    
    %% 跟踪
    for k=1:svN
        if channels{k}.state~=0
            while 1
                % 判断是否有完整的跟踪数据
                if mod(buffHead-channels{k}.trackDataHead,buffSize)>(buffSize/2)
                    break
                end
                % 存跟踪结果（通道参数）
                n = trackResults(k).n;
                trackResults(k).dataIndex(n,:)    = channels{k}.dataIndex;
                trackResults(k).remCodePhase(n,:) = channels{k}.remCodePhase;
                trackResults(k).codeFreq(n,:)     = channels{k}.codeFreq;
                trackResults(k).remCarrPhase(n,:) = channels{k}.remCarrPhase;
                trackResults(k).carrFreq(n,:)     = channels{k}.carrFreq;
                % 基带处理
                trackDataHead = channels{k}.trackDataHead;
                trackDataTail = channels{k}.trackDataTail;
                if trackDataHead>trackDataTail
                    [I_Q, disc] = channels{k}.track(buff(:,trackDataTail:trackDataHead));
                else
                    [I_Q, disc] = channels{k}.track([buff(:,trackDataTail:end),buff(:,1:trackDataHead)]);
                end
                channels{k}.parse(ta); %可以注释掉，只跟踪不解析
                % 存跟踪结果（跟踪结果）
                trackResults(k).I_Q(n,:)          = I_Q;
                trackResults(k).disc(n,:)         = disc;
                trackResults(k).std(n,:)          = sqrt([channels{k}.varCode.D,channels{k}.varPhase.D]);
                trackResults(k).n                 = n + 1;
            end
        end
    end
    
end

%% 关闭文件，关闭进度条
fclose(fileID);
fclose(logID);
close(f);

%% 删除空白数据
for k=1:svN
    trackResults(k) = trackResult_clean(trackResults(k));
end

%% 打印通道日志
clc
listing = dir(log_file); %日志文件信息
if listing.bytes~=0 %如果日志文件不是空才打印
    print_log(log_file, svList);
end

%% 画图
for k=1:svN
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
    title(['PRN = ',num2str(svList(k))])
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
    
    % 调整坐标轴
    set(ax2, 'XLim',[0,msToProcess/1000])
    set(ax3, 'XLim',[0,msToProcess/1000])
    set(ax4, 'XLim',[0,msToProcess/1000])
    set(ax5, 'XLim',[0,msToProcess/1000])
end

%% 计时结束
toc

%% 函数
function trackResult = trackResult_struct(m)
% 跟踪结果结构体
    trackResult.PRN = 0;
    trackResult.n = 1; %指向当前存储的行号
    
    trackResult.dataIndex     = zeros(m,1); %码周期开始采样点在原始数据文件中的位置
    trackResult.remCodePhase  = zeros(m,1); %码周期开始采样点的码相位，码片
    trackResult.codeFreq      = zeros(m,1); %码频率
    trackResult.remCarrPhase  = zeros(m,1); %码周期开始采样点的载波相位，周
    trackResult.carrFreq      = zeros(m,1); %载波频率
    trackResult.I_Q           = zeros(m,8); %[I_P,I_E,I_L,Q_P,Q_E,Q_L, I,Q]
    trackResult.disc          = zeros(m,2); %鉴相器输出
    trackResult.std           = zeros(m,2); %鉴相器输出标准差
end

function trackResult = trackResult_clean(trackResult)
% 清理跟踪结果中的空白空间
    n = trackResult.n;
    
    trackResult.dataIndex(n:end,:)    = [];
    trackResult.remCodePhase(n:end,:) = [];
    trackResult.codeFreq(n:end,:)     = [];
    trackResult.remCarrPhase(n:end,:) = [];
    trackResult.carrFreq(n:end,:)     = [];
    trackResult.I_Q(n:end,:)          = [];
    trackResult.disc(n:end,:)         = [];
    trackResult.std(n:end,:)          = [];
end