% 一颗卫星跟踪20s耗时约9s

clear
clc

%%
tic

%% 文件名
file_path = 'E:\GNSS data\B210_20190726_205109_ch1.dat';

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

%% 卫星列表
svList = 19;
% svList = [19;20;22;36;37;38];
svN = length(svList);

%% 为每颗可能见到的卫星分配跟踪通道
channels = repmat(BDS_B1C_channel_struct(), svN,1); %只分配了场，所有信息都为空
for k=1:svN
    channels(k).PRN = svList(k); %每个通道的卫星号
    channels(k).state = 0; %状态未激活
end
% 根据捕获结果初始化通道
channels(1) = BDS_B1C_channel_init(channels(1), [14319-0, -200], 0, sampleFreq); %当捕获误差差2个采样点时，会误锁到码自相关函数的副峰；4M次采样频率下，码捕获误差最大为0.125码片
% channels(2) = BDS_B1C_channel_init(channels(2), [19294,-2450], 0, sampleFreq);
% channels(3) = BDS_B1C_channel_init(channels(3), [29616, 2300], 0, sampleFreq);
% channels(4) = BDS_B1C_channel_init(channels(4), [13406, 3300], 0, sampleFreq);
% channels(5) = BDS_B1C_channel_init(channels(5), [15648,-2100], 0, sampleFreq);
% channels(6) = BDS_B1C_channel_init(channels(6), [27633,-1900], 0, sampleFreq);

%% 创建跟踪结果存储空间
trackResults = repmat(trackResult_struct(msToProcess), svN,1);
for k=1:svN
    trackResults(k).PRN = svList(k);
end

%% 打开文件，创建进度条
fclose('all');
fileID = fopen(file_path, 'r');
fseek(fileID, round(sample_offset*4), 'bof'); %不取整可能出现文件指针移不过去
if int64(ftell(fileID))~=int64(sample_offset*4)
    error('Sample offset error!');
end
f = waitbar(0, ['0s/',num2str(msToProcess/1000),'s']);

%% 信号处理
for t=1:msToProcess
    if mod(t,1000)==0 %1s步进
        waitbar(t/msToProcess, f, [num2str(t/1000),'s/',num2str(msToProcess/1000),'s']);
    end
    
    buff(:,buffBlkPoint*buffBlkSize+(1:buffBlkSize)) = double(fread(fileID, [2,buffBlkSize], 'int16')); %取数据，行向量，往缓存里放
    buffBlkPoint = buffBlkPoint + 1;
    buffHead = buffBlkPoint * buffBlkSize;
    if buffBlkPoint==buffBlkNum
        buffBlkPoint = 0; %缓存从头开始
    end
    
    for k=1:svN
        if channels(k).state~=0
            while 1
                % 判断是否有完整的跟踪数据
                if mod(buffHead-channels(k).trackDataHead,buffSize)>(buffSize/2)
                    break
                end
                % 存跟踪结果（通道参数）
                n = trackResults(k).n;
                trackResults(k).dataIndex(n,:)    = channels(k).dataIndex;
                trackResults(k).remCodePhase(n,:) = channels(k).remCodePhase;
                trackResults(k).codeFreq(n,:)     = channels(k).codeFreq;
                trackResults(k).remCarrPhase(n,:) = channels(k).remCarrPhase;
                trackResults(k).carrFreq(n,:)     = channels(k).carrFreq;
                % 基带处理
                trackDataHead = channels(k).trackDataHead;
                trackDataTail = channels(k).trackDataTail;
                if trackDataHead>trackDataTail
                    [channels(k), I_Q, disc] = ...
                        BDS_B1C_track(channels(k), sampleFreq, buffSize, buff(:,trackDataTail:trackDataHead));
                else
                    [channels(k), I_Q, disc] = ...
                        BDS_B1C_track(channels(k), sampleFreq, buffSize, [buff(:,trackDataTail:end),buff(:,1:trackDataHead)]);
                end
                % 存跟踪结果（跟踪结果）
                trackResults(k).I_Q(n,:)          = I_Q;
                trackResults(k).disc(n,:)         = disc;
                trackResults(k).n                 = n + 1;
            end
        end
    end
end

%% 关闭文件，关闭进度条
fclose(fileID);
close(f);

%% 删除空白数据
for k=1:svN
    trackResults(k) = trackResult_clean(trackResults(k));
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
    plot(ax1, trackResults(k).I_Q(1001:end,1),trackResults(k).I_Q(1001:end,4), 'LineStyle','none', 'Marker','.') %I/Q图
    plot(ax2, trackResults(k).dataIndex/sampleFreq, trackResults(k).I_Q(:,4)) %I_Q图
    plot(ax2, trackResults(k).dataIndex/sampleFreq, trackResults(k).I_Q(:,1)) %I_P图
    plot(ax4, trackResults(k).dataIndex/sampleFreq, trackResults(k).carrFreq, 'LineWidth',1.5) %载波频率
    
    % 调整坐标轴
    set(ax2, 'XLim',[0,msToProcess/1000])
    set(ax3, 'XLim',[0,msToProcess/1000])
    set(ax4, 'XLim',[0,msToProcess/1000])
    set(ax5, 'XLim',[0,msToProcess/1000])
end

%%
toc