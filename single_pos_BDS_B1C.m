% 北斗B1C单天线定位
% 如果时钟不反馈修正，钟差发散，钟频差常值
% 如果只修钟频，钟频差为0，钟差在短时间应该不飘，要是飘了，说明钟频差算的不对
% 钟差、钟频差全修，钟差和钟频差都是0
% 钟频差对载波多普勒测量的影响主要来源于下变频时频率的偏移，而不是跟踪阶段采样频率不准引起的本地载波频率误差
% 对于3e-9的钟频差，1575.42e6Hz载波下变频频率偏移约4.73Hz
% 对于生成3kHz的本地载波，采样频率误差引起的本地载波测量误差约为1e-5Hz，可以忽略不计
% 定位时间间隔会影响定位噪声特性

% 处理数据例子：F:\GNSS data\20190726\B210_20190726_205109_ch1.dat

clear %清变量
clc %清屏幕
fclose('all'); %清文件

%% 数据文件
%----对话框选择文件
default_path = fileread('.\temp\path_data.txt'); %数据文件所在默认路径
[file, path] = uigetfile([default_path,'\*.dat'], '选择GNSS数据文件'); %文件选择对话框
if file==0
    disp('Invalid file!');
    return
end
if strcmp(file(1:4),'B210')==0
    error('File error!');
end
data_file = [path, file];
data_file_A = data_file;
%----指定文件名
% data_file_A = 'F:\GNSS data\20190726\B210_20190726_205109_ch1.dat';

%% 运行时间
msToProcess = 60*1*1000; %处理总时间
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

%--接收机时间 (BDS时间)
tf = sscanf(data_file_A((end-22):(end-8)), '%4d%02d%02d_%02d%02d%02d')'; %数据文件开始采样时间（日期时间数组）
[tw, ts] = BDS_time(tf); %tw：BDT周数，ts：BDT周内秒数
ta = [ts,0,0] + sample2dt(sample_offset, sampleFreq); %初始化接收机时间（周内秒数），[s,ms,us]
ta = time_carry(round(ta,2)); %取整

%--接收机状态
receiverState = 0; %接收机状态，0表示未初始化，时间还不对，1表示时间已经校正
deltaFreq = 0; %时钟差，理解为百分比，如果差1e-9，生成1500e6Hz的波会差1.5Hz
dtpos = 10; %定位时间间隔，ms
tp = [ta(1),0,0]; %tp为下次定位时间
tp(2) = (floor(ta(2)/dtpos)+1) * dtpos; %加到下个整目标时间
tp = time_carry(tp); %进位

%% 创建北斗处理通道
%--创建日志文件
logFile_BDS_A = '.\temp\log_BDS_A.txt'; %日志文件
logID_BDS_A = fopen(logFile_BDS_A, 'w');

%--捕获一次（临时）
acqResults_BDS = BDS_B1C_acq(data_file_A, sample_offset, 1);
svList_BDS = acqResults_BDS(~isnan(acqResults_BDS(:,2)),1);
acqResults_BDS(isnan(acqResults_BDS(:,2)),:) = [];
acqResults_BDS(:,1) = [];
svN_BDS = length(svList_BDS);

%--卫星列表
% svList_BDS = [19;20;22;36;37;38];
% svN_BDS = length(svList_BDS);

%--创建通道
channels_BDS_A = cell(svN_BDS,1); %创建cell
for k=1:svN_BDS %创建跟踪通道对象
    channels_BDS_A{k} = BDS_B1C_channel(sampleFreq, buffSize, svList_BDS(k), logID_BDS_A);
end

%--初始化通道
for k=1:svN_BDS
    channels_BDS_A{k}.init(acqResults_BDS(k,1:2), 0);
end
% channels_BDS_A{1}.init([14319, -200], 0);
% channels_BDS_A{2}.init([19294,-2450], 0);
% channels_BDS_A{3}.init([29616, 2300], 0);
% channels_BDS_A{4}.init([13406, 3300], 0);
% channels_BDS_A{5}.init([15648,-2100], 0);
% channels_BDS_A{6}.init([27633,-1900], 0);

%--通道输出存储空间
m = msToProcess + 10;
trackResults_BDS = struct('PRN',0, 'n',1, ...
'dataIndex',    zeros(m,1), ...
...'remCodePhase', zeros(m,1), ... %可以不存，注释掉在前面加...
'codeFreq',     zeros(m,1), ...
...'remCarrPhase', zeros(m,1), ... %可以不存
'carrFreq',     zeros(m,1), ...
'I_Q',          zeros(m,8), ...
'disc',         zeros(m,2), ...
'std',          zeros(m,2));
trackResults_BDS_A = repmat(trackResults_BDS, svN_BDS,1);
clearvars trackResults_BDS
for k=1:svN_BDS
    trackResults_BDS_A(k).PRN = svList_BDS(k);
end

%% 接收机输出存储空间
m = msToProcess/dtpos + 100;
output.n        = 1; %当前存储行
output.ta       = NaN(m,1); %接收机时间，s
output.state    = NaN(m,1); %接收机状态
output.pos      = NaN(m,8); %定位，[位置、速度、钟差、钟频差]
output.sv_BDS_A = NaN(svN_BDS,10,m); %北斗卫星信息，[位置、伪距、速度、伪距率、码鉴相器方差、载波鉴相器方差]
output.df       = NaN(m,1); %估计出的钟频差

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
    
    %% 更新接收机时间
    sampleFreq0 = sampleFreq * (1+deltaFreq); %真实采样频率
    ta = time_carry(ta + sample2dt(buffBlkSize, sampleFreq0));
    
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
                    [I_Q, disc] = channels_BDS_A{k}.track(buff_A(:,trackDataTail:trackDataHead));
                else
                    [I_Q, disc] = channels_BDS_A{k}.track([buff_A(:,trackDataTail:end),buff_A(:,1:trackDataHead)]);
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
    dtp = (ta(1)-tp(1)) + (ta(2)-tp(2))/1e3 + (ta(3)-tp(3))/1e6;
    if dtp>=0
        % 测量北斗卫星信息
        sv_BDS_A = NaN(svN_BDS,10);
        for k=1:svN_BDS
            if channels_BDS_A{k}.state==2
                dn = mod(buffHead-channels_BDS_A{k}.trackDataTail+1, buffSize) - 1;
                dtc = dn / sampleFreq0;
                dt = dtc - dtp;
                codePhase = channels_BDS_A{k}.remCodePhase + channels_BDS_A{k}.codeNco*dt;
                ts0 = [floor(channels_BDS_A{k}.ts0/1e3), mod(channels_BDS_A{k}.ts0,1e3), 0] + [0, floor(codePhase/2046), mod(codePhase/2046,1)*1e3]; %北斗考虑子载波时码频率2.046e6Hz
                [sv_BDS_A(k,1:8),~] = BDS_CNAV1_ephemeris_rho(channels_BDS_A{k}.ephemeris, tp, ts0);
                sv_BDS_A(k,8) = -(channels_BDS_A{k}.carrFreq/1575.42e6 + deltaFreq) * 299792458;
                sv_BDS_A(k,8) = sv_BDS_A(k,8) + channels_BDS_A{k}.ephemeris(26)*299792458;
                sv_BDS_A(k,9)  = channels_BDS_A{k}.varCode.D;
                sv_BDS_A(k,10) = channels_BDS_A{k}.varPhase.D;
            end
        end
        % 定位
        sv = sv_BDS_A;
        sv(isnan(sv(:,1)),:) = []; %删除无效卫星
        pos = pos_solve_weight(sv(:,1:8), sv(:,9));
        % 时钟修正
%         if receiverState==1 && ~isnan(pos(7))
%             deltaFreq = deltaFreq + 10*pos(8)*dtpos/1000; %钟频差累加
%             ta = ta - sec2smu(10*pos(7)*dtpos/1000); %时钟修正（可以不用进位，在下次更新时进位）
%         end
        % 存储输出
        n = output.n;
        output.ta(n) = tp(1) + tp(2)/1e3 + tp(3)/1e6;
        output.state(n) = receiverState;
        output.pos(n,:) = pos;
        output.sv_BDS_A(:,:,n) = sv_BDS_A;
        output.df(n) = deltaFreq;
        output.n = n + 1;
        % 检查接收机状态
        if receiverState==0 && ~isnan(pos(7))
            if abs(pos(7))>0.1e-3 %钟差大于0.1ms，修正接收机时间
                ta = ta - sec2smu(pos(7)); %时钟修正
                ta = time_carry(ta);
                tp(1) = ta(1); %更新下次定位时间
                tp(2) = (floor(ta(2)/dtpos)+1) * dtpos;
                tp = time_carry(tp);
            else %钟差小于0.1ms，初始化结束
                receiverState = 1;
            end
        end
        % 更新下次定位时间
        tp = time_carry(tp + [0,dtpos,0]);
    end
    
end

%% 关闭文件，关闭进度条
fclose(fileID_A);
fclose(logID_BDS_A);
close(f);

%% 删除空白数据
trackResults_BDS_A = clean_trackResults(trackResults_BDS_A);
output = clean_receiverOutput(output);

%% 打印通道日志
clc
disp('<--------BDS A-------->')
print_log(logFile_BDS_A, svList_BDS);

%% 清除变量
clearvars -except sampleFreq msToProcess tf ...
                  svList_BDS channels_BDS_A trackResults_BDS_A ...
                  output

%% 计时结束
toc

%% 对话框
answer = questdlg('Plot track result?', ...
	              'Finish', ...
	              'Yes','No','No');
switch answer
    case 'Yes'
        plot_track_BDS_A
    case 'No'
end
clearvars answer