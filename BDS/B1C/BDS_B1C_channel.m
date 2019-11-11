classdef BDS_B1C_channel < BDS_B1C_track
% 继承于跟踪类，做导航电文解析
% https://blog.csdn.net/qq_43575267/article/details/93778020
    
    % 只有类成员可以修改属性值
    properties (GetAccess = public, SetAccess = private)
        state           %通道状态（数字）
        msgStage        %电文解析阶段（字符）
        msgCnt          %电文解析计数器
        Q0              %上次导频分量积分值（用于比特同步）
        bitSyncTable    %比特同步统计表
        bitBuff         %比特缓存
        frameBuff       %帧缓存
        frameBuffPoint  %帧缓存指针
        ephemeris       %星历
        BDT_GPS         %北斗时与GPS时同步参数
        BDT_Galileo     %北斗时与Galileo时同步参数
        BDT_GLONASS     %北斗时与GLONASS时同步参数
    end
    
    methods
        %% 构造
        function obj = BDS_B1C_channel(sampleFreq, buffSize, PRN, logID)
            obj = obj@BDS_B1C_track(sampleFreq, buffSize, PRN, logID);
            obj.state = 0;
        end
        
        %% 初始化
        function init(obj, acqResult, n)
            init@BDS_B1C_track(obj, acqResult, n) %调用父类同名方法
            obj.state = 1;
            obj.msgStage = 'I';
            obj.msgCnt = 0;
            obj.Q0 = 0;
            obj.bitSyncTable = zeros(1,10); %一个比特持续10ms
            obj.bitBuff = zeros(1,10); %最多用10个
            obj.frameBuff = zeros(1,1800); %一帧数据1800比特
            obj.frameBuffPoint = 0;
            obj.ephemeris = NaN(30,1);
            obj.BDT_GPS = NaN(5,1);
            obj.BDT_Galileo = NaN(5,1);
            obj.BDT_GLONASS = NaN(5,1);
        end
        
        %% 电文解析
        % 从捕获到进入比特同步要200ms
        % 比特同步过程要1s，开始帧同步要多一点时间，等待比特边界到达
        % 帧同步过程要500ms，等到下一帧开始才进入星历解析
        % 星历解析18s一次
        function parse(obj, ta) %传入接收机时间，在帧同步时用来确定伪码周期开始时间
            obj.msgCnt = obj.msgCnt + 1; %计数加1
            switch obj.msgStage %I,B,W,F,H,E
                case 'I' %<<====空闲
                    if obj.msgCnt==200 %跟踪200ms
                        obj.msgCnt = 0; %计数器清零
                        obj.msgStage = 'B'; %进入比特同步阶段
                        fprintf(obj.logID, '%2d: Start bit synchronization at %.8fs\r\n', ...
                        obj.PRN, obj.dataIndex/obj.sampleFreq);
                    end
                case 'B' %<<====比特同步
                    if obj.Q0*obj.Q<0 %发现电平翻转
                        index = mod(obj.msgCnt-1,10) + 1;
                        obj.bitSyncTable(index) = obj.bitSyncTable(index) + 1; %统计表中的对应位加1
                    end
                    obj.Q0 = obj.Q;
                    if obj.msgCnt==1000 %1s后检验统计表，此时有100个比特
                        if max(obj.bitSyncTable)>10 && (sum(obj.bitSyncTable)-max(obj.bitSyncTable))<=2
                        % 比特同步成功，确定电平翻转位置（电平翻转大都发生在一个点上）
                            [~,obj.msgCnt] = max(obj.bitSyncTable); %将计数值设为同步表最大值的索引
                            obj.bitSyncTable = zeros(1,10); %比特同步统计表清零
                            obj.msgCnt = -obj.msgCnt + 1; %如果索引为1，下个Q路积分值就为比特开始处
                            if obj.msgCnt==0
                                obj.msgStage = 'F'; %进入帧同步阶段
                                fprintf(obj.logID, '%2d: Start frame synchronization at %.8fs\r\n', ...
                                obj.PRN, obj.dataIndex/obj.sampleFreq);
                            else
                                obj.msgStage = 'W'; %等待比特头
                            end
                        else
                        % 比特同步失败，关闭通道
                            obj.state = 0;
                            fprintf(obj.logID, '%2d: ***Bit synchronization failed at %.8fs\r\n', ...
                            obj.PRN, obj.dataIndex/obj.sampleFreq);
                        end
                    end
                case 'W' %<<====等待比特头
                    if obj.msgCnt==0
                        obj.msgStage = 'F'; %进入帧同步阶段
                        fprintf(obj.logID, '%2d: Start frame synchronization at %.8fs\r\n', ...
                        obj.PRN, obj.dataIndex/obj.sampleFreq);
                    end
                case 'F' %<<====帧同步
                    obj.bitBuff(obj.msgCnt) = obj.Q; %往比特缓存中存数，导频子码
                    if obj.msgCnt==10 %跟踪完一个比特
                        obj.msgCnt = 0; %计数器清零
                        obj.frameBuffPoint = obj.frameBuffPoint + 1; %帧缓存指针加1
                        obj.frameBuff(obj.frameBuffPoint) = (double(sum(obj.bitBuff)>0) - 0.5) * 2; %存储比特值，±1
                        % 采集一段时间导频子码，确定其在子码序列中的位置
                        if obj.frameBuffPoint==50 %存了50个比特
                            R = zeros(1,1800); %50个比特在子码序列不同位置的相关结果
                            code = [obj.codeSub, obj.codeSub(1:49)];
                            x = obj.frameBuff(1:50)'; %列向量
                            for k=1:1800
                                R(k) = code(k:k+49) * x;
                            end
                            [Rmax, index] = max(abs(R)); %寻找相关结果的最大值
                            if Rmax==50 %最大相关值正确
                                ta = ta(1)*1000 + ta(2); %当前接收机时间，ms
                                td = (index+50) *10; %帧内ms
                                t0 = round((ta-td)/18000); %指向最近的整18s
                                obj.set_ts0(t0*18000+td); %设置伪码周期开始时间
                                obj.start_pure_PLL(index+50, R(index)<0); %启动纯锁相环
                                obj.frameBuffPoint = mod(index+49,1800); %帧缓存指针移动
                                if obj.frameBuffPoint==0
                                    obj.msgStage = 'E'; %进入解析星历阶段
                                    fprintf(obj.logID, '%2d: Start parse ephemeris at %.8fs\r\n', ...
                                    obj.PRN, obj.dataIndex/obj.sampleFreq);
                                else
                                    obj.msgStage = 'H'; %等待帧头
                                end
                            else %最大相关值错误
                                obj.frameBuffPoint = 0; %帧缓存指针归位
                                obj.msgStage = 'B'; %返回比特同步阶段
                                fprintf(obj.logID, '%2d: ***Frame synchronization failed at %.8fs\r\n', ...
                                obj.PRN, obj.dataIndex/obj.sampleFreq);
                            end
                        end
                    end
                case 'H' %<<====等待帧头
                    if obj.msgCnt==10 %跟踪完一个比特
                        obj.msgCnt = 0; %计数器清零
                        obj.frameBuffPoint = obj.frameBuffPoint + 1; %帧缓存指针加1
                        if obj.frameBuffPoint==1800
                            obj.frameBuffPoint = 0; %帧缓存指针归位
                            obj.msgStage = 'E'; %进入解析星历阶段
                            fprintf(obj.logID, '%2d: Start parse ephemeris at %.8fs\r\n', ...
                            obj.PRN, obj.dataIndex/obj.sampleFreq);
                        end
                    end
                case 'E' %<<====解析星历
                    obj.bitBuff(obj.msgCnt) = obj.I; %往比特缓存中存数，导航电文
                    if obj.msgCnt==10 %跟踪完一个比特
                        obj.msgCnt = 0; %计数器清零
                        obj.frameBuffPoint = obj.frameBuffPoint + 1; %帧缓存指针加1
                        obj.frameBuff(obj.frameBuffPoint) = (double(sum(obj.bitBuff)>0) - 0.5) * 2; %存储比特值，±1
                        if obj.frameBuffPoint==1800 %存了1800个比特（一帧）
                            obj.frameBuffPoint = 0;
                            [ephemeris0, sf3] = BDS_CNAV1_ephemeris_parse(obj.frameBuff);
                            if ~isempty(ephemeris0) %星历解析成功
                                fprintf(obj.logID, '%2d: Ephemeris is parsed at %.8fs\r\n', ...
                                obj.PRN, obj.dataIndex/obj.sampleFreq);
                                obj.ephemeris = ephemeris0;
                                obj.state = 2;
                                if sf3.pageID==3
                                    switch sf3.BGTO(1) %设置BDT-GNSS时间同步参数
                                        case 1
                                            obj.BDT_GPS = sf3.BGTO(2:6);
                                        case 2
                                            obj.BDT_Galileo = sf3.BGTO(2:6);
                                        case 3
                                            obj.BDT_GLONASS = sf3.BGTO(2:6);
                                    end
                                end
                            else %解析星历错误
                                fprintf(obj.logID, '%2d: ***Ephemeris error at %.8fs\r\n', ...
                                obj.PRN, obj.dataIndex/obj.sampleFreq);
                            end
                        end
                    end
                otherwise
            end
        end
        
    end %end methods
    
end %end classdef