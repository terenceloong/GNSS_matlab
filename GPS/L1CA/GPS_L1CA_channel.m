classdef GPS_L1CA_channel < GPS_L1CA_track

    % 只有类成员可以修改属性值
    properties (GetAccess = public, SetAccess = private)
        state           %通道状态（数字）
        msgStage        %电文解析阶段（字符）
        msgCnt          %电文解析计数器
        I0              %上次I路积分值（用于比特同步）
        bitSyncTable    %比特同步统计表
        bitBuff         %比特缓存
        frameBuff       %帧缓存
        frameBuffPoint  %帧缓存指针
        ephemeris       %星历
        ion             %电离层校正参数
    end %end properties
    
    properties (Constant = true)
        frameHead = [1,-1,-1,-1,1,-1,1,1] %帧头
    end
    
    methods
        %% 构造
        function obj = GPS_L1CA_channel(sampleFreq, buffSize, PRN, logID)
            obj = obj@GPS_L1CA_track(sampleFreq, buffSize, PRN, logID);
            obj.state = 0;
        end
        
        %% 初始化
        function init(obj, acqResult, n)
            init@GPS_L1CA_track(obj, acqResult, n) %调用父类同名方法
            obj.state = 1;
            obj.msgStage = 'I';
            obj.msgCnt = 0;
            obj.I0 = 0;
            obj.bitSyncTable = zeros(1,20); %一个比特持续20ms
            obj.bitBuff = zeros(1,20); %最多用20个
            obj.frameBuff = zeros(1,1502); %一帧数据1500比特
            obj.frameBuffPoint = 0;
            obj.ephemeris = NaN(26,1);
            obj.ion = NaN(8,1);
        end
        
        %% 电文解析
        % 从捕获到进入比特同步要500ms
        % 比特同步过程要2s，开始寻找帧头要多一点时间，等待比特边界到达
        % 比特同步后开始寻找帧头，接收一个完整子帧才能校验帧头，从寻找帧头到解析星历至少6s，最多12s
        % 验证帧头后就可以确定码发射时间
        % 星历解析30s一次
        % 比特同步后可以增加积分时间
        function parse(obj)
            obj.msgCnt = obj.msgCnt + 1; %计数加1
            switch obj.msgStage %I,B,W,H,C,E
                case 'I' %<<====空闲
                    if obj.msgCnt==500 %跟踪500ms
                        obj.msgCnt = 0; %计数器清零
                        obj.msgStage = 'B'; %进入比特同步阶段
                        fprintf(obj.logID, '%2d: Start bit synchronization at %.8fs\r\n', ...
                        obj.PRN, obj.dataIndex/obj.sampleFreq);
                    end
                case 'B' %<<====比特同步
                    if obj.I0*obj.I<0 %发现电平翻转
                        index = mod(obj.msgCnt-1,20) + 1;
                        obj.bitSyncTable(index) = obj.bitSyncTable(index) + 1; %统计表中的对应位加1
                    end
                    obj.I0 = obj.I;
                    if obj.msgCnt==2000 %2s后检验统计表，此时有100个比特
                        obj.I0 = 0;
                        if max(obj.bitSyncTable)>10 && (sum(obj.bitSyncTable)-max(obj.bitSyncTable))<=2
                        % 比特同步成功，确定电平翻转位置（电平翻转大都发生在一个点上）
                            [~,obj.msgCnt] = max(obj.bitSyncTable); %将计数值设为同步表最大值的索引
                            obj.bitSyncTable = zeros(1,20); %比特同步统计表清零
                            obj.msgCnt = -obj.msgCnt + 1; %如果索引为1，下个I路积分值就为比特开始处
                            if obj.msgCnt==0
%                                 obj.set_timeInt(10); %增加积分时间
                                obj.msgStage = 'H'; %进入寻找帧头阶段
                                fprintf(obj.logID, '%2d: Start find head at %.8fs\r\n', ...
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
%                         obj.set_timeInt(10); %增加积分时间
                        obj.msgStage = 'H'; %进入寻找帧头阶段
                        fprintf(obj.logID, '%2d: Start find head at %.8fs\r\n', ...
                        obj.PRN, obj.dataIndex/obj.sampleFreq);
                    end
                otherwise %<<====已经完成比特同步
                    obj.bitBuff(obj.msgCnt) = obj.I; %往比特缓存中存数
                    if obj.msgCnt==obj.pointInt %跟踪完一个比特
                        obj.msgCnt = 0; %计数器清零
                        obj.frameBuffPoint = obj.frameBuffPoint + 1; %帧缓存指针加1
                        obj.frameBuff(obj.frameBuffPoint) = (double(sum(obj.bitBuff(1:obj.pointInt))>0) - 0.5) * 2; %存储比特值，±1
                        switch obj.msgStage
                            case 'H' %<<====寻找帧头
                                if obj.frameBuffPoint>=10 %至少有10个比特，前两个用来校验
                                    if abs(sum(obj.frameBuff(obj.frameBuffPoint+(-7:0)).*obj.frameHead))==8 %检测到疑似帧头
                                        obj.frameBuff(1:10) = obj.frameBuff(obj.frameBuffPoint+(-9:0)); %将帧头提前
                                        obj.frameBuffPoint = 10;
                                        obj.msgStage = 'C'; %进入校验帧头阶段
                                    end
                                    if obj.frameBuffPoint==1502
                                        obj.frameBuffPoint = 0;
                                    end
                                end
                            case 'C' %<<====校验帧头
                                if obj.frameBuffPoint==310 %存储了一个子帧，2+300+8
                                    if GPS_L1CA_check(obj.frameBuff(1:32))==1 && ...
                                       GPS_L1CA_check(obj.frameBuff(31:62))==1 && ...
                                       abs(sum(obj.frameBuff(303:310).*obj.frameHead))==8 %校验通过
                                        % 获取电文时间
                                        % frameBuff(32)为上一字的最后一位，校验时控制电平翻转，为1表示翻转，为0表示不翻转，参见ICD-GPS最后几页
                                        bits = -obj.frameBuff(32) * obj.frameBuff(33:49); %电平翻转，31~47比特
                                        bits = dec2bin(bits>0)'; %±1数组转化为01字符串
                                        TOW = bin2dec(bits); %01字符串转换为十进制数
                                        obj.set_ts0((TOW*6+0.16)*1000); %设置伪码周期开始时间，ms，0.16=8/50
                                        % TOW为下一子帧开始时间，参见《北斗/GPS双模软件接收机原理与实现技术》96页
                                        obj.msgStage = 'E'; %进入解析星历阶段
                                        fprintf(obj.logID, '%2d: Start parse ephemeris at %.8fs\r\n', ...
                                        obj.PRN, obj.dataIndex/obj.sampleFreq);
                                    else %校验未通过
                                        for k=11:310 %检查其他比特中有没有帧头
                                            if abs(sum(obj.frameBuff(k+(-7:0)).*obj.frameHead))==8 %检测到疑似帧头
                                                obj.frameBuff(1:320-k) = obj.frameBuff(k-9:310); %将帧头和后面的比特提前，320-k = 310-(k-9)+1
                                                obj.frameBuffPoint = 320-k; %表示帧缓存中有多少个数
                                                break
                                            end
                                        end
                                        if obj.frameBuffPoint==310 %没检测到疑似帧头
                                            obj.frameBuff(1:9) = obj.frameBuff(302:310); %将未检测的比特提前
                                            obj.frameBuffPoint = 9;
                                            obj.msgStage = 'H'; %再次寻找帧头
                                        end
                                    end
                                end
                            case 'E' %<<====解析星历
                                if obj.frameBuffPoint==1502 %跟踪完5帧
                                    [ephemeris0, ion0] = GPS_L1CA_ephemeris_parse(obj.frameBuff); %解析星历
                                    if ~isempty(ephemeris0) %星历解析成功
                                        if ephemeris0(2)==ephemeris0(3) %更新星历
                                            fprintf(obj.logID, '%2d: Ephemeris is parsed at %.8fs\r\n', ...
                                            obj.PRN, obj.dataIndex/obj.sampleFreq);
                                            obj.ephemeris = ephemeris0;
                                            obj.state = 2;
                                            if ~isempty(ion0) %电离层校正参数
                                                obj.ion = ion0;
                                            end
                                        else %星历改变
                                            fprintf(obj.logID, '%2d: ***Ephemeris changes at %.8fs, IODC=%d, IODE=%d\r\n', ...
                                            obj.PRN, obj.dataIndex/obj.sampleFreq, ephemeris0(2), ephemeris0(3));
                                        end
                                    else  %解析星历错误
                                        fprintf(obj.logID, '%2d: ***Ephemeris error at %.8fs\r\n', ...
                                        obj.PRN, obj.dataIndex/obj.sampleFreq);
                                    end
                                    obj.frameBuff(1:2) = obj.frameBuff(1501:1502); %将最后两个比特提前
                                    obj.frameBuffPoint = 2;
                                end
                        end
                    end
            end
        end
        
    end %end methods
    
end %end classdef