classdef BDS_B1C_track < handle
% 使用句柄类，避免值传递，help->Comparison of Handle and Value Classes
% 句柄类构造时返回一个对象的引用，向函数传递时不需要复制原始对象

    % 只有类成员可以修改属性值
    properties (GetAccess = public, SetAccess = private)
        sampleFreq      %采样频率
        buffSize        %采样缓冲区大小
        logID           %日志文件ID
        PRN             %卫星编号
        codeData        %数据主码（含子载波）
        codePilot       %导频主码（含子载波）
        codeSub         %导频子码
        timeIntMs       %积分时间，ms
        timeIntS        %积分时间，s
        trackDataTail   %跟踪开始点在数据缓存中的位置
        blkSize         %跟踪数据段采样点个数
        trackDataHead   %跟踪结束点在数据缓存中的位置
        dataIndex       %跟踪开始点在文件中的位置
        codeTarget      %当前跟踪目标码相位
        carrNco         %载波发生器频率
        codeNco         %码发生器频率
        carrFreq        %载波频率测量
        codeFreq        %码频率测量
        remCarrPhase    %跟踪开始点的载波相位
        remCodePhase    %跟踪开始点的码相位
        PLL             %锁相环
        DLL             %延迟锁定环
        I               %导频分量积分结果
        Q               %数据分量积分结果
        PLLFlag         %PLL模式标志，0表示180度相位模糊，1表示纯锁相环
        subPhase        %子码相位
        ts0             %当前伪码周期的开始时间，ms
        varCode         %码鉴相器方差统计
        varPhase        %载波鉴相器方差统计
    end %end properties
    
    methods
        %% 构造
        function obj = BDS_B1C_track(sampleFreq, buffSize, PRN, logID)
            obj.sampleFreq = sampleFreq; %设主程序变量
            obj.buffSize = buffSize;
            obj.PRN = PRN;
            obj.logID = logID;
        end
        
        %% 初始化
        function init(obj, acqResult, n)
            % acqResult为捕获结果，第一个数为码相位，第二个数为载波频率
            % n为已经过了多少个采样点
            code = BDS_B1C_code_data(obj.PRN);
            code = reshape([code;-code],10230*2,1);
            obj.codeData = [code(end);code;code(1)]; %列向量
            code = BDS_B1C_code_pilot(obj.PRN);
            code = reshape([code;-code],10230*2,1);
            obj.codePilot = [code(end);code;code(1)]; %列向量
            obj.codeSub = BDS_B1C_code_sub(obj.PRN); %行向量
            obj.timeIntMs = 1;
            obj.timeIntS = 0.001;
            obj.trackDataTail = obj.sampleFreq*0.01 - acqResult(1) + 2;
            obj.blkSize = obj.sampleFreq*0.001;
            obj.trackDataHead = obj.trackDataTail + obj.blkSize - 1;
            obj.dataIndex = obj.trackDataTail + n;
            obj.codeTarget = 2046;
            obj.carrNco = acqResult(2);
            obj.codeNco = (1.023e6 + obj.carrNco/1540) * 2; %因为有子载波，要乘2
            obj.carrFreq = obj.carrNco;
            obj.codeFreq = obj.codeNco;
            obj.remCarrPhase = 0;
            obj.remCodePhase = 0;
            [K1, K2] = orderTwoLoopCoef(25, 0.707, 1);
            obj.PLL.K1 = K1;
            obj.PLL.K2 = K2;
            obj.PLL.Int = obj.carrNco;
            [K1, K2] = orderTwoLoopCoef(2, 0.707, 1);
            obj.DLL.K1 = K1;
            obj.DLL.K2 = K2;
            obj.DLL.Int = obj.codeNco;
            obj.I = 0;
            obj.Q = 0;
            obj.PLLFlag = 0;
            obj.subPhase = 1;
            obj.ts0 = NaN;
            obj.varCode = var_rec(200);
            obj.varPhase = var_rec(200);
        end
        
        %% 跟踪
        function [I_Q, disc] = track(obj, rawSignal)
            %----更新跟踪开始点在文件中的位置（下次跟踪）
            obj.dataIndex = obj.dataIndex + obj.blkSize;
            %----时间序列
            t = (0:obj.blkSize-1) / obj.sampleFreq;
            te = obj.blkSize / obj.sampleFreq;
            %----生成本地载波
            theta = (obj.remCarrPhase + obj.carrNco*t) * 2; %乘2因为后面是以pi为单位求三角函数
            carr_cos = cospi(theta); %本地载波
            carr_sin = sinpi(theta);
            theta_next = obj.remCarrPhase + obj.carrNco*te;
            obj.remCarrPhase = mod(theta_next, 1); %剩余载波相位，周
            %----生成本地码
            tcode = obj.remCodePhase + obj.codeNco*t + 2; %加2保证求滞后码时大于1
%             earlyCodeI  = obj.codeData(floor(tcode+0.3));  %超前码（数据分量）
            promptCodeI = obj.codeData(floor(tcode));      %即时码
%             lateCodeI   = obj.codeData(floor(tcode-0.3));  %滞后码
            earlyCodeQ  = obj.codePilot(floor(tcode+0.3)); %超前码（导频分量）
            promptCodeQ = obj.codePilot(floor(tcode));     %即时码
            lateCodeQ   = obj.codePilot(floor(tcode-0.3)); %滞后码
            obj.remCodePhase = mod(obj.remCodePhase + obj.codeNco*te, 20460); %剩余载波相位，周
            %----原始数据乘载波
            iBasebandSignal = rawSignal(1,:).*carr_cos + rawSignal(2,:).*carr_sin; %乘负载波
            qBasebandSignal = rawSignal(2,:).*carr_cos - rawSignal(1,:).*carr_sin;
            %----六路积分（使用导频分量跟踪，数据分量只解析导航电文）
            I_E = iBasebandSignal * earlyCodeQ;
            Q_E = qBasebandSignal * earlyCodeQ;
            I_P = iBasebandSignal * promptCodeQ;
            Q_P = qBasebandSignal * promptCodeQ;
            I_L = iBasebandSignal * lateCodeQ;
            Q_L = qBasebandSignal * lateCodeQ;
            obj.I = -qBasebandSignal * promptCodeI; %数据分量，幅值比1:sqrt(29/11)，1:624
            obj.Q = I_P;                            %导频分量
            %----码鉴相器
            S_E = sqrt(I_E^2+Q_E^2);
            S_L = sqrt(I_L^2+Q_L^2);
            codeError = (11/30) * (S_E-S_L)/(S_E+S_L); %单位：码片
            %----载波鉴相器
            if obj.PLLFlag==0
                carrError = atan(Q_P/I_P) / (2*pi); %单位：周
            else
                s = obj.codeSub(obj.subPhase); %子码符号
                carrError = atan2(Q_P*s,I_P*s) / (2*pi); %单位：周
            end
            %----PLL
            obj.PLL.Int = obj.PLL.Int + obj.PLL.K2*carrError*obj.timeIntS; %锁相环积分器
            obj.carrNco = obj.PLL.Int + obj.PLL.K1*carrError;
            obj.carrFreq = obj.PLL.Int;
            %----DLL
            obj.DLL.Int = obj.DLL.Int + obj.DLL.K2*codeError*obj.timeIntS; %延迟锁定环积分器
            obj.codeNco = obj.DLL.Int + obj.DLL.K1*codeError;
            obj.codeFreq = obj.DLL.Int;
            %----更新目标码相位、导频子码相位、伪码周期时间
            if obj.codeTarget==20460
                obj.codeTarget = 2046;
                obj.subPhase = mod(obj.subPhase,1800) + 1; %导频子码相位加1（只有通过电文解析确定了导频子码相位才有意义）
                obj.ts0 = obj.ts0 + 10; %一个伪码周期10ms
            else
                obj.codeTarget = obj.codeTarget + 2046; %跟踪目标码相位
            end
            %----更新下一数据块位置
            obj.trackDataTail = obj.trackDataHead + 1;
            if obj.trackDataTail>obj.buffSize
                obj.trackDataTail = 1;
            end
            obj.blkSize = ceil((obj.codeTarget-obj.remCodePhase)/obj.codeNco*obj.sampleFreq);
            obj.trackDataHead = obj.trackDataTail + obj.blkSize - 1;
            if obj.trackDataHead>obj.buffSize
                obj.trackDataHead = obj.trackDataHead - obj.buffSize;
            end
            %----输出
            I_Q = [I_P, I_E, I_L, Q_P, Q_E, Q_L, obj.I, obj.Q];
            disc = [codeError/2, carrError]; %码相位误差除以2，换算成主码相位误差
            %----统计鉴相器输出方差
            obj.varCode.update(codeError/2);
            obj.varPhase.update(carrError);
        end
        
        %% 启动纯锁相环
        function start_pure_PLL(obj, subPhase, phaseFlag)
            if phaseFlag==1
                obj.remCarrPhase = mod(obj.remCarrPhase+0.5, 1); %载波相位翻转
            end
            obj.subPhase = subPhase;
            obj.PLLFlag = 1;
        end
        
        %% 设置伪码周期开始时间
        function set_ts0(obj, ts0)
            obj.ts0 = ts0;
        end
        
        %% 设置采样频率
        function set_sampleFreq(obj, sampleFreq)
            obj.sampleFreq = sampleFreq;
        end
        
    end %end methods
    
end %end classdef