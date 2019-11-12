classdef GPS_L1CA_track < handle

    % 只有类成员可以修改属性值
    properties (GetAccess = public, SetAccess = private)
        sampleFreq      %标称采样频率
        deltaFreq       %频率误差
        buffSize        %采样缓冲区大小
        logID           %日志文件ID
        PRN             %卫星编号
        code            %C/A码
        timeIntMs       %积分时间，ms (1,2,4,5,10,20)
        timeIntS        %积分时间，s
        codeInt         %积分时间内码片个数
        pointInt        %一个比特有多少个积分点，一个比特20ms
        trackDataTail   %跟踪开始点在数据缓存中的位置
        blkSize         %跟踪数据段采样点个数
        trackDataHead   %跟踪结束点在数据缓存中的位置
        dataIndex       %跟踪开始点在文件中的位置
        carrNco         %载波发生器频率
        codeNco         %码发生器频率
        carrFreq        %载波频率测量
        codeFreq        %码频率测量
        remCarrPhase    %跟踪开始点的载波相位
        remCodePhase    %跟踪开始点的码相位
        FLL             %锁频环
        PLL             %锁相环
        DLL             %延迟锁定环
        I               %I路积分结果
        Q               %Q路积分结果
        PLLFlag         %PLL模式标志，0表示锁频环，1表示锁相环
        DLLFlag         %DLL模式标志，0表示纯DLL，1表示XXX
        ts0             %当前伪码周期的开始时间，ms
        varCode         %码鉴相器方差统计
        varPhase        %载波鉴相器方差统计
    end %end properties
    
    methods
        %% 构造
        function obj = GPS_L1CA_track(sampleFreq, buffSize, PRN, logID)
            obj.sampleFreq = sampleFreq;
            obj.buffSize = buffSize;
            obj.PRN = PRN;
            obj.logID = logID;
        end
        
        %% 初始化
        function init(obj, acqResult, n)
            % acqResult为捕获结果，第一个数为码相位，第二个数为载波频率
            % n为已经过了多少个采样点
            obj.deltaFreq = 0;
            codeCA = GPS_L1CA_code(obj.PRN)';
            obj.code = [codeCA(end);codeCA;codeCA(1)]; %列向量
            obj.timeIntMs = 1;
            obj.timeIntS = 0.001;
            obj.codeInt = 1023;
            obj.pointInt = 20;
            obj.trackDataTail = obj.sampleFreq*0.001 - acqResult(1) + 2;
            obj.blkSize = obj.sampleFreq*0.001;
            obj.trackDataHead = obj.trackDataTail + obj.blkSize - 1;
            obj.dataIndex = obj.trackDataTail + n;
            obj.carrNco = acqResult(2);
            obj.codeNco = 1.023e6 + obj.carrNco/1540;
            obj.carrFreq = obj.carrNco;
            obj.codeFreq = obj.codeNco;
            obj.remCarrPhase = 0;
            obj.remCodePhase = 0;
            obj.FLL.K = 40*0.001;
            obj.FLL.Int = obj.carrNco;
            obj.FLL.cnt = 0;
            [K1, K2] = orderTwoLoopCoefDisc(25, 0.707, obj.timeIntS);
            obj.PLL.K1 = K1;
            obj.PLL.K2 = K2;
            obj.PLL.Int = 0;
            [K1, K2] = orderTwoLoopCoefDisc(2, 0.707, obj.timeIntS);
            obj.DLL.K1 = K1;
            obj.DLL.K2 = K2;
            obj.DLL.Int = obj.codeNco;
            obj.I = 1;
            obj.Q = 1;
            obj.PLLFlag = 0;
            obj.DLLFlag = 0;
            obj.ts0 = NaN;
            obj.varCode = var_rec(200);
            obj.varPhase = var_rec(200);
        end
        
        %% 跟踪
        function [I_Q, disc] = track(obj, rawSignal)
            %----修正采样频率
            sampleFreq0 = obj.sampleFreq * (1+obj.deltaFreq);
            %----更新跟踪开始点在文件中的位置（下次跟踪）
            obj.dataIndex = obj.dataIndex + obj.blkSize;
            %----时间序列
            t = (0:obj.blkSize-1) / sampleFreq0;
            te = obj.blkSize / sampleFreq0;
            %----生成本地载波
            theta = (obj.remCarrPhase + obj.carrNco*t) * 2; %乘2因为后面是以pi为单位求三角函数
            carr_cos = cospi(theta); %本地载波
            carr_sin = sinpi(theta);
            theta_next = obj.remCarrPhase + obj.carrNco*te;
            obj.remCarrPhase = mod(theta_next, 1); %剩余载波相位，周
            %----生成本地码
            tcode = obj.remCodePhase + obj.codeNco*t + 2; %加2保证求滞后码时大于1
            earlyCode  = obj.code(floor(tcode+0.5)); %超前码
            promptCode = obj.code(floor(tcode));     %即时码
            lateCode   = obj.code(floor(tcode-0.5)); %滞后码
            obj.remCodePhase = obj.remCodePhase + obj.codeNco*te - obj.codeInt; %剩余载波相位，周
            %----原始数据乘载波
            iBasebandSignal = rawSignal(1,:).*carr_cos + rawSignal(2,:).*carr_sin; %乘负载波
            qBasebandSignal = rawSignal(2,:).*carr_cos - rawSignal(1,:).*carr_sin;
            %----六路积分
            I_E = iBasebandSignal * earlyCode;
            Q_E = qBasebandSignal * earlyCode;
            I_P = iBasebandSignal * promptCode;
            Q_P = qBasebandSignal * promptCode;
            I_L = iBasebandSignal * lateCode;
            Q_L = qBasebandSignal * lateCode;
            %----码鉴相器
            S_E = sqrt(I_E^2+Q_E^2);
            S_L = sqrt(I_L^2+Q_L^2);
            codeError = 0.5 * (S_E-S_L)/(S_E+S_L); %单位：码片，0.5--0.5，0.4--0.6，0.3--0.7，0.25--0.75
            %----载波鉴相器
            carrError = atan(Q_P/I_P) / (2*pi); %单位：周
            %----鉴频器
            yc = obj.I*I_P + obj.Q*Q_P; %I0*I1+Q0*Q1
            ys = obj.I*Q_P - obj.Q*I_P; %I0*Q1-Q0*I1
            freqError = atan(ys/yc)/obj.timeIntS / (2*pi); %单位：Hz
            obj.I = I_P;
            obj.Q = Q_P;
            %----FLL/PLL
            if obj.PLLFlag==0 %-FLL
                obj.FLL.Int = obj.FLL.Int + obj.FLL.K*freqError; %锁频环积分器
                obj.carrNco = obj.FLL.Int;
                obj.carrFreq = obj.FLL.Int;
                obj.FLL.cnt = obj.FLL.cnt + 1;
                if obj.FLL.cnt==200 %锁频200ms后转到锁相环跟踪
                    obj.FLL.cnt = 0;
                    obj.PLL.Int = obj.FLL.Int; %初始化锁相环积分器
                    obj.PLLFlag = 1;
                    fprintf(obj.logID, '%2d: Start PLL tracking at %.8fs\r\n', ...
                    obj.PRN, obj.dataIndex/obj.sampleFreq);
                end
            else %-PLL
                obj.PLL.Int = obj.PLL.Int + obj.PLL.K2*carrError; %锁相环积分器
                obj.carrNco = obj.PLL.Int + obj.PLL.K1*carrError;
                obj.carrFreq = obj.PLL.Int;
            end
            %----DLL
            obj.DLL.Int = obj.DLL.Int + obj.DLL.K2*codeError; %延迟锁定环积分器
            obj.codeNco = obj.DLL.Int + obj.DLL.K1*codeError;
            obj.codeFreq = obj.DLL.Int;
            %----更新伪码周期时间
            obj.ts0 = obj.ts0 + obj.timeIntMs;
            %----更新下一数据块位置
            obj.trackDataTail = obj.trackDataHead + 1;
            if obj.trackDataTail>obj.buffSize
                obj.trackDataTail = 1;
            end
            obj.blkSize = ceil((obj.codeInt-obj.remCodePhase)/obj.codeNco*sampleFreq0);
            obj.trackDataHead = obj.trackDataTail + obj.blkSize - 1;
            if obj.trackDataHead>obj.buffSize
                obj.trackDataHead = obj.trackDataHead - obj.buffSize;
            end
            %----输出
            I_Q = [I_P, I_E, I_L, Q_P, Q_E, Q_L];
            disc = [codeError, carrError, freqError];
            %----统计鉴相器输出方差
            obj.varCode.update(codeError);
            obj.varPhase.update(carrError);
        end
        
        %% 设置伪码周期开始时间
        function set_ts0(obj, ts0)
            obj.ts0 = ts0;
        end
        
        %% 设置频率误差
        function set_deltaFreq(obj, deltaFreq)
            obj.deltaFreq = deltaFreq;
        end
        
        %% 设置积分时间
        function set_timeInt(obj, ti)
            codeCA = GPS_L1CA_code(obj.PRN)';
            obj.code = [codeCA(end);repmat(codeCA,ti,1);codeCA(1)]; %列向量
            obj.timeIntMs = ti;
            obj.timeIntS = ti/1000;
            obj.codeInt = ti*1023;
            obj.pointInt = 20/ti;
            sampleFreq0 = obj.sampleFreq * (1+obj.deltaFreq);
            obj.blkSize = ceil((obj.codeInt-obj.remCodePhase)/obj.codeNco*sampleFreq0);
            obj.trackDataHead = obj.trackDataTail + obj.blkSize - 1;
            if obj.trackDataHead>obj.buffSize
                obj.trackDataHead = obj.trackDataHead - obj.buffSize;
            end
            if ti==10
                [K1, K2] = orderTwoLoopCoefDisc(20, 0.707, obj.timeIntS);
                obj.PLL.K1 = K1;
                obj.PLL.K2 = K2;
                [K1, K2] = orderTwoLoopCoefDisc(1.8, 0.707, obj.timeIntS);
                obj.DLL.K1 = K1;
                obj.DLL.K2 = K2;
            elseif ti==1
                [K1, K2] = orderTwoLoopCoefDisc(25, 0.707, obj.timeIntS);
                obj.PLL.K1 = K1;
                obj.PLL.K2 = K2;
                [K1, K2] = orderTwoLoopCoefDisc(2, 0.707, obj.timeIntS);
                obj.DLL.K1 = K1;
                obj.DLL.K2 = K2;
            end
        end
        
    end %end methods
    
end %end classdef