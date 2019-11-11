classdef GPS_L1CA_channel < GPS_L1CA_track

    % 只有类成员可以修改属性值
    properties (GetAccess = public, SetAccess = private)
        state           %通道状态（数字）
    end %end properties
    
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
        end
        
        %% 电文解析
        function parse(obj, ta)
            
        end
        
    end %end methods
    
end %end classdef