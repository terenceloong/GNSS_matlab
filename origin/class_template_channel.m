classdef GPS_L1CA_channel < handle

    % 只有类成员可以修改属性值
    properties (GetAccess = public, SetAccess = private)
        
    end %end properties
    
    methods
        %% 构造
        function obj = GPS_L1CA_channel(sampleFreq, buffSize, PRN, logID)
            
        end
        
        %% 初始化
        function init(obj, acqResult, n)
            
        end
        
        %% 电文解析
        function parse(obj, ta)
            
        end
        
    end %end methods
    
end %end classdef