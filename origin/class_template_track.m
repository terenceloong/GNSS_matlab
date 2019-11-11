classdef GPS_L1CA_track < handle

    % 只有类成员可以修改属性值
    properties (GetAccess = public, SetAccess = private)
        
    end %end properties
    
    methods
        %% 构造
        function obj = GPS_L1CA_track(sampleFreq, buffSize, PRN, logID)
            
        end
        
        %% 初始化
        function init(obj, acqResult, n)
            
        end
        
        %% 跟踪
        function [I_Q, disc] = track(obj, rawSignal)
            
        end
        
    end %end methods
    
end %end classdef