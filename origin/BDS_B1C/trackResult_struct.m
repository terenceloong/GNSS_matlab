function trackResult = trackResult_struct(m)
% 跟踪结果结构体

trackResult.PRN = 0;
trackResult.n = 1; %指向当前存储的行号

trackResult.dataIndex     = zeros(m,1); %码周期开始采样点在原始数据文件中的位置
trackResult.remCodePhase  = zeros(m,1); %码周期开始采样点的码相位，码片
trackResult.codeFreq      = zeros(m,1); %码频率
trackResult.remCarrPhase  = zeros(m,1); %码周期开始采样点的载波相位，周
trackResult.carrFreq      = zeros(m,1); %载波频率
trackResult.I_Q           = zeros(m,6); %[I_P,I_E,I_L,Q_P,Q_E,Q_L]
trackResult.disc          = zeros(m,2); %鉴相器

end