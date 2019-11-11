function ch = BDS_B1C_channel_init(ch, acqResult, n, sampleFreq)

code = BDS_B1C_data_generate(ch.PRN);
code = reshape([code;-code],10230*2,1)'; %带子载波的码，行向量
ch.codeData = [code(end),code,code(1)]'; %列向量，为了求积分时用矢量相乘加速

code = BDS_B1C_pilot_generate(ch.PRN);
code = reshape([code;-code],10230*2,1)'; %带子载波的码，行向量
ch.codePilot = [code(end),code,code(1)]'; %列向量，为了求积分时用矢量相乘加速

% ch.PRN 卫星号不变
ch.state = 1; %激活通道
ch.timeIntMs = 1;

ch.trackDataTail = sampleFreq*0.01 - acqResult(1) + 2; %%%
ch.blkSize = sampleFreq*0.001;
ch.trackDataHead = ch.trackDataTail + ch.blkSize - 1;
ch.dataIndex = ch.trackDataTail + n;
ch.codeTarget = 2046;

ch.carrNco = acqResult(2);
ch.codeNco = (1.023e6 + ch.carrNco/1540) * 2; %因为有子载波，要乘2
ch.carrFreq = ch.carrNco;
ch.codeFreq = ch.codeNco;
ch.remCarrPhase = 0;
ch.remCodePhase = 0;

[K1, K2] = orderTwoLoopCoef(25, 0.707, 1);
ch.PLL.K1 = K1;
ch.PLL.K2 = K2;
ch.PLL.Int = ch.carrNco;

[K1, K2] = orderTwoLoopCoef(2, 0.707, 1);
ch.DLL.K1 = K1;
ch.DLL.K2 = K2;
ch.DLL.Int = ch.codeNco;

end