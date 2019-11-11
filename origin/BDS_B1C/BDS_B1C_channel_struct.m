function ch = BDS_B1C_channel_struct()
% 声明通道结构体所有场

ch.PRN              = []; %卫星编号
ch.state            = []; %通道状态（数字）
ch.codeData         = []; %数据分量伪码
ch.codePilot        = []; %导频分量伪码
ch.timeIntMs        = []; %积分时间，ms

ch.trackDataTail    = []; %跟踪开始点在数据缓存中的位置
ch.blkSize          = []; %跟踪数据段采样点个数
ch.trackDataHead    = []; %跟踪结束点在数据缓存中的位置
ch.dataIndex        = []; %跟踪开始点在文件中的位置
ch.codeTarget       = []; %当前跟踪目标码相位

ch.carrNco          = []; %载波发生器频率
ch.codeNco          = []; %码发生器频率
ch.carrFreq         = []; %载波频率测量
ch.codeFreq         = []; %码频率测量
ch.remCarrPhase     = []; %跟踪开始点的载波相位
ch.remCodePhase     = []; %跟踪开始点的码相位
ch.PLL              = []; %锁相环（结构体）
ch.DLL              = []; %延迟锁定环（结构体）

end