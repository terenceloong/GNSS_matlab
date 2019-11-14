% 给定文件名，进行一次所有卫星的捕获
% 可以切换数据分量捕获还是导频分量捕获，导频分量功率大，捕获效果好

% clc
sample_offset = 0*4e6;
% acqResults = BDS_B1C_acq('E:\GNSS data\B210_20190726_210916_ch1.dat', sample_offset);
acqResults = BDS_B1C_acq('E:\GNSS data\0823\B210_20190823_194010_ch1.dat', sample_offset);