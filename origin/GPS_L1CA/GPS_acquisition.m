clear
clc
sample_offset = 0*4e6;
% acqResults = GPS_L1_CA_acq('E:\GNSS data\B210_20190726_205109_ch1.dat', sample_offset, 8000);
acqResults = GPS_L1_CA_acq('E:\GNSS data\0823\B210_20190823_194010_ch1.dat', sample_offset, 8000);