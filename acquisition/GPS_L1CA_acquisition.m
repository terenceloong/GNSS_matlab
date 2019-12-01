% 选择文件进行GPS L1CA捕获

default_path = fileread('.\temp\path_data.txt'); %数据文件所在默认路径
[file, path] = uigetfile([default_path,'\*.dat'], '选择GNSS数据文件'); %文件选择对话框
if file==0
    disp('Invalid file!');
    return
end
data_file = [path, file];

acqResults = GPS_L1CA_acq(data_file, 0*4e6, 12000, 1);