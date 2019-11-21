% 保存程序运行结果，将其添加到快捷键中

initialVars = who; %原工作空间中的变量名

default_path = fileread('.\temp\path_result.txt'); %默认结果文件存储路径
[file, path] = uiputfile([default_path,'\matlab.mat']); %选择文件路径，输入文件名，默认文件名matlab.mat，关闭file,path返回0

if file~=0
    save([path,file], initialVars{:})
end

clearvars initialVars default_path file path