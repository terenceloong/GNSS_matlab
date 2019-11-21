function output = clean_receiverOutput(output)
% 自动识别结构体中的场名并清除多余变量

fields = fieldnames(output); %场名
N = size(fields,1); %场数

%---删除结尾多余的数据
index = find(isnan(output.state)); %空状态索引
for k=2:N
    if strcmp(fields{k}(1:2),'sv') %场的前缀是sv
        eval(['output.',fields{k},'(:,:,index) = [];'])
    else
        eval(['output.',fields{k},'(index,:) = [];'])
    end
end

%---删除接收机未初始化的数据
index = find(output.state == 0); %0状态索引
for k=2:N
    if strcmp(fields{k}(1:2),'sv')
        eval(['output.',fields{k},'(:,:,index) = [];'])
    else
        eval(['output.',fields{k},'(index,:) = [];'])
    end
end

output.n = length(output.ta); %数据个数

end