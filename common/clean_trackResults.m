function trackResults = clean_trackResults(trackResults)
% 自动识别结构体中的场名并清除多余变量

fields = fieldnames(trackResults); %场名

M = size(trackResults,1); %结构体行数
N = size(fields,1); %场数

for ki=1:M
    for kj=3:N %从第3个场开始
        eval('n = trackResults(ki).n;')
        eval(['trackResults(ki).',fields{kj},'(n:end,:) = [];'])
    end
end

end