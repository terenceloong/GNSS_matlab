function analysis_measure_BDS
% 分析单天线北斗伪距、伪距率测量精度，几何精度因子
% 给定固定的参考坐标，静止时，伪距伪距率误差为常值，运动时，伪距伪距率误差是载体运动造成的
% 运行结束后查看analysis变量

%% 导入数据
output = evalin('base', 'output');

%% 参考坐标
p0 = [45.73580, 126.62881, 159];
rp = lla2ecef(p0); %ecef
Cen = dcmecef2ned(p0(1), p0(2));

%% 申请空间
n = output.n; %数据点数
svN_BDS = size(output.sv_BDS_A,1); %卫星个数
analysis.ta = output.ta; %时间序列
analysis.BDS_dr  = zeros(n,svN_BDS); %伪距测量误差，m
analysis.BDS_dv  = zeros(n,svN_BDS); %伪距率测量误差，m/s
analysis.BDS_azi = zeros(n,svN_BDS); %方位角，deg
analysis.BDS_ele = zeros(n,svN_BDS); %高度角，deg
analysis.DOP_BDS = NaN(n,7); %北斗星座精度因子

%% 分析
for k=1:n
    %----北斗
    sv_BDS = output.sv_BDS_A(:,1:8,k);
    rs = sv_BDS(:,1:3);
    rsp = ones(svN_BDS,1)*rp - rs;
    rho = sum(rsp.*rsp, 2).^0.5; 
    rspu = rsp ./ (rho*[1,1,1]); 
    vs = sv_BDS(:,5:7); 
    vsp = 0 - vs; 
    rhodot = sum(vsp.*rspu, 2);
    analysis.BDS_dr(k,:) = (sv_BDS(:,4) - rho)';
    analysis.BDS_dv(k,:) = (sv_BDS(:,8) - rhodot)';
    rn = (Cen*rspu')';
    analysis.BDS_ele(k,:) = asind(rn(:,3))';
    analysis.BDS_azi(k,:) = atan2d(-rn(:,2),-rn(:,1))'; 
    rn_BDS = rn(~isnan(rn(:,1)),:);
    if size(rn_BDS,1)>=4
        G = [rn_BDS, -ones(size(rn_BDS,1),1)];
        D = inv(G'*G);
        analysis.DOP_BDS(k,1:4) = sqrt(diag(D));
        analysis.DOP_BDS(k,5) = norm(analysis.DOP_BDS(k,1:2));
        analysis.DOP_BDS(k,6) = norm(analysis.DOP_BDS(k,1:3));
        analysis.DOP_BDS(k,7) = norm(analysis.DOP_BDS(k,1:4));
    end
end

%% 输出
assignin('base', 'analysis', analysis);

end