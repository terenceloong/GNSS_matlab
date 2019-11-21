function analysis_GPS_BDS_pos
% 分析单天线GPS+北斗伪距、伪距率测量精度，几何精度因子
% 给定固定的参考坐标，静止时，伪距伪距率误差为常值，运动时，伪距伪距率误差是载体运动造成的
% 运行结束后查看analysis变量
% 试验发现，某些时候星座几何精度因子不好，定位测速噪声很大

%% 导入数据
output = evalin('base', 'output');

%% 参考坐标
p0 = [45.73580, 126.62881, 159];
rp = lla2ecef(p0); %ecef
Cen = dcmecef2ned(p0(1), p0(2));

%% 申请空间
n = output.n; %数据点数
svN_GPS = size(output.sv_GPS_A,1); %卫星个数
svN_BDS = size(output.sv_BDS_A,1);
analysis.ta = output.ta; %时间序列
analysis.GPS_dr  = zeros(n,svN_GPS); %伪距测量误差，m
analysis.GPS_dv  = zeros(n,svN_GPS); %伪距率测量误差，m/s
analysis.GPS_azi = zeros(n,svN_GPS); %方位角，deg
analysis.GPS_ele = zeros(n,svN_GPS); %高度角，deg
analysis.BDS_dr  = zeros(n,svN_BDS);
analysis.BDS_dv  = zeros(n,svN_BDS);
analysis.BDS_azi = zeros(n,svN_BDS);
analysis.BDS_ele = zeros(n,svN_BDS);
analysis.DOP_GPS     = NaN(n,7); %GPS星座精度因子
analysis.DOP_BDS     = NaN(n,7); %北斗星座精度因子
analysis.DOP_GPS_BDS = NaN(n,7); %GPS+北斗星座精度因子

%% 分析
for k=1:n
    %----GPS
    sv_GPS = output.sv_GPS_A(:,1:8,k);
    rs = sv_GPS(:,1:3);            %卫星位置
    rsp = ones(svN_GPS,1)*rp - rs; %卫星指向接收机矢量
    rho = sum(rsp.*rsp, 2).^0.5;   %理论伪距
    rspu = rsp ./ (rho*[1,1,1]);   %视线单位矢量
    vs = sv_GPS(:,5:7);            %卫星速度
    vsp = 0 - vs;                  %接收机相对卫星的速度
    rhodot = sum(vsp.*rspu, 2);    %理论伪距率
    analysis.GPS_dr(k,:) = (sv_GPS(:,4) - rho)';    %伪距测量误差，测量值减计算值
    analysis.GPS_dv(k,:) = (sv_GPS(:,8) - rhodot)'; %伪距率测量误差
    rn = (Cen*rspu')'; %地理系下视线单位矢量
    analysis.GPS_ele(k,:) = asind(rn(:,3))'; %卫星高度角
    analysis.GPS_azi(k,:) = atan2d(-rn(:,2),-rn(:,1))'; %卫星方位角
    rn_GPS = rn(~isnan(rn(:,1)),:); %提取非空数据
    if size(rn_GPS,1)>=4
        G = [rn_GPS, -ones(size(rn_GPS,1),1)];
        D = inv(G'*G);
        analysis.DOP_GPS(k,1:4) = sqrt(diag(D));
        analysis.DOP_GPS(k,5) = norm(analysis.DOP_GPS(k,1:2)); %HDOP
        analysis.DOP_GPS(k,6) = norm(analysis.DOP_GPS(k,1:3)); %PDOP
        analysis.DOP_GPS(k,7) = norm(analysis.DOP_GPS(k,1:4)); %GDOP
    end
    
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
    
    %----GPS+北斗DOP
    rn_GPS_BDS = [rn_GPS; rn_BDS];
    if size(rn_GPS_BDS,1)>=4
        G = [rn_GPS_BDS, -ones(size(rn_GPS_BDS,1),1)];
        D = inv(G'*G);
        analysis.DOP_GPS_BDS(k,1:4) = sqrt(diag(D));
        analysis.DOP_GPS_BDS(k,5) = norm(analysis.DOP_GPS_BDS(k,1:2));
        analysis.DOP_GPS_BDS(k,6) = norm(analysis.DOP_GPS_BDS(k,1:3));
        analysis.DOP_GPS_BDS(k,7) = norm(analysis.DOP_GPS_BDS(k,1:4));
    end
end

%% 输出
assignin('base', 'analysis', analysis);

end