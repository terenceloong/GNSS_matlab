function analysis_measure_GPS
% 分析单天线GPS伪距、伪距率测量精度，几何精度因子
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
svN_GPS = size(output.sv_GPS_A,1); %卫星个数
analysis.ta = output.ta; %时间序列
analysis.GPS_dr  = zeros(n,svN_GPS); %伪距测量误差，m
analysis.GPS_dv  = zeros(n,svN_GPS); %伪距率测量误差，m/s
analysis.GPS_azi = zeros(n,svN_GPS); %方位角，deg
analysis.GPS_ele = zeros(n,svN_GPS); %高度角，deg
analysis.DOP_GPS = NaN(n,7); %GPS星座精度因子

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
end

%% 输出
assignin('base', 'analysis', analysis);

end