% 测试PV模型导航滤波器
% 有地理系下和ecef系下两种，两种方法稳定时结果一样

clear
clc

%% 仿真条件设置
%====仿真时间
T = 200;        %总时间，s
dt = 0.01;      %时间间隔，s
n = T / dt;     %总仿真点数
t = (1:n)'*dt;  %时间序列（列向量）

%====接收机参数
sigma_rho = 3;        %伪距测量噪声标准差，m
sigma_rhodot = 0.1;   %伪距率测量噪声标准差，m/s
dtr = 1e-8;           %初始钟差，s
dtv = 3e-9;           %初始钟频差，s/s
c = 299792458;        %光速，m/s

%====接收机位置
lat = 46;      %纬度，deg
lon = 126;     %经度，deg
h = 200;       %高度，m

%====卫星位置
% 第一列为方位角，第二列为高度角，deg
sv_info = [  0, 45;
            23, 28;
            58, 80;
           100, 49;
           146, 34;
           186, 78;
           213, 43;
           255, 15;
           310, 20];
rho = 20000000; %卫星到接收机的距离，m

%====计算卫星位置、速度
svN = size(sv_info,1); %卫星个数
sv_real = zeros(svN,8); %存储无误差卫星量测，[x,y,z, rho, vx,vy,vz, rhodot]
Cen = dcmecef2ned(lat, lon);
rp = lla2ecef([lat, lon, h]); %接收机位置矢量（ecef，行向量）
for k=1:svN
    e = [-cosd(sv_info(k,2))*cosd(sv_info(k,1)), ...
         -cosd(sv_info(k,2))*sind(sv_info(k,1)), ...
          sind(sv_info(k,2))]; %卫星指向接收机的单位矢量（地理系，行向量）
    rsp = e * rho; %卫星指向接收机的位置矢量（地理系）
    sv_real(k,1:3) = rp - (rsp*Cen);  %卫星位置矢量（ecef）
    sv_real(k,4) = rho;               %伪距
    sv_real(k,5:7) = 0;               %卫星速度
    sv_real(k,8) = 0;                 %伪距率
end

%====生成伪距、伪距率噪声
% 对比方法使用公共的噪声
noise_rho    = randn(n,svN)*sigma_rho;
noise_rhodot = randn(n,svN)*sigma_rhodot;

%% 测试地理系下导航滤波器
%====申请输出结果空间
output_geog.nav = zeros(n,6); %滤波器导航输出
output_geog.dc  = zeros(n,2); %钟差、钟频差估计
output_geog.P   = zeros(n,8); %滤波器P阵

%====初始化滤波器
a = 6371000; %地球半径
para.P = diag([ ...
               [1/a,secd(lat)/a,1]*5, ... %初始位置误差，[rad,rad,m]
               [1,1,1]*1, ...             %初始速度误差，m/s
               2e-8, ...                  %初始钟差，s
               3e-9 ...
               ])^2;                  %初始钟频差，s/s
para.Q = diag([ ...
               [1/a,secd(lat)/a,1]*0.01 *(dt/1), ...
               [1,1,1]*0.01, ...
               0.03e-9 *(dt/1), ...
               0.03e-9 ...
               ])^2 * dt^2;
NF = navFilter_pv_geog([lat,lon,h], [0,0,0], dt, para);
sigma = [ones(svN,1)*sigma_rho, ones(svN,1)*sigma_rhodot];

%====运行
for k=1:n
    % 生成卫星量测
    dtr = dtr + dtv*dt; %当前钟差，s
    sv = sv_real;
    sv(:,4) = sv(:,4) + noise_rho(k,:)'    + dtr*c; %伪距加噪声
    sv(:,8) = sv(:,8) + noise_rhodot(k,:)' + dtv*c; %伪距率加噪声
    
    % 更新导航滤波器
    NF.update(sv(:,[1:3,5:7]), sv(:,[4,8]), sigma);
    
    % 存储导航结果
    output_geog.nav(k,1:3) = NF.pos;
    output_geog.nav(k,4:6) = NF.vel;
    output_geog.dc(k,1) = NF.dtr - dtr;
    output_geog.dc(k,2) = NF.dtv;
    output_geog.P(k,:) = sqrt(diag(NF.Pk)');
end

%% 测试ecef系下导航滤波器
%====申请输出结果空间
output_ecef.nav = zeros(n,6); %滤波器导航输出
output_ecef.dc  = zeros(n,2); %钟差、钟频差估计
output_ecef.P   = zeros(n,8); %滤波器P阵

%====初始化滤波器
para.P = diag([ ...
               [1,1,1]*5, ... %初始位置误差，[m
               [1,1,1]*1, ... %初始速度误差，m/s
               2e-8, ...      %初始钟差，s
               3e-9 ...       %初始钟频差，s/s
               ])^2;
para.Q = diag([ ...
               [1,1,1]*0.01 *(dt/1), ...
               [1,1,1]*0.01, ...
               0.03e-9 *(dt/1), ...
               0.03e-9 ...
               ])^2 * dt^2;
NF = navFilter_pv_ecef([lat,lon,h], [0,0,0], dt, para);
sigma = [ones(svN,1)*sigma_rho, ones(svN,1)*sigma_rhodot];

%====运行
for k=1:n
    % 生成卫星量测
    dtr = dtr + dtv*dt; %当前钟差，s
    sv = sv_real;
    sv(:,4) = sv(:,4) + noise_rho(k,:)'    + dtr*c; %伪距加噪声
    sv(:,8) = sv(:,8) + noise_rhodot(k,:)' + dtv*c; %伪距率加噪声
    
    % 更新导航滤波器
    NF.update(sv(:,[1:3,5:7]), sv(:,[4,8]), sigma);
    
    % 存储导航结果
    output_ecef.nav(k,1:3) = NF.pos;
    output_ecef.nav(k,4:6) = NF.vel;
    output_ecef.dc(k,1) = NF.dtr - dtr;
    output_ecef.dc(k,2) = NF.dtv;
    output_ecef.P(k,:) = sqrt(diag(NF.Pk)');
end