function measure_error
% 验证静止时伪距、伪距率测量误差
% 可以切换固定参考坐标还是实际解算坐标

%% 导入数据
svList = evalin('base', 'svList'); %卫星编号列表
sv_info = evalin('base', 'output_sv(:,1:8,:)'); %卫星信息，[x,y,z, rho, vx,vy,vz, rhodot]
% p0 = evalin('base', 'output_pos(:,1:3)'); %定位信息

%% 参考坐标
p0 = [45.73105, 126.62487, 207];
rp = lla2ecef(p0); %ecef
Cen = dcmecef2ned(p0(1), p0(2));

%% 数据范围
range = 1:size(sv_info,3); %所有点
% range = 1:6000; %从第几个点到第几个点

%% 输入数据截取
sv_info = sv_info(:,:,range);

%% 申请存储空间
n = size(sv_info,3); %数据点数
svN = length(svList); %卫星个数

output_dr = zeros(n,svN); %伪距误差，每一列为一颗卫星
output_dv = zeros(n,svN); %伪距率误差
output_ele = zeros(n,svN); %高度角
output_azi = zeros(n,svN); %方位角

%% 计算
for k=1:n
%     rp = lla2ecef(p0(k,1:3)); %ecef
%     Cen = dcmecef2ned(p0(k,1), p0(k,2));
    
    rs = sv_info(:,1:3,k);
    rsp = ones(svN,1)*rp - rs; %卫星指向接收机矢量
    rho = sum(rsp.*rsp, 2).^0.5;
    rspu = rsp ./ (rho*[1,1,1]);
    vs = sv_info(:,5:7,k);
    vsp = 0 - vs;
    rhodot = sum(vsp.*rspu, 2);
    output_dr(k,:) = (sv_info(:,4,k) - rho)'; %测量值减计算值
    output_dv(k,:) = (sv_info(:,8,k) - rhodot)';
    rn = (Cen*rspu')'; %地理系下单位矢量
    output_ele(k,:) = asind(rn(:,3))';
    output_azi(k,:) = atan2d(-rn(:,2),-rn(:,1))';
end

%% 输出
assignin('base', 'output_dr', output_dr);
assignin('base', 'output_dv', output_dv);
assignin('base', 'output_ele', output_ele);
assignin('base', 'output_azi', output_azi);

%% 画图
figure
plot(output_dr)
figure
plot(output_dv)

end