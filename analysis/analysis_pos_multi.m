function analysis_pos_multi

%% 导入数据
output = evalin('base', 'output');
n = output.n;

%% 申请空间
pos.GPS = NaN(n,8);
pos.BDS = NaN(n,8);
pos.GPS_BDS = NaN(n,8);

%% 计算
for k=1:n
    %----GPS
    sv_GPS_A = output.sv_GPS_A(:,:,k);
    sv = sv_GPS_A;
    sv(isnan(sv(:,1)),:) = []; %删除无效卫星
    pos.GPS(k,:) = pos_solve_weight(sv(:,1:8), sv(:,9));
    
    %----北斗
    sv_BDS_A = output.sv_BDS_A(:,:,k);
    sv = sv_BDS_A;
    sv(isnan(sv(:,1)),:) = []; %删除无效卫星
    pos.BDS(k,:) = pos_solve_weight(sv(:,1:8), sv(:,9));
    
    %----GPS+北斗
    sv = [sv_GPS_A; sv_BDS_A];
    sv(isnan(sv(:,1)),:) = []; %删除无效卫星
    pos.GPS_BDS(k,:) = pos_solve_weight(sv(:,1:8), sv(:,9));
end

%% 画图
p0 = mean(output.pos(:,1:3), 1, 'omitnan');
p0_ecef = lla2ecef(p0);
Cen = dcmecef2ned(p0(1),p0(2));
gn.GPS = NaN(n,3);
gn.BDS = NaN(n,3);
gn.GPS_BDS = NaN(n,3);
for k=1:n
    gn.GPS(k,:) = (Cen*(lla2ecef(pos.GPS(k,1:3))-p0_ecef)')';
    gn.BDS(k,:) = (Cen*(lla2ecef(pos.BDS(k,1:3))-p0_ecef)')';
    gn.GPS_BDS(k,:) = (Cen*(lla2ecef(pos.GPS_BDS(k,1:3))-p0_ecef)')';
end
figure
plot3(gn.GPS(:,1),gn.GPS(:,2),-gn.GPS(:,3))
hold on
plot3(gn.BDS(:,1),gn.BDS(:,2),-gn.BDS(:,3))
plot3(gn.GPS_BDS(:,1),gn.GPS_BDS(:,2),-gn.GPS_BDS(:,3))
grid on
axis equal
rotate3d on

%% 输出
assignin('base', 'pos', pos);

end