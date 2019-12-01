classdef navFilter_pv_ecef < handle
% ecef系PV模型导航滤波器

    properties (GetAccess = public, SetAccess = private)
        pos       %位置，[lat,lon,h]，[deg,deg,m]
        vel       %速度，[ve,vn,vd]，m/s
        dtr       %钟差，s
        dtv       %钟频差，s/s
        T         %更新周期，s
        Pk        %滤波器P阵
        Qk        %滤波器Q阵
    end %end properties
    
    properties (Access = private) %导航递推使用的变量
        Pxyz      %位置，[x,y,z]，m（行向量）
        Vxyz      %速度，[vx,vy,vz]，m/s（行向量）
    end
    
    methods
        %% 构造
        function obj = navFilter_pv_ecef(p0, v0, T, para)
            % p0：初始位置，[lat,lon,h]，deg
            % v0：初始速度，[ve,vn,vd]，m/s
            % T：更新周期，s
            % para：滤波器参数
            obj.pos = p0;
            obj.vel = v0;
            obj.dtr = 0;
            obj.dtv = 0;
            obj.T = T;
            obj.Pk = para.P;
            obj.Qk = para.Q;
            Cen = dcmecef2ned(p0(1), p0(2));
            obj.Pxyz = lla2ecef(p0);
            obj.Vxyz = v0*Cen;
        end
        
        %% 更新
        function [rho, rhodot] = update(obj, sv, sv_m, sigma)
            % sv = [x,y,z, vx,vy,vz]，卫星位置、速度
            % sv_m = [rho_m, rhodot_m]，测量值，伪距、伪距率
            % sigma = [sigma_rho, sigma_rhodot]，测量值噪声标准差
            % rho, rhodot：使用滤波后位置、速度计算的伪距、伪距率
            
%             Cen = dcmecef2ned(obj.pos(1), obj.pos(2)); %这个函数运行慢
            lat = obj.pos(1) /180*pi; %rad
            lon = obj.pos(2) /180*pi; %rad
            Cen = [-sin(lat)*cos(lon), -sin(lat)*sin(lon),  cos(lat);
                            -sin(lon),           cos(lon),         0;
                   -cos(lat)*cos(lon), -cos(lat)*sin(lon), -sin(lat)];
            
            %==========导航状态递推========================================%
            v = obj.Vxyz; %速度不变
            p = obj.Pxyz + v*obj.T;
            
            %==========滤波器状态方程======================================%
            A = zeros(8);
            A(1,4) = 1;
            A(2,5) = 1;
            A(3,6) = 1;
            A(7,8) = 1;
            Phi = eye(8) + A*obj.T;
            
            %==========量测维数============================================%
            index1 = find(~isnan(sv_m(:,1)))';  %伪距量测索引
            index2 = find(~isnan(sv_m(:,2)))';  %伪距率量测索引
            n1 = length(index1);                %伪距量测个数
            n2 = length(index2);                %伪距率量测个数
            n  = size(sv,1);                    %通道个数
            
            %==========状态变量============================================%
            X = zeros(8,1);
            X(7) = obj.dtr;
            X(8) = obj.dtv;
            
            %==========时间更新============================================%
            X = Phi*X;
            obj.Pk = Phi*obj.Pk*Phi' + obj.Qk;
            
            %==========量测更新============================================%
            if n1>0
                % 1. 计算伪距（所有通道都算，不管有没有数）
                rp = p;                                     %接收机位置矢量（ecef，行向量）
                rs = sv(:,1:3);                             %卫星位置矢量（ecef，行向量）
                rsp = ones(n,1)*rp - rs;                    %卫星指向接收机的位置矢量
                rho = sum(rsp.*rsp, 2).^0.5;                %计算的伪距
                rspu = rsp ./ (rho*[1,1,1]);                %卫星指向接收机的单位矢量（ecef）
                % 2. 计算伪距率（所有通道都算，不管有没有数）
                vp = v;                                     %接收机速度矢量（ecef，行向量）
                vs = sv(:,4:6);                             %卫星速度矢量（ecef，行向量）
                vsp = ones(n,1)*vp - vs;                    %接收机相对卫星的速度矢量
                rhodot = sum(vsp.*rspu, 2);                 %计算的伪距率
                % 3. 量测矩阵、量测量、量测噪声方差阵
                Ha = rspu(index1,:); %取有效的行
                Hb = rspu(index2,:);
                H = zeros(n1+n2, 8);
                H(1:n1,1:3) = Ha;
                H(1:n1,7) = -ones(n1,1)*299792458;
                H((n1+1):end,4:6) = Hb;
                H((n1+1):end,8) = -ones(n2,1)*299792458;
                Z = [   rho(index1) - sv_m(index1,1); ... %伪距差（计算减量测）
                     rhodot(index2) - sv_m(index2,2)];    %伪距率差
                R = diag([sigma(index1,1)', ...
                          sigma(index2,2)'])^2;
                % 4. 滤波更新
                P = obj.Pk;
                K = P*H' / (H*P*H'+R);
                X = X + K*(Z-H*X);
                P = (eye(length(X))-K*H)*P;
                obj.Pk = (P+P')/2;
            end
            
            %==========导航修正============================================%
            obj.Pxyz = p - X(1:3)';
            obj.Vxyz = v - X(4:6)';
            obj.dtr = X(7);
            obj.dtv = X(8);
            
            %==========导航输出============================================%
            obj.pos = ecef2lla(obj.Pxyz); %纬经高，deg
            obj.vel = obj.Vxyz*Cen'; %地理系下速度，m/s
            
            %==========根据滤波结果计算伪距、伪距率=========================%
            % 计算伪距
            rp = obj.Pxyz;
            rs = sv(:,1:3);
            rsp = ones(n,1)*rp - rs;
            rho = sum(rsp.*rsp, 2).^0.5;
            rspu = rsp ./ (rho*[1,1,1]);
            % 计算伪距率
            vp = obj.Vxyz;
            vs = sv(:,4:6);
            vsp = ones(n,1)*vp - vs;
            rhodot = sum(vsp.*rspu, 2);
            % 把钟差、钟频差算上
            rho = rho + obj.dtr*299792458; %钟快使测量的伪距变长
            rhodot = rhodot + obj.dtv*299792458; %钟快使测量的伪距率变大
            
        end
        
    end %end methods
    
end %end classdef