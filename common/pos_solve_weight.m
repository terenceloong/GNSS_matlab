function pos = pos_solve_weight(sv, R)
% 定位解算，加权，天向速度
% sv = [x,y,z, rho, vx,vy,vz, rhodot]
% pos = [lon, lat, h, vn, ne, vu, dtr, dtv]
% R为各卫星量测的方差

W = diag(1./R); %权值，方差的逆

if size(sv,1)<4
    pos = NaN(1,8);
    return
end

pos = zeros(1,8);
n = size(sv,1);

G = -ones(n,4);
R = sv(:,4); %rho, m
V = sv(:,8); %rhodot, m/s
x0 = [0;0;0;0];
% S = zeros(n,1);

cnt = 0;
while 1
%     for k=1:n
%         G(k,1:3) = sv(k,1:3)-x0(1:3)';
%         G(k,1:3) = G(k,1:3)/norm(G(k,1:3));
%         S(k) = G(k,1:3)*sv(k,1:3)';
%     end
        G1 = sv(:,1:3) - ones(n,1)*x0(1:3)';
        rho = sum(G1.*G1, 2).^0.5;
        G1 = G1 ./ (rho*[1,1,1]);
        G(:,1:3) = G1;
        S = sum(G1.*sv(:,1:3), 2);
    x = (G'*W*G)\G'*W*(S-R);
    cnt = cnt+1;
%     if cnt == 10
%         error('Position iteration exceeds the threshold!');
%     end
    if norm(x-x0)<1e-3 || cnt==20
%     if norm(x-x0)<1e-3
        break
    end
    x0 = x;
end
pos(1:3) = ecef2lla(x(1:3)'); %[lat, lon, h], deg
pos(7) = x(4)/299792458; %钟差, s

% for k=1:n
%     G(k,1:3) = sv(k,1:3)-x(1:3)';
%     G(k,1:3) = G(k,1:3)/norm(G(k,1:3));
%     S(k) = G(k,1:3)*sv(k,5:7)';
% end
    G1 = sv(:,1:3) - ones(n,1)*x(1:3)';
    rho = sum(G1.*G1, 2).^0.5;
    G1 = G1 ./ (rho*[1,1,1]);
    G(:,1:3) = G1;
    S = sum(G1.*sv(:,5:7), 2);
v = (G'*W*G)\G'*W*(S-V);
Cen = dcmecef2ned(pos(1), pos(2));
pos(4:6) = (Cen*v(1:3))'; %[vn, ve, vd], m/s
pos(6) = -pos(6);
pos(8) = v(4)/299792458; %钟频差

end