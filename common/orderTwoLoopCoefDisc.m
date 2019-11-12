function [K1, K2] = orderTwoLoopCoefDisc(Bn, zeta, T)
% 离散二阶环路系数
% 参见《Software Defined Radio using MATLAB Simulink and the RTL-SDR》 P601

theta = Bn*T / (zeta+0.25/zeta);

K1 = 4*zeta*theta / (1+2*zeta*theta+theta^2) / T;
K2 = 4*theta^2 / (1+2*zeta*theta+theta^2) / T;

end