
% --- Estimativa de Razão de subida

% - Seguindo a equação 5.122 do Anderson - Aircraft Performance and Design
clc;
close all;

RCmax = @(eta_prop, P, W0, S, wing_eff, LD_max, q_inf, K, CD0) eta_prop*(P/W0) - (2/q_inf)*sqrt(K/(3*CD0)*(W0/S))*1.155/LDmax
%RCmax = @(eta_prop, P_W, W0_kg, S_m2, rho_inf, CD0, LDmax) (eta_prop*P_W/W0_kg) - 0.8776*sqrt( (W0_kg/S_m2)/(rho_inf*CD0) )/(LDmax^(3/2) )

eta_prop = 0.87;
hp_to_W = 745.7;

P_hp = 155;
P_W = P_hp*hp_to_W;

W0_N = 800*9.81;
Sref_m2 = 13.6;

wing_eff = 0.9;

LDmax = 12;

rho = 1.1117;
v_inf_mps = 250/3.6;
q_inf = 0.5*rho*v_inf_mps^2;

AR = 8;
K = 1/(pi*wing_eff*AR);

CD0 = 0.03;

RC_max = RCmax(eta_prop, P_W, W0_N, Sref_m2, wing_eff, LD_max, q_inf, K, CD0)
%RC_max = RCmax(eta_prop, P_W, W0_kg, Sref_m2, q_inf, CD0, LDmax)

