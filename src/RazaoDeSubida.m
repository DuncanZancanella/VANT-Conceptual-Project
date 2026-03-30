
% --- Estimativa de Razão de subida
clc;
close all;

% - Seguindo a equação 5.122 do Anderson - Aircraft Performance and Design

RCmax = @(eta_prop, P, W0, S, wing_eff, LD_max, q_inf, K, CD0) eta_prop*(P/W0) - (2/q_inf)*sqrt(K/(3*CD0)*(W0/S))*1.155/LDmax

% - Assumindo características geométricas/aerodinâmicas similares ao TAI Anka-S
eta_prop = 0.87;
wing_eff = 0.9;

hp_to_W = 745.7;
P_hp = 100;       % - Motor similar ao Bayraktar TB2
P_W = P_hp*hp_to_W;

lb_to_kg = 0.453592;
W0_N = [856.18*lb_to_kg*9.81, 1172.2*lb_to_kg*9.81]; % - Peso máximo da missão 1 e 2
Sref_m2 = 13.6; % - área similar ao TAI Anka-S

LDmax = 18;

rho = 1.1117;   % - Densidade do ar em altitude de 1000m, utilizando modelo de atmosfera padrão
v_inf_mps = 250/3.6;  % - Velocidade de cruzeiro [m/s]
q_inf = 0.5*rho*v_inf_mps^2;

AR = 14;
K = 1/(pi*wing_eff*AR);

CD0 = 0.03;   % - Arrasto parasita aproximado

fprintf(' ---- Cáculo de taxa de subida ---- \n')
for i = 1:length(W0_N)
  W_N = W0_N(i);

  RC_max = RCmax(eta_prop, P_W, W_N, Sref_m2, wing_eff, LDmax, q_inf, K, CD0);
  fprintf('Missão %d | W0: %d N | RC_max: %.2f m/s \n', ...
            i, W0_N(i), RC_max);
end

