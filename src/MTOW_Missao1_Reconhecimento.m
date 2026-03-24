
% --- Calculo MTOW da Missão 1 ---

% -- Considerando na missão:
% 0 - 1 Decolagem
% 1 - 2 Climb
% 2 - 3 Cruzeiro de 500 km a 250 km/h
% 3 - 4 Loiter de 2 horas
% 4 - 5 Cruzeiro de 500 km a 250 km/h
% 5 - 6 Pouso

% -- Pré-requisitos:
CP_kg = 5;
CP_lb = CP_kg * 2.2046 ;
R_km = 500; % Alcance no cruzeiro
R_ft = R_km * 3281;
Vc_kmh = 250; % Velocidade de cruzeiro
Vc_fts = Vc_kmh/ 1.097;
E_h = 2; % Tempo de Loiter
E_s = E_h*3600; %

h_m = 1000; % altitude de decolagem
l_pista = 150; % comprimento da pista
h_s = 2000; % teto de serviço

V_sound_kmh = 1204;
V_sound_fts = V_sound_kmh/ 1.097;

M = Vc_fts/V_sound_fts


% -- Estimativa da aeronave:
Swet_Sref = 5; %Raymer fig 3.6 --> Beech Duchess
Areawet = 1.8; %Raymer fig 3.5
LD_max = 12;
LD_c = LD_max;
LD_l = 0.866 * LD_max;

C_c_hr = 0.4; %consumo específico cruzeiro
C_c_s = C_c_hr/3600;

C_l_hr = 0.5; %consumo específico cruzeiro
C_l_s = C_l_hr/3600;

A = 1.67;
Kvs = 1;
c = -0.16;

% -- Peso Payload
Wpl = CP_lb;

% -- Peso Vazio
%We =  A*(W0^c)*W0;

% -- Peso Combustível

% W1/W0
W1W0 = 0.970;

% W2/W1
W2W1 = 1.0065 - 0.0325*M;

% W3/W2
arg_exp_c = (R_ft*C_c_s/(Vc_fts*LD_c));
W3W2 = exp(-arg_exp_c);

% W4/W3
arg_exp_l = (-E_s*C_l_s/(LD_l));
W4W3 = exp(arg_exp_l);

% W5/W4
W5W4 = W3W2;

% W6/W5
W6W5 = 0.995;

WfW0 = 1.06*(1 - W1W0*W2W1*W3W2*W4W3*W5W4*W6W5);

% --- Resolvendo W0
R = @(W0) W0 - (Wpl + WfW0*W0 + A*(W0^c)*W0) ;
W0 = fzero(R, 200)




