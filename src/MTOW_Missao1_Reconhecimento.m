clc
kmh_to_fts = 0.9113446583067 ;
kg_to_lb = 2.2046226218      ;
km_to_ft = 3280.84;

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
CP_lb = CP_kg * kg_to_lb ;
R_km = 500; % Alcance no cruzeiro
R_ft = R_km * km_to_ft;
Vc_kmh = 250; % Velocidade de cruzeiro
Vc_fts = Vc_kmh * kmh_to_fts;
E_h = 4; % Tempo de Loiter
E_s = E_h * 3600; %

h_m = 1000; % altitude de decolagem
l_pista = 150; % comprimento da pista
h_s = 2000; % teto de serviço

v = Vc_kmh/3.6   ; % Velocidade (m/s)
T = 275.1            ; % Temperatura do ar (K)
a = sqrt(1.4*T*287)  ;
M = v/a              ;



% -- Estimativa da aeronave:
Swet_Sref = 3.5;
LD_max = 18;
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
Kvs = 1
%WeW0 = %@(W0)  A*(W0^c) * Kvs;
We = 631.95 % valor código missão 2
% -- Peso Combustível

% W1/W0
W1W0 = 0.970

% W2/W1
W2W1 = 0.985 %1.0065 - 0.0325*M

% W3/W2
W3W2 = exp(-R_ft*C_c_s/(Vc_fts*LD_c))

% W4/W3
W4W3 = exp(-E_s*C_l_s/(LD_l))

% W5/W4
W5W4 = W3W2

% W6/W5
W6W5 = 0.995

WfW0 = 1.06*(1 - W1W0*W2W1*W3W2*W4W3*W5W4*W6W5);

% --- Resolvendo W0
R = @(W0) W0 - (Wpl + WfW0*W0 + We) ;
W0 = fzero(R, 200)

Wf = WfW0*W0


Wf = WfW0*W0
W1 = W1W0*W0
W2 = W2W1*W1
W3 = W3W2*W2
W4 = W4W3*W3
W5 = W5W4*W4
W6 = W6W5*W5
Wf/W0

x = [0,1,2,3,4,5,6];
plot(x,[W0,W1,W2,W3,W4,W5,W6], 'b--o')
title('Evolução do Peso na Missão 1')
grid('on')




