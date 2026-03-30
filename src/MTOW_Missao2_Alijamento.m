clc
kmh_to_fts = 0.9113446583067 ;
kg_to_lb = 2.2046226218      ;
km_to_ft = 3280.84           ;

% --- Calculo MTOW da Missão 2 ---

% -- Considerando na missão:
% 0 - 1 Decolagem
% 1 - 2 Climb
% 2 - 3 Cruzeiro de 500 km a 150 km/h
% 3 - 4 Loiter de 0.5 horas
% 4 - 5 Alijamento
% 5 - 6 Cruzeiro de 500 km a 150 km/h
% 6 - 7 Pouso

Wdrop = 150 * kg_to_lb

% Fracao de peso vazio
A = 1.67  ;
c = -0.16 ;
Kvs = 1   ;
WeW0 = @(W0) A * W0^c * Kvs;

% Warmp up e decolagem
W1W0 = 0.97

% Subida ate Mach M
v = 150/3.6          ; % Velocidade (m/s)
T = 275.1            ; % Temperatura do ar (K)
a = sqrt(1.4*T*287)  ;
M = v/a              ;
W2W1 = 1.0065 - 0.0325*M
W2W1 = 0.985

% Cruzeiro
C = 0.4/3600        ; % Consumo especifico (lb/s)
R = 500*km_to_ft    ; % Range (ft)
V = 150*kmh_to_fts  ; % Velocidade (ft/s)
LDmax = 12          ;
W3W2 = exp(-R*C / (V*LDmax) )
W6W5 = W3W2

% Loiter
E = 0.5*3600 ;
W4W3 = exp(-E*C/(0.866*LDmax))

% Descida
W7W6 = 0.995

% Alijamento
Wf04 = @(W0) 1.06*(1 - W4W3*W3W2*W2W1*W1W0)*W0;
W4 = @(W0) W0 - Wf04(W0);
W5 = @(W0) W4(W0) - Wdrop;
Wf57 = @(W0) 1.06*(1 - W7W6*W6W5)*(W5(W0));
Wf = @(W0) Wf04(W0) + Wf57(W0);

g = @(W0) W0 - ( Wdrop + WeW0(W0)*W0 + Wf(W0) );

W0 = fzero(g, 1000)
WfW0 = Wf(W0)/W0
Wf = Wf(W0)
We = WeW0(W0)*W0

