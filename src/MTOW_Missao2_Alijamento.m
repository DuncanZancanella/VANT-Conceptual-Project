clc
kmh_to_fts = 0.9113446583067 ;
kg_to_lb = 2.2046226218      ;
% --- Calculo MTOW da Missão 2 - Alijamento

Wdrop = 150 * kg_to_lb

% Fracao de peso vazio
A = 1.67  ;
c = -0.16 ;
Kvs = 1   ;
WeW0 = @(W0) A * W0^c * Kvs;

% Warmp up e decolagem
W1W0 = 0.97

% Subida ate Mach M
v = 150/3.6   ; % Velocidade (m/s)
T = 275.1            ; % Temperatura do ar (K)
a = sqrt(1.4*T*287)  ;
M = v/a              ;
W2W1 = 1.0065 - 0.325*M

% Cruzeiro
C = 0.4/3600      ; % Consumo especifico (lb/s)
E = 500/150*3600  ; % Endurance (s)
LDmax = 12        ;
W3W2 = exp(- E*C / LDmax)
W5W4 = W3W2

% Descida
W6W5 = 0.995

% Alijamento
Wf03 = @(W0) 1.06*(1 - W3W2*W2W1*W1W0)*W0;
W3 = @(W0) W0 - Wf03(W0);
W4 = @(W0) W3(W0) - Wdrop;
Wf46 = @(W0) 1.06*(1 - W5W4*W6W5)*(W4(W0));
Wf = @(W0) Wf03(W0) + Wf46(W0);

g = @(W0) W0 - ( Wdrop + WeW0(W0)*W0 + Wf(W0) );

W0 = fzero(g, 1000)
WfW0 = Wf(W0)/W0
Wf = Wf(W0)
We = WeW0(W0)*W0

