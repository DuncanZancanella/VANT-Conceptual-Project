clc;
close all;
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

Wmotor = 164.2444 % lb

% Fracao de peso vazio
A = 1.67  ;
c = -0.16 ;
Kvs = 1   ;

cargaAlar = 14.4;
potenciaPeso = 0.067;

a  = 0;
b  =  0.69;
c1 = -0.10;
c2 =  0.05;
c3 =  0.1;
c4 = -0.05;
c5 =  0.17;

AR = 14;
Vmax = 161.987;

WeW0 = @(W0) A * W0^c * Kvs;

WeW0 = @(W0) a + b*W0^c1*AR^c2*potenciaPeso^c3*(cargaAlar)^c4*Vmax^c5;

% Warmp up e decolagem
W1W0 = 0.97

% Subida ate Mach M
v = 150/3.6          ; % Velocidade (m/s)
T = 275.1            ; % Temperatura do ar (K)
a = sqrt(1.4*T*287)  ;
M = v/a              ;
%W2W1 = 1.0065 - 0.0325*M
W2W1 = 0.985

% Cruzeiro
C = 0.4/3600        ; % Consumo especifico (lb/s)
R = 500*km_to_ft    ; % Range (ft)
V = 150*kmh_to_fts  ; % Velocidade (ft/s)
LDmax = 18         ;
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

g = @(W0) W0 - ( Wdrop + WeW0(W0)*W0 + Wf(W0) + Wmotor);

W0 = fzero(g, 1000)
WfW0 = Wf(W0)/W0
Wf = Wf(W0)
We = WeW0(W0)*W0

% --- plot
Wf = WfW0*W0
W1 = W1W0 * W0;
W2 = W2W1 * W1;
W3 = W3W2 * W2;
W4 = W4W3 * W3;
W5 = W4 - Wdrop;
W6 = W6W5 * W5;
W7 = W7W6 * W6

x = [0,1,2,3,4,5,6, 7];
plot(x,[W0,W1,W2,W3,W4,W5,W6,W7], 'b--o', 'LineWidth', 3)
grid('on')

title('Evolução do Peso na Missão 2', 'FontSize', 25)
xlabel('Etapa', 'FontSize', 25)
ylabel('Peso (lb)', 'FontSize', 25)

set(gca, 'FontSize', 20)  % controla os números dos eixos (ticks)

