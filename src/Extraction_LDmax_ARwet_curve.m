
% - 0) Definições e conversões de unidades
clc;
close all;

kmh_to_fts = 0.9113446583067 ;
kg_to_lb = 2.2046226218      ;
km_to_ft = 3280.84;

% - 1) Ler arquivo de dados para extrair pontos L/Dmax = L/Dmax(AR_wet)
file = "C:\\Users\\dunca\\Desktop\\26.1\\EVA\\VANT-Conceptual-Project\\src\\Raymer_data\\Raymer_data_fixed_gear_prop_aircraft.csv";

LD_ARwet_data = dlmread(file, ";");% pula a primeira linha

AR_wet = LD_ARwet_data(:, 1);
LDmax = LD_ARwet_data(:, 2);

% - 2) Realizar regressão na forma LDmax = a*AR_wet^b

logX = log(AR_wet);
logY = log(LDmax);

regressao = polyfit(logX, logY, 1);
b = regressao(1);
a = exp(regressao(2));

LDmax_regressao = a * AR_wet.^b;

fprintf('\n=== Resultados da Regressão ===\n');
fprintf('a = %.6f\n', a);
fprintf('b = %.6f\n', b);
fprintf('Modelo: L/D_max = %.6f * AR_wet^{%.6f}\n\n', a, b);

% - 3) Plot
figure;

plot(AR_wet, LDmax, 'ko', 'MarkerSize', 6, 'LineWidth', 1.5);
hold on;

plot(AR_wet, LDmax_regressao, 'r--', 'LineWidth', 2);

xlabel('AR_{wet}', 'FontSize', 12);
ylabel('L/D_{max}', 'FontSize', 12);
title('L/D_{max} vs AR_{wet} - Fixed gear prop aircraft', 'FontSize', 14);

legend('Dados Raymer', 'Modelo de regressão', 'Location', 'northwest');

grid on;
box on;

% - 5) Sensibilidade: ARwet -> L/D -> no alijamento e Wf, W0

% --- Calculo MTOW da Missão 2 - Alijamento

Wdrop = 150 * kg_to_lb

AR_vec = 1:0.1:2.6;
n = length(AR_vec);

AR_wet_arr = zeros(1,n);
LD_arr     = zeros(1,n);
Wf_arr     = zeros(1,n);
W0_arr     = zeros(1,n);

for i = 1:n

  AR_wet_i = AR_vec(i);
  LD_i = a * AR_wet_i.^b;
  % Fracao de peso vazio
  A = 1.67  ;
  c = -0.16 ;
  Kvs = 1   ;
  WeW0 = @(W0) A * W0^c * Kvs;

  % Warmp up e decolagem
  W1W0 = 0.97;

  % Subida ate Mach M
  v = 150/3.6   ; % Velocidade (m/s)
  T = 275.1            ; % Temperatura do ar (K)
  a_mps = sqrt(1.4*T*287)  ;
  M = v/a_mps             ;
  W2W1 = 1.0065 - 0.325*M;

  % Cruzeiro
  C = 0.4/3600        ; % Consumo especifico (lb/s)
  R = 500*km_to_ft  ; % Range (ft)
  V = 150*kmh_to_fts; % Velocidade (ft/s)
  LDmax = LD_i         ;
  W3W2 = exp(-R*C / (V*LDmax) );
  W5W4 = W3W2;

  % Descida
  W6W5 = 0.995;

  % Alijamento
  Wf03 = @(W0) 1.06*(1 - W3W2*W2W1*W1W0)*W0;
  W3 = @(W0) W0 - Wf03(W0);
  W4 = @(W0) W3(W0) - Wdrop;
  Wf46 = @(W0) 1.06*(1 - W5W4*W6W5)*(W4(W0));
  Wf_function = @(W0) Wf03(W0) + Wf46(W0);

  g = @(W0) W0 - ( Wdrop + WeW0(W0)*W0 + Wf_function(W0) );

  W0 = fzero(g, 1000);
  WfW0 = Wf_function(W0)/W0;
  Wf = Wf_function(W0);
  We = WeW0(W0)*W0;

  % --- Salvar dados
  AR_wet_arr(i) = AR_wet_i;
  LD_arr(i)     = LD_i;
  Wf_arr(i)     = Wf;
  W0_arr(i)     = W0;

end

% - 5.1) Plot de sensibilidade

figure;

plot(LD_arr, Wf_arr/kg_to_lb, 'b-', 'MarkerSize', 6, 'LineWidth', 1.5);
hold on;

plot(LD_arr, W0_arr/kg_to_lb, 'r--', 'LineWidth', 2);

xlabel('L/D_{max}', 'FontSize', 12);
ylabel('Peso [kg]', 'FontSize', 12);
title('Pesos característicos - Missão 2: alijamento', 'FontSize', 14);

legend('Wf', 'W0', 'Location', 'northwest');
set(gca, 'YTick', 0:50:800);
grid on;
box on;

% --- sensibilidade 2
figure;

plot(AR_wet_arr, Wf_arr/kg_to_lb, 'b-', 'MarkerSize', 6, 'LineWidth', 1.5);
hold on;

plot(AR_wet_arr, W0_arr/kg_to_lb, 'r--', 'LineWidth', 2);

xlabel('AR_{wet}', 'FontSize', 12);
ylabel('Peso [kg]', 'FontSize', 12);
title('Pesos característicos - Missão 2: alijamento', 'FontSize', 14);

legend('Wf', 'W0', 'Location', 'northwest');
set(gca, 'YTick', 0:50:800);
set(gca, 'XTick', 0.4:0.2:3);
grid on;
box on;

