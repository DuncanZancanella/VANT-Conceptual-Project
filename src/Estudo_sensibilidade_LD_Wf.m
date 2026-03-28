
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

AR_wet_extrapolado = 1:0.1:4.5;
LDmax_regressao_extrapolada = a * AR_wet_extrapolado.^b;;

fprintf('\n=== Resultados da Regressão ===\n');
fprintf('a = %.6f\n', a);
fprintf('b = %.6f\n', b);
fprintf('Modelo: L/D_max = %.6f * AR_wet^{%.6f}\n\n', a, b);

% - 3) Plot
figure;

plot(AR_wet, LDmax, 'ko', 'MarkerSize', 6, 'LineWidth', 5);
hold on;

plot(AR_wet_extrapolado, LDmax_regressao_extrapolada, 'r--', 'LineWidth', 3);

xlabel('AR_{wet}', 'FontSize', 18);
ylabel('L/D_{max}', 'FontSize', 18);
title('L/D_{max} vs AR_{wet} - Fixed gear prop aircraft', 'FontSize', 14);

%legend('Dados Raymer', 'Regressão e extrapolação', 'Location', 'northwest', 'FontSize', 15);

grid on;
box on;

% - 5) Sensibilidade: ARwet -> L/D -> no alijamento e Wf, W0

# - inicialização de vetores para salvar dados
AR_vec = 1:0.1:4;
n = length(AR_vec);

AR_wet_arr = zeros(1,n);
LD_arr     = zeros(1,n);
Wf_arr     = zeros(1,n);
W0_arr     = zeros(1,n);

Wf_arr_missao1     = zeros(1,n);
W0_arr_missao1     = zeros(1,n);


# --- Calculo W0 da Missão 1 - Reconhecimento


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
E_h = 2; % Tempo de Loiter
E_s = E_h * 3600; %

h_m = 1000; % altitude de decolagem
l_pista = 150; % comprimento da pista
h_s = 2000; % teto de serviço

v = Vc_kmh/3.6   ; % Velocidade (m/s)
T = 275.1            ; % Temperatura do ar (K)
a_som = sqrt(1.4*T*287)  ;
M = v/a_som        ;


% -- Estimativa da aeronave:

for i = 1:n
  AR_wet_i = AR_vec(i);
  LD_i_missao1 = a * AR_wet_i.^b;

  Swet_Sref = 5; %Raymer fig 3.6 --> Beech Duchess
  Areawet = 1.8; %Raymer fig 3.5

  LD_max = LD_i_missao1;
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
  Kvs = 1;
  WeW0 = @(W0)  A*(W0^c) * Kvs;

  % -- Peso Combustível

  % W1/W0
  W1W0 = 0.970;

  % W2/W1
  W2W1 = 0.985; %1.0065 - 0.0325*M;

  % W3/W2
  W3W2 = exp(-R_ft*C_c_s/(Vc_fts*LD_c));

  % W4/W3
  W4W3 = exp(-E_s*C_l_s/(LD_l));

  % W5/W4
  W5W4 = W3W2;

  % W6/W5
  W6W5 = 0.995;

  WfW0 = 1.06*(1 - W1W0*W2W1*W3W2*W4W3*W5W4*W6W5);

  % --- Resolvendo W0
  R = @(W0) W0 - (Wpl + WfW0*W0 + WeW0(W0)*W0) ;
  W0 = fzero(R, 200);

  We = WeW0(W0)*W0;
  Wf = WfW0*W0;

  # --- salvar dados
  Wf_arr_missao1(i)     = Wf;
  W0_arr_missao1(i)     = W0;

end


% --- Calculo W0 da Missão 2 - Alijamento

Wdrop = 150 * kg_to_lb;


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
  W2W1 = 0.985; %1.0065 - 0.0325*M;

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

figure('Position', [100 100 900 600]);  % tamanho maior

% --- eixo principal
ax1 = gca;

plot(LD_arr, Wf_arr/kg_to_lb, 'b-', 'LineWidth', 5); hold on;
plot(LD_arr, Wf_arr_missao1/kg_to_lb, 'r-.', 'LineWidth', 5); hold on;


xlabel('L/D_{max}', 'FontSize', 25);
ylabel('Peso [kg]', 'FontSize', 25);

set(gca, 'FontSize', 18);   % ticks maiores

legend('Peso de Combustível (Wf) - Missão 2: Transporte de Insumos  ', ...
       'Peso de Combustível (Wf) - Missão 1: Reconhecimento', 'Location', 'northeast', ...
       'FontSize', 25);

grid on;
box on;

% --- eixo superior
ax2 = axes('Position', get(ax1, 'Position'), ...
           'XAxisLocation', 'top', ...
           'Color', 'none');

set(ax2, 'YTick', []);
set(ax2, 'YColor', 'none');

set(ax2, 'XLim', get(ax1, 'XLim'));

xt = get(ax1, 'XTick');

AR_ticks = zeros(size(xt));
for k = 1:length(xt)
    [~, idx] = min(abs(LD_arr - xt(k)));
    AR_ticks(k) = AR_wet_arr(idx);
end

set(ax2, 'XTick', xt);
set(ax2, 'XTickLabel', num2str(AR_ticks', '%.2f'));
set(ax2, 'FontSize', 18);

linkaxes([ax1, ax2], 'x');

xlabel(ax2, 'AR_{wet}', 'FontSize', 25);

axes(ax1);
set(gca, 'YTick', 0:25:250);
set(gca, 'XTick', 8:1:25);

