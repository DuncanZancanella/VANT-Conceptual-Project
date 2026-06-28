import numpy as np


# ---------------------------
# --- Conversões e constantes
lb_to_kg = 0.453592
g = 9.81

# --- Variáveis
W0_lb = 1475.1 
W0_kg = W0_lb*lb_to_kg # Peso total da aeronave (Missão 2)

# --- Peso sobre cada trem de pouso (dianteiro e traseiro)
W_tdp_dianteiro_N = 0.5*W0_kg*g
W_tdp_traseiro_N  = 0.5*W0_kg*g # (considerando os 2 pneus aguentando esse peso)

W_tdp_traseiro_individual_N = W_tdp_traseiro_N/2 # para cada pneu dos dois pneus traseiros

# ---------------------------
# --- Funções

def TireSizing(W_wheel_N, A, B):
    return A*W_wheel_N**B

# --- Cargas Estáticas e Dinâmicas
def MaxStaticLoad(W_wheel_N, N_a, B):
    return W_wheel_N*N_a/B

def MaxStaticLoad_nose(W_wheel_N, M_f, B):
    return W_wheel_N*M_f/B

def MinStaticLoad_nose(W_wheel_N, M_a, B):
    return W_wheel_N*M_a/B

def DynamicBreakingLoad(W_wheel_N, H, B):
    return 10*H*W_wheel_N/(g*B)

def KE_braking(W_landing_N, V_stall):
    return 0.5*W_landing_N/g*(V_stall**2)

# ---------------------------
# --- Cálculos

# Diâmetro categoria Transport
A_diameter = 5.3
B_diameter = 0.315
# Espessura categoria Transport
A_width = 0.39
B_width = 0.48

# - Cálculo TDP dianteiro
d_TDPdianteiro_cm = TireSizing(W_wheel_N=W_tdp_dianteiro_N, A=A_diameter, B=B_diameter)
espessura_TDPdianteiro_cm =  TireSizing(W_wheel_N=W_tdp_dianteiro_N, A=A_width, B=B_width)
# - Cálculo TDP traseiro
d_TDPtraseiro_cm = TireSizing(W_wheel_N=W_tdp_traseiro_individual_N, A=A_diameter, B=B_diameter)
espessura_TDPtraseiro_cm =  TireSizing(W_wheel_N=W_tdp_traseiro_individual_N, A=A_width, B=B_width)


print('---------------------')
print(f'Peso total da aeronave [kg] = {W0_kg:.2f}')
print('--- TDP dianteiro')
print(f'Diâmetro TDP dianteiro [cm] = {d_TDPdianteiro_cm:.2f}')
print(f'Espessura TDP dianteiro [cm] = {espessura_TDPdianteiro_cm:.2f}')
print('--- TDP traseiro')
# Seguindo Raymer 'Nose tires can be assumed to be about 60-100% the size of the main tires.'
# Então utiliza-se o diâmetro do nariz como 75% do valor calculado
print(f'Diâmetro TDP traseiro [cm] = {0.75*d_TDPtraseiro_cm:.2f}')
print(f'Espessura TDP traseiro [cm] = {espessura_TDPtraseiro_cm:.2f}')


# ---------------------------
# --- Cálculos - Cargas Estáticas e Dinâmicas
x_CG_fwd = 3.1 # [m]
x_CG_aft = 3.5 # [m]

x_TDP_dianteiro = 2
x_TDP_traseiro = 4.2
H = 1.1 # [m], altura do CG relativa ao solo
B = (x_TDP_traseiro - x_TDP_dianteiro) # [m], distância entre pneus

N_a = (x_CG_aft - x_TDP_dianteiro)
N_f = (x_CG_fwd - x_TDP_dianteiro)

M_a = (x_TDP_traseiro - x_CG_aft)
M_f = (x_TDP_traseiro - x_CG_fwd)

print(' ---------------------- ')
print(' --- Cotas ---')
print(f'N_a = {N_a:.1f} m')
print(f'N_f = {N_f:.1f} m')
print(f'M_a = {M_a:.1f} m')
print(f'M_f = {M_f:.1f} m')

print(' ---------------------- ')
print(' --- Carregamentos ---')
print(f'Max Static Load = {MaxStaticLoad(W_wheel_N=W_tdp_traseiro_individual_N, N_a=N_a, B=B):.2f} N')
print(f'Max Static Load (nose) = {MaxStaticLoad_nose(W_wheel_N=W_tdp_dianteiro_N, M_f=M_f, B=B):.2f} N')
print(f'Min Static Load (nose) = {MinStaticLoad_nose(W_wheel_N=W_tdp_dianteiro_N, M_a=M_a, B=B):.2f} N')
print(f'Dynamic Breaking Load (nose) = {DynamicBreakingLoad(W_tdp_dianteiro_N, H=H, B=B):.2f} N')

# ---------------------------
# --- Dimensionamento freio
rho = 1.1117
Sref_m2 = 9.86
CLmax = 0.9*1.63

W_landing_emergency_N = 0.80*W0_kg*g
V_stall_mps_emergency = np.sqrt(2*W_landing_emergency_N/(rho*Sref_m2*CLmax))
KE_freio_emergency = KE_braking(W_landing_emergency_N, V_stall_mps_emergency)

Wf_pouso_missao1_N = W0_kg*g * 1.06*(1 - 0.97*0.9850*0.9565*0.8796*0.9565*0.9950)
V_stall_missao1 = np.sqrt(2*Wf_pouso_missao1_N/(rho*Sref_m2*CLmax))
KE_freio_missao1= KE_braking(W_landing_emergency_N, V_stall_missao1)

Wf_pouso_missao2_N = W0_kg*g * 1.06*(1 - 0.970*0.985*0.9286*0.9873*0.6752*0.9286*0.9950)
V_stall_missao2 = np.sqrt(2*Wf_pouso_missao2_N/(rho*Sref_m2*CLmax))
KE_freio_missao2= KE_braking(W_landing_emergency_N, V_stall_missao2)

print(' ---------------------- ')
print(' --- Dimensionamento Freio ---')
print(f'Pouso de Emergência - KE_freio (por roda) = {KE_freio_emergency/3:.2f} J')
print(f'Pouso Comum Missão 1 - KE_freio (por roda) = {KE_freio_missao1/3:.2f} J')
print(f'Pouso Comum Missão 2 - KE_freio (por roda) = {KE_freio_missao2/3:.2f} J')


# ---------------------------
# --- Dimensionamento Amortecedor
in_to_m = 0.0254

rolling_radius = 10.4*in_to_m # Type III, 8.5-10
S_T = d_TDPtraseiro_cm*0.01/2 - rolling_radius
eta_tire = 0.47 # Tabela 11.4 Raymer
eta_schock_absorver = 0.65 # Oleopneumatic Fixed orifice

N_gear = 3

V_vertical = 3 # [m/s]

def Stroke(V_vertical, N_gear, eta, eta_tire, S_T):
    return (V_vertical**2)/(2*g*eta*N_gear) - (eta_tire/eta)*S_T

print(f'Deslocamento do Amortecedor (TDP principal): {Stroke(V_vertical, N_gear, eta_schock_absorver,
                                                        eta_tire, S_T)*100:.1f} cm')

