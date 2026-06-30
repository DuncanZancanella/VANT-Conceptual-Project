import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

data_pesos_1 = pd.DataFrame(columns=['Componente', 'Peso (lb)', 'Posição (m)'])
data_pesos_1['Componente'] = ['Asa', 'EH', 'EV', 'Fus', 'LG', 'I. EN.', 'Fuel S', 'FC', 'HYD', 'ELEC', 'AV','CP','Fuel']  
data_pesos_1['Posição (m)'] = [3.6686,6.02392,6.02392,2.85728,2.897,5.142,4.52,4.5,1.7125,1.101,1.101,0.585,3.6686]

data_pesos_2 = pd.DataFrame(columns=['Componente', 'Peso (lb)', 'Posição (m)'])
data_pesos_2['Componente'] = ['Asa', 'EH', 'EV', 'Fus', 'LG', 'I. EN.', 'Fuel S', 'FC', 'HYD', 'ELEC', 'AV','CP','Fuel']  
data_pesos_2['Posição (m)'] = [3.6686,6.02392,6.02392,2.85728,2.897,5.142,4.52,4.5,1.7125,1.101,1.101,3.077,3.6686]

class Weights:
    def __init__(self):
        self.design_weight = 1475.1 * 0.8 #lb
        self.limit_load_factor = 2.5
        self.Nz = 1.5* self.limit_load_factor #fator de carga último
        self.S_w = 100.75 #ft^2
        self.A_w = 14
        self.t_c = 0.1 #thickness to chord ratio
        self.lamb =  0.7 #afilamento
        self.sweep = 0
        self.S_csw = self.S_w *0.2
        self.F_w = (3.340+2.44)*3.281 #distância fuselagem até interceptar cauda
        self.B_h = 2.37 # envergadura horizontal tail
        self.S_ht = 10.2257 #ft^2
        self.H_tv = 1 #empenagem convencional?
        self.S_vt = 19.1597 #ft^2
        self.M = 0.2089 #mach design
        self.L_t = 2.44*3.281#tail legnth (MAC asa to MAC cauda)
        self.S_rt = self.S_vt * 0.2 #área do leme
        self.A_vt = 4
        self.sweep_t = 0
        self.l_f = 7.1432*3.281 #ft fuselage length
        self.D_f = 0.9*3.281 #ft fuselage depth
        self.w_f = 0.9*3.281 #ft fuselage width
        self.W_l = 901.42 #peso no pouso
        N_gear = 2
        self.N_l = 1.5*N_gear #fator carga pouso
        self.L_n = 1.1*3.281 #ft nose landing gear length
        self.N_nw = 1 #rodas nose lg
        self.W_en = 163.142 #lb peso motor
        self.N_en = 1 #num motores
        self.fuel_vol_total = 46.874 #gal
        self.Vi_Vt = 1 #integral tanks volume/ Total
        self.N_t = 1 #num tanks
        self.W_uav = 22 #lb peso aviônica não instalada --> 10kg
        self.W_fw = 293.39 #lb peso combustível asa
        self.q = 0.5*1.1117*(250/3.6)**2*0.224809
        self.lamb_ht = 0.7
        self.S_f = 191.167 #ft^2
        self.L_D = 18
        self.b_w = 11.75*3.281 #ft wing span

    def wing_weight(self):
        W_wing = 0.036*self.S_w**0.758*(self.design_weight*self.Nz)**0.49*(self.A_w**0.6)*(self.lamb**0.04)*(np.cos(self.sweep)**-0.3)*(100*self.t_c/np.cos(self.sweep))**-0.3*self.W_fw**0.0035*self.q**0.006
        return W_wing
    
    def ht_weight(self):
        W_ht = 0.016*((self.Nz * self.design_weight)**0.414)*(self.q**0.168)*(self.S_ht)**0.896*((100*self.t_c/np.cos(self.lamb))**-0.12)*((self.A_w /((np.cos(self.lamb_ht))**2))** 0.043)*self.lamb_ht**-0.02
        return W_ht
    
    def vt_weight(self):
        W_vt = 0.073*(1 + 0.2*self.H_tv)*((self.Nz*self.design_weight)**0.376)*(self.q**0.122)*(self.S_vt**0.873)*((100*self.t_c/np.cos(self.lamb_ht))**-0.49)*((self.A_vt/((np.cos(self.lamb_ht))**2))**0.357)*self.lamb_ht**0.039
        return W_vt

    def fus_weight(self):
        W_fus =  0.052*(self.S_f**1.086)*((self.Nz*self.design_weight)**0.177)*(self.L_t**-0.051)*(self.L_D)**-0.072*(self.q**0.241) + 0 #sem penalidade pressurização
        return W_fus

    def landing_gear_weight(self):
        W_nlg = 0.125*((self.W_l*self.N_l)**0.566)*(self.L_n/12)**0.845
        W_mlg = 0.095*((self.W_l*self.N_l)**0.768)*(self.L_n/12)**0.409
        W_lg = W_nlg + W_mlg
        return W_lg
    
    def installed_engine_weight(self):
        W_eng = 2.575*(self.W_en**0.922)*self.N_en
        return W_eng
    
    def fuel_system_weight(self):
        W_fs = 2.49*(self.fuel_vol_total**0.726)*(1/(1+self.Vi_Vt))**0.363*(self.N_t**0.242)*self.N_en**0.157
        return W_fs
    
    def flight_control_weight(self):
        W_fc = 0.053*self.l_f**1.536*self.b_w**0.371*(self.Nz*self.design_weight*10**-4)**0.8
        return W_fc
    
    
    def hyd_weight(self):
        W_hyd = 0.11*self.design_weight**0.8*self.M**0.5
        return W_hyd
    
    def avionics_weight(self):
        W_avionics = 2.117*self.W_uav**0.933
        return W_avionics

    def elec_weight(self):
        W_elec = 12.57*((2.117*self.W_uav**0.933) + (2.49*(self.fuel_vol_total**0.726)*(1/(1+self.Vi_Vt))**0.363*(self.N_t**0.242)*self.N_en**0.157))**0.51
        return W_elec

 
Pesos = Weights()

payload_weight_1 = 11.02
payload_weight_2 = 330.69
fuel_weight_1 = 293.39
fuel_weight_2 = 270.61

data_pesos_1['Peso (lb)'] = [Pesos.wing_weight(), Pesos.ht_weight(), Pesos.vt_weight(), Pesos.fus_weight(), Pesos.landing_gear_weight(), Pesos.installed_engine_weight(), Pesos.fuel_system_weight(), Pesos.flight_control_weight(), Pesos.hyd_weight(), Pesos.elec_weight(), Pesos.avionics_weight(), payload_weight_1, fuel_weight_1]
data_pesos_2['Peso (lb)'] = [Pesos.wing_weight(), Pesos.ht_weight(), Pesos.vt_weight(), Pesos.fus_weight(), Pesos.landing_gear_weight(), Pesos.installed_engine_weight(), Pesos.fuel_system_weight(), Pesos.flight_control_weight(), Pesos.hyd_weight(), Pesos.elec_weight(), Pesos.avionics_weight(), payload_weight_2, fuel_weight_2]


Total_empty = data_pesos_1['Peso (lb)'][:11].sum()
Total_1 = data_pesos_1['Peso (lb)'].sum()
Total_2 = data_pesos_2['Peso (lb)'].sum()

CG_empty = data_pesos_1['Peso (lb)'][:11].dot(data_pesos_1['Posição (m)'][:11]) / Total_empty

CG_full_1 = data_pesos_1['Peso (lb)'].dot(data_pesos_1['Posição (m)']) / Total_1

CG_full_2 = data_pesos_2['Peso (lb)'].dot(data_pesos_2['Posição (m)']) / Total_2


data_pesos_ali = pd.DataFrame(columns=['Componente', 'Peso (lb)', 'Posição (m)'])
data_pesos_ali['Componente'] = ['Asa', 'EH', 'EV', 'Fus', 'LG', 'I. EN.', 'Fuel S', 'FC', 'HYD', 'ELEC', 'AV','CP','Fuel']
data_pesos_ali['Peso (lb)'] = data_pesos_1['Peso (lb)'].copy()
data_pesos_ali['Posição (m)'] = data_pesos_1['Posição (m)'].copy()

data_pesos_ali.loc[data_pesos_ali['Componente'] == 'Fuel', 'Peso (lb)'] = 87.59
data_pesos_ali.loc[data_pesos_ali['Componente'] == 'CP', 'Peso (lb)'] = 0

CG_alij = data_pesos_ali['Peso (lb)'].dot(data_pesos_ali['Posição (m)']) / data_pesos_ali['Peso (lb)'].sum()

Total_ali = data_pesos_ali['Peso (lb)'].sum()


print(f"Peso vazio estimado: {Total_empty:.2f} lb")
print(f"Peso total 1 estimado: {Total_1:.2f} lb")
print(f"Peso total 2 estimado: {Total_2:.2f} lb")
print(f"Peso total após alijamento estimado: {Total_ali:.2f} lb")

print(f"Centro de gravidade vazio estimado: {CG_empty:.2f} m")
print(f"Centro de gravidade cheio 1 estimado: {CG_full_1:.2f} m")
print(f"Centro de gravidade cheio 2 estimado: {CG_full_2:.2f} m")
print(f"Centro de gravidade após alijamento estimado: {CG_alij:.2f} m")
