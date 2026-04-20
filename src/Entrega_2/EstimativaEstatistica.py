import numpy as np

class Equacionamento():
    
    def __init__(self):
        self.aircraft = {'W/S': [],
                         'P/W': []}
        

        # --- Performance
        self.W0_lb = 1172.2

        lb_to_kg = 0.453592

        self.W0_kg = self.W0_lb*lb_to_kg

        self.eta_prop = 0.8 # propeller efficiency

        # --- Missions
        kmph_to_mps = 1/3.6

        kmph_to_ftps = 0.911344

        self.rho_kgpm3_SL = 1.225 # at Sea Level
        self.rho_kgpm3_1000 = 1.1117 # at 1000m altitude
        self.rho_kgpm3_2000 = 0.94 # at 2000m

        self.V_cruise_mps = 250*kmph_to_mps
        self.V_cruise_fts = 250*kmph_to_ftps
        self.Vmax_mps = 300*kmph_to_mps
        self.Vmax_fts = 300*kmph_to_ftps

        self.RoC_mps = 17 # missão 1, reconhecimento
        
        # --- Geometry
        self.AR = 14

        # --- Aerodynamics
        self.CLmax = 1.63

        self.e = 0.9 # wing efficiency
        
        self.CD0 = 0.03
        self.k = 1/(np.pi*self.AR*self.e)
        self.CD = self.CD0 + self.k*(self.CLmax**2)

        self.LDmax = 18
        self.LDcruise = self.LDmax
        self.LDloiter = self.LDmax*0.86
        self.LDmax_climb = self.LDmax # poderia assumir um valor de proporcionalidade
        

        self.V_stall_mps = np.sqrt( (2*self.W0_kg)/(self.rho_kgpm3_1000*self.CLmax*3.5))
        print(f'V_stall_mps = {self.V_stall_mps:.2f}')

    def P_W_vmax(self):
        a = 0.025
        C = 0.22

        P_W = (a*self.Vmax_fts**C)

        return P_W
    
    def T_Wcruise(self):

        T_W = 1/self.LDcruise

        return T_W
    
    def conversionP_W(self, T_W):
        P_W = T_W/(self.eta_prop*550/self.V_cruise_fts)

        return P_W
    
    def conversionT_W(self, P_W):
        T_W = P_W*(self.eta_prop*550/self.V_cruise_fts)

        return T_W
    
    def P_W_subida(self):
        T_Ws = 1/self.LDcruise + (self.RoC_mps/3.28)/self.V_cruise_fts
        P_Ws = T_Ws/(self.eta_prop*550/self.V_cruise_fts)
        return T_Ws, P_Ws



    def P_W_LD(self):
    # Pelo gráfico 5.2 do Sadraey,escolhendo a curva 0-320:
    
    #para h_takeoff = 1000m --> P = 150 hp
    #para h_serviço  = 3000m --> P = 110 hp
        T_Wc = 1/self.LDcruise
        T_cru = 110*self.eta_prop*550/self.V_cruise_fts
        T_to = 150*self.eta_prop*550/self.V_cruise_fts
        T_Wto = T_Wc*(T_to/T_cru)*(self.W0_lb/1120)
        P_Wto = T_Wto/(self.eta_prop*550/self.V_cruise_fts)
        return T_Wc,T_Wto, P_Wto
    

    def W_S_vstall(self):
        q = 0.5*self.rho_kgpm3_1000*(self.V_stall_mps**2)
        ws_stall = q*self.CLmax
        return ws_stall

    def W_S_decolagem(self,P_W_est):
        ws = 500*self.rho_kgpm3_2000/self.rho_kgpm3_SL*P_W_est
        return ws

    def W_S_pouso(self,S_pouso):
        return (S_pouso - 0/3.28)*(self.rho_kgpm3_1000/self.rho_kgpm3_SL)*self.CLmax
        

aircraft = Equacionamento()

#estimativa velocidade máxima:
P_W_vmax = aircraft.P_W_vmax()
P_W_vmax_si = P_W_vmax*745.7/0.453592
print('P/W para velocidade máxima: (hp/lb)', P_W_vmax)
print('P/W para velocidade máxima: (W/kg)', P_W_vmax_si)


#estimativa P/W por L/D:
T_W_cru,T_W_to,P_W_to = aircraft.P_W_LD()
print('T/W por L/D',T_W_to)
print('P/W por L/D corrigido (hp/lb):' ,P_W_to)
print('P/W por L/D corrigido  (W/kg):' ,P_W_to*745.7/0.453592)

#estimativa P_W para subida:
T_W_subida, P_W_subida = aircraft.P_W_subida()
print('T/W subida:',T_W_subida)
print('P/W subida: (hp/lb)',P_W_subida)
print('P/W subida: (W/kg)',P_W_subida*745.7/0.453592)


W_S_stall_si = aircraft.W_S_vstall()
W_S_stall = W_S_stall_si*0.2048
print('W/S para Vstall: (kg/m^2)', W_S_stall_si)
print('W/S para Vstall: (lb/ft^2)', W_S_stall)

#decolagem assumindo P/W=115 W/kg
P_W_est = 120*0.453592/745.7
W_S_decolagem = aircraft.W_S_decolagem(P_W_est)
W_S_decolagem_si = W_S_decolagem/0.2048
print('W/S para decolagem: (kg/m^2)', W_S_decolagem_si)
print('W/S para decolagem: (lb/ft^2)', W_S_decolagem)

#pouso assumindo pista de 150m
S_pouso = 150 
W_S_pouso_si = aircraft.W_S_pouso(S_pouso)
W_S_pouso = W_S_pouso_si*0.2048
print('W/S para pouso: (kg/m^2)', W_S_pouso_si)
print('W/S para pouso: (lb/ft^2)', W_S_pouso)

