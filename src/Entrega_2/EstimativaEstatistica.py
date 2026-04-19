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

        self.V_cruise_mps = 250*kmph_to_mps
        self.V_cruise_fts = 250*kmph_to_ftps
        self.Vmax_mps = 300*kmph_to_mps
        self.Vmax_fts = 300*kmph_to_ftps

        self.RoC_mps = 17 # missão 1, reconhecimento
        
        # --- Geometry
        self.AR = 14

        # --- Aerodynamics
        self.CLmax = 1.8

        self.e = 0.9 # wing efficiency
        
        self.CD0 = 0.03
        self.k = 1/(np.pi*self.AR*self.e)
        self.CD = self.CD0 + self.k*(self.CLmax**2)

        self.LDmax = 18
        self.LDcruise = self.LDmax
        self.LDloiter = self.LDmax*0.86
        self.LDmax_climb = self.LDmax # poderia assumir um valor de proporcionalidade
        

        self.V_stall_mps = np.sqrt( (2*self.W0_kg)/(2*self.rho_kgpm3_SL*self.CLmax))

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
    
    def conversionCruiseTakeoff(self,P_Wcruise):
        # Pelo gráfico 5.2 do Sadraey,escolhendo a curva 0-320:
    
    #para h_takeoff = 1000m --> P = 150 hp
    #para h_serviço  = 3000m --> P = 110 hp
        Pto_Pc = 150/110
        W_takeoff = self.W0_lb
        W_cruise1 = 1120
 
        P_Wtakeoff = P_Wcruise*(W_cruise1/self.W0_lb)*(Pto_Pc)

        return P_Wtakeoff
    
    def W_S_cruise(self):
        q = 0.5*self.rho_kgpm3_SL*(self.V_cruise_mps**2)
        ws = q*np.sqrt(np.pi*self.e*self.AR*self.CD0)
        return ws
    
    def W_S_cruise_loiter(self):
        q = 0.5*self.rho_kgpm3_SL*(self.V_cruise_mps**2)
        wsl = q*np.sqrt(3*np.pi*self.e*self.AR*self.CD0)
        return wsl

    
aircraft = Equacionamento()

#estimativa velocidade máxima:
P_W_vmax = aircraft.P_W_vmax()
P_W_vmax_si = P_W_vmax*745.7/0.453592
print('P/W para velocidade máxima: (hp/lb)', P_W_vmax)
print('P/W para velocidade máxima: (W/kg)', P_W_vmax_si)
print(aircraft.conversionT_W(P_W_vmax))

#estimativa P/W para cruzeiro:
T_Wcruise = aircraft.T_Wcruise()
P_Wcruise = aircraft.conversionP_W(T_Wcruise)
P_Wtakeoff = aircraft.conversionCruiseTakeoff(P_Wcruise)
P_Wtakeoff_si = P_Wtakeoff*745.7/0.453592
print('P/W: (hp/lb)', P_Wtakeoff)
print('P/W: (W/kg)', P_Wtakeoff_si)

#estimativa W/S para cruzeiro:
W_S_cruise_si = aircraft.W_S_cruise()
W_S_cruise = W_S_cruise_si*0.2048
print('W/S para cruzeiro: (kg/m^2)', W_S_cruise_si)
print('W/S para cruzeiro: (lb/ft^2)', W_S_cruise)

W_S_cruise_loiter_si = aircraft.W_S_cruise_loiter()
W_S_cruise_loiter = W_S_cruise_loiter_si*0.2048
print('W/S para loiter: (kg/m^2)', W_S_cruise_loiter_si)
print('W/S para loiter: (lb/ft^2)', W_S_cruise_loiter)
