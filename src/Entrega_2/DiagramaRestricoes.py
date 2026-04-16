
import numpy as np
from matplotlib import pyplot as plt

class AircraftEnvelope():
    
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

        self.rho_kgpm3_SL = 1.225 # at Sea Level

        self.V_cruise_mps = 250*kmph_to_mps
        self.Vmax_mps = 300*kmph_to_mps

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
        self.LDmax_climb = self.LDmax # poderia assumir um valor de proporcionalidade
        

        self.V_stall_mps = np.sqrt( (2*self.W0_kg)/(2*self.rho_kgpm3_SL*self.CLmax))

    def stall_restriction(self, rho_kgpm3):
        """
        Maximum W/S restriction based on Raymer's equation 5.6

        --- --- ---
        INPUTS

        rho [kg/m3]

        V_stall_mps

        CLmax
        --- --- ---
        return W/S

        """
        W_S = 0.5*rho_kgpm3*(self.V_stall_mps**2)*self.CLmax

        return W_S
    
    def takeoff_restriction(self, W_S:np.array, rho_kgpm3, TOP = 500) -> np.array:
        """
        Take-off curve restriction, based on Raymer's equation 5.8
        

        --- --- ---
        TOP = take-off parameter, estimated using Figure 5.4 from Raymer

        rho_kgpm3 = local air density
        """
        
        sigma = rho_kgpm3/self.rho_kgpm3 # density ratio using water sea-level as reference

        CL_TO = 0.90 * self.CLmax # FAR23 approximation

        hp_per_W = W_S/(TOP*sigma*CL_TO)

        return hp_per_W
    
    def takeoff_restriction_Sadraey(self, W_S:np.array, S_TO, mu, pitch_ratio = 0.5):
        """
        Take-off restriction equation based on Sadraey equation 4.75 
        
        --- --- ---
        INPUTS

        W_S = W/S array input [kg/m2]

        S_TO = take-off distance [m]

        mu = total rolling friction [-]

        pitch_ratio = 0.6 for variable pitch propeller or 0.5 for fixed pitch propeller

        """
        fixed_pitch_ratio = 0.5
        variable_pitch_ratio = 0.6

        V_TO_mps = 1.2 * self.V_stall_mps # FAR approximation
        CL_TO = 0.90 * self.CLmax

        K = pitch_ratio*9.81*self.CD*S_TO/W_S
        
        num = mu - (mu + self.CD/CL_TO)*( np.exp(K) )
        den = 1 - np.exp(K)

        P_W = (V_TO_mps/self.eta_prop)*(num/den)

        return P_W


    def maximumSpeed_restriction(self, W_S:np.array, rho_kgpm3):
        """
        Maximum speed restriction, Sadraey equation 4.56

        --- --- ---
        Usually, the maximum speed is assumed between 20% or 30% higher than the cruise speed.
        --- --- ---
        INPUTS

        W_S = W/S [kg/m^2]

        rho = local air density

        """
        Vmax_mps = self.Vmax_mps

        sigma = rho_kgpm3/self.rho_kgpm3_SL

        den = 0.5*self.rho_kgpm3_SL*(Vmax_mps**3)*self.CD0/W_S + (2*self.k)/(rho_kgpm3*sigma*Vmax_mps)*W_S

        W_P = self.eta_prop/den

        return W_P
    
    def RoC_restriction(self, W_S:np.array, RoC_mps, rho_kgpm3, eta_prop_climb = 0.7):
        """
        Rate of climb restriction, based on Sadraey 4.89

        --- --- ---
        INPUTS

        RoC_mps = rate of climb of the specified mission

        eta_prop_climb = propeller efficiency in climb, usually between 0.5 or 0.7 for a cruise-optimized propeller
        
        """
        P_W = (RoC_mps/eta_prop_climb) + np.sqrt( 2*W_S/( rho_kgpm3 * np.sqrt(3*self.CD0/self.k )  ) ) * (1.155/self.LDmax_climb*eta_prop_climb)

        return 1/P_W
    
    def Ceiling_restriction(self, W_S:np.array, RoC_ceiling, rho_kgpm3_ceiling, eta_prop_climb = 0.7):
        """
        Absolute or service ceiling restriction, based on Sadraey equation 4.100

        Similar to the RoC restriction, however RoC depends on the type of ceiling defined.

        --- --- ---
        
        """
        sigma = rho_kgpm3_ceiling/self.rho_kgpm3_SL

        P_W = (RoC_ceiling/eta_prop_climb) + np.sqrt( 2*W_S/( rho_kgpm3_ceiling * np.sqrt(3*self.CD0/self.k )  ) ) * (1.155/self.LDmax_climb*eta_prop_climb)

        return sigma/P_W
    
    def Range_restriction(self, W_S, rho_kgpm3, R_m = 500):
        """
        Equação deduzida
        
        """
        c_bhp = 0.068#*10**6 # kg/Ws

        P_W = (self.eta_prop/(R_m*c_bhp))*np.log(1/0.9286)*np.sqrt(W_S*2/(rho_kgpm3*self.CLmax))
        return 1/P_W
    
    def cruise_restriction(self, W_S, rho_kgpm3):
        """
        
        """
        self.V_cruise_mps = 300/3.6
        q = 0.5*rho_kgpm3*(self.V_cruise_mps**2)*20

        T_W = (q*self.CD0)/(W_S) + (self.k/q)*(W_S)

        return 1/(T_W*self.V_cruise_mps/(self.eta_prop))
    
    def show_RestrictionDiagram(self):
        W_S_array = np.linspace(0.5, 500, 1000)

        W_S_stall = self.stall_restriction(rho_kgpm3=1.225)

        # --- 1) Requisito de cruzeiro
        plt.axvline(x = W_S_stall, linestyle = ':', color='black', label = 'Estol') # região atrás atende

        # --- 2) Requisito de decolagem
        P_W_takeoff = aircraft.takeoff_restriction_Sadraey(W_S_array, S_TO = (2/3)*150, mu = 0.05)
        plt.plot(W_S_array, 1/P_W_takeoff, linestyle='-', linewidth=2, color='black',label='Decolagem') # região abaixo atende

        # --- 3) Requisito de velocidade máxima (assumido 30% acima da velocidade de cruzeiro)
        W_P_maxspeed =  aircraft.maximumSpeed_restriction(W_S_array, rho_kgpm3=1.225)
        plt.plot(W_S_array, W_P_maxspeed, label='Máx. velocidade', linestyle='--', linewidth=2, color='black')

        # --- 4) Requisito de razão de subida
        W_P_roc = aircraft.RoC_restriction(W_S_array, RoC_mps=17, rho_kgpm3=1.225)
        plt.plot(W_S_array, W_P_roc, label='Taxa de subida', linestyle='-.', linewidth=2, color='black')

        # --- 5) Requisito de teto abosluto
        W_P_ceiling = aircraft.Ceiling_restriction(W_S_array, RoC_ceiling=5, rho_kgpm3_ceiling=1.1)
        plt.plot(W_S_array, W_P_ceiling, label='Teto absoluto', linestyle='-', marker='x', markevery=50, linewidth=2, color='grey')

        # --- 6) Requisito de alcance
        #W_P_range = aircraft.Range_restriC', linction(W_S_array, rho_kgpm3=1.225)
        #plt.plot(W_S_array, W_P_range, label='Alcance', linestyle='-.', linewidth=2, color='yellow')

        # --- 7) Requisito de cruzeiro
        #W_P_cruise = aircraft.cruise_restriction(W_S_array, rho_kgpm3=1.225)
        #plt.plot(W_S_array, W_P_cruise, label='Cruzeiro', linestyle='-.', linewidth=2, color='yellow')

        plt.fill_between(W_S_array, 1/P_W_takeoff, np.linspace(0, 0, 1000), alpha=0.1)
        plt.fill_between(W_S_array, W_P_ceiling, np.linspace(0, 0, 1000), alpha=0.2)
        plt.fill_between(W_S_array, W_P_maxspeed, np.linspace(0, 0, 1000), alpha=0.3)

        plt.xlabel('W/S [kg/m^2]')
        plt.ylabel('W/P [kg/W]')
        #plt.ylim(0, 0.2)

        plt.title('Diagrama de Restrições - Metodologia Sadraey')
        plt.legend()
        plt.savefig('RestrictionsDiagram.png')


aircraft = AircraftEnvelope()
aircraft.show_RestrictionDiagram()


