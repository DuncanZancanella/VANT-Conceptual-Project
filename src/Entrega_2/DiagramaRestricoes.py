
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
        self.rho_kgpm3_1000m = 1.1117 # at 1000m altitude

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
        
        k_stall = 3.5 # adjust stall curve to fit range of known aircraft
        self.V_stall_mps = np.sqrt( (2*self.W0_kg)/(self.rho_kgpm3_1000m*self.CLmax*k_stall))
        print(f'V_stall_mps = {self.V_stall_mps:.2f}')

class Sadraey_Methods(AircraftEnvelope):

    def __init__(self):
        super().__init__()

    def stall_restriction(self, rho_kgpm3):
        """
        Maximum W/S restriction based on Sadraey's equation 4.31

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
    
    
    def takeoff_restriction(self, W_S:np.array, S_TO, rho_kgpm3, mu, pitch_ratio = 0.5):
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
        CL_TO = 0.3#0.90 * self.CLmax
        CD_TO = self.CD0 + self.k*self.CLmax**2
        CD_G = CD_TO - mu*CL_TO

        K = pitch_ratio*rho_kgpm3*9.81*self.CD0*S_TO/W_S
        
        num = mu - (mu + self.CD0/CL_TO)*( np.exp(K) )
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
        Vmax_mps = self.V_cruise_mps # used cruise speed as airspeed requirement for constant speed at specified altitude

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
        P_W = (RoC_mps/eta_prop_climb) + np.sqrt( 2*W_S/( rho_kgpm3 * np.sqrt(3*self.CD0/self.k )  ) ) * (1.155/(self.LDmax_climb*eta_prop_climb))

        return 1/P_W
    
    def Ceiling_restriction(self, W_S:np.array, RoC_ceiling, rho_kgpm3_ceiling, eta_prop_climb = 0.7):
        """
        Absolute or service ceiling restriction, based on Sadraey equation 4.100

        Similar to the RoC restriction, however RoC depends on the type of ceiling defined.

        --- --- ---
        
        """
        sigma = rho_kgpm3_ceiling/self.rho_kgpm3_SL

        num = (RoC_ceiling/self.eta_prop) + np.sqrt( 2*W_S/(rho_kgpm3_ceiling*np.sqrt(3*self.CD0/self.k) ) )*(1.155/(self.LDmax*self.eta_prop))

        P_W = num/sigma

        return P_W
    
    def Range_restriction(self, W_S, rho_kgpm3, R_m = 500*10**3):
        """
        Equação deduzida
        
        """
        c_bhp = 0.068*10**(-6) # kg/Ws

        C = c_bhp#*self.V_cruise_mps

        P_W = (self.eta_prop/(R_m*C))*np.log(1/0.9286)*np.sqrt(9.81*W_S*2/(rho_kgpm3*self.CLmax))
        return P_W
    
    def cruise_restriction(self, W_S, rho_kgpm3):
        """
        
        """
        self.V_cruise_mps = 300/3.6
        q = 0.5*rho_kgpm3*(self.V_cruise_mps**2)*20

        T_W = (q*self.CD0)/(W_S) + (self.k/q)*(W_S)

        return 1/(T_W*self.V_cruise_mps/(self.eta_prop))
    
    def show_RestrictionDiagram(self):
        plt.rcParams.update({
            'font.family': 'Arial',
            'font.size': 10,
            'axes.linewidth': 0.8,
            'axes.grid': True,
            'grid.linestyle': '--',
            'grid.linewidth': 0.4,
            'grid.alpha': 0.5,
            'legend.framealpha': 0.95,
            'legend.edgecolor': '0.6',
            'legend.fontsize': 9,
        })

        fig, ax = plt.subplots(figsize=(8, 6), dpi=150)

        W_S_stall = self.stall_restriction(rho_kgpm3=1.225)
        W_S_array = np.linspace(0.5, W_S_stall, 1000)

        # --- Paleta monocromática
        cores = {
            'estol':     ('#000000', ':',          1.5),
            'decolagem': ('#000000', '-',          2.0),
            'cruzeiro':  ('#444444', '-',         2.0),
            'subida':    ('#222222', (0, (5, 2)),  2.0),
            'teto':      ('#777777', '-.',         1.8),
        }

        # --- 1) Estol
        ax.axvline(
            x=W_S_stall,
            linestyle=cores['estol'][1],
            linewidth=cores['estol'][2],
            color=cores['estol'][0],
            label='Limite de estol',
            zorder=5
        )

        # --- 2) Decolagem
        P_W_takeoff = self.takeoff_restriction(
            W_S_array, S_TO=(2/3)*150, rho_kgpm3=1.1117, mu=0.05)
        ax.plot(
            W_S_array, P_W_takeoff,
            linestyle=cores['decolagem'][1],
            linewidth=cores['decolagem'][2],
            color=cores['decolagem'][0],
            label='Decolagem',
            zorder=4
        )

        # --- 3) Cruzeiro
        W_P_maxspeed = self.maximumSpeed_restriction(W_S_array, rho_kgpm3=1.1117)
        ax.plot(
            W_S_array, 1/W_P_maxspeed,
            linestyle=cores['cruzeiro'][1],
            linewidth=cores['cruzeiro'][2],
            color=cores['cruzeiro'][0],
            label='Cruzeiro',
            zorder=4,
            marker='x', markevery=50, markersize=5,
        )

        # --- 4) Taxa de subida
        W_P_roc = self.RoC_restriction(W_S_array, RoC_mps=17, rho_kgpm3=1.1117)
        ax.plot(
            W_S_array, 1/W_P_roc,
            linestyle=cores['subida'][1],
            linewidth=cores['subida'][2],
            color=cores['subida'][0],
            label='Taxa de subida',
            zorder=4
        )

        # --- 5) Teto absoluto
        P_W_ceiling = self.Ceiling_restriction(
            W_S_array, RoC_ceiling=5.0, rho_kgpm3_ceiling=1.1117)
        ax.plot(
            W_S_array, P_W_ceiling,
            linestyle=cores['teto'][1],
            linewidth=cores['teto'][2],
            color=cores['teto'][0],
            label='Teto de serviço',
            zorder=4
        )

        # --- Sombreamento monocromático
        cap = np.full(1000, 150.0)
        ax.fill_between(W_S_array, 1/W_P_maxspeed, cap, alpha=0.18, color='#444444', label='Região viável')
    
        # --- Formatação dos eixos
        ax.set_xlabel(r'$W/S$  [kg/m²]', fontsize=11)
        ax.set_ylabel(r'$P/W$  [W/kg]',  fontsize=11)
        ax.set_ylim(0, 150)
        ax.tick_params(direction='in', length=4, width=0.6, top=True, right=True)

        # --- Eixos secundários (unidades imperiais)
        # Fatores: 1hp = 745.7W, 1kg = 2.2046lb, 1m = 3.28084ft
        secax_x = ax.secondary_xaxis(
            'top',
            functions=(
                lambda x: x * 2.2046 / 3.28084**2,
                lambda x: x / 2.2046 * 3.28084**2
            )
        )
        secax_x.set_xlabel(r'$W/S$  [lb/ft²]', fontsize=11)
        secax_x.tick_params(direction='in', length=4, width=0.6)

        secax_y = ax.secondary_yaxis(
            'right',
            functions=(
                lambda y: y / 745.7 / 2.2046,
                lambda y: y * 745.7 * 2.2046
            )
        )
        secax_y.set_ylabel(r'$P/W$  [hp/lb]', fontsize=11)
        secax_y.tick_params(direction='in', length=4, width=0.6)

        #ax.scatter(60, 128.2, marker='*', s=80, color='blue', label='Ponto de Projeto')


        # --- Legenda
        ax.legend(
            loc='upper left',
            frameon=True,
            ncol=1,
            title='Restrições de projeto',
            title_fontsize=9,
            handlelength=2.5
        )

        # --- Título
        ax.set_title(
            'Diagrama de Restrições — Metodologia Sadraey',
            fontsize=11, fontweight='bold', loc='center', pad=10
        )

        # --- Grade secundária
        ax.minorticks_on()
        ax.grid(which='minor', linestyle=':', linewidth=0.2, alpha=0.4)

        fig.tight_layout()
        fig.savefig('RestrictionsDiagram_Sadraey.pdf', dpi=300, bbox_inches='tight')        #plt.show()


class Gudmundsson_Methods(AircraftEnvelope):

    def __init__(self):
        super().__init__()

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
    
    def takeoff_restriction(self, W_S:np.array, S_G_m, rho_kgpm3, mu) -> np.array:
        """
        Take-off curve restriction, based on Gudmundsson's equation 3-4
        

        --- --- ---
        S_G_m = total take-off distance [m]

        mu = total ground roll friction [-]

        rho_kgpm3 = local air density
        """
        V_LOF_mps = 1.2 * self.V_stall_mps # FAR 23 approximation
        CL_TO = 0.9 * self.CLmax           # FAR 23 approximation

        q_Pa = 0.5*rho_kgpm3*V_LOF_mps**2

        T_W = (V_LOF_mps**2)/(2*9.81*S_G_m) + q_Pa*self.CD/W_S + mu*(1 - (q_Pa*CL_TO)/W_S)

        # --- converting to power
        P_W = T_W * V_LOF_mps/self.eta_prop

        return P_W
    
    def cruise_restriction(self, W_S:np.array, rho_kgpm3):
        """
        Cruise airspeed restriction, based on Gudmundsson equation 3-5
        
        --- --- ---
        rho_kgpm3 = local air density
        """

        q_Pa = 0.5 * rho_kgpm3 * self.V_cruise_mps**2

        T_W = q_Pa*self.CD0/W_S + self.k/q_Pa*W_S

        # --- converting to power
        P_W = T_W * self.V_cruise_mps/self.eta_prop

        return P_W
    
    def service_ceiling_restriction(self, W_S:np.array, rho_kgpm3, RoC_mps = 0.508):
        """
        Service ceiling restriction, based on Gudmundsson equation 3-6

        --- --- ---
        rho_kgpm3 = local air density
        """
        T_W = RoC_mps/(np.sqrt( (2/rho_kgpm3)*W_S*np.sqrt( self.k/(3*self.CD))    )) + 4*np.sqrt(self.k*self.CD/3)

        # --- converting to power
        P_W = T_W * self.V_cruise_mps/self.eta_prop

        return P_W
    
    def climb_restriction(self, W_S:np.array, rho_kgpm3, RoC_mps):
        """
        Rate of climb restriction, based on Gudmundsson equation 3-6

        --- --- ---
        rho_kgpm3 = local air density
        """
        q_Pa = 0.5*rho_kgpm3*(1.1*self.V_stall_mps)**2 # during V_LOF

        V_climb_mps = self.V_cruise_mps

        T_W = RoC_mps/V_climb_mps + q_Pa/W_S*self.CD + self.k/q_Pa*W_S

        # --- converting to power
        P_W = T_W * self.V_cruise_mps/self.eta_prop

        return P_W
    
    def sustained_turn_restriction(self, W_S:np.array, rho_kgpm3, bank_angle_deg):
        """
        Level-constant velocity turn restriction, based on Gudmundsson equation 3-6

        --- --- ---
        rho_kgpm3 = local air density
        """
        q_Pa = 0.5*rho_kgpm3*(1.1*self.V_stall_mps)**2

        n = 1/np.cos( np.deg2rad(bank_angle_deg) )

        T_W = q_Pa*(self.CD/W_S + self.k*W_S*(n/q_Pa)**2)

        # --- converting to power
        P_W = T_W * self.V_cruise_mps/self.eta_prop

        return P_W

    def show_RestrictionDiagram(self):
        plt.rcParams.update({
            'font.family': 'Arial',
            'font.size': 10,
            'axes.linewidth': 0.8,
            'axes.grid': True,
            'grid.linestyle': '--',
            'grid.linewidth': 0.4,
            'grid.alpha': 0.5,
            'legend.framealpha': 0.95,
            'legend.edgecolor': '0.6',
            'legend.fontsize': 9,
        })

        fig, ax = plt.subplots(figsize=(8, 6), dpi=150)

        W_S_stall = self.stall_restriction(rho_kgpm3=1.1117)
        W_S_array = np.linspace(0.5, W_S_stall, 1000)

        # --- Paleta monocromática 
        cores = {
            'estol':      ('#000000', ':',   1.5),
            'decolagem':  ('#000000', '-',   2.0),
            'cruzeiro':   ('#444444', '-',  2.0),
            'teto':       ('#777777', '-.',  1.8),
            'subida':     ('#222222', (0,(5,2)), 2.0),
            'curva':      ('#555555', '-', 2.0),
        }

        # --- 1) Estol
        ax.axvline(
            x=W_S_stall,
            linestyle=cores['estol'][1],
            linewidth=cores['estol'][2],
            color=cores['estol'][0],
            label='Limite de estol',
            zorder=5
        )

        # --- 2) Decolagem
        P_W_takeoff = self.takeoff_restriction(W_S_array, S_G_m=150, rho_kgpm3=1.1117, mu=0.03)
        ax.plot(
            W_S_array, P_W_takeoff,
            linestyle=cores['decolagem'][1],
            linewidth=cores['decolagem'][2],
            color=cores['decolagem'][0],
            label='Decolagem',
            zorder=4
        )

        # --- 3) Cruzeiro
        P_W_cruise = self.cruise_restriction(W_S_array, rho_kgpm3=1.1117)
        ax.plot(
            W_S_array, P_W_cruise,
            linestyle=cores['cruzeiro'][1],
            linewidth=cores['cruzeiro'][2],
            color=cores['cruzeiro'][0],
            label='Cruzeiro',
            zorder=4,
            marker='x', markevery=50, markersize=5
        )

        # --- 4) Teto de serviço
        P_W_ceiling = self.service_ceiling_restriction(W_S_array, rho_kgpm3=1.0066)
        ax.plot(
            W_S_array, P_W_ceiling,
            linestyle=cores['teto'][1],
            linewidth=cores['teto'][2],
            color=cores['teto'][0],
            #marker='x', markevery=50, markersize=5,
            label='Teto de serviço',
            zorder=4
        )

        # --- 5) Taxa de subida
        P_W_roc = self.climb_restriction(W_S_array, rho_kgpm3=1.1117, RoC_mps=17)
        ax.plot(
            W_S_array, P_W_roc,
            linestyle=cores['subida'][1],
            linewidth=cores['subida'][2],
            color=cores['subida'][0],
            label='Taxa de subida',
            zorder=4
        )

        # --- 6) Curva sustentada
        P_W_curve = self.sustained_turn_restriction(W_S_array, rho_kgpm3=1.1117, bank_angle_deg=45)
        ax.plot(
            W_S_array, P_W_curve,
            linestyle=cores['curva'][1],
            linewidth=cores['curva'][2],
            color=cores['curva'][0],
            label='Curva sustentada',
            zorder=4,
            marker='s', markevery=50, markersize=5
        )

        #ax.scatter(60, 128.2, marker='*', s=80, color='blue', label='Ponto de Projeto')

        # --- Formatação dos eixos
        ax.set_xlabel(r'$W/S$  [kg/m²]', fontsize=11)
        ax.set_ylabel(r'$P/W$  [W/kg]',  fontsize=11)
        ax.set_ylim(0, 200)
        ax.tick_params(direction='in', length=4, width=0.6, top=True, right=True)

        # --- Eixos secundários 
        # 1hp = 745.7W, 1kg = 2.2046lb, 1m = 3.28084ft

        secax_x = ax.secondary_xaxis(
            'top',
            functions=(
                lambda x: x * 2.2046 / 3.28084**2,   # kg/m² → lb/ft²
                lambda x: x / 2.2046 * 3.28084**2    # lb/ft² → kg/m²
            )
        )
        secax_x.set_xlabel(r'$W/S$  [lb/ft²]', fontsize=11)
        secax_x.tick_params(direction='in', length=4, width=0.6)

        secax_y = ax.secondary_yaxis(
            'right',
            functions=(
                lambda y: y / 745.7 / 2.2046,   # W/kg → hp/lb
                lambda y: y * 745.7 * 2.2046    # hp/lb → W/kg
            )
        )
        secax_y.set_ylabel(r'$P/W$  [hp/lb]', fontsize=11)
        secax_y.tick_params(direction='in', length=4, width=0.6)

        ax.fill_between(W_S_array, P_W_cruise,     [200]*1000,
                        alpha=0.2, color='#888888', label='Região viável')
        
        # --- Legenda
        ax.legend(
            loc='upper left',
            frameon=True,
            ncol=1,
            title='Restrições de projeto',
            title_fontsize=9,
            handlelength=2.5
        )

        # --- Título
        ax.set_title(
            'Diagrama de Restrições — Metodologia Gudmundsson',
            fontsize=11, fontweight='bold', loc='center', pad=10
        )

        fig.tight_layout()
        fig.savefig('RestrictionsDiagram_Gudmundsson.pdf', dpi=300, bbox_inches='tight')
        #plt.show()

diagram_sadraey =  Sadraey_Methods()
diagram_sadraey.show_RestrictionDiagram()

diagram_Gudmundsson = Gudmundsson_Methods()
diagram_Gudmundsson.show_RestrictionDiagram()


