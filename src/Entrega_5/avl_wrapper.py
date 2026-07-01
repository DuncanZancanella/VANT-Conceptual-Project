import numpy as np
import optvl
import matplotlib.pyplot as plt

# CRUISE ANALYSIS
rho = 1.225 # kg/m3
v = 250/3.6 # m/s
W = 728*9.81 # N
CD0 = 0.01729

ovl = optvl.OVLSolver(geo_file="cruise.avl")
ref_data = ovl.get_reference_data()
Sref = ref_data['Sref']
Bref = ref_data['Bref']
Cref = ref_data['Cref']

CL_trim = W / ((0.5 * rho * v**2) * Sref)
ovl.set_constraint("alpha", "CL", CL_trim)
ovl.set_constraint("Elevator", "Cm", 0)
ovl.execute_run()
print(f"Neutral point: {ovl.get_stab_derivs()['neutral point']:.2f}")

results = []
for CL in np.linspace(0.2, 1.4, 20):
    ovl.set_constraint("alpha", "CL", CL)
    ovl.set_constraint("Elevator", "Cm", 0)
    ovl.execute_run()
    f = ovl.get_total_forces()
    results.append({
        "CL": f["CL"],
        "CDi": f["CD"] + CD0,
        "alpha": ovl.get_variable("alpha"),
        "elevator": ovl.get_control_deflection('Elevator')
    })

CLs = [r["CL"] for r in results]
CDs = [r["CDi"] for r in results] 

plt.figure(figsize=(6, 5))
plt.plot(CDs, CLs, 'o-')
plt.xlabel("CD")
plt.ylabel("CL")
plt.title("Polar de arrasto - Cruzeiro")
plt.grid(True)
plt.tight_layout()
plt.savefig("polar_cruzeiro.pdf", dpi=150)
plt.show()

ovl.plot_geom()

# alpha = np.linspace(-5, 15, 20)
# CL = []
# for a in alpha:
#     ovl.set_variable("alpha", a)
#     ovl.execute_run()
#     CL.append( ovl.get_total_forces()["CL"] )


# ovl.set_constraint("alpha","CL", 1.26)
# ovl.execute_run()
# print("Alpha = ", ovl.get_variable("alpha"))

# plt.plot(alpha, CL)
# plt.xlabel(r"$\alpha$")
# plt.ylabel(r"$C_L$")
# plt.grid()
# plt.show()
