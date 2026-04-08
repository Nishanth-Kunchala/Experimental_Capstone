import numpy as np


# Set the type as "Tran" for translational or "Rota" for Rotational
type = "Rota"
## Cycling PWPF data
data_cyc = np.load(r"C:\Users\anton\OneDrive\Documents\GitHub\Experimental_Capstone\Sim_data\sim_data_cyc_" + type + ".npz")
thrust_cyc = data_cyc["thrust_log"]
t_cyc = data_cyc["t_log"]
state_cyc = data_cyc["state_log"]

dt_cyc = t_cyc[1]-t_cyc[0]
ubar = 25/1000

## varying thrust PWPF data
data_mod = np.load(r"C:\Users\anton\OneDrive\Documents\GitHub\Experimental_Capstone\Sim_data\sim_data_mod_" + type + ".npz")
thrust_mod = data_mod["thrust_log"]
t_mod = data_mod["t_log"]
state_mod = data_mod["state_log"]

dt_mod = t_mod[1]-t_mod[0]

### Original PWPF
#data = np.load(r"C:\Users\anton\OneDrive\Documents\GitHub\Experimental_Capstone\sim_data_" + type + ".npz")
#thrust_log= data["thrust_log"]
#t_log = data["t_log"]
#state_log = data["state_log"]

#dt_log = t_log[1]-t_log[0]

Tot_Im_cyc = sum(sum(thrust_cyc))*dt_cyc

Tot_Im_mod = sum(sum(thrust_mod))*dt_mod # no need to add *ubar, already accounted for in the simulation

#Tot_Im = sum(sum(thrust_log))*dt_log

print("The Convergance for cyc is:", round(max(t_cyc),3),"s")
print("The Convergance for mod is:", round(max(t_mod),3),"s")
#print("The Convergance for standard is:", round(max(t_log),3),"s")
print()
print("The Total Impulse for cyc is:", round(Tot_Im_cyc,3))
print("The Total Impulse for mod is:", round(Tot_Im_mod,3))
#print("The Total Impulse for standard is:", round(Tot_Im,3))

print("The initial State is")
print(state_cyc[0]*180/np.pi)