import numpy as np

#data = np.load("C:\\Users\\anton\\OneDrive\\Documents\\GitHub\\Experimental_Capstone\\sim_data.npz")

data = np.load(r"C:\Users\anton\OneDrive\Documents\GitHub\Experimental_Capstone\sim_data.npz")
thrust_log = data["thrust_log"]
t_log = data["t_log"]
state_log = data["state_log"]

print(thrust_log[1])

dt = t_log[1] - t_log[0]
ubar = 55/1000

Tot_ISP = sum(sum(thrust_log))*dt
print("The Convergance Time is:", round(max(t_log),3),"s")

print("The Total ISP is:", round(Tot_ISP,3))
print(dt)
print(1/1e-2)