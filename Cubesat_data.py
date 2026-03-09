import numpy as np

data = np.load("C:\\Users\\anton\\OneDrive\\Documents\\GitHub\\Experimental_Capstone\\sim_data.npz")

thrust_log = data["thrust_log"]
t_log = data["t_log"]
state_log = data["state_log"]

print(thrust_log[1])