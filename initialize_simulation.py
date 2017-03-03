import numpy as np

def initialize_simulation(nmpc):
	# States vector
	nmpc.sim.rho = np.nan * np.ones((nmpc.pre.n_x, nmpc.pre.N_exp))  
	nmpc.sim.rho[:, [0]] = nmpc.pre.rho_0
	#print nmpc.pre.rho_0
	#print nmpc.sim.rho

	# # Queue size (x[-1])
	# nmpc.sim.que = np.nan * np.ones((1, nmpc.pre.N_exp))
	# nmpc.sim.que[0, 0] = nmpc.sim.rho[-1, 0]
	# #print nmpc.sim.rho_q

	# # Section densities
	# nmpc.sim.rho_n = np.nan * np.ones((nmpc.pre.n, nmpc.pre.N_exp))
	# nmpc.sim.rho_n[:, [0]] = nmpc.sim.rho[:-1, [0]]
	# #print nmpc.sim.rho_n

	# States errors
	nmpc.sim.x = np.nan * np.ones((nmpc.pre.n_x, nmpc.pre.N_exp))
	nmpc.sim.x[:, [0]] = nmpc.sim.rho[:, [0]] - np.vstack((nmpc.pre.rho_e, 0))
	#np.vstack((nmpc.sim.rho_n[:,[0]] - nmpc.pre.rho_e, nmpc.sim.que[0, 0]))
	#print nmpc.sim.x

	# VSL commands
	nmpc.sim.v = np.nan * np.ones((nmpc.pre.n_u, nmpc.pre.N_exp))

	# Define initial guess for the state
	nmpc.pre.xk_IG_0 = nmpc.sim.x[:,[0]]
	nmpc.sim.u = np.nan * np.ones((nmpc.pre.n_u, nmpc.pre.N_exp))
	nmpc.sim.u[:,[0]] = np.zeros((nmpc.pre.n_u, 1))
	# print nmpc.sim.u
	# print nmpc.pre.xk_IG_0

	# Define initial guess for t = 0 for the control input
	nmpc.pre.uk_IG_0 = nmpc.sim.u[:,[0]]

	# NLP solutions
	nmpc.sim.w_star_full = {}

	# NLP Cost function values at each control step
	nmpc.sim.cost2Time = np.zeros((1, nmpc.pre.N_exp))

	# MPC Prediction record at each control step
	nmpc.rec.x_pred = {}
	nmpc.rec.u_pred = {}






