
	



def build_setup(nmpc):
	# cvodes configuration
	nmpc.pre.cvodes_abstol = 1e-8
	nmpc.pre.cvodes_reltol = 1e-6

	# QIH configuration
	nmpc.pre.QIH = False
	nmpc.pre.P_QIH = [[0.0183, 0.0167],
					 [0.0167, 0.0255]]
	nmpc.pre.alpha_QIH = 5

	#create_scenario(nmpc)

	# Maximum iteration loop
	nmpc.pre.max_iter_NLP = 50;
	# Length of one control step in minutes
	nmpc.pre.T_c_minutes = 0.5
	# Lenght of one control step in hours
	nmpc.pre.T_c = nmpc.pre.T_c_minutes / 60
	# Length of prediction horizon in minutes
	nmpc.pre.T_p_minutes = 40
	# Length of prediction horizon in hours
	nmpc.pre.T_p = nmpc.pre.T_p_minutes / 60
	# Prediction horizon in control steps
	nmpc.pre.N_p = nmpc.pre.T_p / nmpc.pre.T_c
	# Control Horizon
	nmpc.pre.N_c = nmpc.pre.N_p

	# Experiment horizon in minutes
	nmpc.pre.exp_minutes = 60;
	# Experiment horizon in control steps
	nmpc.pre.N_exp = nmpc.pre.exp_minutes / nmpc.pre.T_c_minutes
	# Number of integration steps per control interval
	nmpc.pre.M = 1
	# Length of one simulation integrator step (in hours)
	nmpc.pre.DT_sim = nmpc.pre.T_c / nmpc.pre.M
	# Length of one prediction integrator step (in hours)
	nmpc.pre.DT_pre = nmpc.pre.T_c / nmpc.pre.M

	# Degree of interpolating polynomial
	nmpc.pre.dc_p_degree = 2;
	# Type of interpolating polynomial
	nmpc.pre.dc_p_type = 'legendre'

	

