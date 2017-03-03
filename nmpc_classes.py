class pre:
	"""class used to group all prediction related variables together"""
	def __init__(self):
		# MPC objective type ('quadratic' or 'economic')
		self.obj = None
		# The control method applied ('NMPC', 'None')
		self.ctrl = None
		# The integrator used in prediction ('Euler', 'RK4' or 'cvodes')
		self.pred_int = None
		# The NLP solver ('sqp' or 'ipopt')
		self.NLP_solver = None
		# Hessian calculation ('exact' or 'BFGS')
		self.SQP_hessian_approximation = None
		# Direct method ('dss', 'dms' or 'dc')
		self.direct_method = None
		# The absolute tolerance and relative tolerance if cvodes solver is chosen (float)
		self.cvodes_abstol = None
		self.cvodes_reltol = None
		# Quasi-infinite horizon configuration (True or False)
		self.QIH = None
		# P Matrix for QIH
		self.P_QIH = None
		# alpha for QIH (x'Px <= alpha)
		self.alpha_QIH = None

		# Maximum number of NLP solving iterations (int)
		self.max_iter_NLP = None
		# Control sampling time (in minutes)
		self.T_c_minutes = None
		# Control sampling time (in hours)
		self.T_c = None

		# Prediction Horizon (in minutes)
		self.T_p_minutes = None
		# Prediction horizon (in hours)
		self.T_p = None
		# Prediction horizon (in number of control steps)
		self.N_p = None
		# Control horizon
		self.N_c = None
		'''
		This defines the degrees of freedom of the control
		input for the control intervals inside the prediction
		horizon. If it is 1, then the control input is
		constant for the whole horizon. If it is N_p, then
		the control input is allowed to be different for
		each step. Selecting a low N_c value will result in
		faster computations, since it directly effects the
		number of decision variables of the NLP.
		'''

		# Experiment horizon (in minutes)
		self.exp_minutes = None
		# Experiment horizon (in control steps)
		self.N_exp = None
		# Number of integration steps per control interval
		self.M = None

		# Length of one simulation integrator step (in hours)
		self.DT_sim = None
		# Length of one prediction integrator step (in hours)
		self.DT_pre = None

		# Degree of interpolating polynomial
		self.dc_p_degree = None;
		# Type of interpolating polynomial
		self.dc_p_type = None





class sim:
	"""docstring for sim"""
	def __init__(self):
		self.int = None

class ctrl:
	"""docstring for ctrl"""
	def __init__(self):
		self.arg = None
		
		

class nmpc:
	"""class that group sim class, pre class and ctrl class together"""
	def __init__(self, pre, sim, ctrl):
		self.sim = sim
		self.pre = pre
		self.ctrl = ctrl