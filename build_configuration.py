

def build_configuration(nmpc):
	# MPC objective type ('quadratic' or 'economic')
	nmpc.pre.obj = 'quadratic'
	#nmpc.pre.obj = 'economic'

	# The control method applied ('NMPC', 'None')
	#nmpc.pre.ctrl = 'FL'
	nmpc.pre.ctrl = 'NMPC'

	# Choose the simulation integrator
	# nmpc.sim.int = 'RK4'
	# #nmpc.sim.int = 'Euler'

	# The integrator used in prediction ('Euler', 'RK4' or 'cvodes')
	nmpc.pre.int = 'RK4'

	# The NLP solver ('sqp' or 'ipopt')
	nmpc.pre.NLP_solver = 'ipopt'

	# Hessian calculation ('exact' or 'BFGS')
	nmpc.pre.SQP_hessian_approximation = 'exact'

	# Direct method ('dss', 'dms' or 'dc')
	nmpc.pre.direct_method = 'dms'

	#return nmpc
