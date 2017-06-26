from casadi import *
import numpy as np

def build_controller(nmpc):
    if nmpc.pre.ctrl == 'NMPC':
        formulate_NLP(nmpc)
        create_initial_guess(nmpc)


def formulate_NLP(nmpc):
    # Initialize NLP arguments
    w = []
    v = []
    nmpc.ctrl.lbw = [] 
    nmpc.ctrl.ubw = []
    J = 0
    g = []
    nmpc.ctrl.lbg = []
    nmpc.ctrl.ubg = []

    # Initial state
    x0k = MX.sym('x0k', nmpc.pre.n_x, 1)

    # Initialize states
    xk = x0k
    #print x0k.shape
    # Append x0k to NLP parameters vector v
    v = vertcat(v, x0k)
    #print v.shape

    # New NLP variable rate limiting
    u_prev = MX.sym('u_prev', nmpc.pre.n_u, 1)
    v = vertcat(v, u_prev)
    #print v.shape

    # For loop over control intervals
    for k in xrange(int(nmpc.pre.N_p)):
        if k < nmpc.pre.N_c:
            # New NLP variable for control inputs
            uk = MX.sym('uk_'+ str(k), nmpc.pre.n_u, 1)

            # Append uk to NLP variables vector w
            w = vertcat(w, uk)

            # Define constraints on control inputs
            nmpc.ctrl.lbw = vertcat(nmpc.ctrl.lbw, nmpc.pre.u_min)
            nmpc.ctrl.ubw = vertcat(nmpc.ctrl.ubw, nmpc.pre.u_max)

            # Define rate limits in space
            for i in xrange(int(nmpc.pre.n_u)-1):
                
                g = vertcat(g, uk[i+1] - uk[i])
                
                nmpc.ctrl.lbg = vertcat(nmpc.ctrl.lbg, -nmpc.pre.v_pRL_space);
                
                nmpc.ctrl.ubg = vertcat(nmpc.ctrl.ubg, np.inf)

            # Define rate limits in time
            if k == 0:
                
                g = vertcat(g, uk - u_prev)

                nmpc.ctrl.lbg = vertcat(nmpc.ctrl.lbg, -nmpc.pre.v_pRL*np.ones((nmpc.pre.n_u, 1)));
                
                nmpc.ctrl.ubg = vertcat(nmpc.ctrl.ubg, np.inf*np.ones((nmpc.pre.n_u, 1)))
                
                #print nmpc.ctrl.lbg

        # Integrate with Runge Kutta
        xk_end, Jk = nmpc.ctrl.F_pre(xk, uk)

        # Integrate the objective function
        J += Jk

        # New NLP variable for the states at the end of the control interval
        xk = MX.sym('xk_'+'k+1', nmpc.pre.n_x, 1)
        w = vertcat(w, xk)

        nmpc.ctrl.lbw = vertcat(nmpc.ctrl.lbw, -np.inf * np.ones((nmpc.pre.n_x, 1)))
        nmpc.ctrl.ubw = vertcat(nmpc.ctrl.ubw, np.inf * np.ones((nmpc.pre.n_x, 1)))

        # Define the state constraints
        g = vertcat(g, xk)
        nmpc.ctrl.lbg = vertcat(nmpc.ctrl.lbg, nmpc.pre.x_min)
        nmpc.ctrl.ubg = vertcat(nmpc.ctrl.ubg, nmpc.pre.x_max)

        if k == int(nmpc.pre.N_p - 1):
            if nmpc.pre.QIH == 1:
                term_pen = mtimes(mtimes(xk.T, nmpc.pre.P_PIH), xk)
                g = vertcat(g, term_pen)

                nmpc.ctrl.lbg = vertcat(nmpc.ctrl.lbg, 0)
                nmpc.ctrl.ubg = vertcat(nmpc.ctrl.ubg, nmpc.pre.alpha_QIH)

                J = J + term_pen


        # Enforce the new variable for the end of the state
        # to be equal to the one coming from the integrator
        # (this is the main trick of multiple direct)
        g = vertcat(g, xk_end - xk)
        nmpc.ctrl.lbg = vertcat(nmpc.ctrl.lbg, np.zeros((nmpc.pre.n_x, 1)))
        nmpc.ctrl.ubg = vertcat(nmpc.ctrl.ubg, np.zeros((nmpc.pre.n_x, 1)))

    # Create the problem structure
    prob = {'f': J, #objective function
            'x': w, #decision variables
            'p': v, #parameters
            'g': g, #constraints
    }
    #print prob

    # Create function for solving NLP with ipopt
    options = {}
    options['verbose'] = False
    options['ipopt.max_iter'] = nmpc.pre.max_iter_NLP
    options['warn_initial_bounds'] = True

    nmpc.ctrl.f_NLP = nlpsol('f_NLP', 'ipopt', prob, options)

def create_initial_guess(nmpc):
	nmpc.ctrl.w0_IG_0 = []
	for k in xrange(int(nmpc.pre.N_c)):
		nmpc.ctrl.w0_IG_0 = vertcat(nmpc.ctrl.w0_IG_0, nmpc.pre.uk_IG_0)
		nmpc.ctrl.w0_IG_0 = vertcat(nmpc.ctrl.w0_IG_0, nmpc.pre.xk_IG_0)

	for k in xrange(int(nmpc.pre.N_c), int(nmpc.pre.N_p)):
		nmpc.ctrl.w0_IG_0 = vertcat(nmpc.ctrl.w0_IG_0, nmpc.pre.xk_IG_0)
















