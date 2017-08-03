from casadi import *

def define_dynamics(nmpc):
    if nmpc.pre.ctrl == 'NMPC':
        # States
        # Create states as casadi variable
        x = SX.sym('x', nmpc.pre.n_x, 1)

        # Create state derivatives as casadi variable
        x_dot = SX.sym('x_dot', nmpc.pre.n_x, 1)

        # Control inputs
        # Create control inputs as casadi variable
        u = SX.sym('u', nmpc.pre.n_u, 1)

        # System dynamics
        q = compute_flow_rate(nmpc, x, u)
        for i in range(nmpc.pre.n_x-1):
            x_dot[i] = (q[i] - q[i+1]) / nmpc.pre.L[i]
        x_dot[-1] = nmpc.pre.d - q[0] 
        # The system states are error states of [rho0, rho1, ..., rho10, queue]'

        # # The first section
        # x_dot[0] = ( 
        #     fmax(fmin(
        #                      fmin(nmpc.pre.d, nmpc.pre.c[0]), 
        #                      nmpc.pre.w[0] * (nmpc.pre.rho_j[0] - x[0] - nmpc.pre.rho_e[0])), 0) 
        #     - fmax(fmin(
        #                     fmin((u[0] + nmpc.pre.v_e[0]) * (x[0] + nmpc.pre.rho_e[0]),
        #                     nmpc.pre.c[1]),
        #                     nmpc.pre.w[1] * (nmpc.pre.rho_j[1] - x[1] - nmpc.pre.rho_e[1])), 0) 
        #     ) / nmpc.pre.L[0]
        # #print len(nmpc.pre.v_e)

        # #The intermediate section
        # for i in xrange(1, nmpc.pre.n - 1):
        #     x_dot[i] = (
        #         fmax(fmin(
        #                             fmin((u[i-1] + nmpc.pre.v_e[i-1]) * (x[i-1] + nmpc.pre.rho_e[i-1]),
        #                             nmpc.pre.c[i]),
        #                             nmpc.pre.w[i] * (nmpc.pre.rho_j[i] - x[i] - nmpc.pre.rho_e[i]) ), 0)
        #         - fmax(fmin(
        #                             fmin((u[i] + nmpc.pre.v_e[i]) * (x[i] + nmpc.pre.rho_e[i]), 
        #                                 nmpc.pre.c[i+1]), 
        #                             nmpc.pre.w[i+1] * (nmpc.pre.rho_j[i+1] - x[i+1] - nmpc.pre.rho_e[i+1])), 0)
        #         ) / nmpc.pre.L[i]
        # # The bottleneck section
        # x_dot[nmpc.pre.n-1] = (
        #         fmax(fmin(
        #                             fmin((u[nmpc.pre.n-2] + nmpc.pre.v_e[nmpc.pre.n-2]) * (x[nmpc.pre.n-2] + nmpc.pre.rho_e[nmpc.pre.n-2]),
        #                                 nmpc.pre.c[nmpc.pre.n-1]),
        #                             nmpc.pre.w[nmpc.pre.n-1] * (nmpc.pre.rho_j[nmpc.pre.n-1] - x[nmpc.pre.n-1] - nmpc.pre.rho_e[nmpc.pre.n-1])), 0)
        #         - fmax(fmin(
        #                             nmpc.pre.vf * (x[nmpc.pre.n-1] + nmpc.pre.rho_e[nmpc.pre.n-1]),
        #                             nmpc.pre.wb * (nmpc.pre.rho_jb - x[nmpc.pre.n-1] - nmpc.pre.rho_e[nmpc.pre.n-1])
        #                             ), 0) 
        #     ) / nmpc.pre.L[nmpc.pre.n-1]

        # The queue above section 1
        # x_dot[nmpc.pre.n] = (
        #     nmpc.pre.d - 
        #     fmin(
        #         fmin(nmpc.pre.d, nmpc.pre.c[0]), 
        #         nmpc.pre.w[0] * (nmpc.pre.rho_j[0] - x[0] - nmpc.pre.rho_e[0]))  
        #         ) 


        #---------------------------------------------------------------------
        # Stage cost

        if nmpc.pre.obj == 'quadratic':
            L = mtimes(mtimes(x.T, nmpc.pre.Q), x) 
            #print nmpc.pre.Q.shape, x.shape
        if nmpc.pre.obj == 'economic':
            #print x[:-1].shape , nmpc.pre.rho_e.shape
            L = (mtimes((x[:-1] + nmpc.pre.rho_e).T, nmpc.pre.L) + x[-1]) * nmpc.pre.T_c
        
        # ----------------------------------------------------------------
        # System dynamics function f

        nmpc.ctrl.f = Function('f', [x, u], [x_dot, L])

        #print nmpc.pre.f
        #create_simulation_integrator(nmpc)

        if nmpc.pre.ctrl == 'NMPC':
            create_prediction_integrator(nmpc)

def compute_flow_rate(nmpc, x, u):
    '''
    compute the flow rate of each section
    x: system states, SX array
    u: control input, SX array
    returns an SX array
    '''
    q = SX.sym('q', nmpc.pre.n+1, 1)
    q[0] = fmax(fmin(fmin(nmpc.pre.d, nmpc.pre.c[0]), 
        nmpc.pre.w[0] * (nmpc.pre.rho_j[0] - x[0] - nmpc.pre.rho_e[0])), 0)

    for i in range(1, nmpc.pre.n):
        q[i] = fmax(fmin(fmin((u[i-1] + nmpc.pre.v_e[i-1]) * (x[i-1] + nmpc.pre.rho_e[i-1]), nmpc.pre.c[i]),
         nmpc.pre.w[i] * (nmpc.pre.rho_j[i] - x[i] - nmpc.pre.rho_e[i]) ), 0)

    q[nmpc.pre.n] = fmax(fmin(nmpc.pre.vf * (x[nmpc.pre.n-1] + nmpc.pre.rho_e[nmpc.pre.n-1]), 
        nmpc.pre.wb * (nmpc.pre.rho_jb - x[nmpc.pre.n-1] - nmpc.pre.rho_e[nmpc.pre.n-1])), 0) 

    return q



# def create_simulation_integrator(nmpc):
#     # Initial state
#     x0_I = MX.sym('x_I', nmpc.pre.n_x, 1)

#     # Control input
#     u_I = MX.sym('u_I', nmpc.pre.n_u, 1)

#     # Initialize state
#     x_I = x0_I

#     # Initialize objective function
#     J = 0

#     if nmpc.sim.int == 'Euler':
#         for k_I in xrange(nmpc.pre.M):
#             k1, k1_j = nmpc.ctrl.f(x_I, u_I)
#             x_I = x_I + nmpc.pre.DT_sim * k1
#             J = J + nmpc.pre.DT_sim * k1_j

    if nmpc.sim.int == 'RK4':
        k1, k1_j = nmpc.ctrl.f(x_I, u_I)
        k2, k2_j = nmpc.ctrl.f(x_I + nmpc.pre.DT_sim/2 * k1, u_I)
        k3, k3_j = nmpc.ctrl.f(x_I + nmpc.pre.DT_sim/2 * k2, u_I)
        k4, k4_j = nmpc.ctrl.f(x_I + nmpc.pre.DT_sim * k3, u_I)
        x_I = nmpc.pre.DT_sim/6 * (k1 + 2*k2 + 2*k3 + k4)
        J = J + nmpc.pre.DT_sim/6 * (k1_j + 2*k2_j + 2*k3_j + k4_j)


    nmpc.ctrl.F_sim = Function('F_sim', [x0_I, u_I], [x_I, J])

def create_prediction_integrator(nmpc):
    # Initial state
    x0_I = MX.sym('x_I', nmpc.pre.n_x, 1)
    # Control input
    u_I = MX.sym('u_I', nmpc.pre.n_u, 1)
    # Initialize state
    x_I = x0_I
    # Initialize objective function
    J = 0


    if nmpc.pre.int == 'Euler':
        for k_I in xrange(nmpc.pre.M):
            k1, k1_j = nmpc.ctrl.f(x_I, u_I)
            x_I = x_I + nmpc.pre.DT_pre * k1
            J = J + nmpc.pre.DT_pre * k1_j

    if nmpc.pre.int == 'RK4':
        k1, k1_j = nmpc.ctrl.f(x_I, u_I)
        k2, k2_j = nmpc.ctrl.f(x_I + nmpc.pre.DT_pre/2 * k1, u_I)
        k3, k3_j = nmpc.ctrl.f(x_I + nmpc.pre.DT_pre/2 * k2, u_I)
        k4, k4_j = nmpc.ctrl.f(x_I + nmpc.pre.DT_pre * k3, u_I)
        x_I = nmpc.pre.DT_pre/6 * (k1 + 2*k2 + 2*k3 + k4)
        J = J + nmpc.pre.DT_pre/6 * (k1_j + 2*k2_j + 2*k3_j + k4_j)


    nmpc.ctrl.F_pre = Function('F_pre', [x0_I, u_I], [x_I, J])




