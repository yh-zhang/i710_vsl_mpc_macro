import numpy as np

def build_parameters(nmpc):
    # Number of freeway sections
    nmpc.pre.n = 8

    # Number of states and control inputs
    nmpc.pre.n_x = nmpc.pre.n + 1
    nmpc.pre.n_u = nmpc.pre.n - 1

    # Control sampling time (in minutes)
    nmpc.pre.T_c_minutes = 0.5
    
    # Prediction horizon (in minutes)
    nmpc.pre.T_minutes = 20
    
    # Experiment horizon (in minutes)
    nmpc.pre.T_exp_minutes = 90
    
    # QIH configuration
    nmpc.pre.QIH = 0

    # Rounding parameter on VSL commands (mi/h)
    nmpc.pre.v_pR = 5
    
    # Rate limiting parameter on VSL commands (mi/h)
    nmpc.pre.v_pRL = 10
    
    build_time_parameters(nmpc)
    
    build_model_parameters(nmpc)
    
    build_controller_parameters(nmpc)

    nmpc.sim.CPU_time = [0.0] * int(nmpc.pre.N_exp)


def build_time_parameters(nmpc):
    # Control sampling time (in hours)
    nmpc.pre.T_c = nmpc.pre.T_c_minutes / 60
    # Incident starting time (in minutes)
    nmpc.pre.T_start_inc_minutes = 5
    
    # Incident starting time (in control steps)
    nmpc.pre.T_start_inc = int(nmpc.pre.T_start_inc_minutes/nmpc.pre.T_c_minutes)
    
    # Incident removal time (in minutes)
    nmpc.pre.T_remove_inc_minutes = 35
    
    # Incident removal time (in control steps)
    nmpc.pre.T_remove_inc = int(nmpc.pre.T_remove_inc_minutes/nmpc.pre.T_c_minutes)
    
    # Experiment horizon (in control steps)
    nmpc.pre.N_exp = int(nmpc.pre.T_exp_minutes/nmpc.pre.T_c_minutes)
    
    # Length of one simulation integrator step
    nmpc.pre.DT_sim = nmpc.pre.T_c

def build_model_parameters(nmpc):
    # Demand (veh/h)
    nmpc.pre.d = 6500;
    
    # Free-flow speed (mi/h)
    nmpc.pre.vf = 65;

    # Length of freeway sections (mi)
    nmpc.pre.L = np.array([[2921.161, 1611.075, 1924.33, 2001.997, 1539.524, 2131.936, 1895.208, 3763.9]]).transpose() / 5280
    #print nmpc.pre.L

    # Critical density (veh/mi)
    nmpc.pre.rho_c = 105.0 * np.ones((nmpc.pre.n, 1))
    #print nmpc.pre.rho_c

    # Capacity (veh/h)
    nmpc.pre.c = nmpc.pre.vf * nmpc.pre.rho_c
    #print nmpc.pre.c

    # Backward propagating wave speed
    nmpc.pre.w = 14 * np.ones((nmpc.pre.n, 1))
    #print nmpc.pre.w

    # Jam density (veh/mi)
    nmpc.pre.rho_j = nmpc.pre.c / nmpc.pre.w + nmpc.pre.rho_c
    #print nmpc.pre.rho_j

    # Critical density of bottleneck (veh/mi)
    nmpc.pre.rho_cb = 110;

    # Equilibrium Speed of the control segment (mi/h)
    nmpc.pre.ve = 40
    
    # Capacity of bottleneck (veh/h)
    nmpc.pre.cb = nmpc.pre.ve * nmpc.pre.rho_cb;
    
    # Backward propagating wave speed of bottleneck
    nmpc.pre.wb = 40;
    
    # Jam density of bottleneck (veh/mi)
    nmpc.pre.rho_jb = nmpc.pre.cb/nmpc.pre.wb + nmpc.pre.rho_cb;
    
    # Initial queue size at time t = 0 (veh/mi)
    nmpc.pre.que_0 = 0;
    
    # Initial section densities at time t = 0 (veh/mi)
    nmpc.pre.rho_n_0 = (nmpc.pre.d / nmpc.pre.vf) * np.ones((nmpc.pre.n, 1));
    
    # Initial states at time t = 0 (veh/mi)
    # The state vector is [rho1, rho2, ..., rho10, queue]
    nmpc.pre.rho_0 = np.vstack((nmpc.pre.rho_n_0, nmpc.pre.que_0)) 
    #print nmpc.pre.rho_0

    # Equilibrium points
    # The equilibrium vector does not contain the queue length, as the queue length
    # does not converge therefore no equilibrium point
    nmpc.pre.rho_e = nmpc.pre.rho_cb * np.ones((nmpc.pre.n, 1))
    nmpc.pre.rho_e[0] = nmpc.pre.rho_j[0] - nmpc.pre.cb/nmpc.pre.w[0]
    nmpc.pre.v_e = nmpc.pre.cb / nmpc.pre.rho_e
    nmpc.pre.v_e = nmpc.pre.v_e[:-1]

    #print nmpc.pre.rho_e, nmpc.pre.v_e

def build_controller_parameters(nmpc):
    
    # Bounds on VSL commands (mi/h)
    nmpc.pre.v_min = 10;
    nmpc.pre.v_max = 65;

    if nmpc.pre.ctrl == 'FL':
        nmpc.pre.lambda_FL = 50 * np.ones((nmpc.pre.n_u, 1))
        return

    if nmpc.pre.ctrl == 'NMPC':
              
        # Prediction horizon (in number of control steps)
        nmpc.pre.N_p = int(nmpc.pre.T_minutes/nmpc.pre.T_c_minutes);
        
        # Control horizon (in number of control steps)
        nmpc.pre.N_c = nmpc.pre.N_p;
        
        # for_loop_x_b_A_B_191216
        nmpc.pre.P_QIH = np.array([[0.0183, 0.0167], [0.0167, 0.0255]])
        nmpc.pre.alpha_QIH = 5
        #print nmpc.pre.P_QIH
        
        # Error state constraints
        nmpc.pre.x_min = -np.Inf * np.ones((nmpc.pre.n_x, 1));
        nmpc.pre.x_max = np.Inf * np.ones((nmpc.pre.n_x, 1));
        
        # Control input constraints
        nmpc.pre.u_min = nmpc.pre.v_min - nmpc.pre.v_e;
        nmpc.pre.u_max = nmpc.pre.v_max - nmpc.pre.v_e;
        
        # Weighting matrices of objective function
        nmpc.pre.Q = np.identity(nmpc.pre.n_x);
        nmpc.pre.Q[-1,-1] = 0.001;
        nmpc.pre.R = 0.1 * np.identity(nmpc.pre.n_u);
        # print nmpc.pre.Q
        # print nmpc.pre.R
        
        # Maximum NLP iterations
        nmpc.pre.max_iter_NLP = 100;
        
        # Number of integration steps per control interval
        nmpc.pre.M = 1;
        
        # Length of one prediction integrator step
        nmpc.pre.DT_pre = nmpc.pre.T_c/nmpc.pre.M;
        
        # if strcmp(d.p.direct_method,'dc') == 1
            
        #     # Degree of interpolating polynomial
        #     d.p.dc_p_degree = 2;
            
        #     # Type of interpolating polynomial
        #     d.p.dc_p_type = 'legendre';
               

