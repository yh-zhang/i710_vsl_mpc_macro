import numpy as np
from scipy.integrate import ode

def evolve_dynamics(nmpc, t):
    ode15s_incident = ode(i710_const_u)
    ode15s_incident.set_integrator('vode', method = 'bdf', order = 15, nsteps = 3000)
    ode15s_noincident = ode(i710_noincident)
    ode15s_noincident.set_integrator('vode', method = 'bdf', order = 15, nsteps = 3000)
    v = nmpc.sim.v[:, t]
    if t <= nmpc.pre.T_start_inc or t > nmpc.pre.T_remove_inc:
        ode15s_noincident.set_initial_value(nmpc.sim.rho[:, [t]], 0).set_f_params((v, nmpc))
        if ode15s_noincident.successful():
            ode15s_noincident.integrate(nmpc.pre.T_c)
            nmpc.sim.rho[:, [t+1]] = ode15s_noincident.y
    else:
        ode15s_incident.set_initial_value(nmpc.sim.rho[:, [t]], 0).set_f_params((v, nmpc))
        if ode15s_incident.successful():
            ode15s_incident.integrate(nmpc.pre.T_c)
            nmpc.sim.rho[:, [t+1]] = ode15s_incident.y
    nmpc.sim.x[:, [t+1]] = nmpc.sim.rho[:, [t+1]] - np.vstack((nmpc.pre.rho_e, 0))





    # if t < nmpc.pre.N_exp-1:
    #     x_plus, _ = nmpc.ctrl.F_sim(nmpc.sim.x[:, [t]], nmpc.sim.u[:, [t]])
    #     nmpc.sim.x[:, [t+1]] = x_plus.full()
    #     nmpc.sim.rho_n[:, [t+1]] = nmpc.sim.x[:-1, [t+1]] + nmpc.pre.rho_e
    # pass

def i710_const_u(t, rho, (v, nmpc)):
    # state vector [rho_0, rho_1, ... rho_N, queue]
    drho = np.zeros((rho.shape[0], 1))
    q = compute_flow_incident(rho, v, nmpc)
    for i in xrange(nmpc.pre.n):
        drho[i] = (q[i] - q[i+1]) / nmpc.pre.L[i]
    drho[-1] = nmpc.pre.d - q[0]
    return drho

def i710_noincident(t, rho, (v, nmpc)):
    # state vector [rho_0, rho_1, ... rho_N, queue]
    drho = np.zeros((rho.shape[0], 1))
    q = compute_flow_noincident(rho, v, nmpc)
    for i in xrange(nmpc.pre.n):
        drho[i] = (q[i] - q[i+1]) / nmpc.pre.L[i]
    drho[-1] = nmpc.pre.d - q[0]
    return drho


    
def compute_flow_incident(rho, v, nmpc):
    q = np.zeros((nmpc.pre.n+1, 1))
    q[0] = max([0, min([nmpc.pre.d, nmpc.pre.c[0], nmpc.pre.w[0] * (nmpc.pre.rho_j[0] - rho[0])])])
    for i in xrange(1, nmpc.pre.n):
        q[i] = max([0, min([v[i-1] * rho[i-1]], nmpc.pre.c[i], nmpc.pre.w[i] * (nmpc.pre.rho_j[i] - rho[i]))])
    q[nmpc.pre.n] = max(0, min(nmpc.pre.ve * rho[nmpc.pre.n-1], nmpc.pre.wb * (nmpc.pre.rho_jb - rho[nmpc.pre.n-1])))
    return q

def compute_flow_noincident(rho, v, nmpc):
    q = np.zeros((nmpc.pre.n+1, 1))
    q[0] = max([0, min([nmpc.pre.d, nmpc.pre.c[0], nmpc.pre.w[0] * (nmpc.pre.rho_j[0] - rho[0])])])
    for i in xrange(1, nmpc.pre.n):
        q[i] = max([0, min([v[i-1] * rho[i-1]], nmpc.pre.c[i], nmpc.pre.w[i] * (nmpc.pre.rho_j[i] - rho[i]))])
    q[nmpc.pre.n] = max(0, min(nmpc.pre.vf * rho[nmpc.pre.n-1], nmpc.pre.c[-1]))
    return q   

