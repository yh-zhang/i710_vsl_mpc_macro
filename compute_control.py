import numpy as np
from casadi import *
import time
from round_and_rate_limit_control import *

def compute_control(nmpc, t):
    if t < nmpc.pre.T_start_inc or t >= nmpc.pre.T_remove_inc:
            nmpc.sim.v[:, [t]] = nmpc.pre.vf * np.ones((nmpc.pre.n_u, 1))
    else:
        if nmpc.pre.ctrl == 'NC':
            nmpc.sim.v[:,[t]] = nmpc.pre.v_e

        elif nmpc.pre.ctrl == 'FL':
            compute_control_FL(nmpc, t)
            nmpc.sim.v[:,[t]] = nmpc.sim.u[:,[t]] + nmpc.pre.v_e
            round_and_rate_limit_control(nmpc, t)

        elif nmpc.pre.ctrl == 'NMPC':
            compute_control_NMPC(nmpc, t)
            nmpc.sim.v[:,[t]] = nmpc.sim.u[:,[t]] + nmpc.pre.v_e
            round_and_rate_limit_control(nmpc, t)
    # else:
    #     if nmpc.pre.ctrl == 'NC':
    #         nmpc.sim.v[:,[t]] = nmpc.pre.v_e
    #     if nmpc.pre.ctrl == 'FL':
    #         compute_control_FL(nmpc, t)
    #         nmpc.sim.v[:,[t]] = nmpc.sim.u[:,[t]] + nmpc.pre.v_e
    #         round_and_rate_limit_control(nmpc, t)
    #     if nmpc.pre.ctrl == 'NMPC':
    #         compute_control_NMPC(nmpc, t)
    #         nmpc.sim.v[:,[t]] = nmpc.sim.u[:,[t]] + nmpc.pre.v_e
    #         round_and_rate_limit_control(nmpc, t)

def compute_control_FL(nmpc, t):
    #print nmpc.sim.rho
    rrho = nmpc.sim.rho[:-1, [t]]
    #print rrho.shape


    # error of the density of each section 
    e = nmpc.sim.x[:-1, [t]]
    u = np.zeros((nmpc.pre.n_u, 1))
    for i in xrange(nmpc.pre.n_u-1):
        u[i] = 1 / rrho[i] * (- nmpc.pre.v_e[i] * e[i] - nmpc.pre.lambda_FL[i] * nmpc.pre.L[i] *e[i+1])

    N = nmpc.pre.n - 1
    if e[N] <= 0:
        u[N-1] = 1 / rrho[N-1] * (-nmpc.pre.lambda_FL[N-1] * nmpc.pre.L[N] * e[N] - nmpc.pre.v_e[N-1] * e[N-1] + nmpc.pre.ve * e[N])
    else:
        u[N-1] = 1 / rrho[N-1] * (-nmpc.pre.lambda_FL[N-1] * nmpc.pre.L[N] * e[N] - nmpc.pre.v_e[N-1] * e[N-1] - nmpc.pre.wb * e[N])

    nmpc.sim.u[:,[t]] = u;


def compute_control_NMPC(nmpc, t):
    initialize_NMPC(nmpc, t)
    solve_NLP_NMPC(nmpc, t)
    recover_control_NMPC(nmpc, t)

def initialize_NMPC(nmpc, t):
    # Measure states at current time step t
    nmpc.ctrl.x_0 = nmpc.sim.x[:,[t]]

    # Measure previous controls at current time step t
    if t == 0:
        nmpc.ctrl.u_prev = np.zeros((nmpc.pre.n_u, 1))
    else:
        nmpc.ctrl.u_prev = nmpc.sim.v[:,[t-1]] - nmpc.pre.v_e

    # Set current states and previous controls as parameters of the NLP
    #nmpc.pre.p_NLP = np.vstack((nmpc.ctrl.x_0, nmpc.ctrl.u_prev))
    nmpc.pre.p_NLP = vertcat(nmpc.ctrl.x_0, nmpc.ctrl.u_prev)

    # Construct initial guess vector w0 for the current time step t
    # (use values of the previous time step as initial guess)
    if t == 0:
        nmpc.ctrl.w0_IG = nmpc.ctrl.w0_IG_0
    else:
        if not nmpc.sim.w_star_full:
            nmpc.ctrl.w0_IG = nmpc.ctrl.w0_IG_0
        else:
            nmpc.ctrl.w0_IG = nmpc.sim.w_star_full[t-1]

    # Check initial guess feasibility
    lbCheck = (nmpc.ctrl.w0_IG < nmpc.ctrl.lbw)
    #lbCheck[0] = 1
    for i in xrange(lbCheck.shape[0]):
        if lbCheck[i]:
            raise ValueError('Lower bound violated!')

    ubCheck = (nmpc.ctrl.w0_IG > nmpc.ctrl.ubw)
    for i in xrange(ubCheck.shape[0]):
        if ubCheck[i]:
            raise ValueError('Upper bound violated!')



def solve_NLP_NMPC(nmpc, t):
    NLP_data = {
        'x0': nmpc.ctrl.w0_IG,  #initial guess
        'p': nmpc.pre.p_NLP,    #parameters
        'lbx': nmpc.ctrl.lbw,   #lower bounds for box constraints
        'ubx': nmpc.ctrl.ubw,   #upper bounds for box constraints
        'lbg': nmpc.ctrl.lbg,   #lower bounds for general constraints
        'ubg': nmpc.ctrl.ubg,   #upper bounds for general constraints
    }

    tic = time.time()
    nmpc.ctrl.NLP_solution = nmpc.ctrl.f_NLP(
        x0 = NLP_data['x0'],
        p = NLP_data['p'],
        lbx = NLP_data['lbx'],
        ubx = NLP_data['ubx'],
        lbg = NLP_data['lbg'],
        ubg = NLP_data['ubg']
        )
    nmpc.sim.CPU_time[t] = time.time() - tic


def recover_control_NMPC(nmpc, t):
    #temp = nmpc.ctrl.NLP_solution['x']
    w_star_full_raw = nmpc.ctrl.NLP_solution['x'].full()
    nmpc.sim.w_star_full[t] = w_star_full_raw
    w_star_full_reshaped = w_star_full_raw.reshape((nmpc.pre.N_p, nmpc.pre.n_x+nmpc.pre.n_u,)).T
    # Cost function value up to time
    cost2Time = 0

    u_full = np.zeros((nmpc.pre.n_u, int(nmpc.pre.N_p)))
    x_full = np.zeros((nmpc.pre.n_x, int(nmpc.pre.N_p)))

    for k in xrange(int(nmpc.pre.N_p)):
        u_full[:, [k]] = w_star_full_reshaped[:nmpc.pre.n_u, [k]]
        x_full[:, [k]] = w_star_full_reshaped[nmpc.pre.n_u:, [k]]

        cost2Time += mtimes(mtimes(x_full[:, [k]].T, nmpc.pre.Q), x_full[:,[k]])\
                     + mtimes(mtimes(u_full[:, [k]].T, nmpc.pre.R), u_full[:, [k]])
    # Record Stage cost
    nmpc.sim.cost2Time[0, t] = cost2Time
    nmpc.sim.u[:, [t]] = u_full[:, [0]]

    nmpc.rec.x_pred[t] = x_full
    nmpc.rec.u_pred[t] = u_full



    





    

    




