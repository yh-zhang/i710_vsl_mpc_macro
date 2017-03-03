from NMPC_of_VSL import *
import numpy as np 
from bokeh.plotting import figure, output_file, show
from bokeh.charts import Step
from retrieve_data import *

def plot_rho(nmpc, secs):
    '''
    secs(List[int]): section number want to plot
    '''
    rho = nmpc.sim.rho
    t = np.array(range(rho.shape[1])) * nmpc.pre.T_c_minutes
    output_file('rho_'+ nmpc.pre.ctrl + '.html')
    p = figure(title = 'Density (veh/mi)', x_axis_label = 't (min)', y_axis_label = 'rho')
    for sec in secs:
        p.line(t, rho[sec, :], legend = 'rho %d' % sec, line_width = 2)

    show(p)

def plot_v(nmpc, secs):
    v = nmpc.sim.v
    t = np.array(range(v.shape[1])) * nmpc.pre.T_c_minutes
    output_file('v_' + nmpc.pre.ctrl + '.html')
    p = figure(title = 'VSL (mi/h)', x_axis_label = 't (min)', y_axis_label = 'VSL')
    for sec in secs:
        p.line(t, v[sec, :], legend = 'v %d' % sec, line_width = 2)
    show(p)
    

if __name__ == '__main__':
    dataFile = 'data.pckl'
    nmpc = retrieve_data(dataFile)
    plot_rho(nmpc, range(8))
    plot_v(nmpc, range(7))
    pass