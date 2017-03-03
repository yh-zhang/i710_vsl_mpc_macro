# import nmpc_classes as nc
import pickle as pkl
from build_parameters import *
from build_configuration import *
from define_dynamics import *
from initialize_simulation import *
from build_controller import *
from compute_control import *
from evolve_dynamics import *


class pre:
    """docstring for pre"""
    def __init__(self):
        pass

class sim:
    """docstring for sim"""
    def __init__(self):
        pass

class ctrl:
    """docstring for ctrl"""
    def __init__(self):
        pass

class rec:
    """docstring for rec"""
    def __init__(self):
        pass
        
class nmpc:
    """class that group sim class, pre class and ctrl class together"""
    def __init__(self, pre, sim, ctrl, rec):
        self.sim = sim
        self.pre = pre
        self.ctrl = ctrl
        self.rec = rec


                                                        

def NMPC_of_VSL(nmpc):
    # The main script of the VSL NMPC
    # nmpc is an instance of the nmpc class
    build_configuration(nmpc)
    build_parameters(nmpc)
    define_dynamics(nmpc)
    initialize_simulation(nmpc)
    build_controller(nmpc)
    # print nmpc.sim.x[1:, 0]

    # Conduct simulation
    
    for t in xrange(int(nmpc.pre.N_exp-1)):
      print t
      compute_control(nmpc, t)
      evolve_dynamics(nmpc, t)

    # Store the variables into file
    del nmpc.ctrl
    fData = open('data.pckl', 'wb')
    pkl.dump(nmpc, fData)
    fData.close()
    pass
    # fPre = open('Pre.pckl', 'wb')
    # fRec = open('Rec.pckl', 'wb')
    # pkl.dump(nmpc.pre, fPre)
    # pkl.dump(nmpc.rec, fRec)
    # fPre.close()
    # fRec.close()


if __name__ == '__main__':
    p = pre()
    s = sim()
    c = ctrl()
    r = rec()
    mpc = nmpc(p, s, c, r)
    NMPC_of_VSL(mpc)
    #retrive_data
    