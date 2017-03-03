import pickle as pkl
from NMPC_of_VSL import *
from plot_libs import *


def retrieve_data(fileName):
    '''
    fileName: a string of file name
    '''
    f = file(fileName, 'rb')
    return pkl.load(f)


if __name__ == '__main__':
    dataFile = 'data.pckl'
    nmpc = retrieve_data(dataFile)
    # recFile = 'Rec.pckl'
    # preFile = 'Pre.pckl'
    # rec = retrieve_data(recFile)
    # pre = retrieve_data(preFile)
    # rho = retrieve_rho(rec, pre)
    # print rho
    pass