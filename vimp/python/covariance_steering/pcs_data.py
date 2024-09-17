## Define the class to record every matrix occurred in the proximal gradient covariance steering algorithm.
# Hongzhe Yu, 01/06/2023

import pickle as pkl

class PCSParams():
    def __init__(self, nt, tf, x0, xT, Sig0, SigT, epsilon, stepsize) -> None:

        self.nt = nt
        self.tf = tf

        self.x0 = x0
        self.xT = xT
        
        self.epsilon = epsilon
        self.stepsize = stepsize

        self.Sig0 = Sig0
        self.SigT = SigT
        

class PCSIterationData():
    def __init__(self) -> None:       
        """
        Collection of data throughout iterations.
        self._data is a dictionary, {iteratin(int): data(PCSData)}. 
        """
        self._data = {}
        
    def add_map_param(self, map_param):
        """
        Args:
            map_param (PquadCollisionParams): map and collision parameters.
        """
        self._data['map_param'] = map_param
        
    def add_pcs_param(self, exp_param):
        """
        Args:
            pcs_param (PCSParams): PCS experiment parameters.
        """
        self._data['exp_param'] = exp_param
        
    def add_iteration_data(self, iter, data_entry):
        self._data[str(iter)] = data_entry
        
    def dump(self, file_name):
        with open(file_name, 'wb') as f:
            pkl.dump(self._data, f, pkl.HIGHEST_PROTOCOL)
        
    def load(self, file_name):
        with open(file_name, 'rb') as f:
            # The protocol version used is detected automatically, so we do not
            # have to specify it.
            self._data = pkl.load(f)
        
    def print_data(self, iter):
        data_object_i = self._data[str(iter)]
        print(vars(data_object_i))
        
    def data(self):
        """get data.
        Returns:
            PQuadColSDF: map
            PCSData: iteration data
        """
        return self._data.pop('exp_param', None), self._data.pop('map_param', None), self._data
    
    def map_param(self):
        return self._data['map_param']
    
    def exp_param(self):
        return self._data['exp_param']
        
        
class PCSData():
    def __init__(self,
                 Ak, 
                 ak_,
                 hAt, 
                 hat, 
                 hBt,
                 Qk,
                 rk,
                 zt, 
                 Sigt, 
                 Kt=None, 
                 dt=None,
                 ) -> None:
        
        ## Data for 1 iteration
        self.Ak = Ak
        self.ak_ = ak_
        self.hAt = hAt
        self.hat = hat
        self.hBt = hBt
        self.Qk = Qk
        self.rk = rk
        self._zt = zt
        self.Sigt = Sigt
        
        if Kt is not None:
            self.Kt = Kt
        if dt is not None:
            self.dt =dt 

        return    
    
    def zt(self):
        return self._zt
    