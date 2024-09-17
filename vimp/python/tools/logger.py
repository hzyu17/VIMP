## Define logger of 3d time-denpendent matrices
# Hongzhe Yu, 01/06/2023

import wandb
import numpy as np

class Logger():
    """
    The logger class to save all the data during the optimization process.
    Matrices are saved in a hash dictionary ordered according to the iteration number.
    """
    def __init__(self, nt, tf, epsilon, eta, data=None) -> None:
        self.nt = nt
        self.tf = tf
        if data is None:
            self.data = {}
        else:
            self.data = data
         
        # ========== initalize the wandb ========== 
        wandb.login()
        self.wdb = wandb.init(
            project="pgcs-mppi",
            notes="tracking matrices in the experiments",
        )
        wandb.log({"nt": nt, "tf": tf, "epsilon": epsilon, "eta": eta})
        
    # def add_iteration_data(self, iter, data_entry):
    #     """Add i_th iteration data to the logger.

    #     Args:
    #         iter (int): the iteration
    #         data (_type_): data struct containing all the matrices.
    #     """
    #     self.data[str(iter)] = data_entry
        
    # def print_data(self, iter):
    #     data_object_i = self.data[str(iter)]
    #     print(vars(data_object_i))
        
    
    def log_wandb(self, names=None, entries=None):
        """
        Log the local data to wandb html.
        """
        iterations = list(self.data.keys())
        for k in iterations:
            data_object_i = self.data[k]
            varslist = vars(data_object_i) if names is None else names 
            
            for mat_name in varslist:
                mat = getattr(data_object_i, mat_name)

                name_log_var_k = mat_name + '_' + str(k)
                self.log_tv_mat(name_log_var_k, mat_name, mat, n_logs = 50, entries=entries)
        
    def shutdown_wandb(self):
        wandb.finish()
        
    def log_tv_mat(self, log_name, mat_name, mat, n_logs, entries=None):
        """Log time dependent matrix entries, given the log length.

        Args:
            log_name (str): the name of the log
            mat (np.array): 3d matirx (nt, m, n)
            entries (list[list]): the entries to log, [[x indexes],[y indexes]]
            n_logs (int): log length
        """
        
        nstep = int(self.nt/n_logs)
        ts = np.linspace(0, self.tf, n_logs).tolist()
        if entries is None: # log all elements
            _, m, n = mat.shape
            entries = [list(range(m)), list(range(n))]
        
        Mat_list = [ [mat[it, x_indx, y_indx] for it in range(0, self.nt, nstep) ] for x_indx in entries[0] for y_indx in entries[1]  ]
        line_keys = [ mat_name + "_{}{}".format(i+1, j+1) for i in  entries[0] for j in entries[1] ]
        
        wandb.log({log_name: wandb.plot.line_series(xs=ts, 
                                                    ys=Mat_list, 
                                                    keys=line_keys,
                                                    title=log_name,
                                                    xname="Time (s)"
                                                    )
                   })