cimport numpy as np
import numpy as npp
cimport vimp
from vimp cimport shared_ptr
from vimp cimport dynamic_pointer_cast
from vimp cimport make_shared
# C helper function that copies all arguments into a positional list.
cdef list process_args(list keywords, tuple args, dict kwargs):
   cdef str keyword
   cdef int n = len(args), m = len(keywords)
   cdef list params = list(args)
   assert len(args)+len(kwargs) == m, 'Expected {} arguments'.format(m)
   try:
       return params + [kwargs[keyword] for keyword in keywords[n:]]
   except:
       raise ValueError('Epected arguments ' + str(keywords))
from gtsam_eigency.core cimport *
from libcpp cimport bool

from libcpp.pair cimport pair
from libcpp.string cimport string
from cython.operator cimport dereference as deref


cdef class CythonTest:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CCythonTest_ = shared_ptr[CCythonTest]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args([], args, kwargs)
            self.CCythonTest_ = shared_ptr[CCythonTest](new CCythonTest())
        except (AssertionError, ValueError):
            pass
        try:
            __params = process_args(['mat'], args, kwargs)
            mat = <np.ndarray>(__params[0])
            assert isinstance(mat, np.ndarray) and mat.ndim == 2
            mat = mat.astype(float, order='F', copy=False)
            self.CCythonTest_ = shared_ptr[CCythonTest](new CCythonTest(<MatrixXd>(Map[MatrixXd](mat))))
        except (AssertionError, ValueError):
            pass
        if (self.CCythonTest_.use_count()==0):
            raise TypeError('CythonTest construction failed!')

    @staticmethod
    cdef CythonTest cyCreateFromShared(const shared_ptr[CCythonTest]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef CythonTest return_value = CythonTest(cyCreateFromShared=True)
        return_value.CCythonTest_ = other
        return return_value

    def print_mat(self):
        self.CCythonTest_.get().print_mat()



