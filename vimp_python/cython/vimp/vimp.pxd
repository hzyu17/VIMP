from gtsam.gtsam cimport *
from gtsam_eigency.core cimport *
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.pair cimport pair
from libcpp.set cimport set
from libcpp.map cimport map
from libcpp cimport bool

cdef extern from "boost/shared_ptr.hpp" namespace "boost":
    cppclass shared_ptr[T]:
        shared_ptr()
        shared_ptr(T*)
        T* get()
        long use_count() const
        T& operator*()

    cdef shared_ptr[T] dynamic_pointer_cast[T,U](const shared_ptr[U]& r)
    cdef shared_ptr[T] make_shared[T](const T& r)

cdef extern from "vimp/helpers/test_cython.h" namespace "vimp":
    cdef cppclass CCythonTest "vimp::CythonTest":
        CCythonTest() except +
        CCythonTest(const MatrixXd& mat) except +

        void print_mat() except +

cdef class CythonTest:
    cdef shared_ptr[CCythonTest] CCythonTest_
    @staticmethod
    cdef CythonTest cyCreateFromShared(const shared_ptr[CCythonTest]& other)


