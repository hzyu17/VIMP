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

cdef extern from "gpmp2/kinematics/PointRobot.h" namespace "gpmp2":
    cdef cppclass CPointRobot "gpmp2::PointRobot":
        CPointRobot(size_t dof, size_t nr_links) except +

        size_t dof() except +
        MatrixXd forwardKinematicsPose(const VectorXd& jp) except +
        MatrixXd forwardKinematicsPosition(const VectorXd& jp) except +
        MatrixXd forwardKinematicsVel(const VectorXd& jp, const VectorXd& jv) except +
        size_t nr_links() except +

cdef class PointRobot:
    cdef shared_ptr[CPointRobot] CPointRobot_
    @staticmethod
    cdef PointRobot cyCreateFromShared(const shared_ptr[CPointRobot]& other)


cdef extern from "gpmp2/kinematics/RobotModel.h" namespace "gpmp2":
    cdef cppclass CBodySphere "gpmp2::BodySphere":
        CBodySphere(size_t id, double r, const CPoint3& c) except +


cdef class BodySphere:
    cdef shared_ptr[CBodySphere] CBodySphere_
    @staticmethod
    cdef BodySphere cyCreateFromShared(const shared_ptr[CBodySphere]& other)


cdef extern from "gpmp2/kinematics/RobotModel.h" namespace "gpmp2":
    cdef cppclass CBodySphereVector "gpmp2::BodySphereVector":
        CBodySphereVector() except +

        void push_back(const CBodySphere& sphere) except +

cdef class BodySphereVector:
    cdef shared_ptr[CBodySphereVector] CBodySphereVector_
    @staticmethod
    cdef BodySphereVector cyCreateFromShared(const shared_ptr[CBodySphereVector]& other)


cdef extern from "gpmp2/kinematics/PointRobotModel.h" namespace "gpmp2":
    cdef cppclass CPointRobotModel "gpmp2::PointRobotModel":
        CPointRobotModel(const CPointRobot& pR, const CBodySphereVector& spheres) except +

        size_t dof() except +
        CPointRobot fk_model() except +
        size_t nr_body_spheres() except +
        MatrixXd sphereCentersMat(const VectorXd& conf) except +
        double sphere_radius(size_t i) except +

cdef class PointRobotModel:
    cdef shared_ptr[CPointRobotModel] CPointRobotModel_
    @staticmethod
    cdef PointRobotModel cyCreateFromShared(const shared_ptr[CPointRobotModel]& other)


cdef extern from "gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h" namespace "gpmp2":
    cdef cppclass CObstaclePlanarSDFFactorPointRobot "gpmp2::ObstaclePlanarSDFFactorPointRobot"(CNoiseModelFactor):
        CObstaclePlanarSDFFactorPointRobot(size_t posekey, const CPointRobotModel& pR, const CPlanarSDF& sdf, double cost_sigma, double epsilon) except +

        VectorXd evaluateError(const VectorXd& pose) except +

cdef class ObstaclePlanarSDFFactorPointRobot(NoiseModelFactor):
    cdef shared_ptr[CObstaclePlanarSDFFactorPointRobot] CObstaclePlanarSDFFactorPointRobot_
    @staticmethod
    cdef ObstaclePlanarSDFFactorPointRobot cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorPointRobot]& other)


cdef extern from "gpmp2/obstacle/PlanarSDF.h" namespace "gpmp2":
    cdef cppclass CPlanarSDF "gpmp2::PlanarSDF":
        CPlanarSDF(const CPoint2& origin, double cell_size, const MatrixXd& data) except +

        double getSignedDistance(const CPoint2& point) except +
        void print_ "print"(string s) except +

cdef class PlanarSDF:
    cdef shared_ptr[CPlanarSDF] CPlanarSDF_
    @staticmethod
    cdef PlanarSDF cyCreateFromShared(const shared_ptr[CPlanarSDF]& other)


cdef extern from "vimp/instances/PriorColPlanarPointRobot.h" namespace "vimp":
    cdef cppclass CUnaryFactorTranslation2D "vimp::UnaryFactorTranslation2D":
        CUnaryFactorTranslation2D() except +
        CUnaryFactorTranslation2D(size_t key, const VectorXd& conf, const shared_ptr[CnoiseModel_Base]& model) except +

        MatrixXd get_Qc() except +

cdef class UnaryFactorTranslation2D:
    cdef shared_ptr[CUnaryFactorTranslation2D] CUnaryFactorTranslation2D_
    @staticmethod
    cdef UnaryFactorTranslation2D cyCreateFromShared(const shared_ptr[CUnaryFactorTranslation2D]& other)


cdef extern from "vimp/helpers/test_cython.h" namespace "vimp":
    cdef cppclass CCythonTest "vimp::CythonTest":
        CCythonTest() except +
        CCythonTest(const VectorXd& vec, const CUnaryFactorTranslation2D& prior) except +

        MatrixXd f(const MatrixXd& x) except +
        VectorXd vec() except +

cdef class CythonTest:
    cdef shared_ptr[CCythonTest] CCythonTest_
    @staticmethod
    cdef CythonTest cyCreateFromShared(const shared_ptr[CCythonTest]& other)


cdef extern from "vimp/helpers/test_cython.h" namespace "vimp":
    cdef cppclass CCyTest2 "vimp::CyTest2":
        CCyTest2() except +
        CCyTest2(const CPointRobotModel& pR_model) except +


cdef class CyTest2:
    cdef shared_ptr[CCyTest2] CCyTest2_
    @staticmethod
    cdef CyTest2 cyCreateFromShared(const shared_ptr[CCyTest2]& other)


