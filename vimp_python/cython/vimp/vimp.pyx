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


cdef class PointRobot:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPointRobot_ = shared_ptr[CPointRobot]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['dof', 'nr_links'], args, kwargs)
            dof = <size_t>(__params[0])
            nr_links = <size_t>(__params[1])
            self.CPointRobot_ = shared_ptr[CPointRobot](new CPointRobot(dof, nr_links))
        except (AssertionError, ValueError):
            pass
        if (self.CPointRobot_.use_count()==0):
            raise TypeError('PointRobot construction failed!')

    @staticmethod
    cdef PointRobot cyCreateFromShared(const shared_ptr[CPointRobot]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef PointRobot return_value = PointRobot(cyCreateFromShared=True)
        return_value.CPointRobot_ = other
        return return_value

    def dof(self):
        cdef size_t ret = self.CPointRobot_.get().dof()
        return ret
    def forwardKinematicsPose(self, np.ndarray jp):
        jp = jp.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CPointRobot_.get().forwardKinematicsPose(<VectorXd>(Map[VectorXd](jp)))
        return ndarray_copy(ret)
    def forwardKinematicsPosition(self, np.ndarray jp):
        jp = jp.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CPointRobot_.get().forwardKinematicsPosition(<VectorXd>(Map[VectorXd](jp)))
        return ndarray_copy(ret)
    def forwardKinematicsVel(self, np.ndarray jp, np.ndarray jv):
        jp = jp.astype(float, order='F', copy=False)
        jv = jv.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CPointRobot_.get().forwardKinematicsVel(<VectorXd>(Map[VectorXd](jp)), <VectorXd>(Map[VectorXd](jv)))
        return ndarray_copy(ret)
    def nr_links(self):
        cdef size_t ret = self.CPointRobot_.get().nr_links()
        return ret


cdef class BodySphere:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CBodySphere_ = shared_ptr[CBodySphere]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['id', 'r', 'c'], args, kwargs)
            id = <size_t>(__params[0])
            r = <double>(__params[1])
            c = <Point3>(__params[2])
            assert isinstance(c, Point3)
            self.CBodySphere_ = shared_ptr[CBodySphere](new CBodySphere(id, r, deref(c.CPoint3_)))
        except (AssertionError, ValueError):
            pass
        if (self.CBodySphere_.use_count()==0):
            raise TypeError('BodySphere construction failed!')

    @staticmethod
    cdef BodySphere cyCreateFromShared(const shared_ptr[CBodySphere]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef BodySphere return_value = BodySphere(cyCreateFromShared=True)
        return_value.CBodySphere_ = other
        return return_value



cdef class BodySphereVector:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CBodySphereVector_ = shared_ptr[CBodySphereVector]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args([], args, kwargs)
            self.CBodySphereVector_ = shared_ptr[CBodySphereVector](new CBodySphereVector())
        except (AssertionError, ValueError):
            pass
        if (self.CBodySphereVector_.use_count()==0):
            raise TypeError('BodySphereVector construction failed!')

    @staticmethod
    cdef BodySphereVector cyCreateFromShared(const shared_ptr[CBodySphereVector]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef BodySphereVector return_value = BodySphereVector(cyCreateFromShared=True)
        return_value.CBodySphereVector_ = other
        return return_value

    def push_back(self, BodySphere sphere):
        self.CBodySphereVector_.get().push_back(deref(sphere.CBodySphere_))


cdef class PointRobotModel:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPointRobotModel_ = shared_ptr[CPointRobotModel]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['pR', 'spheres'], args, kwargs)
            pR = <PointRobot>(__params[0])
            spheres = <BodySphereVector>(__params[1])
            assert isinstance(pR, PointRobot)
            assert isinstance(spheres, BodySphereVector)
            self.CPointRobotModel_ = shared_ptr[CPointRobotModel](new CPointRobotModel(deref(pR.CPointRobot_), deref(spheres.CBodySphereVector_)))
        except (AssertionError, ValueError):
            pass
        if (self.CPointRobotModel_.use_count()==0):
            raise TypeError('PointRobotModel construction failed!')

    @staticmethod
    cdef PointRobotModel cyCreateFromShared(const shared_ptr[CPointRobotModel]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef PointRobotModel return_value = PointRobotModel(cyCreateFromShared=True)
        return_value.CPointRobotModel_ = other
        return return_value

    def dof(self):
        cdef size_t ret = self.CPointRobotModel_.get().dof()
        return ret
    def fk_model(self):
        cdef shared_ptr[CPointRobot] ret = make_shared[CPointRobot](self.CPointRobotModel_.get().fk_model())
        return PointRobot.cyCreateFromShared(ret)
    def nr_body_spheres(self):
        cdef size_t ret = self.CPointRobotModel_.get().nr_body_spheres()
        return ret
    def sphereCentersMat(self, np.ndarray conf):
        conf = conf.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CPointRobotModel_.get().sphereCentersMat(<VectorXd>(Map[VectorXd](conf)))
        return ndarray_copy(ret)
    def sphere_radius(self, size_t i):
        cdef double ret = self.CPointRobotModel_.get().sphere_radius(i)
        return ret


cdef class ObstaclePlanarSDFFactorPointRobot(NoiseModelFactor):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CObstaclePlanarSDFFactorPointRobot_ = shared_ptr[CObstaclePlanarSDFFactorPointRobot]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['posekey', 'pR', 'sdf', 'cost_sigma', 'epsilon'], args, kwargs)
            posekey = <size_t>(__params[0])
            pR = <PointRobotModel>(__params[1])
            sdf = <PlanarSDF>(__params[2])
            cost_sigma = <double>(__params[3])
            epsilon = <double>(__params[4])
            assert isinstance(pR, PointRobotModel)
            assert isinstance(sdf, PlanarSDF)
            self.CObstaclePlanarSDFFactorPointRobot_ = shared_ptr[CObstaclePlanarSDFFactorPointRobot](new CObstaclePlanarSDFFactorPointRobot(posekey, deref(pR.CPointRobotModel_), deref(sdf.CPlanarSDF_), cost_sigma, epsilon))
        except (AssertionError, ValueError):
            pass
        if (self.CObstaclePlanarSDFFactorPointRobot_.use_count()==0):
            raise TypeError('ObstaclePlanarSDFFactorPointRobot construction failed!')
        self.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(self.CObstaclePlanarSDFFactorPointRobot_)
        self.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(self.CObstaclePlanarSDFFactorPointRobot_)

    @staticmethod
    cdef ObstaclePlanarSDFFactorPointRobot cyCreateFromShared(const shared_ptr[CObstaclePlanarSDFFactorPointRobot]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef ObstaclePlanarSDFFactorPointRobot return_value = ObstaclePlanarSDFFactorPointRobot(cyCreateFromShared=True)
        return_value.CObstaclePlanarSDFFactorPointRobot_ = other
        return_value.CNoiseModelFactor_ = <shared_ptr[CNoiseModelFactor]>(other)
        return_value.CNonlinearFactor_ = <shared_ptr[CNonlinearFactor]>(other)
        return return_value

    def evaluateError(self, np.ndarray pose):
        pose = pose.astype(float, order='F', copy=False)
        cdef VectorXd ret = self.CObstaclePlanarSDFFactorPointRobot_.get().evaluateError(<VectorXd>(Map[VectorXd](pose)))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_ObstaclePlanarSDFFactorPointRobot_NoiseModelFactor(NoiseModelFactor parent):
    try:
        return ObstaclePlanarSDFFactorPointRobot.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorPointRobot]>dynamic_pointer_cast[CObstaclePlanarSDFFactorPointRobot,CNoiseModelFactor](parent.CNoiseModelFactor_))
    except:
        raise TypeError('dynamic cast failed!')
def dynamic_cast_ObstaclePlanarSDFFactorPointRobot_NonlinearFactor(NonlinearFactor parent):
    try:
        return ObstaclePlanarSDFFactorPointRobot.cyCreateFromShared(<shared_ptr[CObstaclePlanarSDFFactorPointRobot]>dynamic_pointer_cast[CObstaclePlanarSDFFactorPointRobot,CNonlinearFactor](parent.CNonlinearFactor_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class PlanarSDF:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPlanarSDF_ = shared_ptr[CPlanarSDF]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['origin', 'cell_size', 'data'], args, kwargs)
            origin = <Point2>(__params[0])
            cell_size = <double>(__params[1])
            data = <np.ndarray>(__params[2])
            assert isinstance(origin, Point2)
            assert isinstance(data, np.ndarray) and data.ndim == 2
            data = data.astype(float, order='F', copy=False)
            self.CPlanarSDF_ = shared_ptr[CPlanarSDF](new CPlanarSDF(deref(origin.CPoint2_), cell_size, <MatrixXd>(Map[MatrixXd](data))))
        except (AssertionError, ValueError):
            pass
        if (self.CPlanarSDF_.use_count()==0):
            raise TypeError('PlanarSDF construction failed!')

    @staticmethod
    cdef PlanarSDF cyCreateFromShared(const shared_ptr[CPlanarSDF]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef PlanarSDF return_value = PlanarSDF(cyCreateFromShared=True)
        return_value.CPlanarSDF_ = other
        return return_value

    def getSignedDistance(self, Point2 point):
        cdef double ret = self.CPlanarSDF_.get().getSignedDistance(deref(point.CPoint2_))
        return ret
    def __str__(self):
        strBuf = RedirectCout()
        self.print_('')
        return strBuf.str()
    def print_(self, string s):
        self.CPlanarSDF_.get().print_(s)


cdef class UnaryFactorTranslation2D:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CUnaryFactorTranslation2D_ = shared_ptr[CUnaryFactorTranslation2D]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args([], args, kwargs)
            self.CUnaryFactorTranslation2D_ = shared_ptr[CUnaryFactorTranslation2D](new CUnaryFactorTranslation2D())
        except (AssertionError, ValueError):
            pass
        try:
            __params = process_args(['key', 'conf', 'model'], args, kwargs)
            key = <size_t>(__params[0])
            conf = <np.ndarray>(__params[1])
            model = <noiseModel_Base>(__params[2])
            assert isinstance(conf, np.ndarray) and conf.ndim == 1
            assert isinstance(model, noiseModel_Base)
            conf = conf.astype(float, order='F', copy=False)
            self.CUnaryFactorTranslation2D_ = shared_ptr[CUnaryFactorTranslation2D](new CUnaryFactorTranslation2D(key, <VectorXd>(Map[VectorXd](conf)), model.CnoiseModel_Base_))
        except (AssertionError, ValueError):
            pass
        if (self.CUnaryFactorTranslation2D_.use_count()==0):
            raise TypeError('UnaryFactorTranslation2D construction failed!')

    @staticmethod
    cdef UnaryFactorTranslation2D cyCreateFromShared(const shared_ptr[CUnaryFactorTranslation2D]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef UnaryFactorTranslation2D return_value = UnaryFactorTranslation2D(cyCreateFromShared=True)
        return_value.CUnaryFactorTranslation2D_ = other
        return return_value

    def get_Qc(self):
        cdef MatrixXd ret = self.CUnaryFactorTranslation2D_.get().get_Qc()
        return ndarray_copy(ret)


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
            __params = process_args(['vec', 'prior'], args, kwargs)
            vec = <np.ndarray>(__params[0])
            prior = <UnaryFactorTranslation2D>(__params[1])
            assert isinstance(vec, np.ndarray) and vec.ndim == 1
            assert isinstance(prior, UnaryFactorTranslation2D)
            vec = vec.astype(float, order='F', copy=False)
            self.CCythonTest_ = shared_ptr[CCythonTest](new CCythonTest(<VectorXd>(Map[VectorXd](vec)), deref(prior.CUnaryFactorTranslation2D_)))
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

    def f(self, np.ndarray x):
        x = x.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CCythonTest_.get().f(<MatrixXd>(Map[MatrixXd](x)))
        return ndarray_copy(ret)
    def vec(self):
        cdef VectorXd ret = self.CCythonTest_.get().vec()
        return ndarray_copy(ret).squeeze()


cdef class CyTest2:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CCyTest2_ = shared_ptr[CCyTest2]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args([], args, kwargs)
            self.CCyTest2_ = shared_ptr[CCyTest2](new CCyTest2())
        except (AssertionError, ValueError):
            pass
        try:
            __params = process_args(['pR_model'], args, kwargs)
            pR_model = <PointRobotModel>(__params[0])
            assert isinstance(pR_model, PointRobotModel)
            self.CCyTest2_ = shared_ptr[CCyTest2](new CCyTest2(deref(pR_model.CPointRobotModel_)))
        except (AssertionError, ValueError):
            pass
        if (self.CCyTest2_.use_count()==0):
            raise TypeError('CyTest2 construction failed!')

    @staticmethod
    cdef CyTest2 cyCreateFromShared(const shared_ptr[CCyTest2]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef CyTest2 return_value = CyTest2(cyCreateFromShared=True)
        return_value.CCyTest2_ = other
        return return_value




