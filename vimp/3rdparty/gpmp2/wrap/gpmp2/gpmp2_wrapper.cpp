#include <wrap/matlab.h>
#include <map>

#include <boost/serialization/export.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <gpmp2/dynamics/VehicleDynamicsFactorPose2.h>
#include <gpmp2/dynamics/VehicleDynamicsFactorPose2Vector.h>
#include <gpmp2/dynamics/VehicleDynamicsFactorVector.h>
#include <gpmp2/geometry/Pose2Vector.h>
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>
#include <gpmp2/gp/GaussianProcessPriorLinear.h>
#include <gpmp2/gp/GaussianProcessPriorPose2.h>
#include <gpmp2/gp/GaussianProcessPriorPose2Vector.h>
#include <gpmp2/kinematics/Arm.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/kinematics/GaussianPriorWorkspaceOrientationArm.h>
#include <gpmp2/kinematics/GaussianPriorWorkspacePoseArm.h>
#include <gpmp2/kinematics/GaussianPriorWorkspacePositionArm.h>
#include <gpmp2/kinematics/GoalFactorArm.h>
#include <gpmp2/kinematics/JointLimitFactorVector.h>
#include <gpmp2/kinematics/PointRobot.h>
#include <gpmp2/kinematics/PointRobotModel.h>
#include <gpmp2/kinematics/Pose2Mobile2Arms.h>
#include <gpmp2/kinematics/Pose2Mobile2ArmsModel.h>
#include <gpmp2/kinematics/Pose2MobileArm.h>
#include <gpmp2/kinematics/Pose2MobileArmModel.h>
#include <gpmp2/kinematics/Pose2MobileBase.h>
#include <gpmp2/kinematics/Pose2MobileBaseModel.h>
#include <gpmp2/kinematics/Pose2MobileVetLin2Arms.h>
#include <gpmp2/kinematics/Pose2MobileVetLin2ArmsModel.h>
#include <gpmp2/kinematics/Pose2MobileVetLinArm.h>
#include <gpmp2/kinematics/Pose2MobileVetLinArmModel.h>
#include <gpmp2/kinematics/RobotModel.h>
#include <gpmp2/kinematics/VelocityLimitFactorVector.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorArm.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPArm.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPointRobot.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2Mobile2Arms.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileArm.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileBase.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPose2Mobile2Arms.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileArm.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileBase.h>
#include <gpmp2/obstacle/ObstacleSDFFactorArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2Mobile2Arms.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileBase.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileVetLin2Arms.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileVetLinArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorPose2Mobile2Arms.h>
#include <gpmp2/obstacle/ObstacleSDFFactorPose2MobileArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorPose2MobileBase.h>
#include <gpmp2/obstacle/ObstacleSDFFactorPose2MobileVetLin2Arms.h>
#include <gpmp2/obstacle/ObstacleSDFFactorPose2MobileVetLinArm.h>
#include <gpmp2/obstacle/PlanarSDF.h>
#include <gpmp2/obstacle/SelfCollisionArm.h>
#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/planner/BatchTrajOptimizer.h>
#include <gpmp2/planner/ISAM2TrajOptimizer.h>
#include <gpmp2/planner/TrajOptimizerSetting.h>
#include <gpmp2/planner/TrajUtils.h>
#include <gpmp2/utils/matlabUtils.h>


BOOST_CLASS_EXPORT_GUID(gpmp2::GaussianProcessPriorLinear, "gpmp2GaussianProcessPriorLinear");
BOOST_CLASS_EXPORT_GUID(gpmp2::ObstaclePlanarSDFFactorArm, "gpmp2ObstaclePlanarSDFFactorArm");

typedef std::set<boost::shared_ptr<gpmp2::Pose2Vector>*> Collector_gpmp2Pose2Vector;
static Collector_gpmp2Pose2Vector collector_gpmp2Pose2Vector;
typedef std::set<boost::shared_ptr<gpmp2::GaussianProcessPriorLinear>*> Collector_gpmp2GaussianProcessPriorLinear;
static Collector_gpmp2GaussianProcessPriorLinear collector_gpmp2GaussianProcessPriorLinear;
typedef std::set<boost::shared_ptr<gpmp2::GaussianProcessPriorPose2>*> Collector_gpmp2GaussianProcessPriorPose2;
static Collector_gpmp2GaussianProcessPriorPose2 collector_gpmp2GaussianProcessPriorPose2;
typedef std::set<boost::shared_ptr<gpmp2::GaussianProcessPriorPose2Vector>*> Collector_gpmp2GaussianProcessPriorPose2Vector;
static Collector_gpmp2GaussianProcessPriorPose2Vector collector_gpmp2GaussianProcessPriorPose2Vector;
typedef std::set<boost::shared_ptr<gpmp2::GaussianProcessInterpolatorLinear>*> Collector_gpmp2GaussianProcessInterpolatorLinear;
static Collector_gpmp2GaussianProcessInterpolatorLinear collector_gpmp2GaussianProcessInterpolatorLinear;
typedef std::set<boost::shared_ptr<gpmp2::Arm>*> Collector_gpmp2Arm;
static Collector_gpmp2Arm collector_gpmp2Arm;
typedef std::set<boost::shared_ptr<gpmp2::Pose2MobileBase>*> Collector_gpmp2Pose2MobileBase;
static Collector_gpmp2Pose2MobileBase collector_gpmp2Pose2MobileBase;
typedef std::set<boost::shared_ptr<gpmp2::Pose2MobileArm>*> Collector_gpmp2Pose2MobileArm;
static Collector_gpmp2Pose2MobileArm collector_gpmp2Pose2MobileArm;
typedef std::set<boost::shared_ptr<gpmp2::Pose2Mobile2Arms>*> Collector_gpmp2Pose2Mobile2Arms;
static Collector_gpmp2Pose2Mobile2Arms collector_gpmp2Pose2Mobile2Arms;
typedef std::set<boost::shared_ptr<gpmp2::Pose2MobileVetLinArm>*> Collector_gpmp2Pose2MobileVetLinArm;
static Collector_gpmp2Pose2MobileVetLinArm collector_gpmp2Pose2MobileVetLinArm;
typedef std::set<boost::shared_ptr<gpmp2::Pose2MobileVetLin2Arms>*> Collector_gpmp2Pose2MobileVetLin2Arms;
static Collector_gpmp2Pose2MobileVetLin2Arms collector_gpmp2Pose2MobileVetLin2Arms;
typedef std::set<boost::shared_ptr<gpmp2::PointRobot>*> Collector_gpmp2PointRobot;
static Collector_gpmp2PointRobot collector_gpmp2PointRobot;
typedef std::set<boost::shared_ptr<gpmp2::BodySphere>*> Collector_gpmp2BodySphere;
static Collector_gpmp2BodySphere collector_gpmp2BodySphere;
typedef std::set<boost::shared_ptr<gpmp2::BodySphereVector>*> Collector_gpmp2BodySphereVector;
static Collector_gpmp2BodySphereVector collector_gpmp2BodySphereVector;
typedef std::set<boost::shared_ptr<gpmp2::ArmModel>*> Collector_gpmp2ArmModel;
static Collector_gpmp2ArmModel collector_gpmp2ArmModel;
typedef std::set<boost::shared_ptr<gpmp2::Pose2MobileBaseModel>*> Collector_gpmp2Pose2MobileBaseModel;
static Collector_gpmp2Pose2MobileBaseModel collector_gpmp2Pose2MobileBaseModel;
typedef std::set<boost::shared_ptr<gpmp2::Pose2MobileArmModel>*> Collector_gpmp2Pose2MobileArmModel;
static Collector_gpmp2Pose2MobileArmModel collector_gpmp2Pose2MobileArmModel;
typedef std::set<boost::shared_ptr<gpmp2::Pose2Mobile2ArmsModel>*> Collector_gpmp2Pose2Mobile2ArmsModel;
static Collector_gpmp2Pose2Mobile2ArmsModel collector_gpmp2Pose2Mobile2ArmsModel;
typedef std::set<boost::shared_ptr<gpmp2::Pose2MobileVetLinArmModel>*> Collector_gpmp2Pose2MobileVetLinArmModel;
static Collector_gpmp2Pose2MobileVetLinArmModel collector_gpmp2Pose2MobileVetLinArmModel;
typedef std::set<boost::shared_ptr<gpmp2::Pose2MobileVetLin2ArmsModel>*> Collector_gpmp2Pose2MobileVetLin2ArmsModel;
static Collector_gpmp2Pose2MobileVetLin2ArmsModel collector_gpmp2Pose2MobileVetLin2ArmsModel;
typedef std::set<boost::shared_ptr<gpmp2::PointRobotModel>*> Collector_gpmp2PointRobotModel;
static Collector_gpmp2PointRobotModel collector_gpmp2PointRobotModel;
typedef std::set<boost::shared_ptr<gpmp2::GoalFactorArm>*> Collector_gpmp2GoalFactorArm;
static Collector_gpmp2GoalFactorArm collector_gpmp2GoalFactorArm;
typedef std::set<boost::shared_ptr<gpmp2::JointLimitFactorVector>*> Collector_gpmp2JointLimitFactorVector;
static Collector_gpmp2JointLimitFactorVector collector_gpmp2JointLimitFactorVector;
typedef std::set<boost::shared_ptr<gpmp2::VelocityLimitFactorVector>*> Collector_gpmp2VelocityLimitFactorVector;
static Collector_gpmp2VelocityLimitFactorVector collector_gpmp2VelocityLimitFactorVector;
typedef std::set<boost::shared_ptr<gpmp2::GaussianPriorWorkspacePositionArm>*> Collector_gpmp2GaussianPriorWorkspacePositionArm;
static Collector_gpmp2GaussianPriorWorkspacePositionArm collector_gpmp2GaussianPriorWorkspacePositionArm;
typedef std::set<boost::shared_ptr<gpmp2::GaussianPriorWorkspaceOrientationArm>*> Collector_gpmp2GaussianPriorWorkspaceOrientationArm;
static Collector_gpmp2GaussianPriorWorkspaceOrientationArm collector_gpmp2GaussianPriorWorkspaceOrientationArm;
typedef std::set<boost::shared_ptr<gpmp2::GaussianPriorWorkspacePoseArm>*> Collector_gpmp2GaussianPriorWorkspacePoseArm;
static Collector_gpmp2GaussianPriorWorkspacePoseArm collector_gpmp2GaussianPriorWorkspacePoseArm;
typedef std::set<boost::shared_ptr<gpmp2::VehicleDynamicsFactorPose2>*> Collector_gpmp2VehicleDynamicsFactorPose2;
static Collector_gpmp2VehicleDynamicsFactorPose2 collector_gpmp2VehicleDynamicsFactorPose2;
typedef std::set<boost::shared_ptr<gpmp2::VehicleDynamicsFactorPose2Vector>*> Collector_gpmp2VehicleDynamicsFactorPose2Vector;
static Collector_gpmp2VehicleDynamicsFactorPose2Vector collector_gpmp2VehicleDynamicsFactorPose2Vector;
typedef std::set<boost::shared_ptr<gpmp2::VehicleDynamicsFactorVector>*> Collector_gpmp2VehicleDynamicsFactorVector;
static Collector_gpmp2VehicleDynamicsFactorVector collector_gpmp2VehicleDynamicsFactorVector;
typedef std::set<boost::shared_ptr<gpmp2::SignedDistanceField>*> Collector_gpmp2SignedDistanceField;
static Collector_gpmp2SignedDistanceField collector_gpmp2SignedDistanceField;
typedef std::set<boost::shared_ptr<gpmp2::PlanarSDF>*> Collector_gpmp2PlanarSDF;
static Collector_gpmp2PlanarSDF collector_gpmp2PlanarSDF;
typedef std::set<boost::shared_ptr<gpmp2::ObstacleSDFFactorArm>*> Collector_gpmp2ObstacleSDFFactorArm;
static Collector_gpmp2ObstacleSDFFactorArm collector_gpmp2ObstacleSDFFactorArm;
typedef std::set<boost::shared_ptr<gpmp2::ObstacleSDFFactorGPArm>*> Collector_gpmp2ObstacleSDFFactorGPArm;
static Collector_gpmp2ObstacleSDFFactorGPArm collector_gpmp2ObstacleSDFFactorGPArm;
typedef std::set<boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorArm>*> Collector_gpmp2ObstaclePlanarSDFFactorArm;
static Collector_gpmp2ObstaclePlanarSDFFactorArm collector_gpmp2ObstaclePlanarSDFFactorArm;
typedef std::set<boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPArm>*> Collector_gpmp2ObstaclePlanarSDFFactorGPArm;
static Collector_gpmp2ObstaclePlanarSDFFactorGPArm collector_gpmp2ObstaclePlanarSDFFactorGPArm;
typedef std::set<boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPointRobot>*> Collector_gpmp2ObstaclePlanarSDFFactorPointRobot;
static Collector_gpmp2ObstaclePlanarSDFFactorPointRobot collector_gpmp2ObstaclePlanarSDFFactorPointRobot;
typedef std::set<boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPointRobot>*> Collector_gpmp2ObstaclePlanarSDFFactorGPPointRobot;
static Collector_gpmp2ObstaclePlanarSDFFactorGPPointRobot collector_gpmp2ObstaclePlanarSDFFactorGPPointRobot;
typedef std::set<boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2MobileBase>*> Collector_gpmp2ObstaclePlanarSDFFactorPose2MobileBase;
static Collector_gpmp2ObstaclePlanarSDFFactorPose2MobileBase collector_gpmp2ObstaclePlanarSDFFactorPose2MobileBase;
typedef std::set<boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPose2MobileBase>*> Collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase;
static Collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase;
typedef std::set<boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2MobileArm>*> Collector_gpmp2ObstaclePlanarSDFFactorPose2MobileArm;
static Collector_gpmp2ObstaclePlanarSDFFactorPose2MobileArm collector_gpmp2ObstaclePlanarSDFFactorPose2MobileArm;
typedef std::set<boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPose2MobileArm>*> Collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm;
static Collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm;
typedef std::set<boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2Mobile2Arms>*> Collector_gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms;
static Collector_gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms collector_gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms;
typedef std::set<boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPose2Mobile2Arms>*> Collector_gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms;
static Collector_gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms collector_gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms;
typedef std::set<boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileBase>*> Collector_gpmp2ObstacleSDFFactorPose2MobileBase;
static Collector_gpmp2ObstacleSDFFactorPose2MobileBase collector_gpmp2ObstacleSDFFactorPose2MobileBase;
typedef std::set<boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileBase>*> Collector_gpmp2ObstacleSDFFactorGPPose2MobileBase;
static Collector_gpmp2ObstacleSDFFactorGPPose2MobileBase collector_gpmp2ObstacleSDFFactorGPPose2MobileBase;
typedef std::set<boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileArm>*> Collector_gpmp2ObstacleSDFFactorPose2MobileArm;
static Collector_gpmp2ObstacleSDFFactorPose2MobileArm collector_gpmp2ObstacleSDFFactorPose2MobileArm;
typedef std::set<boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileArm>*> Collector_gpmp2ObstacleSDFFactorGPPose2MobileArm;
static Collector_gpmp2ObstacleSDFFactorGPPose2MobileArm collector_gpmp2ObstacleSDFFactorGPPose2MobileArm;
typedef std::set<boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2Mobile2Arms>*> Collector_gpmp2ObstacleSDFFactorPose2Mobile2Arms;
static Collector_gpmp2ObstacleSDFFactorPose2Mobile2Arms collector_gpmp2ObstacleSDFFactorPose2Mobile2Arms;
typedef std::set<boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2Mobile2Arms>*> Collector_gpmp2ObstacleSDFFactorGPPose2Mobile2Arms;
static Collector_gpmp2ObstacleSDFFactorGPPose2Mobile2Arms collector_gpmp2ObstacleSDFFactorGPPose2Mobile2Arms;
typedef std::set<boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileVetLinArm>*> Collector_gpmp2ObstacleSDFFactorPose2MobileVetLinArm;
static Collector_gpmp2ObstacleSDFFactorPose2MobileVetLinArm collector_gpmp2ObstacleSDFFactorPose2MobileVetLinArm;
typedef std::set<boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileVetLinArm>*> Collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm;
static Collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm;
typedef std::set<boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileVetLin2Arms>*> Collector_gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms;
static Collector_gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms collector_gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms;
typedef std::set<boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileVetLin2Arms>*> Collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms;
static Collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms;
typedef std::set<boost::shared_ptr<gpmp2::SelfCollisionArm>*> Collector_gpmp2SelfCollisionArm;
static Collector_gpmp2SelfCollisionArm collector_gpmp2SelfCollisionArm;
typedef std::set<boost::shared_ptr<gpmp2::TrajOptimizerSetting>*> Collector_gpmp2TrajOptimizerSetting;
static Collector_gpmp2TrajOptimizerSetting collector_gpmp2TrajOptimizerSetting;
typedef std::set<boost::shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm>*> Collector_gpmp2ISAM2TrajOptimizer2DArm;
static Collector_gpmp2ISAM2TrajOptimizer2DArm collector_gpmp2ISAM2TrajOptimizer2DArm;
typedef std::set<boost::shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm>*> Collector_gpmp2ISAM2TrajOptimizer3DArm;
static Collector_gpmp2ISAM2TrajOptimizer3DArm collector_gpmp2ISAM2TrajOptimizer3DArm;
typedef std::set<boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D>*> Collector_gpmp2ISAM2TrajOptimizerPose2MobileArm2D;
static Collector_gpmp2ISAM2TrajOptimizerPose2MobileArm2D collector_gpmp2ISAM2TrajOptimizerPose2MobileArm2D;
typedef std::set<boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm>*> Collector_gpmp2ISAM2TrajOptimizerPose2MobileArm;
static Collector_gpmp2ISAM2TrajOptimizerPose2MobileArm collector_gpmp2ISAM2TrajOptimizerPose2MobileArm;
typedef std::set<boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms>*> Collector_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms;
static Collector_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms collector_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms;
typedef std::set<boost::shared_ptr<gpmp2::PriorFactorPose2Vector>*> Collector_gpmp2PriorFactorPose2Vector;
static Collector_gpmp2PriorFactorPose2Vector collector_gpmp2PriorFactorPose2Vector;

void _deleteAllObjects()
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  bool anyDeleted = false;
  { for(Collector_gpmp2Pose2Vector::iterator iter = collector_gpmp2Pose2Vector.begin();
      iter != collector_gpmp2Pose2Vector.end(); ) {
    delete *iter;
    collector_gpmp2Pose2Vector.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2GaussianProcessPriorLinear::iterator iter = collector_gpmp2GaussianProcessPriorLinear.begin();
      iter != collector_gpmp2GaussianProcessPriorLinear.end(); ) {
    delete *iter;
    collector_gpmp2GaussianProcessPriorLinear.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2GaussianProcessPriorPose2::iterator iter = collector_gpmp2GaussianProcessPriorPose2.begin();
      iter != collector_gpmp2GaussianProcessPriorPose2.end(); ) {
    delete *iter;
    collector_gpmp2GaussianProcessPriorPose2.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2GaussianProcessPriorPose2Vector::iterator iter = collector_gpmp2GaussianProcessPriorPose2Vector.begin();
      iter != collector_gpmp2GaussianProcessPriorPose2Vector.end(); ) {
    delete *iter;
    collector_gpmp2GaussianProcessPriorPose2Vector.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2GaussianProcessInterpolatorLinear::iterator iter = collector_gpmp2GaussianProcessInterpolatorLinear.begin();
      iter != collector_gpmp2GaussianProcessInterpolatorLinear.end(); ) {
    delete *iter;
    collector_gpmp2GaussianProcessInterpolatorLinear.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2Arm::iterator iter = collector_gpmp2Arm.begin();
      iter != collector_gpmp2Arm.end(); ) {
    delete *iter;
    collector_gpmp2Arm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2Pose2MobileBase::iterator iter = collector_gpmp2Pose2MobileBase.begin();
      iter != collector_gpmp2Pose2MobileBase.end(); ) {
    delete *iter;
    collector_gpmp2Pose2MobileBase.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2Pose2MobileArm::iterator iter = collector_gpmp2Pose2MobileArm.begin();
      iter != collector_gpmp2Pose2MobileArm.end(); ) {
    delete *iter;
    collector_gpmp2Pose2MobileArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2Pose2Mobile2Arms::iterator iter = collector_gpmp2Pose2Mobile2Arms.begin();
      iter != collector_gpmp2Pose2Mobile2Arms.end(); ) {
    delete *iter;
    collector_gpmp2Pose2Mobile2Arms.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2Pose2MobileVetLinArm::iterator iter = collector_gpmp2Pose2MobileVetLinArm.begin();
      iter != collector_gpmp2Pose2MobileVetLinArm.end(); ) {
    delete *iter;
    collector_gpmp2Pose2MobileVetLinArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2Pose2MobileVetLin2Arms::iterator iter = collector_gpmp2Pose2MobileVetLin2Arms.begin();
      iter != collector_gpmp2Pose2MobileVetLin2Arms.end(); ) {
    delete *iter;
    collector_gpmp2Pose2MobileVetLin2Arms.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2PointRobot::iterator iter = collector_gpmp2PointRobot.begin();
      iter != collector_gpmp2PointRobot.end(); ) {
    delete *iter;
    collector_gpmp2PointRobot.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2BodySphere::iterator iter = collector_gpmp2BodySphere.begin();
      iter != collector_gpmp2BodySphere.end(); ) {
    delete *iter;
    collector_gpmp2BodySphere.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2BodySphereVector::iterator iter = collector_gpmp2BodySphereVector.begin();
      iter != collector_gpmp2BodySphereVector.end(); ) {
    delete *iter;
    collector_gpmp2BodySphereVector.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ArmModel::iterator iter = collector_gpmp2ArmModel.begin();
      iter != collector_gpmp2ArmModel.end(); ) {
    delete *iter;
    collector_gpmp2ArmModel.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2Pose2MobileBaseModel::iterator iter = collector_gpmp2Pose2MobileBaseModel.begin();
      iter != collector_gpmp2Pose2MobileBaseModel.end(); ) {
    delete *iter;
    collector_gpmp2Pose2MobileBaseModel.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2Pose2MobileArmModel::iterator iter = collector_gpmp2Pose2MobileArmModel.begin();
      iter != collector_gpmp2Pose2MobileArmModel.end(); ) {
    delete *iter;
    collector_gpmp2Pose2MobileArmModel.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2Pose2Mobile2ArmsModel::iterator iter = collector_gpmp2Pose2Mobile2ArmsModel.begin();
      iter != collector_gpmp2Pose2Mobile2ArmsModel.end(); ) {
    delete *iter;
    collector_gpmp2Pose2Mobile2ArmsModel.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2Pose2MobileVetLinArmModel::iterator iter = collector_gpmp2Pose2MobileVetLinArmModel.begin();
      iter != collector_gpmp2Pose2MobileVetLinArmModel.end(); ) {
    delete *iter;
    collector_gpmp2Pose2MobileVetLinArmModel.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2Pose2MobileVetLin2ArmsModel::iterator iter = collector_gpmp2Pose2MobileVetLin2ArmsModel.begin();
      iter != collector_gpmp2Pose2MobileVetLin2ArmsModel.end(); ) {
    delete *iter;
    collector_gpmp2Pose2MobileVetLin2ArmsModel.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2PointRobotModel::iterator iter = collector_gpmp2PointRobotModel.begin();
      iter != collector_gpmp2PointRobotModel.end(); ) {
    delete *iter;
    collector_gpmp2PointRobotModel.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2GoalFactorArm::iterator iter = collector_gpmp2GoalFactorArm.begin();
      iter != collector_gpmp2GoalFactorArm.end(); ) {
    delete *iter;
    collector_gpmp2GoalFactorArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2JointLimitFactorVector::iterator iter = collector_gpmp2JointLimitFactorVector.begin();
      iter != collector_gpmp2JointLimitFactorVector.end(); ) {
    delete *iter;
    collector_gpmp2JointLimitFactorVector.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2VelocityLimitFactorVector::iterator iter = collector_gpmp2VelocityLimitFactorVector.begin();
      iter != collector_gpmp2VelocityLimitFactorVector.end(); ) {
    delete *iter;
    collector_gpmp2VelocityLimitFactorVector.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2GaussianPriorWorkspacePositionArm::iterator iter = collector_gpmp2GaussianPriorWorkspacePositionArm.begin();
      iter != collector_gpmp2GaussianPriorWorkspacePositionArm.end(); ) {
    delete *iter;
    collector_gpmp2GaussianPriorWorkspacePositionArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2GaussianPriorWorkspaceOrientationArm::iterator iter = collector_gpmp2GaussianPriorWorkspaceOrientationArm.begin();
      iter != collector_gpmp2GaussianPriorWorkspaceOrientationArm.end(); ) {
    delete *iter;
    collector_gpmp2GaussianPriorWorkspaceOrientationArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2GaussianPriorWorkspacePoseArm::iterator iter = collector_gpmp2GaussianPriorWorkspacePoseArm.begin();
      iter != collector_gpmp2GaussianPriorWorkspacePoseArm.end(); ) {
    delete *iter;
    collector_gpmp2GaussianPriorWorkspacePoseArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2VehicleDynamicsFactorPose2::iterator iter = collector_gpmp2VehicleDynamicsFactorPose2.begin();
      iter != collector_gpmp2VehicleDynamicsFactorPose2.end(); ) {
    delete *iter;
    collector_gpmp2VehicleDynamicsFactorPose2.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2VehicleDynamicsFactorPose2Vector::iterator iter = collector_gpmp2VehicleDynamicsFactorPose2Vector.begin();
      iter != collector_gpmp2VehicleDynamicsFactorPose2Vector.end(); ) {
    delete *iter;
    collector_gpmp2VehicleDynamicsFactorPose2Vector.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2VehicleDynamicsFactorVector::iterator iter = collector_gpmp2VehicleDynamicsFactorVector.begin();
      iter != collector_gpmp2VehicleDynamicsFactorVector.end(); ) {
    delete *iter;
    collector_gpmp2VehicleDynamicsFactorVector.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2SignedDistanceField::iterator iter = collector_gpmp2SignedDistanceField.begin();
      iter != collector_gpmp2SignedDistanceField.end(); ) {
    delete *iter;
    collector_gpmp2SignedDistanceField.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2PlanarSDF::iterator iter = collector_gpmp2PlanarSDF.begin();
      iter != collector_gpmp2PlanarSDF.end(); ) {
    delete *iter;
    collector_gpmp2PlanarSDF.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstacleSDFFactorArm::iterator iter = collector_gpmp2ObstacleSDFFactorArm.begin();
      iter != collector_gpmp2ObstacleSDFFactorArm.end(); ) {
    delete *iter;
    collector_gpmp2ObstacleSDFFactorArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstacleSDFFactorGPArm::iterator iter = collector_gpmp2ObstacleSDFFactorGPArm.begin();
      iter != collector_gpmp2ObstacleSDFFactorGPArm.end(); ) {
    delete *iter;
    collector_gpmp2ObstacleSDFFactorGPArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstaclePlanarSDFFactorArm::iterator iter = collector_gpmp2ObstaclePlanarSDFFactorArm.begin();
      iter != collector_gpmp2ObstaclePlanarSDFFactorArm.end(); ) {
    delete *iter;
    collector_gpmp2ObstaclePlanarSDFFactorArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstaclePlanarSDFFactorGPArm::iterator iter = collector_gpmp2ObstaclePlanarSDFFactorGPArm.begin();
      iter != collector_gpmp2ObstaclePlanarSDFFactorGPArm.end(); ) {
    delete *iter;
    collector_gpmp2ObstaclePlanarSDFFactorGPArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstaclePlanarSDFFactorPointRobot::iterator iter = collector_gpmp2ObstaclePlanarSDFFactorPointRobot.begin();
      iter != collector_gpmp2ObstaclePlanarSDFFactorPointRobot.end(); ) {
    delete *iter;
    collector_gpmp2ObstaclePlanarSDFFactorPointRobot.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstaclePlanarSDFFactorGPPointRobot::iterator iter = collector_gpmp2ObstaclePlanarSDFFactorGPPointRobot.begin();
      iter != collector_gpmp2ObstaclePlanarSDFFactorGPPointRobot.end(); ) {
    delete *iter;
    collector_gpmp2ObstaclePlanarSDFFactorGPPointRobot.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstaclePlanarSDFFactorPose2MobileBase::iterator iter = collector_gpmp2ObstaclePlanarSDFFactorPose2MobileBase.begin();
      iter != collector_gpmp2ObstaclePlanarSDFFactorPose2MobileBase.end(); ) {
    delete *iter;
    collector_gpmp2ObstaclePlanarSDFFactorPose2MobileBase.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase::iterator iter = collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase.begin();
      iter != collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase.end(); ) {
    delete *iter;
    collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstaclePlanarSDFFactorPose2MobileArm::iterator iter = collector_gpmp2ObstaclePlanarSDFFactorPose2MobileArm.begin();
      iter != collector_gpmp2ObstaclePlanarSDFFactorPose2MobileArm.end(); ) {
    delete *iter;
    collector_gpmp2ObstaclePlanarSDFFactorPose2MobileArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm::iterator iter = collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm.begin();
      iter != collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm.end(); ) {
    delete *iter;
    collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms::iterator iter = collector_gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms.begin();
      iter != collector_gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms.end(); ) {
    delete *iter;
    collector_gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms::iterator iter = collector_gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms.begin();
      iter != collector_gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms.end(); ) {
    delete *iter;
    collector_gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstacleSDFFactorPose2MobileBase::iterator iter = collector_gpmp2ObstacleSDFFactorPose2MobileBase.begin();
      iter != collector_gpmp2ObstacleSDFFactorPose2MobileBase.end(); ) {
    delete *iter;
    collector_gpmp2ObstacleSDFFactorPose2MobileBase.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstacleSDFFactorGPPose2MobileBase::iterator iter = collector_gpmp2ObstacleSDFFactorGPPose2MobileBase.begin();
      iter != collector_gpmp2ObstacleSDFFactorGPPose2MobileBase.end(); ) {
    delete *iter;
    collector_gpmp2ObstacleSDFFactorGPPose2MobileBase.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstacleSDFFactorPose2MobileArm::iterator iter = collector_gpmp2ObstacleSDFFactorPose2MobileArm.begin();
      iter != collector_gpmp2ObstacleSDFFactorPose2MobileArm.end(); ) {
    delete *iter;
    collector_gpmp2ObstacleSDFFactorPose2MobileArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstacleSDFFactorGPPose2MobileArm::iterator iter = collector_gpmp2ObstacleSDFFactorGPPose2MobileArm.begin();
      iter != collector_gpmp2ObstacleSDFFactorGPPose2MobileArm.end(); ) {
    delete *iter;
    collector_gpmp2ObstacleSDFFactorGPPose2MobileArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstacleSDFFactorPose2Mobile2Arms::iterator iter = collector_gpmp2ObstacleSDFFactorPose2Mobile2Arms.begin();
      iter != collector_gpmp2ObstacleSDFFactorPose2Mobile2Arms.end(); ) {
    delete *iter;
    collector_gpmp2ObstacleSDFFactorPose2Mobile2Arms.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstacleSDFFactorGPPose2Mobile2Arms::iterator iter = collector_gpmp2ObstacleSDFFactorGPPose2Mobile2Arms.begin();
      iter != collector_gpmp2ObstacleSDFFactorGPPose2Mobile2Arms.end(); ) {
    delete *iter;
    collector_gpmp2ObstacleSDFFactorGPPose2Mobile2Arms.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstacleSDFFactorPose2MobileVetLinArm::iterator iter = collector_gpmp2ObstacleSDFFactorPose2MobileVetLinArm.begin();
      iter != collector_gpmp2ObstacleSDFFactorPose2MobileVetLinArm.end(); ) {
    delete *iter;
    collector_gpmp2ObstacleSDFFactorPose2MobileVetLinArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm::iterator iter = collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm.begin();
      iter != collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm.end(); ) {
    delete *iter;
    collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms::iterator iter = collector_gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms.begin();
      iter != collector_gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms.end(); ) {
    delete *iter;
    collector_gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms::iterator iter = collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms.begin();
      iter != collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms.end(); ) {
    delete *iter;
    collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2SelfCollisionArm::iterator iter = collector_gpmp2SelfCollisionArm.begin();
      iter != collector_gpmp2SelfCollisionArm.end(); ) {
    delete *iter;
    collector_gpmp2SelfCollisionArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2TrajOptimizerSetting::iterator iter = collector_gpmp2TrajOptimizerSetting.begin();
      iter != collector_gpmp2TrajOptimizerSetting.end(); ) {
    delete *iter;
    collector_gpmp2TrajOptimizerSetting.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ISAM2TrajOptimizer2DArm::iterator iter = collector_gpmp2ISAM2TrajOptimizer2DArm.begin();
      iter != collector_gpmp2ISAM2TrajOptimizer2DArm.end(); ) {
    delete *iter;
    collector_gpmp2ISAM2TrajOptimizer2DArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ISAM2TrajOptimizer3DArm::iterator iter = collector_gpmp2ISAM2TrajOptimizer3DArm.begin();
      iter != collector_gpmp2ISAM2TrajOptimizer3DArm.end(); ) {
    delete *iter;
    collector_gpmp2ISAM2TrajOptimizer3DArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ISAM2TrajOptimizerPose2MobileArm2D::iterator iter = collector_gpmp2ISAM2TrajOptimizerPose2MobileArm2D.begin();
      iter != collector_gpmp2ISAM2TrajOptimizerPose2MobileArm2D.end(); ) {
    delete *iter;
    collector_gpmp2ISAM2TrajOptimizerPose2MobileArm2D.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ISAM2TrajOptimizerPose2MobileArm::iterator iter = collector_gpmp2ISAM2TrajOptimizerPose2MobileArm.begin();
      iter != collector_gpmp2ISAM2TrajOptimizerPose2MobileArm.end(); ) {
    delete *iter;
    collector_gpmp2ISAM2TrajOptimizerPose2MobileArm.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms::iterator iter = collector_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms.begin();
      iter != collector_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms.end(); ) {
    delete *iter;
    collector_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gpmp2PriorFactorPose2Vector::iterator iter = collector_gpmp2PriorFactorPose2Vector.begin();
      iter != collector_gpmp2PriorFactorPose2Vector.end(); ) {
    delete *iter;
    collector_gpmp2PriorFactorPose2Vector.erase(iter++);
    anyDeleted = true;
  } }
  if(anyDeleted)
    cout <<
      "WARNING:  Wrap modules with variables in the workspace have been reloaded due to\n"
      "calling destructors, call 'clear all' again if you plan to now recompile a wrap\n"
      "module, so that your recompiled module is used instead of the old one." << endl;
  std::cout.rdbuf(outbuf);
}

void _gpmp2_RTTIRegister() {
  const mxArray *alreadyCreated = mexGetVariablePtr("global", "gtsam_gpmp2_rttiRegistry_created");
  if(!alreadyCreated) {
    std::map<std::string, std::string> types;
    types.insert(std::make_pair(typeid(gpmp2::GaussianProcessPriorLinear).name(), "gpmp2.GaussianProcessPriorLinear"));
    types.insert(std::make_pair(typeid(gpmp2::GaussianProcessPriorPose2).name(), "gpmp2.GaussianProcessPriorPose2"));
    types.insert(std::make_pair(typeid(gpmp2::GaussianProcessPriorPose2Vector).name(), "gpmp2.GaussianProcessPriorPose2Vector"));
    types.insert(std::make_pair(typeid(gpmp2::GoalFactorArm).name(), "gpmp2.GoalFactorArm"));
    types.insert(std::make_pair(typeid(gpmp2::JointLimitFactorVector).name(), "gpmp2.JointLimitFactorVector"));
    types.insert(std::make_pair(typeid(gpmp2::VelocityLimitFactorVector).name(), "gpmp2.VelocityLimitFactorVector"));
    types.insert(std::make_pair(typeid(gpmp2::GaussianPriorWorkspacePositionArm).name(), "gpmp2.GaussianPriorWorkspacePositionArm"));
    types.insert(std::make_pair(typeid(gpmp2::GaussianPriorWorkspaceOrientationArm).name(), "gpmp2.GaussianPriorWorkspaceOrientationArm"));
    types.insert(std::make_pair(typeid(gpmp2::GaussianPriorWorkspacePoseArm).name(), "gpmp2.GaussianPriorWorkspacePoseArm"));
    types.insert(std::make_pair(typeid(gpmp2::VehicleDynamicsFactorPose2).name(), "gpmp2.VehicleDynamicsFactorPose2"));
    types.insert(std::make_pair(typeid(gpmp2::VehicleDynamicsFactorPose2Vector).name(), "gpmp2.VehicleDynamicsFactorPose2Vector"));
    types.insert(std::make_pair(typeid(gpmp2::VehicleDynamicsFactorVector).name(), "gpmp2.VehicleDynamicsFactorVector"));
    types.insert(std::make_pair(typeid(gpmp2::ObstacleSDFFactorArm).name(), "gpmp2.ObstacleSDFFactorArm"));
    types.insert(std::make_pair(typeid(gpmp2::ObstacleSDFFactorGPArm).name(), "gpmp2.ObstacleSDFFactorGPArm"));
    types.insert(std::make_pair(typeid(gpmp2::ObstaclePlanarSDFFactorArm).name(), "gpmp2.ObstaclePlanarSDFFactorArm"));
    types.insert(std::make_pair(typeid(gpmp2::ObstaclePlanarSDFFactorGPArm).name(), "gpmp2.ObstaclePlanarSDFFactorGPArm"));
    types.insert(std::make_pair(typeid(gpmp2::ObstaclePlanarSDFFactorPointRobot).name(), "gpmp2.ObstaclePlanarSDFFactorPointRobot"));
    types.insert(std::make_pair(typeid(gpmp2::ObstaclePlanarSDFFactorGPPointRobot).name(), "gpmp2.ObstaclePlanarSDFFactorGPPointRobot"));
    types.insert(std::make_pair(typeid(gpmp2::ObstaclePlanarSDFFactorPose2MobileBase).name(), "gpmp2.ObstaclePlanarSDFFactorPose2MobileBase"));
    types.insert(std::make_pair(typeid(gpmp2::ObstaclePlanarSDFFactorGPPose2MobileBase).name(), "gpmp2.ObstaclePlanarSDFFactorGPPose2MobileBase"));
    types.insert(std::make_pair(typeid(gpmp2::ObstaclePlanarSDFFactorPose2MobileArm).name(), "gpmp2.ObstaclePlanarSDFFactorPose2MobileArm"));
    types.insert(std::make_pair(typeid(gpmp2::ObstaclePlanarSDFFactorGPPose2MobileArm).name(), "gpmp2.ObstaclePlanarSDFFactorGPPose2MobileArm"));
    types.insert(std::make_pair(typeid(gpmp2::ObstaclePlanarSDFFactorPose2Mobile2Arms).name(), "gpmp2.ObstaclePlanarSDFFactorPose2Mobile2Arms"));
    types.insert(std::make_pair(typeid(gpmp2::ObstaclePlanarSDFFactorGPPose2Mobile2Arms).name(), "gpmp2.ObstaclePlanarSDFFactorGPPose2Mobile2Arms"));
    types.insert(std::make_pair(typeid(gpmp2::ObstacleSDFFactorPose2MobileBase).name(), "gpmp2.ObstacleSDFFactorPose2MobileBase"));
    types.insert(std::make_pair(typeid(gpmp2::ObstacleSDFFactorGPPose2MobileBase).name(), "gpmp2.ObstacleSDFFactorGPPose2MobileBase"));
    types.insert(std::make_pair(typeid(gpmp2::ObstacleSDFFactorPose2MobileArm).name(), "gpmp2.ObstacleSDFFactorPose2MobileArm"));
    types.insert(std::make_pair(typeid(gpmp2::ObstacleSDFFactorGPPose2MobileArm).name(), "gpmp2.ObstacleSDFFactorGPPose2MobileArm"));
    types.insert(std::make_pair(typeid(gpmp2::ObstacleSDFFactorPose2Mobile2Arms).name(), "gpmp2.ObstacleSDFFactorPose2Mobile2Arms"));
    types.insert(std::make_pair(typeid(gpmp2::ObstacleSDFFactorGPPose2Mobile2Arms).name(), "gpmp2.ObstacleSDFFactorGPPose2Mobile2Arms"));
    types.insert(std::make_pair(typeid(gpmp2::ObstacleSDFFactorPose2MobileVetLinArm).name(), "gpmp2.ObstacleSDFFactorPose2MobileVetLinArm"));
    types.insert(std::make_pair(typeid(gpmp2::ObstacleSDFFactorGPPose2MobileVetLinArm).name(), "gpmp2.ObstacleSDFFactorGPPose2MobileVetLinArm"));
    types.insert(std::make_pair(typeid(gpmp2::ObstacleSDFFactorPose2MobileVetLin2Arms).name(), "gpmp2.ObstacleSDFFactorPose2MobileVetLin2Arms"));
    types.insert(std::make_pair(typeid(gpmp2::ObstacleSDFFactorGPPose2MobileVetLin2Arms).name(), "gpmp2.ObstacleSDFFactorGPPose2MobileVetLin2Arms"));
    types.insert(std::make_pair(typeid(gpmp2::SelfCollisionArm).name(), "gpmp2.SelfCollisionArm"));
    types.insert(std::make_pair(typeid(gpmp2::PriorFactorPose2Vector).name(), "gpmp2.PriorFactorPose2Vector"));

    mxArray *registry = mexGetVariable("global", "gtsamwrap_rttiRegistry");
    if(!registry)
      registry = mxCreateStructMatrix(1, 1, 0, NULL);
    typedef std::pair<std::string, std::string> StringPair;
    for(const StringPair& rtti_matlab: types) {
      int fieldId = mxAddField(registry, rtti_matlab.first.c_str());
      if(fieldId < 0)
        mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
      mxArray *matlabName = mxCreateString(rtti_matlab.second.c_str());
      mxSetFieldByNumber(registry, 0, fieldId, matlabName);
    }
    if(mexPutVariable("global", "gtsamwrap_rttiRegistry", registry) != 0)
      mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
    mxDestroyArray(registry);
    
    mxArray *newAlreadyCreated = mxCreateNumericMatrix(0, 0, mxINT8_CLASS, mxREAL);
    if(mexPutVariable("global", "gtsam_gpmp2_rttiRegistry_created", newAlreadyCreated) != 0)
      mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
    mxDestroyArray(newAlreadyCreated);
  }
}

void gpmp2Pose2Vector_collectorInsertAndMakeBase_0(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2Vector> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2Pose2Vector.insert(self);
}

void gpmp2Pose2Vector_constructor_1(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2Vector> Shared;

  Shared *self = new Shared(new gpmp2::Pose2Vector());
  collector_gpmp2Pose2Vector.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Pose2Vector_constructor_2(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2Vector> Shared;

  gtsam::Pose2& pose = *unwrap_shared_ptr< gtsam::Pose2 >(in[0], "ptr_gtsamPose2");
  Vector c = unwrap< Vector >(in[1]);
  Shared *self = new Shared(new gpmp2::Pose2Vector(pose,c));
  collector_gpmp2Pose2Vector.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Pose2Vector_deconstructor_3(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2Vector> Shared;
  checkArguments("delete_gpmp2Pose2Vector",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2Pose2Vector::iterator item;
  item = collector_gpmp2Pose2Vector.find(self);
  if(item != collector_gpmp2Pose2Vector.end()) {
    delete self;
    collector_gpmp2Pose2Vector.erase(item);
  }
}

void gpmp2Pose2Vector_configuration_4(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2Vector> Shared;
  checkArguments("configuration",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2Vector>(in[0], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Vector >(obj->configuration());
}

void gpmp2Pose2Vector_pose_5(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Pose2> SharedPose2;
  typedef boost::shared_ptr<gpmp2::Pose2Vector> Shared;
  checkArguments("pose",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2Vector>(in[0], "ptr_gpmp2Pose2Vector");
  out[0] = wrap_shared_ptr(SharedPose2(new gtsam::Pose2(obj->pose())),"gtsam.Pose2", false);
}

void gpmp2Pose2Vector_print_6(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2Vector> Shared;
  checkArguments("print",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2Vector>(in[0], "ptr_gpmp2Pose2Vector");
  string s = unwrap< string >(in[1]);
  obj->print(s);
}

void gpmp2GaussianProcessPriorLinear_collectorInsertAndMakeBase_7(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianProcessPriorLinear> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2GaussianProcessPriorLinear.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2GaussianProcessPriorLinear_upcastFromVoid_8(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianProcessPriorLinear> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::GaussianProcessPriorLinear>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2GaussianProcessPriorLinear_constructor_9(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianProcessPriorLinear> Shared;

  size_t key1 = unwrap< size_t >(in[0]);
  size_t key2 = unwrap< size_t >(in[1]);
  size_t key3 = unwrap< size_t >(in[2]);
  size_t key4 = unwrap< size_t >(in[3]);
  double delta = unwrap< double >(in[4]);
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[5], "ptr_gtsamnoiseModelBase");
  Shared *self = new Shared(new gpmp2::GaussianProcessPriorLinear(key1,key2,key3,key4,delta,Qc_model));
  collector_gpmp2GaussianProcessPriorLinear.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2GaussianProcessPriorLinear_deconstructor_10(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::GaussianProcessPriorLinear> Shared;
  checkArguments("delete_gpmp2GaussianProcessPriorLinear",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2GaussianProcessPriorLinear::iterator item;
  item = collector_gpmp2GaussianProcessPriorLinear.find(self);
  if(item != collector_gpmp2GaussianProcessPriorLinear.end()) {
    delete self;
    collector_gpmp2GaussianProcessPriorLinear.erase(item);
  }
}

void gpmp2GaussianProcessPriorLinear_evaluateError_11(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::GaussianProcessPriorLinear> Shared;
  checkArguments("evaluateError",nargout,nargin-1,4);
  Shared obj = unwrap_shared_ptr<gpmp2::GaussianProcessPriorLinear>(in[0], "ptr_gpmp2GaussianProcessPriorLinear");
  Vector pose1 = unwrap< Vector >(in[1]);
  Vector vel1 = unwrap< Vector >(in[2]);
  Vector pose2 = unwrap< Vector >(in[3]);
  Vector vel2 = unwrap< Vector >(in[4]);
  out[0] = wrap< Vector >(obj->evaluateError(pose1,vel1,pose2,vel2));
}

void gpmp2GaussianProcessPriorLinear_string_serialize_12(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::GaussianProcessPriorLinear> Shared;
  checkArguments("string_serialize",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::GaussianProcessPriorLinear>(in[0], "ptr_gpmp2GaussianProcessPriorLinear");
  ostringstream out_archive_stream;
  boost::archive::text_oarchive out_archive(out_archive_stream);
  out_archive << *obj;
  out[0] = wrap< string >(out_archive_stream.str());
}
void gpmp2GaussianProcessPriorLinear_string_deserialize_13(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::GaussianProcessPriorLinear> Shared;
  checkArguments("gpmp2GaussianProcessPriorLinear.string_deserialize",nargout,nargin,1);
  string serialized = unwrap< string >(in[0]);
  istringstream in_archive_stream(serialized);
  boost::archive::text_iarchive in_archive(in_archive_stream);
  Shared output(new gpmp2::GaussianProcessPriorLinear());
  in_archive >> *output;
  out[0] = wrap_shared_ptr(output,"gpmp2.GaussianProcessPriorLinear", false);
}
void gpmp2GaussianProcessPriorPose2_collectorInsertAndMakeBase_14(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianProcessPriorPose2> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2GaussianProcessPriorPose2.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2GaussianProcessPriorPose2_upcastFromVoid_15(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianProcessPriorPose2> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::GaussianProcessPriorPose2>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2GaussianProcessPriorPose2_constructor_16(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianProcessPriorPose2> Shared;

  size_t key1 = unwrap< size_t >(in[0]);
  size_t key2 = unwrap< size_t >(in[1]);
  size_t key3 = unwrap< size_t >(in[2]);
  size_t key4 = unwrap< size_t >(in[3]);
  double delta = unwrap< double >(in[4]);
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[5], "ptr_gtsamnoiseModelBase");
  Shared *self = new Shared(new gpmp2::GaussianProcessPriorPose2(key1,key2,key3,key4,delta,Qc_model));
  collector_gpmp2GaussianProcessPriorPose2.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2GaussianProcessPriorPose2_deconstructor_17(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::GaussianProcessPriorPose2> Shared;
  checkArguments("delete_gpmp2GaussianProcessPriorPose2",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2GaussianProcessPriorPose2::iterator item;
  item = collector_gpmp2GaussianProcessPriorPose2.find(self);
  if(item != collector_gpmp2GaussianProcessPriorPose2.end()) {
    delete self;
    collector_gpmp2GaussianProcessPriorPose2.erase(item);
  }
}

void gpmp2GaussianProcessPriorPose2Vector_collectorInsertAndMakeBase_18(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianProcessPriorPose2Vector> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2GaussianProcessPriorPose2Vector.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2GaussianProcessPriorPose2Vector_upcastFromVoid_19(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianProcessPriorPose2Vector> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::GaussianProcessPriorPose2Vector>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2GaussianProcessPriorPose2Vector_constructor_20(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianProcessPriorPose2Vector> Shared;

  size_t key1 = unwrap< size_t >(in[0]);
  size_t key2 = unwrap< size_t >(in[1]);
  size_t key3 = unwrap< size_t >(in[2]);
  size_t key4 = unwrap< size_t >(in[3]);
  double delta = unwrap< double >(in[4]);
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[5], "ptr_gtsamnoiseModelBase");
  Shared *self = new Shared(new gpmp2::GaussianProcessPriorPose2Vector(key1,key2,key3,key4,delta,Qc_model));
  collector_gpmp2GaussianProcessPriorPose2Vector.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2GaussianProcessPriorPose2Vector_deconstructor_21(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::GaussianProcessPriorPose2Vector> Shared;
  checkArguments("delete_gpmp2GaussianProcessPriorPose2Vector",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2GaussianProcessPriorPose2Vector::iterator item;
  item = collector_gpmp2GaussianProcessPriorPose2Vector.find(self);
  if(item != collector_gpmp2GaussianProcessPriorPose2Vector.end()) {
    delete self;
    collector_gpmp2GaussianProcessPriorPose2Vector.erase(item);
  }
}

void gpmp2GaussianProcessInterpolatorLinear_collectorInsertAndMakeBase_22(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianProcessInterpolatorLinear> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2GaussianProcessInterpolatorLinear.insert(self);
}

void gpmp2GaussianProcessInterpolatorLinear_constructor_23(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianProcessInterpolatorLinear> Shared;

  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[0], "ptr_gtsamnoiseModelBase");
  double delta_t = unwrap< double >(in[1]);
  double tau = unwrap< double >(in[2]);
  Shared *self = new Shared(new gpmp2::GaussianProcessInterpolatorLinear(Qc_model,delta_t,tau));
  collector_gpmp2GaussianProcessInterpolatorLinear.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2GaussianProcessInterpolatorLinear_deconstructor_24(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::GaussianProcessInterpolatorLinear> Shared;
  checkArguments("delete_gpmp2GaussianProcessInterpolatorLinear",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2GaussianProcessInterpolatorLinear::iterator item;
  item = collector_gpmp2GaussianProcessInterpolatorLinear.find(self);
  if(item != collector_gpmp2GaussianProcessInterpolatorLinear.end()) {
    delete self;
    collector_gpmp2GaussianProcessInterpolatorLinear.erase(item);
  }
}

void gpmp2GaussianProcessInterpolatorLinear_interpolatePose_25(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::GaussianProcessInterpolatorLinear> Shared;
  checkArguments("interpolatePose",nargout,nargin-1,4);
  Shared obj = unwrap_shared_ptr<gpmp2::GaussianProcessInterpolatorLinear>(in[0], "ptr_gpmp2GaussianProcessInterpolatorLinear");
  Vector pose1 = unwrap< Vector >(in[1]);
  Vector vel1 = unwrap< Vector >(in[2]);
  Vector pose2 = unwrap< Vector >(in[3]);
  Vector vel2 = unwrap< Vector >(in[4]);
  out[0] = wrap< Vector >(obj->interpolatePose(pose1,vel1,pose2,vel2));
}

void gpmp2GaussianProcessInterpolatorLinear_interpolateVelocity_26(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::GaussianProcessInterpolatorLinear> Shared;
  checkArguments("interpolateVelocity",nargout,nargin-1,4);
  Shared obj = unwrap_shared_ptr<gpmp2::GaussianProcessInterpolatorLinear>(in[0], "ptr_gpmp2GaussianProcessInterpolatorLinear");
  Vector pose1 = unwrap< Vector >(in[1]);
  Vector vel1 = unwrap< Vector >(in[2]);
  Vector pose2 = unwrap< Vector >(in[3]);
  Vector vel2 = unwrap< Vector >(in[4]);
  out[0] = wrap< Vector >(obj->interpolateVelocity(pose1,vel1,pose2,vel2));
}

void gpmp2Arm_collectorInsertAndMakeBase_27(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Arm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2Arm.insert(self);
}

void gpmp2Arm_constructor_28(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Arm> Shared;

  size_t dof = unwrap< size_t >(in[0]);
  Vector a = unwrap< Vector >(in[1]);
  Vector alpha = unwrap< Vector >(in[2]);
  Vector d = unwrap< Vector >(in[3]);
  Shared *self = new Shared(new gpmp2::Arm(dof,a,alpha,d));
  collector_gpmp2Arm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Arm_constructor_29(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Arm> Shared;

  size_t dof = unwrap< size_t >(in[0]);
  Vector a = unwrap< Vector >(in[1]);
  Vector alpha = unwrap< Vector >(in[2]);
  Vector d = unwrap< Vector >(in[3]);
  gtsam::Pose3& base_pose = *unwrap_shared_ptr< gtsam::Pose3 >(in[4], "ptr_gtsamPose3");
  Shared *self = new Shared(new gpmp2::Arm(dof,a,alpha,d,base_pose));
  collector_gpmp2Arm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Arm_constructor_30(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Arm> Shared;

  size_t dof = unwrap< size_t >(in[0]);
  Vector a = unwrap< Vector >(in[1]);
  Vector alpha = unwrap< Vector >(in[2]);
  Vector d = unwrap< Vector >(in[3]);
  gtsam::Pose3& base_pose = *unwrap_shared_ptr< gtsam::Pose3 >(in[4], "ptr_gtsamPose3");
  Vector theta_bias = unwrap< Vector >(in[5]);
  Shared *self = new Shared(new gpmp2::Arm(dof,a,alpha,d,base_pose,theta_bias));
  collector_gpmp2Arm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Arm_deconstructor_31(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Arm> Shared;
  checkArguments("delete_gpmp2Arm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2Arm::iterator item;
  item = collector_gpmp2Arm.find(self);
  if(item != collector_gpmp2Arm.end()) {
    delete self;
    collector_gpmp2Arm.erase(item);
  }
}

void gpmp2Arm_a_32(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Arm> Shared;
  checkArguments("a",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Arm>(in[0], "ptr_gpmp2Arm");
  out[0] = wrap< Vector >(obj->a());
}

void gpmp2Arm_alpha_33(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Arm> Shared;
  checkArguments("alpha",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Arm>(in[0], "ptr_gpmp2Arm");
  out[0] = wrap< Vector >(obj->alpha());
}

void gpmp2Arm_base_pose_34(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Pose3> SharedPose3;
  typedef boost::shared_ptr<gpmp2::Arm> Shared;
  checkArguments("base_pose",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Arm>(in[0], "ptr_gpmp2Arm");
  out[0] = wrap_shared_ptr(SharedPose3(new gtsam::Pose3(obj->base_pose())),"gtsam.Pose3", false);
}

void gpmp2Arm_d_35(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Arm> Shared;
  checkArguments("d",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Arm>(in[0], "ptr_gpmp2Arm");
  out[0] = wrap< Vector >(obj->d());
}

void gpmp2Arm_dof_36(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Arm> Shared;
  checkArguments("dof",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Arm>(in[0], "ptr_gpmp2Arm");
  out[0] = wrap< size_t >(obj->dof());
}

void gpmp2Arm_forwardKinematicsPose_37(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Arm> Shared;
  checkArguments("forwardKinematicsPose",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Arm>(in[0], "ptr_gpmp2Arm");
  Vector jp = unwrap< Vector >(in[1]);
  out[0] = wrap< Matrix >(obj->forwardKinematicsPose(jp));
}

void gpmp2Arm_forwardKinematicsPosition_38(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Arm> Shared;
  checkArguments("forwardKinematicsPosition",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Arm>(in[0], "ptr_gpmp2Arm");
  Vector jp = unwrap< Vector >(in[1]);
  out[0] = wrap< Matrix >(obj->forwardKinematicsPosition(jp));
}

void gpmp2Arm_forwardKinematicsVel_39(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Arm> Shared;
  checkArguments("forwardKinematicsVel",nargout,nargin-1,2);
  Shared obj = unwrap_shared_ptr<gpmp2::Arm>(in[0], "ptr_gpmp2Arm");
  Vector jp = unwrap< Vector >(in[1]);
  Vector jv = unwrap< Vector >(in[2]);
  out[0] = wrap< Matrix >(obj->forwardKinematicsVel(jp,jv));
}

void gpmp2Pose2MobileBase_collectorInsertAndMakeBase_40(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileBase> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2Pose2MobileBase.insert(self);
}

void gpmp2Pose2MobileBase_constructor_41(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileBase> Shared;

  Shared *self = new Shared(new gpmp2::Pose2MobileBase());
  collector_gpmp2Pose2MobileBase.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Pose2MobileBase_deconstructor_42(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileBase> Shared;
  checkArguments("delete_gpmp2Pose2MobileBase",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2Pose2MobileBase::iterator item;
  item = collector_gpmp2Pose2MobileBase.find(self);
  if(item != collector_gpmp2Pose2MobileBase.end()) {
    delete self;
    collector_gpmp2Pose2MobileBase.erase(item);
  }
}

void gpmp2Pose2MobileBase_dof_43(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileBase> Shared;
  checkArguments("dof",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileBase>(in[0], "ptr_gpmp2Pose2MobileBase");
  out[0] = wrap< size_t >(obj->dof());
}

void gpmp2Pose2MobileBase_forwardKinematicsPose_44(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileBase> Shared;
  checkArguments("forwardKinematicsPose",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileBase>(in[0], "ptr_gpmp2Pose2MobileBase");
  gtsam::Pose2& jp = *unwrap_shared_ptr< gtsam::Pose2 >(in[1], "ptr_gtsamPose2");
  out[0] = wrap< Matrix >(obj->forwardKinematicsPose(jp));
}

void gpmp2Pose2MobileBase_forwardKinematicsPosition_45(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileBase> Shared;
  checkArguments("forwardKinematicsPosition",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileBase>(in[0], "ptr_gpmp2Pose2MobileBase");
  gtsam::Pose2& jp = *unwrap_shared_ptr< gtsam::Pose2 >(in[1], "ptr_gtsamPose2");
  out[0] = wrap< Matrix >(obj->forwardKinematicsPosition(jp));
}

void gpmp2Pose2MobileBase_forwardKinematicsVel_46(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileBase> Shared;
  checkArguments("forwardKinematicsVel",nargout,nargin-1,2);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileBase>(in[0], "ptr_gpmp2Pose2MobileBase");
  gtsam::Pose2& jp = *unwrap_shared_ptr< gtsam::Pose2 >(in[1], "ptr_gtsamPose2");
  Vector jv = unwrap< Vector >(in[2]);
  out[0] = wrap< Matrix >(obj->forwardKinematicsVel(jp,jv));
}

void gpmp2Pose2MobileBase_nr_links_47(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileBase> Shared;
  checkArguments("nr_links",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileBase>(in[0], "ptr_gpmp2Pose2MobileBase");
  out[0] = wrap< size_t >(obj->nr_links());
}

void gpmp2Pose2MobileArm_collectorInsertAndMakeBase_48(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2Pose2MobileArm.insert(self);
}

void gpmp2Pose2MobileArm_constructor_49(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileArm> Shared;

  gpmp2::Arm& arm = *unwrap_shared_ptr< gpmp2::Arm >(in[0], "ptr_gpmp2Arm");
  Shared *self = new Shared(new gpmp2::Pose2MobileArm(arm));
  collector_gpmp2Pose2MobileArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Pose2MobileArm_constructor_50(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileArm> Shared;

  gpmp2::Arm& arm = *unwrap_shared_ptr< gpmp2::Arm >(in[0], "ptr_gpmp2Arm");
  gtsam::Pose3& base_T_arm = *unwrap_shared_ptr< gtsam::Pose3 >(in[1], "ptr_gtsamPose3");
  Shared *self = new Shared(new gpmp2::Pose2MobileArm(arm,base_T_arm));
  collector_gpmp2Pose2MobileArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Pose2MobileArm_deconstructor_51(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileArm> Shared;
  checkArguments("delete_gpmp2Pose2MobileArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2Pose2MobileArm::iterator item;
  item = collector_gpmp2Pose2MobileArm.find(self);
  if(item != collector_gpmp2Pose2MobileArm.end()) {
    delete self;
    collector_gpmp2Pose2MobileArm.erase(item);
  }
}

void gpmp2Pose2MobileArm_arm_52(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Arm> SharedArm;
  typedef boost::shared_ptr<gpmp2::Pose2MobileArm> Shared;
  checkArguments("arm",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileArm>(in[0], "ptr_gpmp2Pose2MobileArm");
  out[0] = wrap_shared_ptr(SharedArm(new gpmp2::Arm(obj->arm())),"gpmp2.Arm", false);
}

void gpmp2Pose2MobileArm_base_T_arm_53(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Pose3> SharedPose3;
  typedef boost::shared_ptr<gpmp2::Pose2MobileArm> Shared;
  checkArguments("base_T_arm",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileArm>(in[0], "ptr_gpmp2Pose2MobileArm");
  out[0] = wrap_shared_ptr(SharedPose3(new gtsam::Pose3(obj->base_T_arm())),"gtsam.Pose3", false);
}

void gpmp2Pose2MobileArm_dof_54(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileArm> Shared;
  checkArguments("dof",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileArm>(in[0], "ptr_gpmp2Pose2MobileArm");
  out[0] = wrap< size_t >(obj->dof());
}

void gpmp2Pose2MobileArm_forwardKinematicsPose_55(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileArm> Shared;
  checkArguments("forwardKinematicsPose",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileArm>(in[0], "ptr_gpmp2Pose2MobileArm");
  gpmp2::Pose2Vector& jp = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Matrix >(obj->forwardKinematicsPose(jp));
}

void gpmp2Pose2MobileArm_forwardKinematicsPosition_56(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileArm> Shared;
  checkArguments("forwardKinematicsPosition",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileArm>(in[0], "ptr_gpmp2Pose2MobileArm");
  gpmp2::Pose2Vector& jp = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Matrix >(obj->forwardKinematicsPosition(jp));
}

void gpmp2Pose2MobileArm_forwardKinematicsVel_57(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileArm> Shared;
  checkArguments("forwardKinematicsVel",nargout,nargin-1,2);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileArm>(in[0], "ptr_gpmp2Pose2MobileArm");
  gpmp2::Pose2Vector& jp = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  Vector jv = unwrap< Vector >(in[2]);
  out[0] = wrap< Matrix >(obj->forwardKinematicsVel(jp,jv));
}

void gpmp2Pose2MobileArm_nr_links_58(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileArm> Shared;
  checkArguments("nr_links",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileArm>(in[0], "ptr_gpmp2Pose2MobileArm");
  out[0] = wrap< size_t >(obj->nr_links());
}

void gpmp2Pose2Mobile2Arms_collectorInsertAndMakeBase_59(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2Arms> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2Pose2Mobile2Arms.insert(self);
}

void gpmp2Pose2Mobile2Arms_constructor_60(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2Arms> Shared;

  gpmp2::Arm& arm1 = *unwrap_shared_ptr< gpmp2::Arm >(in[0], "ptr_gpmp2Arm");
  gpmp2::Arm& arm2 = *unwrap_shared_ptr< gpmp2::Arm >(in[1], "ptr_gpmp2Arm");
  Shared *self = new Shared(new gpmp2::Pose2Mobile2Arms(arm1,arm2));
  collector_gpmp2Pose2Mobile2Arms.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Pose2Mobile2Arms_constructor_61(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2Arms> Shared;

  gpmp2::Arm& arm1 = *unwrap_shared_ptr< gpmp2::Arm >(in[0], "ptr_gpmp2Arm");
  gpmp2::Arm& arm2 = *unwrap_shared_ptr< gpmp2::Arm >(in[1], "ptr_gpmp2Arm");
  gtsam::Pose3& base_T_arm1 = *unwrap_shared_ptr< gtsam::Pose3 >(in[2], "ptr_gtsamPose3");
  gtsam::Pose3& base_T_arm2 = *unwrap_shared_ptr< gtsam::Pose3 >(in[3], "ptr_gtsamPose3");
  Shared *self = new Shared(new gpmp2::Pose2Mobile2Arms(arm1,arm2,base_T_arm1,base_T_arm2));
  collector_gpmp2Pose2Mobile2Arms.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Pose2Mobile2Arms_deconstructor_62(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2Arms> Shared;
  checkArguments("delete_gpmp2Pose2Mobile2Arms",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2Pose2Mobile2Arms::iterator item;
  item = collector_gpmp2Pose2Mobile2Arms.find(self);
  if(item != collector_gpmp2Pose2Mobile2Arms.end()) {
    delete self;
    collector_gpmp2Pose2Mobile2Arms.erase(item);
  }
}

void gpmp2Pose2Mobile2Arms_arm1_63(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Arm> SharedArm;
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2Arms> Shared;
  checkArguments("arm1",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2Mobile2Arms>(in[0], "ptr_gpmp2Pose2Mobile2Arms");
  out[0] = wrap_shared_ptr(SharedArm(new gpmp2::Arm(obj->arm1())),"gpmp2.Arm", false);
}

void gpmp2Pose2Mobile2Arms_arm2_64(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Arm> SharedArm;
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2Arms> Shared;
  checkArguments("arm2",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2Mobile2Arms>(in[0], "ptr_gpmp2Pose2Mobile2Arms");
  out[0] = wrap_shared_ptr(SharedArm(new gpmp2::Arm(obj->arm2())),"gpmp2.Arm", false);
}

void gpmp2Pose2Mobile2Arms_base_T_arm1_65(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Pose3> SharedPose3;
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2Arms> Shared;
  checkArguments("base_T_arm1",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2Mobile2Arms>(in[0], "ptr_gpmp2Pose2Mobile2Arms");
  out[0] = wrap_shared_ptr(SharedPose3(new gtsam::Pose3(obj->base_T_arm1())),"gtsam.Pose3", false);
}

void gpmp2Pose2Mobile2Arms_base_T_arm2_66(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Pose3> SharedPose3;
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2Arms> Shared;
  checkArguments("base_T_arm2",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2Mobile2Arms>(in[0], "ptr_gpmp2Pose2Mobile2Arms");
  out[0] = wrap_shared_ptr(SharedPose3(new gtsam::Pose3(obj->base_T_arm2())),"gtsam.Pose3", false);
}

void gpmp2Pose2Mobile2Arms_dof_67(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2Arms> Shared;
  checkArguments("dof",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2Mobile2Arms>(in[0], "ptr_gpmp2Pose2Mobile2Arms");
  out[0] = wrap< size_t >(obj->dof());
}

void gpmp2Pose2Mobile2Arms_forwardKinematicsPose_68(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2Arms> Shared;
  checkArguments("forwardKinematicsPose",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2Mobile2Arms>(in[0], "ptr_gpmp2Pose2Mobile2Arms");
  gpmp2::Pose2Vector& jp = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Matrix >(obj->forwardKinematicsPose(jp));
}

void gpmp2Pose2Mobile2Arms_forwardKinematicsPosition_69(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2Arms> Shared;
  checkArguments("forwardKinematicsPosition",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2Mobile2Arms>(in[0], "ptr_gpmp2Pose2Mobile2Arms");
  gpmp2::Pose2Vector& jp = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Matrix >(obj->forwardKinematicsPosition(jp));
}

void gpmp2Pose2Mobile2Arms_forwardKinematicsVel_70(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2Arms> Shared;
  checkArguments("forwardKinematicsVel",nargout,nargin-1,2);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2Mobile2Arms>(in[0], "ptr_gpmp2Pose2Mobile2Arms");
  gpmp2::Pose2Vector& jp = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  Vector jv = unwrap< Vector >(in[2]);
  out[0] = wrap< Matrix >(obj->forwardKinematicsVel(jp,jv));
}

void gpmp2Pose2Mobile2Arms_nr_links_71(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2Arms> Shared;
  checkArguments("nr_links",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2Mobile2Arms>(in[0], "ptr_gpmp2Pose2Mobile2Arms");
  out[0] = wrap< size_t >(obj->nr_links());
}

void gpmp2Pose2MobileVetLinArm_collectorInsertAndMakeBase_72(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2Pose2MobileVetLinArm.insert(self);
}

void gpmp2Pose2MobileVetLinArm_constructor_73(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArm> Shared;

  gpmp2::Arm& arm = *unwrap_shared_ptr< gpmp2::Arm >(in[0], "ptr_gpmp2Arm");
  Shared *self = new Shared(new gpmp2::Pose2MobileVetLinArm(arm));
  collector_gpmp2Pose2MobileVetLinArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Pose2MobileVetLinArm_constructor_74(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArm> Shared;

  gpmp2::Arm& arm = *unwrap_shared_ptr< gpmp2::Arm >(in[0], "ptr_gpmp2Arm");
  gtsam::Pose3& base_T_torso = *unwrap_shared_ptr< gtsam::Pose3 >(in[1], "ptr_gtsamPose3");
  gtsam::Pose3& torso_T_arm = *unwrap_shared_ptr< gtsam::Pose3 >(in[2], "ptr_gtsamPose3");
  bool reverse_linact = unwrap< bool >(in[3]);
  Shared *self = new Shared(new gpmp2::Pose2MobileVetLinArm(arm,base_T_torso,torso_T_arm,reverse_linact));
  collector_gpmp2Pose2MobileVetLinArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Pose2MobileVetLinArm_deconstructor_75(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArm> Shared;
  checkArguments("delete_gpmp2Pose2MobileVetLinArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2Pose2MobileVetLinArm::iterator item;
  item = collector_gpmp2Pose2MobileVetLinArm.find(self);
  if(item != collector_gpmp2Pose2MobileVetLinArm.end()) {
    delete self;
    collector_gpmp2Pose2MobileVetLinArm.erase(item);
  }
}

void gpmp2Pose2MobileVetLinArm_arm_76(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Arm> SharedArm;
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArm> Shared;
  checkArguments("arm",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLinArm>(in[0], "ptr_gpmp2Pose2MobileVetLinArm");
  out[0] = wrap_shared_ptr(SharedArm(new gpmp2::Arm(obj->arm())),"gpmp2.Arm", false);
}

void gpmp2Pose2MobileVetLinArm_base_T_torso_77(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Pose3> SharedPose3;
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArm> Shared;
  checkArguments("base_T_torso",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLinArm>(in[0], "ptr_gpmp2Pose2MobileVetLinArm");
  out[0] = wrap_shared_ptr(SharedPose3(new gtsam::Pose3(obj->base_T_torso())),"gtsam.Pose3", false);
}

void gpmp2Pose2MobileVetLinArm_dof_78(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArm> Shared;
  checkArguments("dof",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLinArm>(in[0], "ptr_gpmp2Pose2MobileVetLinArm");
  out[0] = wrap< size_t >(obj->dof());
}

void gpmp2Pose2MobileVetLinArm_forwardKinematicsPose_79(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArm> Shared;
  checkArguments("forwardKinematicsPose",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLinArm>(in[0], "ptr_gpmp2Pose2MobileVetLinArm");
  gpmp2::Pose2Vector& jp = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Matrix >(obj->forwardKinematicsPose(jp));
}

void gpmp2Pose2MobileVetLinArm_forwardKinematicsPosition_80(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArm> Shared;
  checkArguments("forwardKinematicsPosition",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLinArm>(in[0], "ptr_gpmp2Pose2MobileVetLinArm");
  gpmp2::Pose2Vector& jp = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Matrix >(obj->forwardKinematicsPosition(jp));
}

void gpmp2Pose2MobileVetLinArm_forwardKinematicsVel_81(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArm> Shared;
  checkArguments("forwardKinematicsVel",nargout,nargin-1,2);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLinArm>(in[0], "ptr_gpmp2Pose2MobileVetLinArm");
  gpmp2::Pose2Vector& jp = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  Vector jv = unwrap< Vector >(in[2]);
  out[0] = wrap< Matrix >(obj->forwardKinematicsVel(jp,jv));
}

void gpmp2Pose2MobileVetLinArm_nr_links_82(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArm> Shared;
  checkArguments("nr_links",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLinArm>(in[0], "ptr_gpmp2Pose2MobileVetLinArm");
  out[0] = wrap< size_t >(obj->nr_links());
}

void gpmp2Pose2MobileVetLinArm_reverse_linact_83(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArm> Shared;
  checkArguments("reverse_linact",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLinArm>(in[0], "ptr_gpmp2Pose2MobileVetLinArm");
  out[0] = wrap< bool >(obj->reverse_linact());
}

void gpmp2Pose2MobileVetLinArm_torso_T_arm_84(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Pose3> SharedPose3;
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArm> Shared;
  checkArguments("torso_T_arm",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLinArm>(in[0], "ptr_gpmp2Pose2MobileVetLinArm");
  out[0] = wrap_shared_ptr(SharedPose3(new gtsam::Pose3(obj->torso_T_arm())),"gtsam.Pose3", false);
}

void gpmp2Pose2MobileVetLin2Arms_collectorInsertAndMakeBase_85(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2Arms> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2Pose2MobileVetLin2Arms.insert(self);
}

void gpmp2Pose2MobileVetLin2Arms_constructor_86(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2Arms> Shared;

  gpmp2::Arm& arm1 = *unwrap_shared_ptr< gpmp2::Arm >(in[0], "ptr_gpmp2Arm");
  gpmp2::Arm& arm2 = *unwrap_shared_ptr< gpmp2::Arm >(in[1], "ptr_gpmp2Arm");
  Shared *self = new Shared(new gpmp2::Pose2MobileVetLin2Arms(arm1,arm2));
  collector_gpmp2Pose2MobileVetLin2Arms.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Pose2MobileVetLin2Arms_constructor_87(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2Arms> Shared;

  gpmp2::Arm& arm1 = *unwrap_shared_ptr< gpmp2::Arm >(in[0], "ptr_gpmp2Arm");
  gpmp2::Arm& arm2 = *unwrap_shared_ptr< gpmp2::Arm >(in[1], "ptr_gpmp2Arm");
  gtsam::Pose3& base_T_torso = *unwrap_shared_ptr< gtsam::Pose3 >(in[2], "ptr_gtsamPose3");
  gtsam::Pose3& torso_T_arm1 = *unwrap_shared_ptr< gtsam::Pose3 >(in[3], "ptr_gtsamPose3");
  gtsam::Pose3& torso_T_arm2 = *unwrap_shared_ptr< gtsam::Pose3 >(in[4], "ptr_gtsamPose3");
  bool reverse_linact = unwrap< bool >(in[5]);
  Shared *self = new Shared(new gpmp2::Pose2MobileVetLin2Arms(arm1,arm2,base_T_torso,torso_T_arm1,torso_T_arm2,reverse_linact));
  collector_gpmp2Pose2MobileVetLin2Arms.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Pose2MobileVetLin2Arms_deconstructor_88(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2Arms> Shared;
  checkArguments("delete_gpmp2Pose2MobileVetLin2Arms",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2Pose2MobileVetLin2Arms::iterator item;
  item = collector_gpmp2Pose2MobileVetLin2Arms.find(self);
  if(item != collector_gpmp2Pose2MobileVetLin2Arms.end()) {
    delete self;
    collector_gpmp2Pose2MobileVetLin2Arms.erase(item);
  }
}

void gpmp2Pose2MobileVetLin2Arms_arm1_89(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Arm> SharedArm;
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2Arms> Shared;
  checkArguments("arm1",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLin2Arms>(in[0], "ptr_gpmp2Pose2MobileVetLin2Arms");
  out[0] = wrap_shared_ptr(SharedArm(new gpmp2::Arm(obj->arm1())),"gpmp2.Arm", false);
}

void gpmp2Pose2MobileVetLin2Arms_arm2_90(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Arm> SharedArm;
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2Arms> Shared;
  checkArguments("arm2",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLin2Arms>(in[0], "ptr_gpmp2Pose2MobileVetLin2Arms");
  out[0] = wrap_shared_ptr(SharedArm(new gpmp2::Arm(obj->arm2())),"gpmp2.Arm", false);
}

void gpmp2Pose2MobileVetLin2Arms_base_T_torso_91(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Pose3> SharedPose3;
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2Arms> Shared;
  checkArguments("base_T_torso",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLin2Arms>(in[0], "ptr_gpmp2Pose2MobileVetLin2Arms");
  out[0] = wrap_shared_ptr(SharedPose3(new gtsam::Pose3(obj->base_T_torso())),"gtsam.Pose3", false);
}

void gpmp2Pose2MobileVetLin2Arms_dof_92(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2Arms> Shared;
  checkArguments("dof",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLin2Arms>(in[0], "ptr_gpmp2Pose2MobileVetLin2Arms");
  out[0] = wrap< size_t >(obj->dof());
}

void gpmp2Pose2MobileVetLin2Arms_forwardKinematicsPose_93(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2Arms> Shared;
  checkArguments("forwardKinematicsPose",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLin2Arms>(in[0], "ptr_gpmp2Pose2MobileVetLin2Arms");
  gpmp2::Pose2Vector& jp = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Matrix >(obj->forwardKinematicsPose(jp));
}

void gpmp2Pose2MobileVetLin2Arms_forwardKinematicsPosition_94(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2Arms> Shared;
  checkArguments("forwardKinematicsPosition",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLin2Arms>(in[0], "ptr_gpmp2Pose2MobileVetLin2Arms");
  gpmp2::Pose2Vector& jp = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Matrix >(obj->forwardKinematicsPosition(jp));
}

void gpmp2Pose2MobileVetLin2Arms_forwardKinematicsVel_95(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2Arms> Shared;
  checkArguments("forwardKinematicsVel",nargout,nargin-1,2);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLin2Arms>(in[0], "ptr_gpmp2Pose2MobileVetLin2Arms");
  gpmp2::Pose2Vector& jp = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  Vector jv = unwrap< Vector >(in[2]);
  out[0] = wrap< Matrix >(obj->forwardKinematicsVel(jp,jv));
}

void gpmp2Pose2MobileVetLin2Arms_nr_links_96(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2Arms> Shared;
  checkArguments("nr_links",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLin2Arms>(in[0], "ptr_gpmp2Pose2MobileVetLin2Arms");
  out[0] = wrap< size_t >(obj->nr_links());
}

void gpmp2Pose2MobileVetLin2Arms_reverse_linact_97(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2Arms> Shared;
  checkArguments("reverse_linact",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLin2Arms>(in[0], "ptr_gpmp2Pose2MobileVetLin2Arms");
  out[0] = wrap< bool >(obj->reverse_linact());
}

void gpmp2Pose2MobileVetLin2Arms_torso_T_arm1_98(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Pose3> SharedPose3;
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2Arms> Shared;
  checkArguments("torso_T_arm1",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLin2Arms>(in[0], "ptr_gpmp2Pose2MobileVetLin2Arms");
  out[0] = wrap_shared_ptr(SharedPose3(new gtsam::Pose3(obj->torso_T_arm1())),"gtsam.Pose3", false);
}

void gpmp2Pose2MobileVetLin2Arms_torso_T_arm2_99(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Pose3> SharedPose3;
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2Arms> Shared;
  checkArguments("torso_T_arm2",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLin2Arms>(in[0], "ptr_gpmp2Pose2MobileVetLin2Arms");
  out[0] = wrap_shared_ptr(SharedPose3(new gtsam::Pose3(obj->torso_T_arm2())),"gtsam.Pose3", false);
}

void gpmp2PointRobot_collectorInsertAndMakeBase_100(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::PointRobot> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2PointRobot.insert(self);
}

void gpmp2PointRobot_constructor_101(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::PointRobot> Shared;

  size_t dof = unwrap< size_t >(in[0]);
  size_t nr_links = unwrap< size_t >(in[1]);
  Shared *self = new Shared(new gpmp2::PointRobot(dof,nr_links));
  collector_gpmp2PointRobot.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2PointRobot_deconstructor_102(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::PointRobot> Shared;
  checkArguments("delete_gpmp2PointRobot",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2PointRobot::iterator item;
  item = collector_gpmp2PointRobot.find(self);
  if(item != collector_gpmp2PointRobot.end()) {
    delete self;
    collector_gpmp2PointRobot.erase(item);
  }
}

void gpmp2PointRobot_dof_103(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::PointRobot> Shared;
  checkArguments("dof",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::PointRobot>(in[0], "ptr_gpmp2PointRobot");
  out[0] = wrap< size_t >(obj->dof());
}

void gpmp2PointRobot_forwardKinematicsPose_104(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::PointRobot> Shared;
  checkArguments("forwardKinematicsPose",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::PointRobot>(in[0], "ptr_gpmp2PointRobot");
  Vector jp = unwrap< Vector >(in[1]);
  out[0] = wrap< Matrix >(obj->forwardKinematicsPose(jp));
}

void gpmp2PointRobot_forwardKinematicsPosition_105(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::PointRobot> Shared;
  checkArguments("forwardKinematicsPosition",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::PointRobot>(in[0], "ptr_gpmp2PointRobot");
  Vector jp = unwrap< Vector >(in[1]);
  out[0] = wrap< Matrix >(obj->forwardKinematicsPosition(jp));
}

void gpmp2PointRobot_forwardKinematicsVel_106(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::PointRobot> Shared;
  checkArguments("forwardKinematicsVel",nargout,nargin-1,2);
  Shared obj = unwrap_shared_ptr<gpmp2::PointRobot>(in[0], "ptr_gpmp2PointRobot");
  Vector jp = unwrap< Vector >(in[1]);
  Vector jv = unwrap< Vector >(in[2]);
  out[0] = wrap< Matrix >(obj->forwardKinematicsVel(jp,jv));
}

void gpmp2PointRobot_nr_links_107(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::PointRobot> Shared;
  checkArguments("nr_links",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::PointRobot>(in[0], "ptr_gpmp2PointRobot");
  out[0] = wrap< size_t >(obj->nr_links());
}

void gpmp2BodySphere_collectorInsertAndMakeBase_108(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::BodySphere> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2BodySphere.insert(self);
}

void gpmp2BodySphere_constructor_109(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::BodySphere> Shared;

  size_t id = unwrap< size_t >(in[0]);
  double r = unwrap< double >(in[1]);
  gtsam::Point3& c = *unwrap_shared_ptr< gtsam::Point3 >(in[2], "ptr_gtsamPoint3");
  Shared *self = new Shared(new gpmp2::BodySphere(id,r,c));
  collector_gpmp2BodySphere.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2BodySphere_deconstructor_110(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::BodySphere> Shared;
  checkArguments("delete_gpmp2BodySphere",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2BodySphere::iterator item;
  item = collector_gpmp2BodySphere.find(self);
  if(item != collector_gpmp2BodySphere.end()) {
    delete self;
    collector_gpmp2BodySphere.erase(item);
  }
}

void gpmp2BodySphereVector_collectorInsertAndMakeBase_111(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::BodySphereVector> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2BodySphereVector.insert(self);
}

void gpmp2BodySphereVector_constructor_112(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::BodySphereVector> Shared;

  Shared *self = new Shared(new gpmp2::BodySphereVector());
  collector_gpmp2BodySphereVector.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2BodySphereVector_deconstructor_113(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::BodySphereVector> Shared;
  checkArguments("delete_gpmp2BodySphereVector",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2BodySphereVector::iterator item;
  item = collector_gpmp2BodySphereVector.find(self);
  if(item != collector_gpmp2BodySphereVector.end()) {
    delete self;
    collector_gpmp2BodySphereVector.erase(item);
  }
}

void gpmp2BodySphereVector_push_back_114(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::BodySphereVector> Shared;
  checkArguments("push_back",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::BodySphereVector>(in[0], "ptr_gpmp2BodySphereVector");
  gpmp2::BodySphere& sphere = *unwrap_shared_ptr< gpmp2::BodySphere >(in[1], "ptr_gpmp2BodySphere");
  obj->push_back(sphere);
}

void gpmp2ArmModel_collectorInsertAndMakeBase_115(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ArmModel> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ArmModel.insert(self);
}

void gpmp2ArmModel_constructor_116(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ArmModel> Shared;

  gpmp2::Arm& arm = *unwrap_shared_ptr< gpmp2::Arm >(in[0], "ptr_gpmp2Arm");
  gpmp2::BodySphereVector& spheres = *unwrap_shared_ptr< gpmp2::BodySphereVector >(in[1], "ptr_gpmp2BodySphereVector");
  Shared *self = new Shared(new gpmp2::ArmModel(arm,spheres));
  collector_gpmp2ArmModel.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2ArmModel_deconstructor_117(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ArmModel> Shared;
  checkArguments("delete_gpmp2ArmModel",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ArmModel::iterator item;
  item = collector_gpmp2ArmModel.find(self);
  if(item != collector_gpmp2ArmModel.end()) {
    delete self;
    collector_gpmp2ArmModel.erase(item);
  }
}

void gpmp2ArmModel_dof_118(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ArmModel> Shared;
  checkArguments("dof",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ArmModel>(in[0], "ptr_gpmp2ArmModel");
  out[0] = wrap< size_t >(obj->dof());
}

void gpmp2ArmModel_fk_model_119(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Arm> SharedArm;
  typedef boost::shared_ptr<gpmp2::ArmModel> Shared;
  checkArguments("fk_model",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ArmModel>(in[0], "ptr_gpmp2ArmModel");
  out[0] = wrap_shared_ptr(SharedArm(new gpmp2::Arm(obj->fk_model())),"gpmp2.Arm", false);
}

void gpmp2ArmModel_nr_body_spheres_120(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ArmModel> Shared;
  checkArguments("nr_body_spheres",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ArmModel>(in[0], "ptr_gpmp2ArmModel");
  out[0] = wrap< size_t >(obj->nr_body_spheres());
}

void gpmp2ArmModel_sphereCentersMat_121(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ArmModel> Shared;
  checkArguments("sphereCentersMat",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ArmModel>(in[0], "ptr_gpmp2ArmModel");
  Vector conf = unwrap< Vector >(in[1]);
  out[0] = wrap< Matrix >(obj->sphereCentersMat(conf));
}

void gpmp2ArmModel_sphere_radius_122(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ArmModel> Shared;
  checkArguments("sphere_radius",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ArmModel>(in[0], "ptr_gpmp2ArmModel");
  size_t i = unwrap< size_t >(in[1]);
  out[0] = wrap< double >(obj->sphere_radius(i));
}

void gpmp2Pose2MobileBaseModel_collectorInsertAndMakeBase_123(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileBaseModel> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2Pose2MobileBaseModel.insert(self);
}

void gpmp2Pose2MobileBaseModel_constructor_124(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileBaseModel> Shared;

  gpmp2::Pose2MobileBase& r = *unwrap_shared_ptr< gpmp2::Pose2MobileBase >(in[0], "ptr_gpmp2Pose2MobileBase");
  gpmp2::BodySphereVector& spheres = *unwrap_shared_ptr< gpmp2::BodySphereVector >(in[1], "ptr_gpmp2BodySphereVector");
  Shared *self = new Shared(new gpmp2::Pose2MobileBaseModel(r,spheres));
  collector_gpmp2Pose2MobileBaseModel.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Pose2MobileBaseModel_deconstructor_125(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileBaseModel> Shared;
  checkArguments("delete_gpmp2Pose2MobileBaseModel",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2Pose2MobileBaseModel::iterator item;
  item = collector_gpmp2Pose2MobileBaseModel.find(self);
  if(item != collector_gpmp2Pose2MobileBaseModel.end()) {
    delete self;
    collector_gpmp2Pose2MobileBaseModel.erase(item);
  }
}

void gpmp2Pose2MobileBaseModel_dof_126(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileBaseModel> Shared;
  checkArguments("dof",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileBaseModel>(in[0], "ptr_gpmp2Pose2MobileBaseModel");
  out[0] = wrap< size_t >(obj->dof());
}

void gpmp2Pose2MobileBaseModel_fk_model_127(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileBase> SharedPose2MobileBase;
  typedef boost::shared_ptr<gpmp2::Pose2MobileBaseModel> Shared;
  checkArguments("fk_model",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileBaseModel>(in[0], "ptr_gpmp2Pose2MobileBaseModel");
  out[0] = wrap_shared_ptr(SharedPose2MobileBase(new gpmp2::Pose2MobileBase(obj->fk_model())),"gpmp2.Pose2MobileBase", false);
}

void gpmp2Pose2MobileBaseModel_nr_body_spheres_128(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileBaseModel> Shared;
  checkArguments("nr_body_spheres",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileBaseModel>(in[0], "ptr_gpmp2Pose2MobileBaseModel");
  out[0] = wrap< size_t >(obj->nr_body_spheres());
}

void gpmp2Pose2MobileBaseModel_sphereCentersMat_129(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileBaseModel> Shared;
  checkArguments("sphereCentersMat",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileBaseModel>(in[0], "ptr_gpmp2Pose2MobileBaseModel");
  gtsam::Pose2& conf = *unwrap_shared_ptr< gtsam::Pose2 >(in[1], "ptr_gtsamPose2");
  out[0] = wrap< Matrix >(obj->sphereCentersMat(conf));
}

void gpmp2Pose2MobileBaseModel_sphere_radius_130(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileBaseModel> Shared;
  checkArguments("sphere_radius",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileBaseModel>(in[0], "ptr_gpmp2Pose2MobileBaseModel");
  size_t i = unwrap< size_t >(in[1]);
  out[0] = wrap< double >(obj->sphere_radius(i));
}

void gpmp2Pose2MobileArmModel_collectorInsertAndMakeBase_131(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileArmModel> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2Pose2MobileArmModel.insert(self);
}

void gpmp2Pose2MobileArmModel_constructor_132(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileArmModel> Shared;

  gpmp2::Pose2MobileArm& r = *unwrap_shared_ptr< gpmp2::Pose2MobileArm >(in[0], "ptr_gpmp2Pose2MobileArm");
  gpmp2::BodySphereVector& spheres = *unwrap_shared_ptr< gpmp2::BodySphereVector >(in[1], "ptr_gpmp2BodySphereVector");
  Shared *self = new Shared(new gpmp2::Pose2MobileArmModel(r,spheres));
  collector_gpmp2Pose2MobileArmModel.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Pose2MobileArmModel_deconstructor_133(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileArmModel> Shared;
  checkArguments("delete_gpmp2Pose2MobileArmModel",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2Pose2MobileArmModel::iterator item;
  item = collector_gpmp2Pose2MobileArmModel.find(self);
  if(item != collector_gpmp2Pose2MobileArmModel.end()) {
    delete self;
    collector_gpmp2Pose2MobileArmModel.erase(item);
  }
}

void gpmp2Pose2MobileArmModel_dof_134(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileArmModel> Shared;
  checkArguments("dof",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileArmModel>(in[0], "ptr_gpmp2Pose2MobileArmModel");
  out[0] = wrap< size_t >(obj->dof());
}

void gpmp2Pose2MobileArmModel_fk_model_135(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileArm> SharedPose2MobileArm;
  typedef boost::shared_ptr<gpmp2::Pose2MobileArmModel> Shared;
  checkArguments("fk_model",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileArmModel>(in[0], "ptr_gpmp2Pose2MobileArmModel");
  out[0] = wrap_shared_ptr(SharedPose2MobileArm(new gpmp2::Pose2MobileArm(obj->fk_model())),"gpmp2.Pose2MobileArm", false);
}

void gpmp2Pose2MobileArmModel_nr_body_spheres_136(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileArmModel> Shared;
  checkArguments("nr_body_spheres",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileArmModel>(in[0], "ptr_gpmp2Pose2MobileArmModel");
  out[0] = wrap< size_t >(obj->nr_body_spheres());
}

void gpmp2Pose2MobileArmModel_sphereCentersMat_137(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileArmModel> Shared;
  checkArguments("sphereCentersMat",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileArmModel>(in[0], "ptr_gpmp2Pose2MobileArmModel");
  gpmp2::Pose2Vector& conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Matrix >(obj->sphereCentersMat(conf));
}

void gpmp2Pose2MobileArmModel_sphere_radius_138(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileArmModel> Shared;
  checkArguments("sphere_radius",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileArmModel>(in[0], "ptr_gpmp2Pose2MobileArmModel");
  size_t i = unwrap< size_t >(in[1]);
  out[0] = wrap< double >(obj->sphere_radius(i));
}

void gpmp2Pose2Mobile2ArmsModel_collectorInsertAndMakeBase_139(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2ArmsModel> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2Pose2Mobile2ArmsModel.insert(self);
}

void gpmp2Pose2Mobile2ArmsModel_constructor_140(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2ArmsModel> Shared;

  gpmp2::Pose2Mobile2Arms& r = *unwrap_shared_ptr< gpmp2::Pose2Mobile2Arms >(in[0], "ptr_gpmp2Pose2Mobile2Arms");
  gpmp2::BodySphereVector& spheres = *unwrap_shared_ptr< gpmp2::BodySphereVector >(in[1], "ptr_gpmp2BodySphereVector");
  Shared *self = new Shared(new gpmp2::Pose2Mobile2ArmsModel(r,spheres));
  collector_gpmp2Pose2Mobile2ArmsModel.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Pose2Mobile2ArmsModel_deconstructor_141(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2ArmsModel> Shared;
  checkArguments("delete_gpmp2Pose2Mobile2ArmsModel",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2Pose2Mobile2ArmsModel::iterator item;
  item = collector_gpmp2Pose2Mobile2ArmsModel.find(self);
  if(item != collector_gpmp2Pose2Mobile2ArmsModel.end()) {
    delete self;
    collector_gpmp2Pose2Mobile2ArmsModel.erase(item);
  }
}

void gpmp2Pose2Mobile2ArmsModel_dof_142(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2ArmsModel> Shared;
  checkArguments("dof",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2Mobile2ArmsModel>(in[0], "ptr_gpmp2Pose2Mobile2ArmsModel");
  out[0] = wrap< size_t >(obj->dof());
}

void gpmp2Pose2Mobile2ArmsModel_fk_model_143(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2Arms> SharedPose2Mobile2Arms;
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2ArmsModel> Shared;
  checkArguments("fk_model",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2Mobile2ArmsModel>(in[0], "ptr_gpmp2Pose2Mobile2ArmsModel");
  out[0] = wrap_shared_ptr(SharedPose2Mobile2Arms(new gpmp2::Pose2Mobile2Arms(obj->fk_model())),"gpmp2.Pose2Mobile2Arms", false);
}

void gpmp2Pose2Mobile2ArmsModel_nr_body_spheres_144(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2ArmsModel> Shared;
  checkArguments("nr_body_spheres",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2Mobile2ArmsModel>(in[0], "ptr_gpmp2Pose2Mobile2ArmsModel");
  out[0] = wrap< size_t >(obj->nr_body_spheres());
}

void gpmp2Pose2Mobile2ArmsModel_sphereCentersMat_145(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2ArmsModel> Shared;
  checkArguments("sphereCentersMat",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2Mobile2ArmsModel>(in[0], "ptr_gpmp2Pose2Mobile2ArmsModel");
  gpmp2::Pose2Vector& conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Matrix >(obj->sphereCentersMat(conf));
}

void gpmp2Pose2Mobile2ArmsModel_sphere_radius_146(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2Mobile2ArmsModel> Shared;
  checkArguments("sphere_radius",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2Mobile2ArmsModel>(in[0], "ptr_gpmp2Pose2Mobile2ArmsModel");
  size_t i = unwrap< size_t >(in[1]);
  out[0] = wrap< double >(obj->sphere_radius(i));
}

void gpmp2Pose2MobileVetLinArmModel_collectorInsertAndMakeBase_147(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArmModel> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2Pose2MobileVetLinArmModel.insert(self);
}

void gpmp2Pose2MobileVetLinArmModel_constructor_148(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArmModel> Shared;

  gpmp2::Pose2MobileVetLinArm& r = *unwrap_shared_ptr< gpmp2::Pose2MobileVetLinArm >(in[0], "ptr_gpmp2Pose2MobileVetLinArm");
  gpmp2::BodySphereVector& spheres = *unwrap_shared_ptr< gpmp2::BodySphereVector >(in[1], "ptr_gpmp2BodySphereVector");
  Shared *self = new Shared(new gpmp2::Pose2MobileVetLinArmModel(r,spheres));
  collector_gpmp2Pose2MobileVetLinArmModel.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Pose2MobileVetLinArmModel_deconstructor_149(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArmModel> Shared;
  checkArguments("delete_gpmp2Pose2MobileVetLinArmModel",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2Pose2MobileVetLinArmModel::iterator item;
  item = collector_gpmp2Pose2MobileVetLinArmModel.find(self);
  if(item != collector_gpmp2Pose2MobileVetLinArmModel.end()) {
    delete self;
    collector_gpmp2Pose2MobileVetLinArmModel.erase(item);
  }
}

void gpmp2Pose2MobileVetLinArmModel_dof_150(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArmModel> Shared;
  checkArguments("dof",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLinArmModel>(in[0], "ptr_gpmp2Pose2MobileVetLinArmModel");
  out[0] = wrap< size_t >(obj->dof());
}

void gpmp2Pose2MobileVetLinArmModel_fk_model_151(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArm> SharedPose2MobileVetLinArm;
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArmModel> Shared;
  checkArguments("fk_model",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLinArmModel>(in[0], "ptr_gpmp2Pose2MobileVetLinArmModel");
  out[0] = wrap_shared_ptr(SharedPose2MobileVetLinArm(new gpmp2::Pose2MobileVetLinArm(obj->fk_model())),"gpmp2.Pose2MobileVetLinArm", false);
}

void gpmp2Pose2MobileVetLinArmModel_nr_body_spheres_152(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArmModel> Shared;
  checkArguments("nr_body_spheres",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLinArmModel>(in[0], "ptr_gpmp2Pose2MobileVetLinArmModel");
  out[0] = wrap< size_t >(obj->nr_body_spheres());
}

void gpmp2Pose2MobileVetLinArmModel_sphereCentersMat_153(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArmModel> Shared;
  checkArguments("sphereCentersMat",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLinArmModel>(in[0], "ptr_gpmp2Pose2MobileVetLinArmModel");
  gpmp2::Pose2Vector& conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Matrix >(obj->sphereCentersMat(conf));
}

void gpmp2Pose2MobileVetLinArmModel_sphere_radius_154(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLinArmModel> Shared;
  checkArguments("sphere_radius",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLinArmModel>(in[0], "ptr_gpmp2Pose2MobileVetLinArmModel");
  size_t i = unwrap< size_t >(in[1]);
  out[0] = wrap< double >(obj->sphere_radius(i));
}

void gpmp2Pose2MobileVetLin2ArmsModel_collectorInsertAndMakeBase_155(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2ArmsModel> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2Pose2MobileVetLin2ArmsModel.insert(self);
}

void gpmp2Pose2MobileVetLin2ArmsModel_constructor_156(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2ArmsModel> Shared;

  gpmp2::Pose2MobileVetLin2Arms& r = *unwrap_shared_ptr< gpmp2::Pose2MobileVetLin2Arms >(in[0], "ptr_gpmp2Pose2MobileVetLin2Arms");
  gpmp2::BodySphereVector& spheres = *unwrap_shared_ptr< gpmp2::BodySphereVector >(in[1], "ptr_gpmp2BodySphereVector");
  Shared *self = new Shared(new gpmp2::Pose2MobileVetLin2ArmsModel(r,spheres));
  collector_gpmp2Pose2MobileVetLin2ArmsModel.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2Pose2MobileVetLin2ArmsModel_deconstructor_157(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2ArmsModel> Shared;
  checkArguments("delete_gpmp2Pose2MobileVetLin2ArmsModel",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2Pose2MobileVetLin2ArmsModel::iterator item;
  item = collector_gpmp2Pose2MobileVetLin2ArmsModel.find(self);
  if(item != collector_gpmp2Pose2MobileVetLin2ArmsModel.end()) {
    delete self;
    collector_gpmp2Pose2MobileVetLin2ArmsModel.erase(item);
  }
}

void gpmp2Pose2MobileVetLin2ArmsModel_dof_158(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2ArmsModel> Shared;
  checkArguments("dof",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLin2ArmsModel>(in[0], "ptr_gpmp2Pose2MobileVetLin2ArmsModel");
  out[0] = wrap< size_t >(obj->dof());
}

void gpmp2Pose2MobileVetLin2ArmsModel_fk_model_159(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2Arms> SharedPose2MobileVetLin2Arms;
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2ArmsModel> Shared;
  checkArguments("fk_model",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLin2ArmsModel>(in[0], "ptr_gpmp2Pose2MobileVetLin2ArmsModel");
  out[0] = wrap_shared_ptr(SharedPose2MobileVetLin2Arms(new gpmp2::Pose2MobileVetLin2Arms(obj->fk_model())),"gpmp2.Pose2MobileVetLin2Arms", false);
}

void gpmp2Pose2MobileVetLin2ArmsModel_nr_body_spheres_160(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2ArmsModel> Shared;
  checkArguments("nr_body_spheres",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLin2ArmsModel>(in[0], "ptr_gpmp2Pose2MobileVetLin2ArmsModel");
  out[0] = wrap< size_t >(obj->nr_body_spheres());
}

void gpmp2Pose2MobileVetLin2ArmsModel_sphereCentersMat_161(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2ArmsModel> Shared;
  checkArguments("sphereCentersMat",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLin2ArmsModel>(in[0], "ptr_gpmp2Pose2MobileVetLin2ArmsModel");
  gpmp2::Pose2Vector& conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Matrix >(obj->sphereCentersMat(conf));
}

void gpmp2Pose2MobileVetLin2ArmsModel_sphere_radius_162(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2MobileVetLin2ArmsModel> Shared;
  checkArguments("sphere_radius",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::Pose2MobileVetLin2ArmsModel>(in[0], "ptr_gpmp2Pose2MobileVetLin2ArmsModel");
  size_t i = unwrap< size_t >(in[1]);
  out[0] = wrap< double >(obj->sphere_radius(i));
}

void gpmp2PointRobotModel_collectorInsertAndMakeBase_163(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::PointRobotModel> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2PointRobotModel.insert(self);
}

void gpmp2PointRobotModel_constructor_164(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::PointRobotModel> Shared;

  gpmp2::PointRobot& pR = *unwrap_shared_ptr< gpmp2::PointRobot >(in[0], "ptr_gpmp2PointRobot");
  gpmp2::BodySphereVector& spheres = *unwrap_shared_ptr< gpmp2::BodySphereVector >(in[1], "ptr_gpmp2BodySphereVector");
  Shared *self = new Shared(new gpmp2::PointRobotModel(pR,spheres));
  collector_gpmp2PointRobotModel.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2PointRobotModel_deconstructor_165(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::PointRobotModel> Shared;
  checkArguments("delete_gpmp2PointRobotModel",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2PointRobotModel::iterator item;
  item = collector_gpmp2PointRobotModel.find(self);
  if(item != collector_gpmp2PointRobotModel.end()) {
    delete self;
    collector_gpmp2PointRobotModel.erase(item);
  }
}

void gpmp2PointRobotModel_dof_166(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::PointRobotModel> Shared;
  checkArguments("dof",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::PointRobotModel>(in[0], "ptr_gpmp2PointRobotModel");
  out[0] = wrap< size_t >(obj->dof());
}

void gpmp2PointRobotModel_fk_model_167(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::PointRobot> SharedPointRobot;
  typedef boost::shared_ptr<gpmp2::PointRobotModel> Shared;
  checkArguments("fk_model",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::PointRobotModel>(in[0], "ptr_gpmp2PointRobotModel");
  out[0] = wrap_shared_ptr(SharedPointRobot(new gpmp2::PointRobot(obj->fk_model())),"gpmp2.PointRobot", false);
}

void gpmp2PointRobotModel_nr_body_spheres_168(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::PointRobotModel> Shared;
  checkArguments("nr_body_spheres",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::PointRobotModel>(in[0], "ptr_gpmp2PointRobotModel");
  out[0] = wrap< size_t >(obj->nr_body_spheres());
}

void gpmp2PointRobotModel_sphereCentersMat_169(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::PointRobotModel> Shared;
  checkArguments("sphereCentersMat",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::PointRobotModel>(in[0], "ptr_gpmp2PointRobotModel");
  Vector conf = unwrap< Vector >(in[1]);
  out[0] = wrap< Matrix >(obj->sphereCentersMat(conf));
}

void gpmp2PointRobotModel_sphere_radius_170(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::PointRobotModel> Shared;
  checkArguments("sphere_radius",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::PointRobotModel>(in[0], "ptr_gpmp2PointRobotModel");
  size_t i = unwrap< size_t >(in[1]);
  out[0] = wrap< double >(obj->sphere_radius(i));
}

void gpmp2GoalFactorArm_collectorInsertAndMakeBase_171(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GoalFactorArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2GoalFactorArm.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2GoalFactorArm_upcastFromVoid_172(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GoalFactorArm> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::GoalFactorArm>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2GoalFactorArm_constructor_173(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GoalFactorArm> Shared;

  size_t poseKey = unwrap< size_t >(in[0]);
  boost::shared_ptr<gtsam::noiseModel::Base> cost_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[1], "ptr_gtsamnoiseModelBase");
  gpmp2::Arm& arm = *unwrap_shared_ptr< gpmp2::Arm >(in[2], "ptr_gpmp2Arm");
  gtsam::Point3& dest_point = *unwrap_shared_ptr< gtsam::Point3 >(in[3], "ptr_gtsamPoint3");
  Shared *self = new Shared(new gpmp2::GoalFactorArm(poseKey,cost_model,arm,dest_point));
  collector_gpmp2GoalFactorArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2GoalFactorArm_deconstructor_174(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::GoalFactorArm> Shared;
  checkArguments("delete_gpmp2GoalFactorArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2GoalFactorArm::iterator item;
  item = collector_gpmp2GoalFactorArm.find(self);
  if(item != collector_gpmp2GoalFactorArm.end()) {
    delete self;
    collector_gpmp2GoalFactorArm.erase(item);
  }
}

void gpmp2JointLimitFactorVector_collectorInsertAndMakeBase_175(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::JointLimitFactorVector> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2JointLimitFactorVector.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2JointLimitFactorVector_upcastFromVoid_176(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::JointLimitFactorVector> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::JointLimitFactorVector>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2JointLimitFactorVector_constructor_177(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::JointLimitFactorVector> Shared;

  size_t key = unwrap< size_t >(in[0]);
  boost::shared_ptr<gtsam::noiseModel::Base> cost_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[1], "ptr_gtsamnoiseModelBase");
  Vector down_limit = unwrap< Vector >(in[2]);
  Vector up_limit = unwrap< Vector >(in[3]);
  Vector limit_thresh = unwrap< Vector >(in[4]);
  Shared *self = new Shared(new gpmp2::JointLimitFactorVector(key,cost_model,down_limit,up_limit,limit_thresh));
  collector_gpmp2JointLimitFactorVector.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2JointLimitFactorVector_deconstructor_178(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::JointLimitFactorVector> Shared;
  checkArguments("delete_gpmp2JointLimitFactorVector",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2JointLimitFactorVector::iterator item;
  item = collector_gpmp2JointLimitFactorVector.find(self);
  if(item != collector_gpmp2JointLimitFactorVector.end()) {
    delete self;
    collector_gpmp2JointLimitFactorVector.erase(item);
  }
}

void gpmp2VelocityLimitFactorVector_collectorInsertAndMakeBase_179(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::VelocityLimitFactorVector> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2VelocityLimitFactorVector.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2VelocityLimitFactorVector_upcastFromVoid_180(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::VelocityLimitFactorVector> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::VelocityLimitFactorVector>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2VelocityLimitFactorVector_constructor_181(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::VelocityLimitFactorVector> Shared;

  size_t key = unwrap< size_t >(in[0]);
  boost::shared_ptr<gtsam::noiseModel::Base> cost_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[1], "ptr_gtsamnoiseModelBase");
  Vector vel_limit = unwrap< Vector >(in[2]);
  Vector limit_thresh = unwrap< Vector >(in[3]);
  Shared *self = new Shared(new gpmp2::VelocityLimitFactorVector(key,cost_model,vel_limit,limit_thresh));
  collector_gpmp2VelocityLimitFactorVector.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2VelocityLimitFactorVector_deconstructor_182(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::VelocityLimitFactorVector> Shared;
  checkArguments("delete_gpmp2VelocityLimitFactorVector",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2VelocityLimitFactorVector::iterator item;
  item = collector_gpmp2VelocityLimitFactorVector.find(self);
  if(item != collector_gpmp2VelocityLimitFactorVector.end()) {
    delete self;
    collector_gpmp2VelocityLimitFactorVector.erase(item);
  }
}

void gpmp2GaussianPriorWorkspacePositionArm_collectorInsertAndMakeBase_183(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianPriorWorkspacePositionArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2GaussianPriorWorkspacePositionArm.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2GaussianPriorWorkspacePositionArm_upcastFromVoid_184(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianPriorWorkspacePositionArm> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::GaussianPriorWorkspacePositionArm>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2GaussianPriorWorkspacePositionArm_constructor_185(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianPriorWorkspacePositionArm> Shared;

  size_t poseKey = unwrap< size_t >(in[0]);
  gpmp2::ArmModel& arm = *unwrap_shared_ptr< gpmp2::ArmModel >(in[1], "ptr_gpmp2ArmModel");
  int joint = unwrap< int >(in[2]);
  gtsam::Point3& des_position = *unwrap_shared_ptr< gtsam::Point3 >(in[3], "ptr_gtsamPoint3");
  boost::shared_ptr<gtsam::noiseModel::Base> cost_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[4], "ptr_gtsamnoiseModelBase");
  Shared *self = new Shared(new gpmp2::GaussianPriorWorkspacePositionArm(poseKey,arm,joint,des_position,cost_model));
  collector_gpmp2GaussianPriorWorkspacePositionArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2GaussianPriorWorkspacePositionArm_deconstructor_186(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::GaussianPriorWorkspacePositionArm> Shared;
  checkArguments("delete_gpmp2GaussianPriorWorkspacePositionArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2GaussianPriorWorkspacePositionArm::iterator item;
  item = collector_gpmp2GaussianPriorWorkspacePositionArm.find(self);
  if(item != collector_gpmp2GaussianPriorWorkspacePositionArm.end()) {
    delete self;
    collector_gpmp2GaussianPriorWorkspacePositionArm.erase(item);
  }
}

void gpmp2GaussianPriorWorkspaceOrientationArm_collectorInsertAndMakeBase_187(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianPriorWorkspaceOrientationArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2GaussianPriorWorkspaceOrientationArm.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2GaussianPriorWorkspaceOrientationArm_upcastFromVoid_188(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianPriorWorkspaceOrientationArm> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::GaussianPriorWorkspaceOrientationArm>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2GaussianPriorWorkspaceOrientationArm_constructor_189(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianPriorWorkspaceOrientationArm> Shared;

  size_t poseKey = unwrap< size_t >(in[0]);
  gpmp2::ArmModel& arm = *unwrap_shared_ptr< gpmp2::ArmModel >(in[1], "ptr_gpmp2ArmModel");
  int joint = unwrap< int >(in[2]);
  gtsam::Rot3& des_orientation = *unwrap_shared_ptr< gtsam::Rot3 >(in[3], "ptr_gtsamRot3");
  boost::shared_ptr<gtsam::noiseModel::Base> cost_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[4], "ptr_gtsamnoiseModelBase");
  Shared *self = new Shared(new gpmp2::GaussianPriorWorkspaceOrientationArm(poseKey,arm,joint,des_orientation,cost_model));
  collector_gpmp2GaussianPriorWorkspaceOrientationArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2GaussianPriorWorkspaceOrientationArm_deconstructor_190(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::GaussianPriorWorkspaceOrientationArm> Shared;
  checkArguments("delete_gpmp2GaussianPriorWorkspaceOrientationArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2GaussianPriorWorkspaceOrientationArm::iterator item;
  item = collector_gpmp2GaussianPriorWorkspaceOrientationArm.find(self);
  if(item != collector_gpmp2GaussianPriorWorkspaceOrientationArm.end()) {
    delete self;
    collector_gpmp2GaussianPriorWorkspaceOrientationArm.erase(item);
  }
}

void gpmp2GaussianPriorWorkspacePoseArm_collectorInsertAndMakeBase_191(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianPriorWorkspacePoseArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2GaussianPriorWorkspacePoseArm.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2GaussianPriorWorkspacePoseArm_upcastFromVoid_192(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianPriorWorkspacePoseArm> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::GaussianPriorWorkspacePoseArm>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2GaussianPriorWorkspacePoseArm_constructor_193(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::GaussianPriorWorkspacePoseArm> Shared;

  size_t poseKey = unwrap< size_t >(in[0]);
  gpmp2::ArmModel& arm = *unwrap_shared_ptr< gpmp2::ArmModel >(in[1], "ptr_gpmp2ArmModel");
  int joint = unwrap< int >(in[2]);
  gtsam::Pose3& des_pose = *unwrap_shared_ptr< gtsam::Pose3 >(in[3], "ptr_gtsamPose3");
  boost::shared_ptr<gtsam::noiseModel::Base> cost_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[4], "ptr_gtsamnoiseModelBase");
  Shared *self = new Shared(new gpmp2::GaussianPriorWorkspacePoseArm(poseKey,arm,joint,des_pose,cost_model));
  collector_gpmp2GaussianPriorWorkspacePoseArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2GaussianPriorWorkspacePoseArm_deconstructor_194(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::GaussianPriorWorkspacePoseArm> Shared;
  checkArguments("delete_gpmp2GaussianPriorWorkspacePoseArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2GaussianPriorWorkspacePoseArm::iterator item;
  item = collector_gpmp2GaussianPriorWorkspacePoseArm.find(self);
  if(item != collector_gpmp2GaussianPriorWorkspacePoseArm.end()) {
    delete self;
    collector_gpmp2GaussianPriorWorkspacePoseArm.erase(item);
  }
}

void gpmp2VehicleDynamicsFactorPose2_collectorInsertAndMakeBase_195(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::VehicleDynamicsFactorPose2> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2VehicleDynamicsFactorPose2.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2VehicleDynamicsFactorPose2_upcastFromVoid_196(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::VehicleDynamicsFactorPose2> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::VehicleDynamicsFactorPose2>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2VehicleDynamicsFactorPose2_constructor_197(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::VehicleDynamicsFactorPose2> Shared;

  size_t poseKey = unwrap< size_t >(in[0]);
  size_t velKey = unwrap< size_t >(in[1]);
  double cost_sigma = unwrap< double >(in[2]);
  Shared *self = new Shared(new gpmp2::VehicleDynamicsFactorPose2(poseKey,velKey,cost_sigma));
  collector_gpmp2VehicleDynamicsFactorPose2.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2VehicleDynamicsFactorPose2_deconstructor_198(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::VehicleDynamicsFactorPose2> Shared;
  checkArguments("delete_gpmp2VehicleDynamicsFactorPose2",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2VehicleDynamicsFactorPose2::iterator item;
  item = collector_gpmp2VehicleDynamicsFactorPose2.find(self);
  if(item != collector_gpmp2VehicleDynamicsFactorPose2.end()) {
    delete self;
    collector_gpmp2VehicleDynamicsFactorPose2.erase(item);
  }
}

void gpmp2VehicleDynamicsFactorPose2Vector_collectorInsertAndMakeBase_199(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::VehicleDynamicsFactorPose2Vector> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2VehicleDynamicsFactorPose2Vector.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2VehicleDynamicsFactorPose2Vector_upcastFromVoid_200(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::VehicleDynamicsFactorPose2Vector> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::VehicleDynamicsFactorPose2Vector>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2VehicleDynamicsFactorPose2Vector_constructor_201(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::VehicleDynamicsFactorPose2Vector> Shared;

  size_t poseKey = unwrap< size_t >(in[0]);
  size_t velKey = unwrap< size_t >(in[1]);
  double cost_sigma = unwrap< double >(in[2]);
  Shared *self = new Shared(new gpmp2::VehicleDynamicsFactorPose2Vector(poseKey,velKey,cost_sigma));
  collector_gpmp2VehicleDynamicsFactorPose2Vector.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2VehicleDynamicsFactorPose2Vector_deconstructor_202(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::VehicleDynamicsFactorPose2Vector> Shared;
  checkArguments("delete_gpmp2VehicleDynamicsFactorPose2Vector",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2VehicleDynamicsFactorPose2Vector::iterator item;
  item = collector_gpmp2VehicleDynamicsFactorPose2Vector.find(self);
  if(item != collector_gpmp2VehicleDynamicsFactorPose2Vector.end()) {
    delete self;
    collector_gpmp2VehicleDynamicsFactorPose2Vector.erase(item);
  }
}

void gpmp2VehicleDynamicsFactorVector_collectorInsertAndMakeBase_203(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::VehicleDynamicsFactorVector> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2VehicleDynamicsFactorVector.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2VehicleDynamicsFactorVector_upcastFromVoid_204(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::VehicleDynamicsFactorVector> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::VehicleDynamicsFactorVector>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2VehicleDynamicsFactorVector_constructor_205(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::VehicleDynamicsFactorVector> Shared;

  size_t poseKey = unwrap< size_t >(in[0]);
  size_t velKey = unwrap< size_t >(in[1]);
  double cost_sigma = unwrap< double >(in[2]);
  Shared *self = new Shared(new gpmp2::VehicleDynamicsFactorVector(poseKey,velKey,cost_sigma));
  collector_gpmp2VehicleDynamicsFactorVector.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2VehicleDynamicsFactorVector_deconstructor_206(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::VehicleDynamicsFactorVector> Shared;
  checkArguments("delete_gpmp2VehicleDynamicsFactorVector",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2VehicleDynamicsFactorVector::iterator item;
  item = collector_gpmp2VehicleDynamicsFactorVector.find(self);
  if(item != collector_gpmp2VehicleDynamicsFactorVector.end()) {
    delete self;
    collector_gpmp2VehicleDynamicsFactorVector.erase(item);
  }
}

void gpmp2SignedDistanceField_collectorInsertAndMakeBase_207(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::SignedDistanceField> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2SignedDistanceField.insert(self);
}

void gpmp2SignedDistanceField_constructor_208(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::SignedDistanceField> Shared;

  Shared *self = new Shared(new gpmp2::SignedDistanceField());
  collector_gpmp2SignedDistanceField.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2SignedDistanceField_constructor_209(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::SignedDistanceField> Shared;

  gtsam::Point3& origin = *unwrap_shared_ptr< gtsam::Point3 >(in[0], "ptr_gtsamPoint3");
  double cell_size = unwrap< double >(in[1]);
  size_t field_rows = unwrap< size_t >(in[2]);
  size_t field_cols = unwrap< size_t >(in[3]);
  size_t field_z = unwrap< size_t >(in[4]);
  Shared *self = new Shared(new gpmp2::SignedDistanceField(origin,cell_size,field_rows,field_cols,field_z));
  collector_gpmp2SignedDistanceField.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2SignedDistanceField_deconstructor_210(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::SignedDistanceField> Shared;
  checkArguments("delete_gpmp2SignedDistanceField",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2SignedDistanceField::iterator item;
  item = collector_gpmp2SignedDistanceField.find(self);
  if(item != collector_gpmp2SignedDistanceField.end()) {
    delete self;
    collector_gpmp2SignedDistanceField.erase(item);
  }
}

void gpmp2SignedDistanceField_getSignedDistance_211(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::SignedDistanceField> Shared;
  checkArguments("getSignedDistance",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::SignedDistanceField>(in[0], "ptr_gpmp2SignedDistanceField");
  gtsam::Point3& point = *unwrap_shared_ptr< gtsam::Point3 >(in[1], "ptr_gtsamPoint3");
  out[0] = wrap< double >(obj->getSignedDistance(point));
}

void gpmp2SignedDistanceField_initFieldData_212(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::SignedDistanceField> Shared;
  checkArguments("initFieldData",nargout,nargin-1,2);
  Shared obj = unwrap_shared_ptr<gpmp2::SignedDistanceField>(in[0], "ptr_gpmp2SignedDistanceField");
  size_t z_idx = unwrap< size_t >(in[1]);
  Matrix field_layer = unwrap< Matrix >(in[2]);
  obj->initFieldData(z_idx,field_layer);
}

void gpmp2SignedDistanceField_loadSDF_213(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::SignedDistanceField> Shared;
  checkArguments("loadSDF",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::SignedDistanceField>(in[0], "ptr_gpmp2SignedDistanceField");
  string filename = unwrap< string >(in[1]);
  obj->loadSDF(filename);
}

void gpmp2SignedDistanceField_print_214(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::SignedDistanceField> Shared;
  checkArguments("print",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::SignedDistanceField>(in[0], "ptr_gpmp2SignedDistanceField");
  string s = unwrap< string >(in[1]);
  obj->print(s);
}

void gpmp2SignedDistanceField_saveSDF_215(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::SignedDistanceField> Shared;
  checkArguments("saveSDF",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::SignedDistanceField>(in[0], "ptr_gpmp2SignedDistanceField");
  string filename = unwrap< string >(in[1]);
  obj->saveSDF(filename);
}

void gpmp2PlanarSDF_collectorInsertAndMakeBase_216(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::PlanarSDF> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2PlanarSDF.insert(self);
}

void gpmp2PlanarSDF_constructor_217(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::PlanarSDF> Shared;

  gtsam::Point2& origin = *unwrap_shared_ptr< gtsam::Point2 >(in[0], "ptr_gtsamPoint2");
  double cell_size = unwrap< double >(in[1]);
  Matrix data = unwrap< Matrix >(in[2]);
  Shared *self = new Shared(new gpmp2::PlanarSDF(origin,cell_size,data));
  collector_gpmp2PlanarSDF.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2PlanarSDF_deconstructor_218(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::PlanarSDF> Shared;
  checkArguments("delete_gpmp2PlanarSDF",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2PlanarSDF::iterator item;
  item = collector_gpmp2PlanarSDF.find(self);
  if(item != collector_gpmp2PlanarSDF.end()) {
    delete self;
    collector_gpmp2PlanarSDF.erase(item);
  }
}

void gpmp2PlanarSDF_getSignedDistance_219(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::PlanarSDF> Shared;
  checkArguments("getSignedDistance",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::PlanarSDF>(in[0], "ptr_gpmp2PlanarSDF");
  gtsam::Point2& point = *unwrap_shared_ptr< gtsam::Point2 >(in[1], "ptr_gtsamPoint2");
  out[0] = wrap< double >(obj->getSignedDistance(point));
}

void gpmp2PlanarSDF_print_220(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::PlanarSDF> Shared;
  checkArguments("print",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::PlanarSDF>(in[0], "ptr_gpmp2PlanarSDF");
  string s = unwrap< string >(in[1]);
  obj->print(s);
}

void gpmp2ObstacleSDFFactorArm_collectorInsertAndMakeBase_221(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstacleSDFFactorArm.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorArm_upcastFromVoid_222(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorArm> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstacleSDFFactorArm>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstacleSDFFactorArm_constructor_223(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorArm> Shared;

  size_t poseKey = unwrap< size_t >(in[0]);
  gpmp2::ArmModel& arm = *unwrap_shared_ptr< gpmp2::ArmModel >(in[1], "ptr_gpmp2ArmModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[2], "ptr_gpmp2SignedDistanceField");
  double cost_sigma = unwrap< double >(in[3]);
  double epsilon = unwrap< double >(in[4]);
  Shared *self = new Shared(new gpmp2::ObstacleSDFFactorArm(poseKey,arm,sdf,cost_sigma,epsilon));
  collector_gpmp2ObstacleSDFFactorArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorArm_deconstructor_224(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorArm> Shared;
  checkArguments("delete_gpmp2ObstacleSDFFactorArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstacleSDFFactorArm::iterator item;
  item = collector_gpmp2ObstacleSDFFactorArm.find(self);
  if(item != collector_gpmp2ObstacleSDFFactorArm.end()) {
    delete self;
    collector_gpmp2ObstacleSDFFactorArm.erase(item);
  }
}

void gpmp2ObstacleSDFFactorArm_evaluateError_225(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorArm> Shared;
  checkArguments("evaluateError",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ObstacleSDFFactorArm>(in[0], "ptr_gpmp2ObstacleSDFFactorArm");
  Vector pose = unwrap< Vector >(in[1]);
  out[0] = wrap< Vector >(obj->evaluateError(pose));
}

void gpmp2ObstacleSDFFactorGPArm_collectorInsertAndMakeBase_226(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstacleSDFFactorGPArm.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorGPArm_upcastFromVoid_227(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPArm> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstacleSDFFactorGPArm>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstacleSDFFactorGPArm_constructor_228(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPArm> Shared;

  size_t pose1Key = unwrap< size_t >(in[0]);
  size_t vel1Key = unwrap< size_t >(in[1]);
  size_t pose2Key = unwrap< size_t >(in[2]);
  size_t vel2Key = unwrap< size_t >(in[3]);
  gpmp2::ArmModel& arm = *unwrap_shared_ptr< gpmp2::ArmModel >(in[4], "ptr_gpmp2ArmModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[5], "ptr_gpmp2SignedDistanceField");
  double cost_sigma = unwrap< double >(in[6]);
  double epsilon = unwrap< double >(in[7]);
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[8], "ptr_gtsamnoiseModelBase");
  double delta_t = unwrap< double >(in[9]);
  double tau = unwrap< double >(in[10]);
  Shared *self = new Shared(new gpmp2::ObstacleSDFFactorGPArm(pose1Key,vel1Key,pose2Key,vel2Key,arm,sdf,cost_sigma,epsilon,Qc_model,delta_t,tau));
  collector_gpmp2ObstacleSDFFactorGPArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorGPArm_deconstructor_229(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPArm> Shared;
  checkArguments("delete_gpmp2ObstacleSDFFactorGPArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstacleSDFFactorGPArm::iterator item;
  item = collector_gpmp2ObstacleSDFFactorGPArm.find(self);
  if(item != collector_gpmp2ObstacleSDFFactorGPArm.end()) {
    delete self;
    collector_gpmp2ObstacleSDFFactorGPArm.erase(item);
  }
}

void gpmp2ObstaclePlanarSDFFactorArm_collectorInsertAndMakeBase_230(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstaclePlanarSDFFactorArm.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorArm_upcastFromVoid_231(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorArm> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstaclePlanarSDFFactorArm>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstaclePlanarSDFFactorArm_constructor_232(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorArm> Shared;

  size_t posekey = unwrap< size_t >(in[0]);
  gpmp2::ArmModel& arm = *unwrap_shared_ptr< gpmp2::ArmModel >(in[1], "ptr_gpmp2ArmModel");
  gpmp2::PlanarSDF& sdf = *unwrap_shared_ptr< gpmp2::PlanarSDF >(in[2], "ptr_gpmp2PlanarSDF");
  double cost_sigma = unwrap< double >(in[3]);
  double epsilon = unwrap< double >(in[4]);
  Shared *self = new Shared(new gpmp2::ObstaclePlanarSDFFactorArm(posekey,arm,sdf,cost_sigma,epsilon));
  collector_gpmp2ObstaclePlanarSDFFactorArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorArm_deconstructor_233(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorArm> Shared;
  checkArguments("delete_gpmp2ObstaclePlanarSDFFactorArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstaclePlanarSDFFactorArm::iterator item;
  item = collector_gpmp2ObstaclePlanarSDFFactorArm.find(self);
  if(item != collector_gpmp2ObstaclePlanarSDFFactorArm.end()) {
    delete self;
    collector_gpmp2ObstaclePlanarSDFFactorArm.erase(item);
  }
}

void gpmp2ObstaclePlanarSDFFactorArm_evaluateError_234(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorArm> Shared;
  checkArguments("evaluateError",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ObstaclePlanarSDFFactorArm>(in[0], "ptr_gpmp2ObstaclePlanarSDFFactorArm");
  Vector pose = unwrap< Vector >(in[1]);
  out[0] = wrap< Vector >(obj->evaluateError(pose));
}

void gpmp2ObstaclePlanarSDFFactorArm_string_serialize_235(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorArm> Shared;
  checkArguments("string_serialize",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ObstaclePlanarSDFFactorArm>(in[0], "ptr_gpmp2ObstaclePlanarSDFFactorArm");
  ostringstream out_archive_stream;
  boost::archive::text_oarchive out_archive(out_archive_stream);
  out_archive << *obj;
  out[0] = wrap< string >(out_archive_stream.str());
}
void gpmp2ObstaclePlanarSDFFactorArm_string_deserialize_236(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorArm> Shared;
  checkArguments("gpmp2ObstaclePlanarSDFFactorArm.string_deserialize",nargout,nargin,1);
  string serialized = unwrap< string >(in[0]);
  istringstream in_archive_stream(serialized);
  boost::archive::text_iarchive in_archive(in_archive_stream);
  Shared output(new gpmp2::ObstaclePlanarSDFFactorArm());
  in_archive >> *output;
  out[0] = wrap_shared_ptr(output,"gpmp2.ObstaclePlanarSDFFactorArm", false);
}
void gpmp2ObstaclePlanarSDFFactorGPArm_collectorInsertAndMakeBase_237(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstaclePlanarSDFFactorGPArm.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorGPArm_upcastFromVoid_238(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPArm> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstaclePlanarSDFFactorGPArm>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstaclePlanarSDFFactorGPArm_constructor_239(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPArm> Shared;

  size_t pose1key = unwrap< size_t >(in[0]);
  size_t vel1key = unwrap< size_t >(in[1]);
  size_t pose2key = unwrap< size_t >(in[2]);
  size_t vel2key = unwrap< size_t >(in[3]);
  gpmp2::ArmModel& arm = *unwrap_shared_ptr< gpmp2::ArmModel >(in[4], "ptr_gpmp2ArmModel");
  gpmp2::PlanarSDF& sdf = *unwrap_shared_ptr< gpmp2::PlanarSDF >(in[5], "ptr_gpmp2PlanarSDF");
  double cost_sigma = unwrap< double >(in[6]);
  double epsilon = unwrap< double >(in[7]);
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[8], "ptr_gtsamnoiseModelBase");
  double delta_t = unwrap< double >(in[9]);
  double tau = unwrap< double >(in[10]);
  Shared *self = new Shared(new gpmp2::ObstaclePlanarSDFFactorGPArm(pose1key,vel1key,pose2key,vel2key,arm,sdf,cost_sigma,epsilon,Qc_model,delta_t,tau));
  collector_gpmp2ObstaclePlanarSDFFactorGPArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorGPArm_deconstructor_240(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPArm> Shared;
  checkArguments("delete_gpmp2ObstaclePlanarSDFFactorGPArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstaclePlanarSDFFactorGPArm::iterator item;
  item = collector_gpmp2ObstaclePlanarSDFFactorGPArm.find(self);
  if(item != collector_gpmp2ObstaclePlanarSDFFactorGPArm.end()) {
    delete self;
    collector_gpmp2ObstaclePlanarSDFFactorGPArm.erase(item);
  }
}

void gpmp2ObstaclePlanarSDFFactorPointRobot_collectorInsertAndMakeBase_241(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPointRobot> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstaclePlanarSDFFactorPointRobot.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorPointRobot_upcastFromVoid_242(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPointRobot> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstaclePlanarSDFFactorPointRobot>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstaclePlanarSDFFactorPointRobot_constructor_243(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPointRobot> Shared;

  size_t posekey = unwrap< size_t >(in[0]);
  gpmp2::PointRobotModel& pR = *unwrap_shared_ptr< gpmp2::PointRobotModel >(in[1], "ptr_gpmp2PointRobotModel");
  gpmp2::PlanarSDF& sdf = *unwrap_shared_ptr< gpmp2::PlanarSDF >(in[2], "ptr_gpmp2PlanarSDF");
  double cost_sigma = unwrap< double >(in[3]);
  double epsilon = unwrap< double >(in[4]);
  Shared *self = new Shared(new gpmp2::ObstaclePlanarSDFFactorPointRobot(posekey,pR,sdf,cost_sigma,epsilon));
  collector_gpmp2ObstaclePlanarSDFFactorPointRobot.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorPointRobot_deconstructor_244(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPointRobot> Shared;
  checkArguments("delete_gpmp2ObstaclePlanarSDFFactorPointRobot",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstaclePlanarSDFFactorPointRobot::iterator item;
  item = collector_gpmp2ObstaclePlanarSDFFactorPointRobot.find(self);
  if(item != collector_gpmp2ObstaclePlanarSDFFactorPointRobot.end()) {
    delete self;
    collector_gpmp2ObstaclePlanarSDFFactorPointRobot.erase(item);
  }
}

void gpmp2ObstaclePlanarSDFFactorPointRobot_evaluateError_245(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPointRobot> Shared;
  checkArguments("evaluateError",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ObstaclePlanarSDFFactorPointRobot>(in[0], "ptr_gpmp2ObstaclePlanarSDFFactorPointRobot");
  Vector pose = unwrap< Vector >(in[1]);
  out[0] = wrap< Vector >(obj->evaluateError(pose));
}

void gpmp2ObstaclePlanarSDFFactorGPPointRobot_collectorInsertAndMakeBase_246(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPointRobot> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstaclePlanarSDFFactorGPPointRobot.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorGPPointRobot_upcastFromVoid_247(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPointRobot> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstaclePlanarSDFFactorGPPointRobot>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstaclePlanarSDFFactorGPPointRobot_constructor_248(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPointRobot> Shared;

  size_t pose1key = unwrap< size_t >(in[0]);
  size_t vel1key = unwrap< size_t >(in[1]);
  size_t pose2key = unwrap< size_t >(in[2]);
  size_t vel2key = unwrap< size_t >(in[3]);
  gpmp2::PointRobotModel& pR = *unwrap_shared_ptr< gpmp2::PointRobotModel >(in[4], "ptr_gpmp2PointRobotModel");
  gpmp2::PlanarSDF& sdf = *unwrap_shared_ptr< gpmp2::PlanarSDF >(in[5], "ptr_gpmp2PlanarSDF");
  double cost_sigma = unwrap< double >(in[6]);
  double epsilon = unwrap< double >(in[7]);
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[8], "ptr_gtsamnoiseModelBase");
  double delta_t = unwrap< double >(in[9]);
  double tau = unwrap< double >(in[10]);
  Shared *self = new Shared(new gpmp2::ObstaclePlanarSDFFactorGPPointRobot(pose1key,vel1key,pose2key,vel2key,pR,sdf,cost_sigma,epsilon,Qc_model,delta_t,tau));
  collector_gpmp2ObstaclePlanarSDFFactorGPPointRobot.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorGPPointRobot_deconstructor_249(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPointRobot> Shared;
  checkArguments("delete_gpmp2ObstaclePlanarSDFFactorGPPointRobot",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstaclePlanarSDFFactorGPPointRobot::iterator item;
  item = collector_gpmp2ObstaclePlanarSDFFactorGPPointRobot.find(self);
  if(item != collector_gpmp2ObstaclePlanarSDFFactorGPPointRobot.end()) {
    delete self;
    collector_gpmp2ObstaclePlanarSDFFactorGPPointRobot.erase(item);
  }
}

void gpmp2ObstaclePlanarSDFFactorPose2MobileBase_collectorInsertAndMakeBase_250(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2MobileBase> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstaclePlanarSDFFactorPose2MobileBase.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorPose2MobileBase_upcastFromVoid_251(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2MobileBase> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstaclePlanarSDFFactorPose2MobileBase>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstaclePlanarSDFFactorPose2MobileBase_constructor_252(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2MobileBase> Shared;

  size_t posekey = unwrap< size_t >(in[0]);
  gpmp2::Pose2MobileBaseModel& robot = *unwrap_shared_ptr< gpmp2::Pose2MobileBaseModel >(in[1], "ptr_gpmp2Pose2MobileBaseModel");
  gpmp2::PlanarSDF& sdf = *unwrap_shared_ptr< gpmp2::PlanarSDF >(in[2], "ptr_gpmp2PlanarSDF");
  double cost_sigma = unwrap< double >(in[3]);
  double epsilon = unwrap< double >(in[4]);
  Shared *self = new Shared(new gpmp2::ObstaclePlanarSDFFactorPose2MobileBase(posekey,robot,sdf,cost_sigma,epsilon));
  collector_gpmp2ObstaclePlanarSDFFactorPose2MobileBase.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorPose2MobileBase_deconstructor_253(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2MobileBase> Shared;
  checkArguments("delete_gpmp2ObstaclePlanarSDFFactorPose2MobileBase",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstaclePlanarSDFFactorPose2MobileBase::iterator item;
  item = collector_gpmp2ObstaclePlanarSDFFactorPose2MobileBase.find(self);
  if(item != collector_gpmp2ObstaclePlanarSDFFactorPose2MobileBase.end()) {
    delete self;
    collector_gpmp2ObstaclePlanarSDFFactorPose2MobileBase.erase(item);
  }
}

void gpmp2ObstaclePlanarSDFFactorPose2MobileBase_evaluateError_254(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2MobileBase> Shared;
  checkArguments("evaluateError",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2MobileBase>(in[0], "ptr_gpmp2ObstaclePlanarSDFFactorPose2MobileBase");
  gtsam::Pose2& pose = *unwrap_shared_ptr< gtsam::Pose2 >(in[1], "ptr_gtsamPose2");
  out[0] = wrap< Vector >(obj->evaluateError(pose));
}

void gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase_collectorInsertAndMakeBase_255(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPose2MobileBase> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase_upcastFromVoid_256(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPose2MobileBase> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstaclePlanarSDFFactorGPPose2MobileBase>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase_constructor_257(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPose2MobileBase> Shared;

  size_t pose1key = unwrap< size_t >(in[0]);
  size_t vel1key = unwrap< size_t >(in[1]);
  size_t pose2key = unwrap< size_t >(in[2]);
  size_t vel2key = unwrap< size_t >(in[3]);
  gpmp2::Pose2MobileBaseModel& robot = *unwrap_shared_ptr< gpmp2::Pose2MobileBaseModel >(in[4], "ptr_gpmp2Pose2MobileBaseModel");
  gpmp2::PlanarSDF& sdf = *unwrap_shared_ptr< gpmp2::PlanarSDF >(in[5], "ptr_gpmp2PlanarSDF");
  double cost_sigma = unwrap< double >(in[6]);
  double epsilon = unwrap< double >(in[7]);
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[8], "ptr_gtsamnoiseModelBase");
  double delta_t = unwrap< double >(in[9]);
  double tau = unwrap< double >(in[10]);
  Shared *self = new Shared(new gpmp2::ObstaclePlanarSDFFactorGPPose2MobileBase(pose1key,vel1key,pose2key,vel2key,robot,sdf,cost_sigma,epsilon,Qc_model,delta_t,tau));
  collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase_deconstructor_258(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPose2MobileBase> Shared;
  checkArguments("delete_gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase::iterator item;
  item = collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase.find(self);
  if(item != collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase.end()) {
    delete self;
    collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase.erase(item);
  }
}

void gpmp2ObstaclePlanarSDFFactorPose2MobileArm_collectorInsertAndMakeBase_259(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2MobileArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstaclePlanarSDFFactorPose2MobileArm.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorPose2MobileArm_upcastFromVoid_260(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2MobileArm> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstaclePlanarSDFFactorPose2MobileArm>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstaclePlanarSDFFactorPose2MobileArm_constructor_261(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2MobileArm> Shared;

  size_t posekey = unwrap< size_t >(in[0]);
  gpmp2::Pose2MobileArmModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileArmModel >(in[1], "ptr_gpmp2Pose2MobileArmModel");
  gpmp2::PlanarSDF& sdf = *unwrap_shared_ptr< gpmp2::PlanarSDF >(in[2], "ptr_gpmp2PlanarSDF");
  double cost_sigma = unwrap< double >(in[3]);
  double epsilon = unwrap< double >(in[4]);
  Shared *self = new Shared(new gpmp2::ObstaclePlanarSDFFactorPose2MobileArm(posekey,marm,sdf,cost_sigma,epsilon));
  collector_gpmp2ObstaclePlanarSDFFactorPose2MobileArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorPose2MobileArm_deconstructor_262(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2MobileArm> Shared;
  checkArguments("delete_gpmp2ObstaclePlanarSDFFactorPose2MobileArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstaclePlanarSDFFactorPose2MobileArm::iterator item;
  item = collector_gpmp2ObstaclePlanarSDFFactorPose2MobileArm.find(self);
  if(item != collector_gpmp2ObstaclePlanarSDFFactorPose2MobileArm.end()) {
    delete self;
    collector_gpmp2ObstaclePlanarSDFFactorPose2MobileArm.erase(item);
  }
}

void gpmp2ObstaclePlanarSDFFactorPose2MobileArm_evaluateError_263(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2MobileArm> Shared;
  checkArguments("evaluateError",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2MobileArm>(in[0], "ptr_gpmp2ObstaclePlanarSDFFactorPose2MobileArm");
  gpmp2::Pose2Vector& pose = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Vector >(obj->evaluateError(pose));
}

void gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm_collectorInsertAndMakeBase_264(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPose2MobileArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm_upcastFromVoid_265(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPose2MobileArm> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstaclePlanarSDFFactorGPPose2MobileArm>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm_constructor_266(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPose2MobileArm> Shared;

  size_t pose1key = unwrap< size_t >(in[0]);
  size_t vel1key = unwrap< size_t >(in[1]);
  size_t pose2key = unwrap< size_t >(in[2]);
  size_t vel2key = unwrap< size_t >(in[3]);
  gpmp2::Pose2MobileArmModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileArmModel >(in[4], "ptr_gpmp2Pose2MobileArmModel");
  gpmp2::PlanarSDF& sdf = *unwrap_shared_ptr< gpmp2::PlanarSDF >(in[5], "ptr_gpmp2PlanarSDF");
  double cost_sigma = unwrap< double >(in[6]);
  double epsilon = unwrap< double >(in[7]);
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[8], "ptr_gtsamnoiseModelBase");
  double delta_t = unwrap< double >(in[9]);
  double tau = unwrap< double >(in[10]);
  Shared *self = new Shared(new gpmp2::ObstaclePlanarSDFFactorGPPose2MobileArm(pose1key,vel1key,pose2key,vel2key,marm,sdf,cost_sigma,epsilon,Qc_model,delta_t,tau));
  collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm_deconstructor_267(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPose2MobileArm> Shared;
  checkArguments("delete_gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm::iterator item;
  item = collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm.find(self);
  if(item != collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm.end()) {
    delete self;
    collector_gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm.erase(item);
  }
}

void gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms_collectorInsertAndMakeBase_268(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2Mobile2Arms> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms_upcastFromVoid_269(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2Mobile2Arms> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstaclePlanarSDFFactorPose2Mobile2Arms>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms_constructor_270(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2Mobile2Arms> Shared;

  size_t posekey = unwrap< size_t >(in[0]);
  gpmp2::Pose2Mobile2ArmsModel& marm = *unwrap_shared_ptr< gpmp2::Pose2Mobile2ArmsModel >(in[1], "ptr_gpmp2Pose2Mobile2ArmsModel");
  gpmp2::PlanarSDF& sdf = *unwrap_shared_ptr< gpmp2::PlanarSDF >(in[2], "ptr_gpmp2PlanarSDF");
  double cost_sigma = unwrap< double >(in[3]);
  double epsilon = unwrap< double >(in[4]);
  Shared *self = new Shared(new gpmp2::ObstaclePlanarSDFFactorPose2Mobile2Arms(posekey,marm,sdf,cost_sigma,epsilon));
  collector_gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms_deconstructor_271(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2Mobile2Arms> Shared;
  checkArguments("delete_gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms::iterator item;
  item = collector_gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms.find(self);
  if(item != collector_gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms.end()) {
    delete self;
    collector_gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms.erase(item);
  }
}

void gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms_evaluateError_272(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2Mobile2Arms> Shared;
  checkArguments("evaluateError",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ObstaclePlanarSDFFactorPose2Mobile2Arms>(in[0], "ptr_gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms");
  gpmp2::Pose2Vector& pose = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Vector >(obj->evaluateError(pose));
}

void gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms_collectorInsertAndMakeBase_273(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPose2Mobile2Arms> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms_upcastFromVoid_274(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPose2Mobile2Arms> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstaclePlanarSDFFactorGPPose2Mobile2Arms>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms_constructor_275(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPose2Mobile2Arms> Shared;

  size_t pose1key = unwrap< size_t >(in[0]);
  size_t vel1key = unwrap< size_t >(in[1]);
  size_t pose2key = unwrap< size_t >(in[2]);
  size_t vel2key = unwrap< size_t >(in[3]);
  gpmp2::Pose2Mobile2ArmsModel& marm = *unwrap_shared_ptr< gpmp2::Pose2Mobile2ArmsModel >(in[4], "ptr_gpmp2Pose2Mobile2ArmsModel");
  gpmp2::PlanarSDF& sdf = *unwrap_shared_ptr< gpmp2::PlanarSDF >(in[5], "ptr_gpmp2PlanarSDF");
  double cost_sigma = unwrap< double >(in[6]);
  double epsilon = unwrap< double >(in[7]);
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[8], "ptr_gtsamnoiseModelBase");
  double delta_t = unwrap< double >(in[9]);
  double tau = unwrap< double >(in[10]);
  Shared *self = new Shared(new gpmp2::ObstaclePlanarSDFFactorGPPose2Mobile2Arms(pose1key,vel1key,pose2key,vel2key,marm,sdf,cost_sigma,epsilon,Qc_model,delta_t,tau));
  collector_gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms_deconstructor_276(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstaclePlanarSDFFactorGPPose2Mobile2Arms> Shared;
  checkArguments("delete_gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms::iterator item;
  item = collector_gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms.find(self);
  if(item != collector_gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms.end()) {
    delete self;
    collector_gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms.erase(item);
  }
}

void gpmp2ObstacleSDFFactorPose2MobileBase_collectorInsertAndMakeBase_277(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileBase> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstacleSDFFactorPose2MobileBase.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorPose2MobileBase_upcastFromVoid_278(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileBase> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstacleSDFFactorPose2MobileBase>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstacleSDFFactorPose2MobileBase_constructor_279(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileBase> Shared;

  size_t posekey = unwrap< size_t >(in[0]);
  gpmp2::Pose2MobileBaseModel& robot = *unwrap_shared_ptr< gpmp2::Pose2MobileBaseModel >(in[1], "ptr_gpmp2Pose2MobileBaseModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[2], "ptr_gpmp2SignedDistanceField");
  double cost_sigma = unwrap< double >(in[3]);
  double epsilon = unwrap< double >(in[4]);
  Shared *self = new Shared(new gpmp2::ObstacleSDFFactorPose2MobileBase(posekey,robot,sdf,cost_sigma,epsilon));
  collector_gpmp2ObstacleSDFFactorPose2MobileBase.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorPose2MobileBase_deconstructor_280(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileBase> Shared;
  checkArguments("delete_gpmp2ObstacleSDFFactorPose2MobileBase",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstacleSDFFactorPose2MobileBase::iterator item;
  item = collector_gpmp2ObstacleSDFFactorPose2MobileBase.find(self);
  if(item != collector_gpmp2ObstacleSDFFactorPose2MobileBase.end()) {
    delete self;
    collector_gpmp2ObstacleSDFFactorPose2MobileBase.erase(item);
  }
}

void gpmp2ObstacleSDFFactorPose2MobileBase_evaluateError_281(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileBase> Shared;
  checkArguments("evaluateError",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileBase>(in[0], "ptr_gpmp2ObstacleSDFFactorPose2MobileBase");
  gtsam::Pose2& pose = *unwrap_shared_ptr< gtsam::Pose2 >(in[1], "ptr_gtsamPose2");
  out[0] = wrap< Vector >(obj->evaluateError(pose));
}

void gpmp2ObstacleSDFFactorGPPose2MobileBase_collectorInsertAndMakeBase_282(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileBase> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstacleSDFFactorGPPose2MobileBase.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorGPPose2MobileBase_upcastFromVoid_283(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileBase> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstacleSDFFactorGPPose2MobileBase>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstacleSDFFactorGPPose2MobileBase_constructor_284(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileBase> Shared;

  size_t pose1key = unwrap< size_t >(in[0]);
  size_t vel1key = unwrap< size_t >(in[1]);
  size_t pose2key = unwrap< size_t >(in[2]);
  size_t vel2key = unwrap< size_t >(in[3]);
  gpmp2::Pose2MobileBaseModel& robot = *unwrap_shared_ptr< gpmp2::Pose2MobileBaseModel >(in[4], "ptr_gpmp2Pose2MobileBaseModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[5], "ptr_gpmp2SignedDistanceField");
  double cost_sigma = unwrap< double >(in[6]);
  double epsilon = unwrap< double >(in[7]);
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[8], "ptr_gtsamnoiseModelBase");
  double delta_t = unwrap< double >(in[9]);
  double tau = unwrap< double >(in[10]);
  Shared *self = new Shared(new gpmp2::ObstacleSDFFactorGPPose2MobileBase(pose1key,vel1key,pose2key,vel2key,robot,sdf,cost_sigma,epsilon,Qc_model,delta_t,tau));
  collector_gpmp2ObstacleSDFFactorGPPose2MobileBase.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorGPPose2MobileBase_deconstructor_285(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileBase> Shared;
  checkArguments("delete_gpmp2ObstacleSDFFactorGPPose2MobileBase",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstacleSDFFactorGPPose2MobileBase::iterator item;
  item = collector_gpmp2ObstacleSDFFactorGPPose2MobileBase.find(self);
  if(item != collector_gpmp2ObstacleSDFFactorGPPose2MobileBase.end()) {
    delete self;
    collector_gpmp2ObstacleSDFFactorGPPose2MobileBase.erase(item);
  }
}

void gpmp2ObstacleSDFFactorPose2MobileArm_collectorInsertAndMakeBase_286(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstacleSDFFactorPose2MobileArm.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorPose2MobileArm_upcastFromVoid_287(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileArm> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstacleSDFFactorPose2MobileArm>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstacleSDFFactorPose2MobileArm_constructor_288(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileArm> Shared;

  size_t posekey = unwrap< size_t >(in[0]);
  gpmp2::Pose2MobileArmModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileArmModel >(in[1], "ptr_gpmp2Pose2MobileArmModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[2], "ptr_gpmp2SignedDistanceField");
  double cost_sigma = unwrap< double >(in[3]);
  double epsilon = unwrap< double >(in[4]);
  Shared *self = new Shared(new gpmp2::ObstacleSDFFactorPose2MobileArm(posekey,marm,sdf,cost_sigma,epsilon));
  collector_gpmp2ObstacleSDFFactorPose2MobileArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorPose2MobileArm_deconstructor_289(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileArm> Shared;
  checkArguments("delete_gpmp2ObstacleSDFFactorPose2MobileArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstacleSDFFactorPose2MobileArm::iterator item;
  item = collector_gpmp2ObstacleSDFFactorPose2MobileArm.find(self);
  if(item != collector_gpmp2ObstacleSDFFactorPose2MobileArm.end()) {
    delete self;
    collector_gpmp2ObstacleSDFFactorPose2MobileArm.erase(item);
  }
}

void gpmp2ObstacleSDFFactorPose2MobileArm_evaluateError_290(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileArm> Shared;
  checkArguments("evaluateError",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileArm>(in[0], "ptr_gpmp2ObstacleSDFFactorPose2MobileArm");
  gpmp2::Pose2Vector& pose = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Vector >(obj->evaluateError(pose));
}

void gpmp2ObstacleSDFFactorGPPose2MobileArm_collectorInsertAndMakeBase_291(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstacleSDFFactorGPPose2MobileArm.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorGPPose2MobileArm_upcastFromVoid_292(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileArm> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstacleSDFFactorGPPose2MobileArm>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstacleSDFFactorGPPose2MobileArm_constructor_293(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileArm> Shared;

  size_t pose1key = unwrap< size_t >(in[0]);
  size_t vel1key = unwrap< size_t >(in[1]);
  size_t pose2key = unwrap< size_t >(in[2]);
  size_t vel2key = unwrap< size_t >(in[3]);
  gpmp2::Pose2MobileArmModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileArmModel >(in[4], "ptr_gpmp2Pose2MobileArmModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[5], "ptr_gpmp2SignedDistanceField");
  double cost_sigma = unwrap< double >(in[6]);
  double epsilon = unwrap< double >(in[7]);
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[8], "ptr_gtsamnoiseModelBase");
  double delta_t = unwrap< double >(in[9]);
  double tau = unwrap< double >(in[10]);
  Shared *self = new Shared(new gpmp2::ObstacleSDFFactorGPPose2MobileArm(pose1key,vel1key,pose2key,vel2key,marm,sdf,cost_sigma,epsilon,Qc_model,delta_t,tau));
  collector_gpmp2ObstacleSDFFactorGPPose2MobileArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorGPPose2MobileArm_deconstructor_294(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileArm> Shared;
  checkArguments("delete_gpmp2ObstacleSDFFactorGPPose2MobileArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstacleSDFFactorGPPose2MobileArm::iterator item;
  item = collector_gpmp2ObstacleSDFFactorGPPose2MobileArm.find(self);
  if(item != collector_gpmp2ObstacleSDFFactorGPPose2MobileArm.end()) {
    delete self;
    collector_gpmp2ObstacleSDFFactorGPPose2MobileArm.erase(item);
  }
}

void gpmp2ObstacleSDFFactorPose2Mobile2Arms_collectorInsertAndMakeBase_295(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2Mobile2Arms> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstacleSDFFactorPose2Mobile2Arms.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorPose2Mobile2Arms_upcastFromVoid_296(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2Mobile2Arms> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstacleSDFFactorPose2Mobile2Arms>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstacleSDFFactorPose2Mobile2Arms_constructor_297(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2Mobile2Arms> Shared;

  size_t posekey = unwrap< size_t >(in[0]);
  gpmp2::Pose2Mobile2ArmsModel& marm = *unwrap_shared_ptr< gpmp2::Pose2Mobile2ArmsModel >(in[1], "ptr_gpmp2Pose2Mobile2ArmsModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[2], "ptr_gpmp2SignedDistanceField");
  double cost_sigma = unwrap< double >(in[3]);
  double epsilon = unwrap< double >(in[4]);
  Shared *self = new Shared(new gpmp2::ObstacleSDFFactorPose2Mobile2Arms(posekey,marm,sdf,cost_sigma,epsilon));
  collector_gpmp2ObstacleSDFFactorPose2Mobile2Arms.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorPose2Mobile2Arms_deconstructor_298(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2Mobile2Arms> Shared;
  checkArguments("delete_gpmp2ObstacleSDFFactorPose2Mobile2Arms",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstacleSDFFactorPose2Mobile2Arms::iterator item;
  item = collector_gpmp2ObstacleSDFFactorPose2Mobile2Arms.find(self);
  if(item != collector_gpmp2ObstacleSDFFactorPose2Mobile2Arms.end()) {
    delete self;
    collector_gpmp2ObstacleSDFFactorPose2Mobile2Arms.erase(item);
  }
}

void gpmp2ObstacleSDFFactorPose2Mobile2Arms_evaluateError_299(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2Mobile2Arms> Shared;
  checkArguments("evaluateError",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ObstacleSDFFactorPose2Mobile2Arms>(in[0], "ptr_gpmp2ObstacleSDFFactorPose2Mobile2Arms");
  gpmp2::Pose2Vector& pose = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Vector >(obj->evaluateError(pose));
}

void gpmp2ObstacleSDFFactorGPPose2Mobile2Arms_collectorInsertAndMakeBase_300(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2Mobile2Arms> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstacleSDFFactorGPPose2Mobile2Arms.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorGPPose2Mobile2Arms_upcastFromVoid_301(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2Mobile2Arms> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstacleSDFFactorGPPose2Mobile2Arms>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstacleSDFFactorGPPose2Mobile2Arms_constructor_302(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2Mobile2Arms> Shared;

  size_t pose1key = unwrap< size_t >(in[0]);
  size_t vel1key = unwrap< size_t >(in[1]);
  size_t pose2key = unwrap< size_t >(in[2]);
  size_t vel2key = unwrap< size_t >(in[3]);
  gpmp2::Pose2Mobile2ArmsModel& marm = *unwrap_shared_ptr< gpmp2::Pose2Mobile2ArmsModel >(in[4], "ptr_gpmp2Pose2Mobile2ArmsModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[5], "ptr_gpmp2SignedDistanceField");
  double cost_sigma = unwrap< double >(in[6]);
  double epsilon = unwrap< double >(in[7]);
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[8], "ptr_gtsamnoiseModelBase");
  double delta_t = unwrap< double >(in[9]);
  double tau = unwrap< double >(in[10]);
  Shared *self = new Shared(new gpmp2::ObstacleSDFFactorGPPose2Mobile2Arms(pose1key,vel1key,pose2key,vel2key,marm,sdf,cost_sigma,epsilon,Qc_model,delta_t,tau));
  collector_gpmp2ObstacleSDFFactorGPPose2Mobile2Arms.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorGPPose2Mobile2Arms_deconstructor_303(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2Mobile2Arms> Shared;
  checkArguments("delete_gpmp2ObstacleSDFFactorGPPose2Mobile2Arms",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstacleSDFFactorGPPose2Mobile2Arms::iterator item;
  item = collector_gpmp2ObstacleSDFFactorGPPose2Mobile2Arms.find(self);
  if(item != collector_gpmp2ObstacleSDFFactorGPPose2Mobile2Arms.end()) {
    delete self;
    collector_gpmp2ObstacleSDFFactorGPPose2Mobile2Arms.erase(item);
  }
}

void gpmp2ObstacleSDFFactorPose2MobileVetLinArm_collectorInsertAndMakeBase_304(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileVetLinArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstacleSDFFactorPose2MobileVetLinArm.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorPose2MobileVetLinArm_upcastFromVoid_305(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileVetLinArm> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstacleSDFFactorPose2MobileVetLinArm>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstacleSDFFactorPose2MobileVetLinArm_constructor_306(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileVetLinArm> Shared;

  size_t posekey = unwrap< size_t >(in[0]);
  gpmp2::Pose2MobileVetLinArmModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileVetLinArmModel >(in[1], "ptr_gpmp2Pose2MobileVetLinArmModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[2], "ptr_gpmp2SignedDistanceField");
  double cost_sigma = unwrap< double >(in[3]);
  double epsilon = unwrap< double >(in[4]);
  Shared *self = new Shared(new gpmp2::ObstacleSDFFactorPose2MobileVetLinArm(posekey,marm,sdf,cost_sigma,epsilon));
  collector_gpmp2ObstacleSDFFactorPose2MobileVetLinArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorPose2MobileVetLinArm_deconstructor_307(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileVetLinArm> Shared;
  checkArguments("delete_gpmp2ObstacleSDFFactorPose2MobileVetLinArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstacleSDFFactorPose2MobileVetLinArm::iterator item;
  item = collector_gpmp2ObstacleSDFFactorPose2MobileVetLinArm.find(self);
  if(item != collector_gpmp2ObstacleSDFFactorPose2MobileVetLinArm.end()) {
    delete self;
    collector_gpmp2ObstacleSDFFactorPose2MobileVetLinArm.erase(item);
  }
}

void gpmp2ObstacleSDFFactorPose2MobileVetLinArm_evaluateError_308(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileVetLinArm> Shared;
  checkArguments("evaluateError",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileVetLinArm>(in[0], "ptr_gpmp2ObstacleSDFFactorPose2MobileVetLinArm");
  gpmp2::Pose2Vector& pose = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Vector >(obj->evaluateError(pose));
}

void gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm_collectorInsertAndMakeBase_309(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileVetLinArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm_upcastFromVoid_310(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileVetLinArm> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstacleSDFFactorGPPose2MobileVetLinArm>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm_constructor_311(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileVetLinArm> Shared;

  size_t pose1key = unwrap< size_t >(in[0]);
  size_t vel1key = unwrap< size_t >(in[1]);
  size_t pose2key = unwrap< size_t >(in[2]);
  size_t vel2key = unwrap< size_t >(in[3]);
  gpmp2::Pose2MobileVetLinArmModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileVetLinArmModel >(in[4], "ptr_gpmp2Pose2MobileVetLinArmModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[5], "ptr_gpmp2SignedDistanceField");
  double cost_sigma = unwrap< double >(in[6]);
  double epsilon = unwrap< double >(in[7]);
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[8], "ptr_gtsamnoiseModelBase");
  double delta_t = unwrap< double >(in[9]);
  double tau = unwrap< double >(in[10]);
  Shared *self = new Shared(new gpmp2::ObstacleSDFFactorGPPose2MobileVetLinArm(pose1key,vel1key,pose2key,vel2key,marm,sdf,cost_sigma,epsilon,Qc_model,delta_t,tau));
  collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm_deconstructor_312(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileVetLinArm> Shared;
  checkArguments("delete_gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm::iterator item;
  item = collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm.find(self);
  if(item != collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm.end()) {
    delete self;
    collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm.erase(item);
  }
}

void gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms_collectorInsertAndMakeBase_313(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileVetLin2Arms> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms_upcastFromVoid_314(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileVetLin2Arms> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstacleSDFFactorPose2MobileVetLin2Arms>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms_constructor_315(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileVetLin2Arms> Shared;

  size_t posekey = unwrap< size_t >(in[0]);
  gpmp2::Pose2MobileVetLin2ArmsModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileVetLin2ArmsModel >(in[1], "ptr_gpmp2Pose2MobileVetLin2ArmsModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[2], "ptr_gpmp2SignedDistanceField");
  double cost_sigma = unwrap< double >(in[3]);
  double epsilon = unwrap< double >(in[4]);
  Shared *self = new Shared(new gpmp2::ObstacleSDFFactorPose2MobileVetLin2Arms(posekey,marm,sdf,cost_sigma,epsilon));
  collector_gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms_deconstructor_316(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileVetLin2Arms> Shared;
  checkArguments("delete_gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms::iterator item;
  item = collector_gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms.find(self);
  if(item != collector_gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms.end()) {
    delete self;
    collector_gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms.erase(item);
  }
}

void gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms_evaluateError_317(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileVetLin2Arms> Shared;
  checkArguments("evaluateError",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ObstacleSDFFactorPose2MobileVetLin2Arms>(in[0], "ptr_gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms");
  gpmp2::Pose2Vector& pose = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  out[0] = wrap< Vector >(obj->evaluateError(pose));
}

void gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms_collectorInsertAndMakeBase_318(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileVetLin2Arms> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms_upcastFromVoid_319(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileVetLin2Arms> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::ObstacleSDFFactorGPPose2MobileVetLin2Arms>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms_constructor_320(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileVetLin2Arms> Shared;

  size_t pose1key = unwrap< size_t >(in[0]);
  size_t vel1key = unwrap< size_t >(in[1]);
  size_t pose2key = unwrap< size_t >(in[2]);
  size_t vel2key = unwrap< size_t >(in[3]);
  gpmp2::Pose2MobileVetLin2ArmsModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileVetLin2ArmsModel >(in[4], "ptr_gpmp2Pose2MobileVetLin2ArmsModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[5], "ptr_gpmp2SignedDistanceField");
  double cost_sigma = unwrap< double >(in[6]);
  double epsilon = unwrap< double >(in[7]);
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[8], "ptr_gtsamnoiseModelBase");
  double delta_t = unwrap< double >(in[9]);
  double tau = unwrap< double >(in[10]);
  Shared *self = new Shared(new gpmp2::ObstacleSDFFactorGPPose2MobileVetLin2Arms(pose1key,vel1key,pose2key,vel2key,marm,sdf,cost_sigma,epsilon,Qc_model,delta_t,tau));
  collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms_deconstructor_321(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ObstacleSDFFactorGPPose2MobileVetLin2Arms> Shared;
  checkArguments("delete_gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms::iterator item;
  item = collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms.find(self);
  if(item != collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms.end()) {
    delete self;
    collector_gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms.erase(item);
  }
}

void gpmp2SelfCollisionArm_collectorInsertAndMakeBase_322(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::SelfCollisionArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2SelfCollisionArm.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2SelfCollisionArm_upcastFromVoid_323(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::SelfCollisionArm> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::SelfCollisionArm>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2SelfCollisionArm_constructor_324(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::SelfCollisionArm> Shared;

  size_t poseKey = unwrap< size_t >(in[0]);
  gpmp2::ArmModel& arm = *unwrap_shared_ptr< gpmp2::ArmModel >(in[1], "ptr_gpmp2ArmModel");
  Matrix data = unwrap< Matrix >(in[2]);
  Shared *self = new Shared(new gpmp2::SelfCollisionArm(poseKey,arm,data));
  collector_gpmp2SelfCollisionArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2SelfCollisionArm_deconstructor_325(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::SelfCollisionArm> Shared;
  checkArguments("delete_gpmp2SelfCollisionArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2SelfCollisionArm::iterator item;
  item = collector_gpmp2SelfCollisionArm.find(self);
  if(item != collector_gpmp2SelfCollisionArm.end()) {
    delete self;
    collector_gpmp2SelfCollisionArm.erase(item);
  }
}

void gpmp2SelfCollisionArm_evaluateError_326(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::SelfCollisionArm> Shared;
  checkArguments("evaluateError",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::SelfCollisionArm>(in[0], "ptr_gpmp2SelfCollisionArm");
  Vector pose = unwrap< Vector >(in[1]);
  out[0] = wrap< Vector >(obj->evaluateError(pose));
}

void gpmp2TrajOptimizerSetting_collectorInsertAndMakeBase_327(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2TrajOptimizerSetting.insert(self);
}

void gpmp2TrajOptimizerSetting_constructor_328(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;

  size_t dof = unwrap< size_t >(in[0]);
  Shared *self = new Shared(new gpmp2::TrajOptimizerSetting(dof));
  collector_gpmp2TrajOptimizerSetting.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2TrajOptimizerSetting_deconstructor_329(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("delete_gpmp2TrajOptimizerSetting",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2TrajOptimizerSetting::iterator item;
  item = collector_gpmp2TrajOptimizerSetting.find(self);
  if(item != collector_gpmp2TrajOptimizerSetting.end()) {
    delete self;
    collector_gpmp2TrajOptimizerSetting.erase(item);
  }
}

void gpmp2TrajOptimizerSetting_setDogleg_330(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("setDogleg",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  obj->setDogleg();
}

void gpmp2TrajOptimizerSetting_setGaussNewton_331(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("setGaussNewton",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  obj->setGaussNewton();
}

void gpmp2TrajOptimizerSetting_setLM_332(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("setLM",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  obj->setLM();
}

void gpmp2TrajOptimizerSetting_setOptimizationNoIncrase_333(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("setOptimizationNoIncrase",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  bool flag = unwrap< bool >(in[1]);
  obj->setOptimizationNoIncrase(flag);
}

void gpmp2TrajOptimizerSetting_setVerbosityError_334(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("setVerbosityError",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  obj->setVerbosityError();
}

void gpmp2TrajOptimizerSetting_setVerbosityNone_335(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("setVerbosityNone",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  obj->setVerbosityNone();
}

void gpmp2TrajOptimizerSetting_set_Qc_model_336(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_Qc_model",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  Matrix Qc = unwrap< Matrix >(in[1]);
  obj->set_Qc_model(Qc);
}

void gpmp2TrajOptimizerSetting_set_conf_prior_model_337(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_conf_prior_model",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  double sigma = unwrap< double >(in[1]);
  obj->set_conf_prior_model(sigma);
}

void gpmp2TrajOptimizerSetting_set_cost_sigma_338(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_cost_sigma",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  double sigma = unwrap< double >(in[1]);
  obj->set_cost_sigma(sigma);
}

void gpmp2TrajOptimizerSetting_set_epsilon_339(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_epsilon",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  double eps = unwrap< double >(in[1]);
  obj->set_epsilon(eps);
}

void gpmp2TrajOptimizerSetting_set_flag_pos_limit_340(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_flag_pos_limit",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  bool flag = unwrap< bool >(in[1]);
  obj->set_flag_pos_limit(flag);
}

void gpmp2TrajOptimizerSetting_set_flag_vel_limit_341(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_flag_vel_limit",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  bool flag = unwrap< bool >(in[1]);
  obj->set_flag_vel_limit(flag);
}

void gpmp2TrajOptimizerSetting_set_joint_pos_limits_down_342(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_joint_pos_limits_down",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  Vector v = unwrap< Vector >(in[1]);
  obj->set_joint_pos_limits_down(v);
}

void gpmp2TrajOptimizerSetting_set_joint_pos_limits_up_343(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_joint_pos_limits_up",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  Vector v = unwrap< Vector >(in[1]);
  obj->set_joint_pos_limits_up(v);
}

void gpmp2TrajOptimizerSetting_set_max_iter_344(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_max_iter",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  size_t iter = unwrap< size_t >(in[1]);
  obj->set_max_iter(iter);
}

void gpmp2TrajOptimizerSetting_set_obs_check_inter_345(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_obs_check_inter",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  size_t inter = unwrap< size_t >(in[1]);
  obj->set_obs_check_inter(inter);
}

void gpmp2TrajOptimizerSetting_set_pos_limit_model_346(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_pos_limit_model",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  Vector v = unwrap< Vector >(in[1]);
  obj->set_pos_limit_model(v);
}

void gpmp2TrajOptimizerSetting_set_pos_limit_thresh_347(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_pos_limit_thresh",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  Vector v = unwrap< Vector >(in[1]);
  obj->set_pos_limit_thresh(v);
}

void gpmp2TrajOptimizerSetting_set_rel_thresh_348(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_rel_thresh",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  double thresh = unwrap< double >(in[1]);
  obj->set_rel_thresh(thresh);
}

void gpmp2TrajOptimizerSetting_set_total_step_349(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_total_step",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  size_t step = unwrap< size_t >(in[1]);
  obj->set_total_step(step);
}

void gpmp2TrajOptimizerSetting_set_total_time_350(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_total_time",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  double time = unwrap< double >(in[1]);
  obj->set_total_time(time);
}

void gpmp2TrajOptimizerSetting_set_vel_limit_model_351(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_vel_limit_model",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  Vector v = unwrap< Vector >(in[1]);
  obj->set_vel_limit_model(v);
}

void gpmp2TrajOptimizerSetting_set_vel_limit_thresh_352(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_vel_limit_thresh",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  Vector v = unwrap< Vector >(in[1]);
  obj->set_vel_limit_thresh(v);
}

void gpmp2TrajOptimizerSetting_set_vel_limits_353(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_vel_limits",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  Vector v = unwrap< Vector >(in[1]);
  obj->set_vel_limits(v);
}

void gpmp2TrajOptimizerSetting_set_vel_prior_model_354(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::TrajOptimizerSetting> Shared;
  checkArguments("set_vel_prior_model",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::TrajOptimizerSetting>(in[0], "ptr_gpmp2TrajOptimizerSetting");
  double sigma = unwrap< double >(in[1]);
  obj->set_vel_prior_model(sigma);
}

void gpmp2ISAM2TrajOptimizer2DArm_collectorInsertAndMakeBase_355(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ISAM2TrajOptimizer2DArm.insert(self);
}

void gpmp2ISAM2TrajOptimizer2DArm_constructor_356(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm> Shared;

  gpmp2::ArmModel& arm = *unwrap_shared_ptr< gpmp2::ArmModel >(in[0], "ptr_gpmp2ArmModel");
  gpmp2::PlanarSDF& sdf = *unwrap_shared_ptr< gpmp2::PlanarSDF >(in[1], "ptr_gpmp2PlanarSDF");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[2], "ptr_gpmp2TrajOptimizerSetting");
  Shared *self = new Shared(new gpmp2::ISAM2TrajOptimizer2DArm(arm,sdf,setting));
  collector_gpmp2ISAM2TrajOptimizer2DArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2ISAM2TrajOptimizer2DArm_deconstructor_357(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm> Shared;
  checkArguments("delete_gpmp2ISAM2TrajOptimizer2DArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ISAM2TrajOptimizer2DArm::iterator item;
  item = collector_gpmp2ISAM2TrajOptimizer2DArm.find(self);
  if(item != collector_gpmp2ISAM2TrajOptimizer2DArm.end()) {
    delete self;
    collector_gpmp2ISAM2TrajOptimizer2DArm.erase(item);
  }
}

void gpmp2ISAM2TrajOptimizer2DArm_addPoseEstimate_358(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm> Shared;
  checkArguments("addPoseEstimate",nargout,nargin-1,3);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer2DArm");
  size_t state_idx = unwrap< size_t >(in[1]);
  Vector pose = unwrap< Vector >(in[2]);
  Matrix pose_cov = unwrap< Matrix >(in[3]);
  obj->addPoseEstimate(state_idx,pose,pose_cov);
}

void gpmp2ISAM2TrajOptimizer2DArm_addStateEstimate_359(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm> Shared;
  checkArguments("addStateEstimate",nargout,nargin-1,5);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer2DArm");
  size_t state_idx = unwrap< size_t >(in[1]);
  Vector pose = unwrap< Vector >(in[2]);
  Matrix pose_cov = unwrap< Matrix >(in[3]);
  Vector vel = unwrap< Vector >(in[4]);
  Matrix vel_cov = unwrap< Matrix >(in[5]);
  obj->addStateEstimate(state_idx,pose,pose_cov,vel,vel_cov);
}

void gpmp2ISAM2TrajOptimizer2DArm_changeGoalConfigAndVel_360(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm> Shared;
  checkArguments("changeGoalConfigAndVel",nargout,nargin-1,2);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer2DArm");
  Vector goal_conf = unwrap< Vector >(in[1]);
  Vector goal_vel = unwrap< Vector >(in[2]);
  obj->changeGoalConfigAndVel(goal_conf,goal_vel);
}

void gpmp2ISAM2TrajOptimizer2DArm_fixConfigAndVel_361(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm> Shared;
  checkArguments("fixConfigAndVel",nargout,nargin-1,3);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer2DArm");
  size_t state_idx = unwrap< size_t >(in[1]);
  Vector conf_fix = unwrap< Vector >(in[2]);
  Vector vel_fix = unwrap< Vector >(in[3]);
  obj->fixConfigAndVel(state_idx,conf_fix,vel_fix);
}

void gpmp2ISAM2TrajOptimizer2DArm_initFactorGraph_362(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm> Shared;
  checkArguments("initFactorGraph",nargout,nargin-1,4);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer2DArm");
  Vector start_conf = unwrap< Vector >(in[1]);
  Vector start_vel = unwrap< Vector >(in[2]);
  Vector goal_conf = unwrap< Vector >(in[3]);
  Vector goal_vel = unwrap< Vector >(in[4]);
  obj->initFactorGraph(start_conf,start_vel,goal_conf,goal_vel);
}

void gpmp2ISAM2TrajOptimizer2DArm_initValues_363(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm> Shared;
  checkArguments("initValues",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer2DArm");
  gtsam::Values& init_values = *unwrap_shared_ptr< gtsam::Values >(in[1], "ptr_gtsamValues");
  obj->initValues(init_values);
}

void gpmp2ISAM2TrajOptimizer2DArm_removeGoalConfigAndVel_364(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm> Shared;
  checkArguments("removeGoalConfigAndVel",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer2DArm");
  obj->removeGoalConfigAndVel();
}

void gpmp2ISAM2TrajOptimizer2DArm_update_365(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm> Shared;
  checkArguments("update",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer2DArm");
  obj->update();
}

void gpmp2ISAM2TrajOptimizer2DArm_values_366(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm> Shared;
  checkArguments("values",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer2DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer2DArm");
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(obj->values())),"gtsam.Values", false);
}

void gpmp2ISAM2TrajOptimizer3DArm_collectorInsertAndMakeBase_367(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ISAM2TrajOptimizer3DArm.insert(self);
}

void gpmp2ISAM2TrajOptimizer3DArm_constructor_368(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm> Shared;

  gpmp2::ArmModel& arm = *unwrap_shared_ptr< gpmp2::ArmModel >(in[0], "ptr_gpmp2ArmModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[1], "ptr_gpmp2SignedDistanceField");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[2], "ptr_gpmp2TrajOptimizerSetting");
  Shared *self = new Shared(new gpmp2::ISAM2TrajOptimizer3DArm(arm,sdf,setting));
  collector_gpmp2ISAM2TrajOptimizer3DArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2ISAM2TrajOptimizer3DArm_deconstructor_369(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm> Shared;
  checkArguments("delete_gpmp2ISAM2TrajOptimizer3DArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ISAM2TrajOptimizer3DArm::iterator item;
  item = collector_gpmp2ISAM2TrajOptimizer3DArm.find(self);
  if(item != collector_gpmp2ISAM2TrajOptimizer3DArm.end()) {
    delete self;
    collector_gpmp2ISAM2TrajOptimizer3DArm.erase(item);
  }
}

void gpmp2ISAM2TrajOptimizer3DArm_addPoseEstimate_370(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm> Shared;
  checkArguments("addPoseEstimate",nargout,nargin-1,3);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer3DArm");
  size_t state_idx = unwrap< size_t >(in[1]);
  Vector pose = unwrap< Vector >(in[2]);
  Matrix pose_cov = unwrap< Matrix >(in[3]);
  obj->addPoseEstimate(state_idx,pose,pose_cov);
}

void gpmp2ISAM2TrajOptimizer3DArm_addStateEstimate_371(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm> Shared;
  checkArguments("addStateEstimate",nargout,nargin-1,5);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer3DArm");
  size_t state_idx = unwrap< size_t >(in[1]);
  Vector pose = unwrap< Vector >(in[2]);
  Matrix pose_cov = unwrap< Matrix >(in[3]);
  Vector vel = unwrap< Vector >(in[4]);
  Matrix vel_cov = unwrap< Matrix >(in[5]);
  obj->addStateEstimate(state_idx,pose,pose_cov,vel,vel_cov);
}

void gpmp2ISAM2TrajOptimizer3DArm_changeGoalConfigAndVel_372(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm> Shared;
  checkArguments("changeGoalConfigAndVel",nargout,nargin-1,2);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer3DArm");
  Vector goal_conf = unwrap< Vector >(in[1]);
  Vector goal_vel = unwrap< Vector >(in[2]);
  obj->changeGoalConfigAndVel(goal_conf,goal_vel);
}

void gpmp2ISAM2TrajOptimizer3DArm_fixConfigAndVel_373(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm> Shared;
  checkArguments("fixConfigAndVel",nargout,nargin-1,3);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer3DArm");
  size_t state_idx = unwrap< size_t >(in[1]);
  Vector conf_fix = unwrap< Vector >(in[2]);
  Vector vel_fix = unwrap< Vector >(in[3]);
  obj->fixConfigAndVel(state_idx,conf_fix,vel_fix);
}

void gpmp2ISAM2TrajOptimizer3DArm_initFactorGraph_374(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm> Shared;
  checkArguments("initFactorGraph",nargout,nargin-1,4);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer3DArm");
  Vector start_conf = unwrap< Vector >(in[1]);
  Vector start_vel = unwrap< Vector >(in[2]);
  Vector goal_conf = unwrap< Vector >(in[3]);
  Vector goal_vel = unwrap< Vector >(in[4]);
  obj->initFactorGraph(start_conf,start_vel,goal_conf,goal_vel);
}

void gpmp2ISAM2TrajOptimizer3DArm_initValues_375(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm> Shared;
  checkArguments("initValues",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer3DArm");
  gtsam::Values& init_values = *unwrap_shared_ptr< gtsam::Values >(in[1], "ptr_gtsamValues");
  obj->initValues(init_values);
}

void gpmp2ISAM2TrajOptimizer3DArm_removeGoalConfigAndVel_376(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm> Shared;
  checkArguments("removeGoalConfigAndVel",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer3DArm");
  obj->removeGoalConfigAndVel();
}

void gpmp2ISAM2TrajOptimizer3DArm_update_377(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm> Shared;
  checkArguments("update",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer3DArm");
  obj->update();
}

void gpmp2ISAM2TrajOptimizer3DArm_values_378(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm> Shared;
  checkArguments("values",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizer3DArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizer3DArm");
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(obj->values())),"gtsam.Values", false);
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm2D_collectorInsertAndMakeBase_379(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ISAM2TrajOptimizerPose2MobileArm2D.insert(self);
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm2D_constructor_380(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D> Shared;

  gpmp2::Pose2MobileArmModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileArmModel >(in[0], "ptr_gpmp2Pose2MobileArmModel");
  gpmp2::PlanarSDF& sdf = *unwrap_shared_ptr< gpmp2::PlanarSDF >(in[1], "ptr_gpmp2PlanarSDF");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[2], "ptr_gpmp2TrajOptimizerSetting");
  Shared *self = new Shared(new gpmp2::ISAM2TrajOptimizerPose2MobileArm2D(marm,sdf,setting));
  collector_gpmp2ISAM2TrajOptimizerPose2MobileArm2D.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm2D_deconstructor_381(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D> Shared;
  checkArguments("delete_gpmp2ISAM2TrajOptimizerPose2MobileArm2D",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ISAM2TrajOptimizerPose2MobileArm2D::iterator item;
  item = collector_gpmp2ISAM2TrajOptimizerPose2MobileArm2D.find(self);
  if(item != collector_gpmp2ISAM2TrajOptimizerPose2MobileArm2D.end()) {
    delete self;
    collector_gpmp2ISAM2TrajOptimizerPose2MobileArm2D.erase(item);
  }
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm2D_addPoseEstimate_382(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D> Shared;
  checkArguments("addPoseEstimate",nargout,nargin-1,3);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm2D");
  size_t state_idx = unwrap< size_t >(in[1]);
  gpmp2::Pose2Vector& pose = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[2], "ptr_gpmp2Pose2Vector");
  Matrix pose_cov = unwrap< Matrix >(in[3]);
  obj->addPoseEstimate(state_idx,pose,pose_cov);
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm2D_addStateEstimate_383(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D> Shared;
  checkArguments("addStateEstimate",nargout,nargin-1,5);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm2D");
  size_t state_idx = unwrap< size_t >(in[1]);
  gpmp2::Pose2Vector& pose = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[2], "ptr_gpmp2Pose2Vector");
  Matrix pose_cov = unwrap< Matrix >(in[3]);
  Vector vel = unwrap< Vector >(in[4]);
  Matrix vel_cov = unwrap< Matrix >(in[5]);
  obj->addStateEstimate(state_idx,pose,pose_cov,vel,vel_cov);
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm2D_changeGoalConfigAndVel_384(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D> Shared;
  checkArguments("changeGoalConfigAndVel",nargout,nargin-1,2);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm2D");
  gpmp2::Pose2Vector& goal_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  Vector goal_vel = unwrap< Vector >(in[2]);
  obj->changeGoalConfigAndVel(goal_conf,goal_vel);
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm2D_fixConfigAndVel_385(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D> Shared;
  checkArguments("fixConfigAndVel",nargout,nargin-1,3);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm2D");
  size_t state_idx = unwrap< size_t >(in[1]);
  gpmp2::Pose2Vector& conf_fix = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[2], "ptr_gpmp2Pose2Vector");
  Vector vel_fix = unwrap< Vector >(in[3]);
  obj->fixConfigAndVel(state_idx,conf_fix,vel_fix);
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm2D_initFactorGraph_386(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D> Shared;
  checkArguments("initFactorGraph",nargout,nargin-1,4);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm2D");
  gpmp2::Pose2Vector& start_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  Vector start_vel = unwrap< Vector >(in[2]);
  gpmp2::Pose2Vector& goal_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[3], "ptr_gpmp2Pose2Vector");
  Vector goal_vel = unwrap< Vector >(in[4]);
  obj->initFactorGraph(start_conf,start_vel,goal_conf,goal_vel);
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm2D_initValues_387(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D> Shared;
  checkArguments("initValues",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm2D");
  gtsam::Values& init_values = *unwrap_shared_ptr< gtsam::Values >(in[1], "ptr_gtsamValues");
  obj->initValues(init_values);
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm2D_removeGoalConfigAndVel_388(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D> Shared;
  checkArguments("removeGoalConfigAndVel",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm2D");
  obj->removeGoalConfigAndVel();
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm2D_update_389(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D> Shared;
  checkArguments("update",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm2D");
  obj->update();
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm2D_values_390(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D> Shared;
  checkArguments("values",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm2D>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm2D");
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(obj->values())),"gtsam.Values", false);
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm_collectorInsertAndMakeBase_391(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ISAM2TrajOptimizerPose2MobileArm.insert(self);
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm_constructor_392(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm> Shared;

  gpmp2::Pose2MobileArmModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileArmModel >(in[0], "ptr_gpmp2Pose2MobileArmModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[1], "ptr_gpmp2SignedDistanceField");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[2], "ptr_gpmp2TrajOptimizerSetting");
  Shared *self = new Shared(new gpmp2::ISAM2TrajOptimizerPose2MobileArm(marm,sdf,setting));
  collector_gpmp2ISAM2TrajOptimizerPose2MobileArm.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm_deconstructor_393(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm> Shared;
  checkArguments("delete_gpmp2ISAM2TrajOptimizerPose2MobileArm",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ISAM2TrajOptimizerPose2MobileArm::iterator item;
  item = collector_gpmp2ISAM2TrajOptimizerPose2MobileArm.find(self);
  if(item != collector_gpmp2ISAM2TrajOptimizerPose2MobileArm.end()) {
    delete self;
    collector_gpmp2ISAM2TrajOptimizerPose2MobileArm.erase(item);
  }
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm_addPoseEstimate_394(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm> Shared;
  checkArguments("addPoseEstimate",nargout,nargin-1,3);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm");
  size_t state_idx = unwrap< size_t >(in[1]);
  gpmp2::Pose2Vector& pose = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[2], "ptr_gpmp2Pose2Vector");
  Matrix pose_cov = unwrap< Matrix >(in[3]);
  obj->addPoseEstimate(state_idx,pose,pose_cov);
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm_addStateEstimate_395(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm> Shared;
  checkArguments("addStateEstimate",nargout,nargin-1,5);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm");
  size_t state_idx = unwrap< size_t >(in[1]);
  gpmp2::Pose2Vector& pose = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[2], "ptr_gpmp2Pose2Vector");
  Matrix pose_cov = unwrap< Matrix >(in[3]);
  Vector vel = unwrap< Vector >(in[4]);
  Matrix vel_cov = unwrap< Matrix >(in[5]);
  obj->addStateEstimate(state_idx,pose,pose_cov,vel,vel_cov);
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm_changeGoalConfigAndVel_396(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm> Shared;
  checkArguments("changeGoalConfigAndVel",nargout,nargin-1,2);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm");
  gpmp2::Pose2Vector& goal_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  Vector goal_vel = unwrap< Vector >(in[2]);
  obj->changeGoalConfigAndVel(goal_conf,goal_vel);
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm_fixConfigAndVel_397(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm> Shared;
  checkArguments("fixConfigAndVel",nargout,nargin-1,3);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm");
  size_t state_idx = unwrap< size_t >(in[1]);
  gpmp2::Pose2Vector& conf_fix = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[2], "ptr_gpmp2Pose2Vector");
  Vector vel_fix = unwrap< Vector >(in[3]);
  obj->fixConfigAndVel(state_idx,conf_fix,vel_fix);
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm_initFactorGraph_398(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm> Shared;
  checkArguments("initFactorGraph",nargout,nargin-1,4);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm");
  gpmp2::Pose2Vector& start_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  Vector start_vel = unwrap< Vector >(in[2]);
  gpmp2::Pose2Vector& goal_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[3], "ptr_gpmp2Pose2Vector");
  Vector goal_vel = unwrap< Vector >(in[4]);
  obj->initFactorGraph(start_conf,start_vel,goal_conf,goal_vel);
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm_initValues_399(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm> Shared;
  checkArguments("initValues",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm");
  gtsam::Values& init_values = *unwrap_shared_ptr< gtsam::Values >(in[1], "ptr_gtsamValues");
  obj->initValues(init_values);
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm_removeGoalConfigAndVel_400(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm> Shared;
  checkArguments("removeGoalConfigAndVel",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm");
  obj->removeGoalConfigAndVel();
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm_update_401(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm> Shared;
  checkArguments("update",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm");
  obj->update();
}

void gpmp2ISAM2TrajOptimizerPose2MobileArm_values_402(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm> Shared;
  checkArguments("values",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileArm>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileArm");
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(obj->values())),"gtsam.Values", false);
}

void gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_collectorInsertAndMakeBase_403(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms.insert(self);
}

void gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_constructor_404(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms> Shared;

  gpmp2::Pose2MobileVetLin2ArmsModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileVetLin2ArmsModel >(in[0], "ptr_gpmp2Pose2MobileVetLin2ArmsModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[1], "ptr_gpmp2SignedDistanceField");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[2], "ptr_gpmp2TrajOptimizerSetting");
  Shared *self = new Shared(new gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms(marm,sdf,setting));
  collector_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_deconstructor_405(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms> Shared;
  checkArguments("delete_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms::iterator item;
  item = collector_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms.find(self);
  if(item != collector_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms.end()) {
    delete self;
    collector_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms.erase(item);
  }
}

void gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_addPoseEstimate_406(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms> Shared;
  checkArguments("addPoseEstimate",nargout,nargin-1,3);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms");
  size_t state_idx = unwrap< size_t >(in[1]);
  gpmp2::Pose2Vector& pose = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[2], "ptr_gpmp2Pose2Vector");
  Matrix pose_cov = unwrap< Matrix >(in[3]);
  obj->addPoseEstimate(state_idx,pose,pose_cov);
}

void gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_addStateEstimate_407(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms> Shared;
  checkArguments("addStateEstimate",nargout,nargin-1,5);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms");
  size_t state_idx = unwrap< size_t >(in[1]);
  gpmp2::Pose2Vector& pose = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[2], "ptr_gpmp2Pose2Vector");
  Matrix pose_cov = unwrap< Matrix >(in[3]);
  Vector vel = unwrap< Vector >(in[4]);
  Matrix vel_cov = unwrap< Matrix >(in[5]);
  obj->addStateEstimate(state_idx,pose,pose_cov,vel,vel_cov);
}

void gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_changeGoalConfigAndVel_408(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms> Shared;
  checkArguments("changeGoalConfigAndVel",nargout,nargin-1,2);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms");
  gpmp2::Pose2Vector& goal_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  Vector goal_vel = unwrap< Vector >(in[2]);
  obj->changeGoalConfigAndVel(goal_conf,goal_vel);
}

void gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_fixConfigAndVel_409(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms> Shared;
  checkArguments("fixConfigAndVel",nargout,nargin-1,3);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms");
  size_t state_idx = unwrap< size_t >(in[1]);
  gpmp2::Pose2Vector& conf_fix = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[2], "ptr_gpmp2Pose2Vector");
  Vector vel_fix = unwrap< Vector >(in[3]);
  obj->fixConfigAndVel(state_idx,conf_fix,vel_fix);
}

void gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_initFactorGraph_410(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms> Shared;
  checkArguments("initFactorGraph",nargout,nargin-1,4);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms");
  gpmp2::Pose2Vector& start_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  Vector start_vel = unwrap< Vector >(in[2]);
  gpmp2::Pose2Vector& goal_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[3], "ptr_gpmp2Pose2Vector");
  Vector goal_vel = unwrap< Vector >(in[4]);
  obj->initFactorGraph(start_conf,start_vel,goal_conf,goal_vel);
}

void gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_initValues_411(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms> Shared;
  checkArguments("initValues",nargout,nargin-1,1);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms");
  gtsam::Values& init_values = *unwrap_shared_ptr< gtsam::Values >(in[1], "ptr_gtsamValues");
  obj->initValues(init_values);
}

void gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_removeGoalConfigAndVel_412(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms> Shared;
  checkArguments("removeGoalConfigAndVel",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms");
  obj->removeGoalConfigAndVel();
}

void gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_update_413(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms> Shared;
  checkArguments("update",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms");
  obj->update();
}

void gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_values_414(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  typedef boost::shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms> Shared;
  checkArguments("values",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gpmp2::ISAM2TrajOptimizerPose2MobileVetLin2Arms>(in[0], "ptr_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms");
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(obj->values())),"gtsam.Values", false);
}

void gpmp2PriorFactorPose2Vector_collectorInsertAndMakeBase_415(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::PriorFactorPose2Vector> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gpmp2PriorFactorPose2Vector.insert(self);

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void gpmp2PriorFactorPose2Vector_upcastFromVoid_416(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::PriorFactorPose2Vector> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<gpmp2::PriorFactorPose2Vector>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void gpmp2PriorFactorPose2Vector_constructor_417(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gpmp2::PriorFactorPose2Vector> Shared;

  size_t poseKey = unwrap< size_t >(in[0]);
  gpmp2::Pose2Vector& value = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  boost::shared_ptr<gtsam::noiseModel::Base> model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[2], "ptr_gtsamnoiseModelBase");
  Shared *self = new Shared(new gpmp2::PriorFactorPose2Vector(poseKey,value,model));
  collector_gpmp2PriorFactorPose2Vector.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<gtsam::NoiseModelFactor> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void gpmp2PriorFactorPose2Vector_deconstructor_418(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::PriorFactorPose2Vector> Shared;
  checkArguments("delete_gpmp2PriorFactorPose2Vector",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gpmp2PriorFactorPose2Vector::iterator item;
  item = collector_gpmp2PriorFactorPose2Vector.find(self);
  if(item != collector_gpmp2PriorFactorPose2Vector.end()) {
    delete self;
    collector_gpmp2PriorFactorPose2Vector.erase(item);
  }
}

void gpmp2BatchTrajOptimize2DArm_419(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  checkArguments("gpmp2BatchTrajOptimize2DArm",nargout,nargin,8);
  gpmp2::ArmModel& arm = *unwrap_shared_ptr< gpmp2::ArmModel >(in[0], "ptr_gpmp2ArmModel");
  gpmp2::PlanarSDF& sdf = *unwrap_shared_ptr< gpmp2::PlanarSDF >(in[1], "ptr_gpmp2PlanarSDF");
  Vector start_conf = unwrap< Vector >(in[2]);
  Vector start_vel = unwrap< Vector >(in[3]);
  Vector end_conf = unwrap< Vector >(in[4]);
  Vector end_vel = unwrap< Vector >(in[5]);
  gtsam::Values& init_values = *unwrap_shared_ptr< gtsam::Values >(in[6], "ptr_gtsamValues");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[7], "ptr_gpmp2TrajOptimizerSetting");
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(gpmp2::BatchTrajOptimize2DArm(arm,sdf,start_conf,start_vel,end_conf,end_vel,init_values,setting))),"gtsam.Values", false);
}
void gpmp2BatchTrajOptimize3DArm_420(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  checkArguments("gpmp2BatchTrajOptimize3DArm",nargout,nargin,8);
  gpmp2::ArmModel& arm = *unwrap_shared_ptr< gpmp2::ArmModel >(in[0], "ptr_gpmp2ArmModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[1], "ptr_gpmp2SignedDistanceField");
  Vector start_conf = unwrap< Vector >(in[2]);
  Vector start_vel = unwrap< Vector >(in[3]);
  Vector end_conf = unwrap< Vector >(in[4]);
  Vector end_vel = unwrap< Vector >(in[5]);
  gtsam::Values& init_values = *unwrap_shared_ptr< gtsam::Values >(in[6], "ptr_gtsamValues");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[7], "ptr_gpmp2TrajOptimizerSetting");
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(gpmp2::BatchTrajOptimize3DArm(arm,sdf,start_conf,start_vel,end_conf,end_vel,init_values,setting))),"gtsam.Values", false);
}
void gpmp2BatchTrajOptimizePose2Mobile2Arms_421(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  checkArguments("gpmp2BatchTrajOptimizePose2Mobile2Arms",nargout,nargin,8);
  gpmp2::Pose2Mobile2ArmsModel& marm = *unwrap_shared_ptr< gpmp2::Pose2Mobile2ArmsModel >(in[0], "ptr_gpmp2Pose2Mobile2ArmsModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[1], "ptr_gpmp2SignedDistanceField");
  gpmp2::Pose2Vector& start_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[2], "ptr_gpmp2Pose2Vector");
  Vector start_vel = unwrap< Vector >(in[3]);
  gpmp2::Pose2Vector& end_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[4], "ptr_gpmp2Pose2Vector");
  Vector end_vel = unwrap< Vector >(in[5]);
  gtsam::Values& init_values = *unwrap_shared_ptr< gtsam::Values >(in[6], "ptr_gtsamValues");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[7], "ptr_gpmp2TrajOptimizerSetting");
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(gpmp2::BatchTrajOptimizePose2Mobile2Arms(marm,sdf,start_conf,start_vel,end_conf,end_vel,init_values,setting))),"gtsam.Values", false);
}
void gpmp2BatchTrajOptimizePose2MobileArm_422(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  checkArguments("gpmp2BatchTrajOptimizePose2MobileArm",nargout,nargin,8);
  gpmp2::Pose2MobileArmModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileArmModel >(in[0], "ptr_gpmp2Pose2MobileArmModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[1], "ptr_gpmp2SignedDistanceField");
  gpmp2::Pose2Vector& start_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[2], "ptr_gpmp2Pose2Vector");
  Vector start_vel = unwrap< Vector >(in[3]);
  gpmp2::Pose2Vector& end_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[4], "ptr_gpmp2Pose2Vector");
  Vector end_vel = unwrap< Vector >(in[5]);
  gtsam::Values& init_values = *unwrap_shared_ptr< gtsam::Values >(in[6], "ptr_gtsamValues");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[7], "ptr_gpmp2TrajOptimizerSetting");
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(gpmp2::BatchTrajOptimizePose2MobileArm(marm,sdf,start_conf,start_vel,end_conf,end_vel,init_values,setting))),"gtsam.Values", false);
}
void gpmp2BatchTrajOptimizePose2MobileArm2D_423(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  checkArguments("gpmp2BatchTrajOptimizePose2MobileArm2D",nargout,nargin,8);
  gpmp2::Pose2MobileArmModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileArmModel >(in[0], "ptr_gpmp2Pose2MobileArmModel");
  gpmp2::PlanarSDF& sdf = *unwrap_shared_ptr< gpmp2::PlanarSDF >(in[1], "ptr_gpmp2PlanarSDF");
  gpmp2::Pose2Vector& start_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[2], "ptr_gpmp2Pose2Vector");
  Vector start_vel = unwrap< Vector >(in[3]);
  gpmp2::Pose2Vector& end_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[4], "ptr_gpmp2Pose2Vector");
  Vector end_vel = unwrap< Vector >(in[5]);
  gtsam::Values& init_values = *unwrap_shared_ptr< gtsam::Values >(in[6], "ptr_gtsamValues");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[7], "ptr_gpmp2TrajOptimizerSetting");
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(gpmp2::BatchTrajOptimizePose2MobileArm2D(marm,sdf,start_conf,start_vel,end_conf,end_vel,init_values,setting))),"gtsam.Values", false);
}
void gpmp2BatchTrajOptimizePose2MobileVetLin2Arms_424(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  checkArguments("gpmp2BatchTrajOptimizePose2MobileVetLin2Arms",nargout,nargin,8);
  gpmp2::Pose2MobileVetLin2ArmsModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileVetLin2ArmsModel >(in[0], "ptr_gpmp2Pose2MobileVetLin2ArmsModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[1], "ptr_gpmp2SignedDistanceField");
  gpmp2::Pose2Vector& start_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[2], "ptr_gpmp2Pose2Vector");
  Vector start_vel = unwrap< Vector >(in[3]);
  gpmp2::Pose2Vector& end_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[4], "ptr_gpmp2Pose2Vector");
  Vector end_vel = unwrap< Vector >(in[5]);
  gtsam::Values& init_values = *unwrap_shared_ptr< gtsam::Values >(in[6], "ptr_gtsamValues");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[7], "ptr_gpmp2TrajOptimizerSetting");
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(gpmp2::BatchTrajOptimizePose2MobileVetLin2Arms(marm,sdf,start_conf,start_vel,end_conf,end_vel,init_values,setting))),"gtsam.Values", false);
}
void gpmp2BatchTrajOptimizePose2MobileVetLinArm_425(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  checkArguments("gpmp2BatchTrajOptimizePose2MobileVetLinArm",nargout,nargin,8);
  gpmp2::Pose2MobileVetLinArmModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileVetLinArmModel >(in[0], "ptr_gpmp2Pose2MobileVetLinArmModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[1], "ptr_gpmp2SignedDistanceField");
  gpmp2::Pose2Vector& start_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[2], "ptr_gpmp2Pose2Vector");
  Vector start_vel = unwrap< Vector >(in[3]);
  gpmp2::Pose2Vector& end_conf = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[4], "ptr_gpmp2Pose2Vector");
  Vector end_vel = unwrap< Vector >(in[5]);
  gtsam::Values& init_values = *unwrap_shared_ptr< gtsam::Values >(in[6], "ptr_gtsamValues");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[7], "ptr_gpmp2TrajOptimizerSetting");
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(gpmp2::BatchTrajOptimizePose2MobileVetLinArm(marm,sdf,start_conf,start_vel,end_conf,end_vel,init_values,setting))),"gtsam.Values", false);
}
void gpmp2CollisionCost2DArm_426(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("gpmp2CollisionCost2DArm",nargout,nargin,4);
  gpmp2::ArmModel& arm = *unwrap_shared_ptr< gpmp2::ArmModel >(in[0], "ptr_gpmp2ArmModel");
  gpmp2::PlanarSDF& sdf = *unwrap_shared_ptr< gpmp2::PlanarSDF >(in[1], "ptr_gpmp2PlanarSDF");
  gtsam::Values& result = *unwrap_shared_ptr< gtsam::Values >(in[2], "ptr_gtsamValues");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[3], "ptr_gpmp2TrajOptimizerSetting");
  out[0] = wrap< double >(gpmp2::CollisionCost2DArm(arm,sdf,result,setting));
}
void gpmp2CollisionCost3DArm_427(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("gpmp2CollisionCost3DArm",nargout,nargin,4);
  gpmp2::ArmModel& arm = *unwrap_shared_ptr< gpmp2::ArmModel >(in[0], "ptr_gpmp2ArmModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[1], "ptr_gpmp2SignedDistanceField");
  gtsam::Values& result = *unwrap_shared_ptr< gtsam::Values >(in[2], "ptr_gtsamValues");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[3], "ptr_gpmp2TrajOptimizerSetting");
  out[0] = wrap< double >(gpmp2::CollisionCost3DArm(arm,sdf,result,setting));
}
void gpmp2CollisionCostPose2Mobile2Arms_428(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("gpmp2CollisionCostPose2Mobile2Arms",nargout,nargin,4);
  gpmp2::Pose2Mobile2ArmsModel& marm = *unwrap_shared_ptr< gpmp2::Pose2Mobile2ArmsModel >(in[0], "ptr_gpmp2Pose2Mobile2ArmsModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[1], "ptr_gpmp2SignedDistanceField");
  gtsam::Values& result = *unwrap_shared_ptr< gtsam::Values >(in[2], "ptr_gtsamValues");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[3], "ptr_gpmp2TrajOptimizerSetting");
  out[0] = wrap< double >(gpmp2::CollisionCostPose2Mobile2Arms(marm,sdf,result,setting));
}
void gpmp2CollisionCostPose2MobileArm_429(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("gpmp2CollisionCostPose2MobileArm",nargout,nargin,4);
  gpmp2::Pose2MobileArmModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileArmModel >(in[0], "ptr_gpmp2Pose2MobileArmModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[1], "ptr_gpmp2SignedDistanceField");
  gtsam::Values& result = *unwrap_shared_ptr< gtsam::Values >(in[2], "ptr_gtsamValues");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[3], "ptr_gpmp2TrajOptimizerSetting");
  out[0] = wrap< double >(gpmp2::CollisionCostPose2MobileArm(marm,sdf,result,setting));
}
void gpmp2CollisionCostPose2MobileArm2D_430(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("gpmp2CollisionCostPose2MobileArm2D",nargout,nargin,4);
  gpmp2::Pose2MobileArmModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileArmModel >(in[0], "ptr_gpmp2Pose2MobileArmModel");
  gpmp2::PlanarSDF& sdf = *unwrap_shared_ptr< gpmp2::PlanarSDF >(in[1], "ptr_gpmp2PlanarSDF");
  gtsam::Values& result = *unwrap_shared_ptr< gtsam::Values >(in[2], "ptr_gtsamValues");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[3], "ptr_gpmp2TrajOptimizerSetting");
  out[0] = wrap< double >(gpmp2::CollisionCostPose2MobileArm2D(marm,sdf,result,setting));
}
void gpmp2CollisionCostPose2MobileBase_431(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("gpmp2CollisionCostPose2MobileBase",nargout,nargin,4);
  gpmp2::Pose2MobileBaseModel& robot = *unwrap_shared_ptr< gpmp2::Pose2MobileBaseModel >(in[0], "ptr_gpmp2Pose2MobileBaseModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[1], "ptr_gpmp2SignedDistanceField");
  gtsam::Values& result = *unwrap_shared_ptr< gtsam::Values >(in[2], "ptr_gtsamValues");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[3], "ptr_gpmp2TrajOptimizerSetting");
  out[0] = wrap< double >(gpmp2::CollisionCostPose2MobileBase(robot,sdf,result,setting));
}
void gpmp2CollisionCostPose2MobileBase2D_432(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("gpmp2CollisionCostPose2MobileBase2D",nargout,nargin,4);
  gpmp2::Pose2MobileBaseModel& robot = *unwrap_shared_ptr< gpmp2::Pose2MobileBaseModel >(in[0], "ptr_gpmp2Pose2MobileBaseModel");
  gpmp2::PlanarSDF& sdf = *unwrap_shared_ptr< gpmp2::PlanarSDF >(in[1], "ptr_gpmp2PlanarSDF");
  gtsam::Values& result = *unwrap_shared_ptr< gtsam::Values >(in[2], "ptr_gtsamValues");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[3], "ptr_gpmp2TrajOptimizerSetting");
  out[0] = wrap< double >(gpmp2::CollisionCostPose2MobileBase2D(robot,sdf,result,setting));
}
void gpmp2CollisionCostPose2MobileVetLin2Arms_433(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("gpmp2CollisionCostPose2MobileVetLin2Arms",nargout,nargin,4);
  gpmp2::Pose2MobileVetLin2ArmsModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileVetLin2ArmsModel >(in[0], "ptr_gpmp2Pose2MobileVetLin2ArmsModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[1], "ptr_gpmp2SignedDistanceField");
  gtsam::Values& result = *unwrap_shared_ptr< gtsam::Values >(in[2], "ptr_gtsamValues");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[3], "ptr_gpmp2TrajOptimizerSetting");
  out[0] = wrap< double >(gpmp2::CollisionCostPose2MobileVetLin2Arms(marm,sdf,result,setting));
}
void gpmp2CollisionCostPose2MobileVetLinArm_434(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("gpmp2CollisionCostPose2MobileVetLinArm",nargout,nargin,4);
  gpmp2::Pose2MobileVetLinArmModel& marm = *unwrap_shared_ptr< gpmp2::Pose2MobileVetLinArmModel >(in[0], "ptr_gpmp2Pose2MobileVetLinArmModel");
  gpmp2::SignedDistanceField& sdf = *unwrap_shared_ptr< gpmp2::SignedDistanceField >(in[1], "ptr_gpmp2SignedDistanceField");
  gtsam::Values& result = *unwrap_shared_ptr< gtsam::Values >(in[2], "ptr_gtsamValues");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[3], "ptr_gpmp2TrajOptimizerSetting");
  out[0] = wrap< double >(gpmp2::CollisionCostPose2MobileVetLinArm(marm,sdf,result,setting));
}
void gpmp2atPose2VectorValues_435(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gpmp2::Pose2Vector> SharedPose2Vector;
  checkArguments("gpmp2atPose2VectorValues",nargout,nargin,2);
  size_t key = unwrap< size_t >(in[0]);
  gtsam::Values& values = *unwrap_shared_ptr< gtsam::Values >(in[1], "ptr_gtsamValues");
  out[0] = wrap_shared_ptr(SharedPose2Vector(new gpmp2::Pose2Vector(gpmp2::atPose2VectorValues(key,values))),"gpmp2.Pose2Vector", false);
}
void gpmp2initArmTrajStraightLine_436(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  checkArguments("gpmp2initArmTrajStraightLine",nargout,nargin,3);
  Vector init_conf = unwrap< Vector >(in[0]);
  Vector end_conf = unwrap< Vector >(in[1]);
  size_t total_step = unwrap< size_t >(in[2]);
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(gpmp2::initArmTrajStraightLine(init_conf,end_conf,total_step))),"gtsam.Values", false);
}
void gpmp2initPose2TrajStraightLine_437(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  checkArguments("gpmp2initPose2TrajStraightLine",nargout,nargin,3);
  gtsam::Pose2& init_pose = *unwrap_shared_ptr< gtsam::Pose2 >(in[0], "ptr_gtsamPose2");
  gtsam::Pose2& end_pose = *unwrap_shared_ptr< gtsam::Pose2 >(in[1], "ptr_gtsamPose2");
  size_t total_step = unwrap< size_t >(in[2]);
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(gpmp2::initPose2TrajStraightLine(init_pose,end_pose,total_step))),"gtsam.Values", false);
}
void gpmp2initPose2VectorTrajStraightLine_438(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  checkArguments("gpmp2initPose2VectorTrajStraightLine",nargout,nargin,5);
  gtsam::Pose2& init_pose = *unwrap_shared_ptr< gtsam::Pose2 >(in[0], "ptr_gtsamPose2");
  Vector init_conf = unwrap< Vector >(in[1]);
  gtsam::Pose2& end_pose = *unwrap_shared_ptr< gtsam::Pose2 >(in[2], "ptr_gtsamPose2");
  Vector end_conf = unwrap< Vector >(in[3]);
  size_t total_step = unwrap< size_t >(in[4]);
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(gpmp2::initPose2VectorTrajStraightLine(init_pose,init_conf,end_pose,end_conf,total_step))),"gtsam.Values", false);
}
void gpmp2insertPose2VectorInValues_439(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("gpmp2insertPose2VectorInValues",nargout,nargin,3);
  size_t key = unwrap< size_t >(in[0]);
  gpmp2::Pose2Vector& p = *unwrap_shared_ptr< gpmp2::Pose2Vector >(in[1], "ptr_gpmp2Pose2Vector");
  gtsam::Values& values = *unwrap_shared_ptr< gtsam::Values >(in[2], "ptr_gtsamValues");
gpmp2::insertPose2VectorInValues(key,p,values);
}
void gpmp2interpolateArmTraj_440(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  checkArguments("gpmp2interpolateArmTraj",nargout,nargin,4);
  gtsam::Values& opt_values = *unwrap_shared_ptr< gtsam::Values >(in[0], "ptr_gtsamValues");
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[1], "ptr_gtsamnoiseModelBase");
  double delta_t = unwrap< double >(in[2]);
  size_t inter_step = unwrap< size_t >(in[3]);
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(gpmp2::interpolateArmTraj(opt_values,Qc_model,delta_t,inter_step))),"gtsam.Values", false);
}
void gpmp2interpolateArmTraj_441(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  checkArguments("gpmp2interpolateArmTraj",nargout,nargin,6);
  gtsam::Values& opt_values = *unwrap_shared_ptr< gtsam::Values >(in[0], "ptr_gtsamValues");
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[1], "ptr_gtsamnoiseModelBase");
  double delta_t = unwrap< double >(in[2]);
  size_t inter_step = unwrap< size_t >(in[3]);
  size_t start_index = unwrap< size_t >(in[4]);
  size_t end_index = unwrap< size_t >(in[5]);
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(gpmp2::interpolateArmTraj(opt_values,Qc_model,delta_t,inter_step,start_index,end_index))),"gtsam.Values", false);
}
void gpmp2interpolatePose2MobileArmTraj_442(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  checkArguments("gpmp2interpolatePose2MobileArmTraj",nargout,nargin,6);
  gtsam::Values& opt_values = *unwrap_shared_ptr< gtsam::Values >(in[0], "ptr_gtsamValues");
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[1], "ptr_gtsamnoiseModelBase");
  double delta_t = unwrap< double >(in[2]);
  size_t inter_step = unwrap< size_t >(in[3]);
  size_t start_index = unwrap< size_t >(in[4]);
  size_t end_index = unwrap< size_t >(in[5]);
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(gpmp2::interpolatePose2MobileArmTraj(opt_values,Qc_model,delta_t,inter_step,start_index,end_index))),"gtsam.Values", false);
}
void gpmp2interpolatePose2Traj_443(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  checkArguments("gpmp2interpolatePose2Traj",nargout,nargin,6);
  gtsam::Values& opt_values = *unwrap_shared_ptr< gtsam::Values >(in[0], "ptr_gtsamValues");
  boost::shared_ptr<gtsam::noiseModel::Base> Qc_model = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[1], "ptr_gtsamnoiseModelBase");
  double delta_t = unwrap< double >(in[2]);
  size_t inter_step = unwrap< size_t >(in[3]);
  size_t start_index = unwrap< size_t >(in[4]);
  size_t end_index = unwrap< size_t >(in[5]);
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(gpmp2::interpolatePose2Traj(opt_values,Qc_model,delta_t,inter_step,start_index,end_index))),"gtsam.Values", false);
}
void gpmp2optimize_444(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Values> SharedValues;
  checkArguments("gpmp2optimize",nargout,nargin,4);
  gtsam::NonlinearFactorGraph& graph = *unwrap_shared_ptr< gtsam::NonlinearFactorGraph >(in[0], "ptr_gtsamNonlinearFactorGraph");
  gtsam::Values& init_values = *unwrap_shared_ptr< gtsam::Values >(in[1], "ptr_gtsamValues");
  gpmp2::TrajOptimizerSetting& setting = *unwrap_shared_ptr< gpmp2::TrajOptimizerSetting >(in[2], "ptr_gpmp2TrajOptimizerSetting");
  bool iter_no_increase = unwrap< bool >(in[3]);
  out[0] = wrap_shared_ptr(SharedValues(new gtsam::Values(gpmp2::optimize(graph,init_values,setting,iter_no_increase))),"gtsam.Values", false);
}

void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  _gpmp2_RTTIRegister();

  int id = unwrap<int>(in[0]);

  try {
    switch(id) {
    case 0:
      gpmp2Pose2Vector_collectorInsertAndMakeBase_0(nargout, out, nargin-1, in+1);
      break;
    case 1:
      gpmp2Pose2Vector_constructor_1(nargout, out, nargin-1, in+1);
      break;
    case 2:
      gpmp2Pose2Vector_constructor_2(nargout, out, nargin-1, in+1);
      break;
    case 3:
      gpmp2Pose2Vector_deconstructor_3(nargout, out, nargin-1, in+1);
      break;
    case 4:
      gpmp2Pose2Vector_configuration_4(nargout, out, nargin-1, in+1);
      break;
    case 5:
      gpmp2Pose2Vector_pose_5(nargout, out, nargin-1, in+1);
      break;
    case 6:
      gpmp2Pose2Vector_print_6(nargout, out, nargin-1, in+1);
      break;
    case 7:
      gpmp2GaussianProcessPriorLinear_collectorInsertAndMakeBase_7(nargout, out, nargin-1, in+1);
      break;
    case 8:
      gpmp2GaussianProcessPriorLinear_upcastFromVoid_8(nargout, out, nargin-1, in+1);
      break;
    case 9:
      gpmp2GaussianProcessPriorLinear_constructor_9(nargout, out, nargin-1, in+1);
      break;
    case 10:
      gpmp2GaussianProcessPriorLinear_deconstructor_10(nargout, out, nargin-1, in+1);
      break;
    case 11:
      gpmp2GaussianProcessPriorLinear_evaluateError_11(nargout, out, nargin-1, in+1);
      break;
    case 12:
      gpmp2GaussianProcessPriorLinear_string_serialize_12(nargout, out, nargin-1, in+1);
      break;
    case 13:
      gpmp2GaussianProcessPriorLinear_string_deserialize_13(nargout, out, nargin-1, in+1);
      break;
    case 14:
      gpmp2GaussianProcessPriorPose2_collectorInsertAndMakeBase_14(nargout, out, nargin-1, in+1);
      break;
    case 15:
      gpmp2GaussianProcessPriorPose2_upcastFromVoid_15(nargout, out, nargin-1, in+1);
      break;
    case 16:
      gpmp2GaussianProcessPriorPose2_constructor_16(nargout, out, nargin-1, in+1);
      break;
    case 17:
      gpmp2GaussianProcessPriorPose2_deconstructor_17(nargout, out, nargin-1, in+1);
      break;
    case 18:
      gpmp2GaussianProcessPriorPose2Vector_collectorInsertAndMakeBase_18(nargout, out, nargin-1, in+1);
      break;
    case 19:
      gpmp2GaussianProcessPriorPose2Vector_upcastFromVoid_19(nargout, out, nargin-1, in+1);
      break;
    case 20:
      gpmp2GaussianProcessPriorPose2Vector_constructor_20(nargout, out, nargin-1, in+1);
      break;
    case 21:
      gpmp2GaussianProcessPriorPose2Vector_deconstructor_21(nargout, out, nargin-1, in+1);
      break;
    case 22:
      gpmp2GaussianProcessInterpolatorLinear_collectorInsertAndMakeBase_22(nargout, out, nargin-1, in+1);
      break;
    case 23:
      gpmp2GaussianProcessInterpolatorLinear_constructor_23(nargout, out, nargin-1, in+1);
      break;
    case 24:
      gpmp2GaussianProcessInterpolatorLinear_deconstructor_24(nargout, out, nargin-1, in+1);
      break;
    case 25:
      gpmp2GaussianProcessInterpolatorLinear_interpolatePose_25(nargout, out, nargin-1, in+1);
      break;
    case 26:
      gpmp2GaussianProcessInterpolatorLinear_interpolateVelocity_26(nargout, out, nargin-1, in+1);
      break;
    case 27:
      gpmp2Arm_collectorInsertAndMakeBase_27(nargout, out, nargin-1, in+1);
      break;
    case 28:
      gpmp2Arm_constructor_28(nargout, out, nargin-1, in+1);
      break;
    case 29:
      gpmp2Arm_constructor_29(nargout, out, nargin-1, in+1);
      break;
    case 30:
      gpmp2Arm_constructor_30(nargout, out, nargin-1, in+1);
      break;
    case 31:
      gpmp2Arm_deconstructor_31(nargout, out, nargin-1, in+1);
      break;
    case 32:
      gpmp2Arm_a_32(nargout, out, nargin-1, in+1);
      break;
    case 33:
      gpmp2Arm_alpha_33(nargout, out, nargin-1, in+1);
      break;
    case 34:
      gpmp2Arm_base_pose_34(nargout, out, nargin-1, in+1);
      break;
    case 35:
      gpmp2Arm_d_35(nargout, out, nargin-1, in+1);
      break;
    case 36:
      gpmp2Arm_dof_36(nargout, out, nargin-1, in+1);
      break;
    case 37:
      gpmp2Arm_forwardKinematicsPose_37(nargout, out, nargin-1, in+1);
      break;
    case 38:
      gpmp2Arm_forwardKinematicsPosition_38(nargout, out, nargin-1, in+1);
      break;
    case 39:
      gpmp2Arm_forwardKinematicsVel_39(nargout, out, nargin-1, in+1);
      break;
    case 40:
      gpmp2Pose2MobileBase_collectorInsertAndMakeBase_40(nargout, out, nargin-1, in+1);
      break;
    case 41:
      gpmp2Pose2MobileBase_constructor_41(nargout, out, nargin-1, in+1);
      break;
    case 42:
      gpmp2Pose2MobileBase_deconstructor_42(nargout, out, nargin-1, in+1);
      break;
    case 43:
      gpmp2Pose2MobileBase_dof_43(nargout, out, nargin-1, in+1);
      break;
    case 44:
      gpmp2Pose2MobileBase_forwardKinematicsPose_44(nargout, out, nargin-1, in+1);
      break;
    case 45:
      gpmp2Pose2MobileBase_forwardKinematicsPosition_45(nargout, out, nargin-1, in+1);
      break;
    case 46:
      gpmp2Pose2MobileBase_forwardKinematicsVel_46(nargout, out, nargin-1, in+1);
      break;
    case 47:
      gpmp2Pose2MobileBase_nr_links_47(nargout, out, nargin-1, in+1);
      break;
    case 48:
      gpmp2Pose2MobileArm_collectorInsertAndMakeBase_48(nargout, out, nargin-1, in+1);
      break;
    case 49:
      gpmp2Pose2MobileArm_constructor_49(nargout, out, nargin-1, in+1);
      break;
    case 50:
      gpmp2Pose2MobileArm_constructor_50(nargout, out, nargin-1, in+1);
      break;
    case 51:
      gpmp2Pose2MobileArm_deconstructor_51(nargout, out, nargin-1, in+1);
      break;
    case 52:
      gpmp2Pose2MobileArm_arm_52(nargout, out, nargin-1, in+1);
      break;
    case 53:
      gpmp2Pose2MobileArm_base_T_arm_53(nargout, out, nargin-1, in+1);
      break;
    case 54:
      gpmp2Pose2MobileArm_dof_54(nargout, out, nargin-1, in+1);
      break;
    case 55:
      gpmp2Pose2MobileArm_forwardKinematicsPose_55(nargout, out, nargin-1, in+1);
      break;
    case 56:
      gpmp2Pose2MobileArm_forwardKinematicsPosition_56(nargout, out, nargin-1, in+1);
      break;
    case 57:
      gpmp2Pose2MobileArm_forwardKinematicsVel_57(nargout, out, nargin-1, in+1);
      break;
    case 58:
      gpmp2Pose2MobileArm_nr_links_58(nargout, out, nargin-1, in+1);
      break;
    case 59:
      gpmp2Pose2Mobile2Arms_collectorInsertAndMakeBase_59(nargout, out, nargin-1, in+1);
      break;
    case 60:
      gpmp2Pose2Mobile2Arms_constructor_60(nargout, out, nargin-1, in+1);
      break;
    case 61:
      gpmp2Pose2Mobile2Arms_constructor_61(nargout, out, nargin-1, in+1);
      break;
    case 62:
      gpmp2Pose2Mobile2Arms_deconstructor_62(nargout, out, nargin-1, in+1);
      break;
    case 63:
      gpmp2Pose2Mobile2Arms_arm1_63(nargout, out, nargin-1, in+1);
      break;
    case 64:
      gpmp2Pose2Mobile2Arms_arm2_64(nargout, out, nargin-1, in+1);
      break;
    case 65:
      gpmp2Pose2Mobile2Arms_base_T_arm1_65(nargout, out, nargin-1, in+1);
      break;
    case 66:
      gpmp2Pose2Mobile2Arms_base_T_arm2_66(nargout, out, nargin-1, in+1);
      break;
    case 67:
      gpmp2Pose2Mobile2Arms_dof_67(nargout, out, nargin-1, in+1);
      break;
    case 68:
      gpmp2Pose2Mobile2Arms_forwardKinematicsPose_68(nargout, out, nargin-1, in+1);
      break;
    case 69:
      gpmp2Pose2Mobile2Arms_forwardKinematicsPosition_69(nargout, out, nargin-1, in+1);
      break;
    case 70:
      gpmp2Pose2Mobile2Arms_forwardKinematicsVel_70(nargout, out, nargin-1, in+1);
      break;
    case 71:
      gpmp2Pose2Mobile2Arms_nr_links_71(nargout, out, nargin-1, in+1);
      break;
    case 72:
      gpmp2Pose2MobileVetLinArm_collectorInsertAndMakeBase_72(nargout, out, nargin-1, in+1);
      break;
    case 73:
      gpmp2Pose2MobileVetLinArm_constructor_73(nargout, out, nargin-1, in+1);
      break;
    case 74:
      gpmp2Pose2MobileVetLinArm_constructor_74(nargout, out, nargin-1, in+1);
      break;
    case 75:
      gpmp2Pose2MobileVetLinArm_deconstructor_75(nargout, out, nargin-1, in+1);
      break;
    case 76:
      gpmp2Pose2MobileVetLinArm_arm_76(nargout, out, nargin-1, in+1);
      break;
    case 77:
      gpmp2Pose2MobileVetLinArm_base_T_torso_77(nargout, out, nargin-1, in+1);
      break;
    case 78:
      gpmp2Pose2MobileVetLinArm_dof_78(nargout, out, nargin-1, in+1);
      break;
    case 79:
      gpmp2Pose2MobileVetLinArm_forwardKinematicsPose_79(nargout, out, nargin-1, in+1);
      break;
    case 80:
      gpmp2Pose2MobileVetLinArm_forwardKinematicsPosition_80(nargout, out, nargin-1, in+1);
      break;
    case 81:
      gpmp2Pose2MobileVetLinArm_forwardKinematicsVel_81(nargout, out, nargin-1, in+1);
      break;
    case 82:
      gpmp2Pose2MobileVetLinArm_nr_links_82(nargout, out, nargin-1, in+1);
      break;
    case 83:
      gpmp2Pose2MobileVetLinArm_reverse_linact_83(nargout, out, nargin-1, in+1);
      break;
    case 84:
      gpmp2Pose2MobileVetLinArm_torso_T_arm_84(nargout, out, nargin-1, in+1);
      break;
    case 85:
      gpmp2Pose2MobileVetLin2Arms_collectorInsertAndMakeBase_85(nargout, out, nargin-1, in+1);
      break;
    case 86:
      gpmp2Pose2MobileVetLin2Arms_constructor_86(nargout, out, nargin-1, in+1);
      break;
    case 87:
      gpmp2Pose2MobileVetLin2Arms_constructor_87(nargout, out, nargin-1, in+1);
      break;
    case 88:
      gpmp2Pose2MobileVetLin2Arms_deconstructor_88(nargout, out, nargin-1, in+1);
      break;
    case 89:
      gpmp2Pose2MobileVetLin2Arms_arm1_89(nargout, out, nargin-1, in+1);
      break;
    case 90:
      gpmp2Pose2MobileVetLin2Arms_arm2_90(nargout, out, nargin-1, in+1);
      break;
    case 91:
      gpmp2Pose2MobileVetLin2Arms_base_T_torso_91(nargout, out, nargin-1, in+1);
      break;
    case 92:
      gpmp2Pose2MobileVetLin2Arms_dof_92(nargout, out, nargin-1, in+1);
      break;
    case 93:
      gpmp2Pose2MobileVetLin2Arms_forwardKinematicsPose_93(nargout, out, nargin-1, in+1);
      break;
    case 94:
      gpmp2Pose2MobileVetLin2Arms_forwardKinematicsPosition_94(nargout, out, nargin-1, in+1);
      break;
    case 95:
      gpmp2Pose2MobileVetLin2Arms_forwardKinematicsVel_95(nargout, out, nargin-1, in+1);
      break;
    case 96:
      gpmp2Pose2MobileVetLin2Arms_nr_links_96(nargout, out, nargin-1, in+1);
      break;
    case 97:
      gpmp2Pose2MobileVetLin2Arms_reverse_linact_97(nargout, out, nargin-1, in+1);
      break;
    case 98:
      gpmp2Pose2MobileVetLin2Arms_torso_T_arm1_98(nargout, out, nargin-1, in+1);
      break;
    case 99:
      gpmp2Pose2MobileVetLin2Arms_torso_T_arm2_99(nargout, out, nargin-1, in+1);
      break;
    case 100:
      gpmp2PointRobot_collectorInsertAndMakeBase_100(nargout, out, nargin-1, in+1);
      break;
    case 101:
      gpmp2PointRobot_constructor_101(nargout, out, nargin-1, in+1);
      break;
    case 102:
      gpmp2PointRobot_deconstructor_102(nargout, out, nargin-1, in+1);
      break;
    case 103:
      gpmp2PointRobot_dof_103(nargout, out, nargin-1, in+1);
      break;
    case 104:
      gpmp2PointRobot_forwardKinematicsPose_104(nargout, out, nargin-1, in+1);
      break;
    case 105:
      gpmp2PointRobot_forwardKinematicsPosition_105(nargout, out, nargin-1, in+1);
      break;
    case 106:
      gpmp2PointRobot_forwardKinematicsVel_106(nargout, out, nargin-1, in+1);
      break;
    case 107:
      gpmp2PointRobot_nr_links_107(nargout, out, nargin-1, in+1);
      break;
    case 108:
      gpmp2BodySphere_collectorInsertAndMakeBase_108(nargout, out, nargin-1, in+1);
      break;
    case 109:
      gpmp2BodySphere_constructor_109(nargout, out, nargin-1, in+1);
      break;
    case 110:
      gpmp2BodySphere_deconstructor_110(nargout, out, nargin-1, in+1);
      break;
    case 111:
      gpmp2BodySphereVector_collectorInsertAndMakeBase_111(nargout, out, nargin-1, in+1);
      break;
    case 112:
      gpmp2BodySphereVector_constructor_112(nargout, out, nargin-1, in+1);
      break;
    case 113:
      gpmp2BodySphereVector_deconstructor_113(nargout, out, nargin-1, in+1);
      break;
    case 114:
      gpmp2BodySphereVector_push_back_114(nargout, out, nargin-1, in+1);
      break;
    case 115:
      gpmp2ArmModel_collectorInsertAndMakeBase_115(nargout, out, nargin-1, in+1);
      break;
    case 116:
      gpmp2ArmModel_constructor_116(nargout, out, nargin-1, in+1);
      break;
    case 117:
      gpmp2ArmModel_deconstructor_117(nargout, out, nargin-1, in+1);
      break;
    case 118:
      gpmp2ArmModel_dof_118(nargout, out, nargin-1, in+1);
      break;
    case 119:
      gpmp2ArmModel_fk_model_119(nargout, out, nargin-1, in+1);
      break;
    case 120:
      gpmp2ArmModel_nr_body_spheres_120(nargout, out, nargin-1, in+1);
      break;
    case 121:
      gpmp2ArmModel_sphereCentersMat_121(nargout, out, nargin-1, in+1);
      break;
    case 122:
      gpmp2ArmModel_sphere_radius_122(nargout, out, nargin-1, in+1);
      break;
    case 123:
      gpmp2Pose2MobileBaseModel_collectorInsertAndMakeBase_123(nargout, out, nargin-1, in+1);
      break;
    case 124:
      gpmp2Pose2MobileBaseModel_constructor_124(nargout, out, nargin-1, in+1);
      break;
    case 125:
      gpmp2Pose2MobileBaseModel_deconstructor_125(nargout, out, nargin-1, in+1);
      break;
    case 126:
      gpmp2Pose2MobileBaseModel_dof_126(nargout, out, nargin-1, in+1);
      break;
    case 127:
      gpmp2Pose2MobileBaseModel_fk_model_127(nargout, out, nargin-1, in+1);
      break;
    case 128:
      gpmp2Pose2MobileBaseModel_nr_body_spheres_128(nargout, out, nargin-1, in+1);
      break;
    case 129:
      gpmp2Pose2MobileBaseModel_sphereCentersMat_129(nargout, out, nargin-1, in+1);
      break;
    case 130:
      gpmp2Pose2MobileBaseModel_sphere_radius_130(nargout, out, nargin-1, in+1);
      break;
    case 131:
      gpmp2Pose2MobileArmModel_collectorInsertAndMakeBase_131(nargout, out, nargin-1, in+1);
      break;
    case 132:
      gpmp2Pose2MobileArmModel_constructor_132(nargout, out, nargin-1, in+1);
      break;
    case 133:
      gpmp2Pose2MobileArmModel_deconstructor_133(nargout, out, nargin-1, in+1);
      break;
    case 134:
      gpmp2Pose2MobileArmModel_dof_134(nargout, out, nargin-1, in+1);
      break;
    case 135:
      gpmp2Pose2MobileArmModel_fk_model_135(nargout, out, nargin-1, in+1);
      break;
    case 136:
      gpmp2Pose2MobileArmModel_nr_body_spheres_136(nargout, out, nargin-1, in+1);
      break;
    case 137:
      gpmp2Pose2MobileArmModel_sphereCentersMat_137(nargout, out, nargin-1, in+1);
      break;
    case 138:
      gpmp2Pose2MobileArmModel_sphere_radius_138(nargout, out, nargin-1, in+1);
      break;
    case 139:
      gpmp2Pose2Mobile2ArmsModel_collectorInsertAndMakeBase_139(nargout, out, nargin-1, in+1);
      break;
    case 140:
      gpmp2Pose2Mobile2ArmsModel_constructor_140(nargout, out, nargin-1, in+1);
      break;
    case 141:
      gpmp2Pose2Mobile2ArmsModel_deconstructor_141(nargout, out, nargin-1, in+1);
      break;
    case 142:
      gpmp2Pose2Mobile2ArmsModel_dof_142(nargout, out, nargin-1, in+1);
      break;
    case 143:
      gpmp2Pose2Mobile2ArmsModel_fk_model_143(nargout, out, nargin-1, in+1);
      break;
    case 144:
      gpmp2Pose2Mobile2ArmsModel_nr_body_spheres_144(nargout, out, nargin-1, in+1);
      break;
    case 145:
      gpmp2Pose2Mobile2ArmsModel_sphereCentersMat_145(nargout, out, nargin-1, in+1);
      break;
    case 146:
      gpmp2Pose2Mobile2ArmsModel_sphere_radius_146(nargout, out, nargin-1, in+1);
      break;
    case 147:
      gpmp2Pose2MobileVetLinArmModel_collectorInsertAndMakeBase_147(nargout, out, nargin-1, in+1);
      break;
    case 148:
      gpmp2Pose2MobileVetLinArmModel_constructor_148(nargout, out, nargin-1, in+1);
      break;
    case 149:
      gpmp2Pose2MobileVetLinArmModel_deconstructor_149(nargout, out, nargin-1, in+1);
      break;
    case 150:
      gpmp2Pose2MobileVetLinArmModel_dof_150(nargout, out, nargin-1, in+1);
      break;
    case 151:
      gpmp2Pose2MobileVetLinArmModel_fk_model_151(nargout, out, nargin-1, in+1);
      break;
    case 152:
      gpmp2Pose2MobileVetLinArmModel_nr_body_spheres_152(nargout, out, nargin-1, in+1);
      break;
    case 153:
      gpmp2Pose2MobileVetLinArmModel_sphereCentersMat_153(nargout, out, nargin-1, in+1);
      break;
    case 154:
      gpmp2Pose2MobileVetLinArmModel_sphere_radius_154(nargout, out, nargin-1, in+1);
      break;
    case 155:
      gpmp2Pose2MobileVetLin2ArmsModel_collectorInsertAndMakeBase_155(nargout, out, nargin-1, in+1);
      break;
    case 156:
      gpmp2Pose2MobileVetLin2ArmsModel_constructor_156(nargout, out, nargin-1, in+1);
      break;
    case 157:
      gpmp2Pose2MobileVetLin2ArmsModel_deconstructor_157(nargout, out, nargin-1, in+1);
      break;
    case 158:
      gpmp2Pose2MobileVetLin2ArmsModel_dof_158(nargout, out, nargin-1, in+1);
      break;
    case 159:
      gpmp2Pose2MobileVetLin2ArmsModel_fk_model_159(nargout, out, nargin-1, in+1);
      break;
    case 160:
      gpmp2Pose2MobileVetLin2ArmsModel_nr_body_spheres_160(nargout, out, nargin-1, in+1);
      break;
    case 161:
      gpmp2Pose2MobileVetLin2ArmsModel_sphereCentersMat_161(nargout, out, nargin-1, in+1);
      break;
    case 162:
      gpmp2Pose2MobileVetLin2ArmsModel_sphere_radius_162(nargout, out, nargin-1, in+1);
      break;
    case 163:
      gpmp2PointRobotModel_collectorInsertAndMakeBase_163(nargout, out, nargin-1, in+1);
      break;
    case 164:
      gpmp2PointRobotModel_constructor_164(nargout, out, nargin-1, in+1);
      break;
    case 165:
      gpmp2PointRobotModel_deconstructor_165(nargout, out, nargin-1, in+1);
      break;
    case 166:
      gpmp2PointRobotModel_dof_166(nargout, out, nargin-1, in+1);
      break;
    case 167:
      gpmp2PointRobotModel_fk_model_167(nargout, out, nargin-1, in+1);
      break;
    case 168:
      gpmp2PointRobotModel_nr_body_spheres_168(nargout, out, nargin-1, in+1);
      break;
    case 169:
      gpmp2PointRobotModel_sphereCentersMat_169(nargout, out, nargin-1, in+1);
      break;
    case 170:
      gpmp2PointRobotModel_sphere_radius_170(nargout, out, nargin-1, in+1);
      break;
    case 171:
      gpmp2GoalFactorArm_collectorInsertAndMakeBase_171(nargout, out, nargin-1, in+1);
      break;
    case 172:
      gpmp2GoalFactorArm_upcastFromVoid_172(nargout, out, nargin-1, in+1);
      break;
    case 173:
      gpmp2GoalFactorArm_constructor_173(nargout, out, nargin-1, in+1);
      break;
    case 174:
      gpmp2GoalFactorArm_deconstructor_174(nargout, out, nargin-1, in+1);
      break;
    case 175:
      gpmp2JointLimitFactorVector_collectorInsertAndMakeBase_175(nargout, out, nargin-1, in+1);
      break;
    case 176:
      gpmp2JointLimitFactorVector_upcastFromVoid_176(nargout, out, nargin-1, in+1);
      break;
    case 177:
      gpmp2JointLimitFactorVector_constructor_177(nargout, out, nargin-1, in+1);
      break;
    case 178:
      gpmp2JointLimitFactorVector_deconstructor_178(nargout, out, nargin-1, in+1);
      break;
    case 179:
      gpmp2VelocityLimitFactorVector_collectorInsertAndMakeBase_179(nargout, out, nargin-1, in+1);
      break;
    case 180:
      gpmp2VelocityLimitFactorVector_upcastFromVoid_180(nargout, out, nargin-1, in+1);
      break;
    case 181:
      gpmp2VelocityLimitFactorVector_constructor_181(nargout, out, nargin-1, in+1);
      break;
    case 182:
      gpmp2VelocityLimitFactorVector_deconstructor_182(nargout, out, nargin-1, in+1);
      break;
    case 183:
      gpmp2GaussianPriorWorkspacePositionArm_collectorInsertAndMakeBase_183(nargout, out, nargin-1, in+1);
      break;
    case 184:
      gpmp2GaussianPriorWorkspacePositionArm_upcastFromVoid_184(nargout, out, nargin-1, in+1);
      break;
    case 185:
      gpmp2GaussianPriorWorkspacePositionArm_constructor_185(nargout, out, nargin-1, in+1);
      break;
    case 186:
      gpmp2GaussianPriorWorkspacePositionArm_deconstructor_186(nargout, out, nargin-1, in+1);
      break;
    case 187:
      gpmp2GaussianPriorWorkspaceOrientationArm_collectorInsertAndMakeBase_187(nargout, out, nargin-1, in+1);
      break;
    case 188:
      gpmp2GaussianPriorWorkspaceOrientationArm_upcastFromVoid_188(nargout, out, nargin-1, in+1);
      break;
    case 189:
      gpmp2GaussianPriorWorkspaceOrientationArm_constructor_189(nargout, out, nargin-1, in+1);
      break;
    case 190:
      gpmp2GaussianPriorWorkspaceOrientationArm_deconstructor_190(nargout, out, nargin-1, in+1);
      break;
    case 191:
      gpmp2GaussianPriorWorkspacePoseArm_collectorInsertAndMakeBase_191(nargout, out, nargin-1, in+1);
      break;
    case 192:
      gpmp2GaussianPriorWorkspacePoseArm_upcastFromVoid_192(nargout, out, nargin-1, in+1);
      break;
    case 193:
      gpmp2GaussianPriorWorkspacePoseArm_constructor_193(nargout, out, nargin-1, in+1);
      break;
    case 194:
      gpmp2GaussianPriorWorkspacePoseArm_deconstructor_194(nargout, out, nargin-1, in+1);
      break;
    case 195:
      gpmp2VehicleDynamicsFactorPose2_collectorInsertAndMakeBase_195(nargout, out, nargin-1, in+1);
      break;
    case 196:
      gpmp2VehicleDynamicsFactorPose2_upcastFromVoid_196(nargout, out, nargin-1, in+1);
      break;
    case 197:
      gpmp2VehicleDynamicsFactorPose2_constructor_197(nargout, out, nargin-1, in+1);
      break;
    case 198:
      gpmp2VehicleDynamicsFactorPose2_deconstructor_198(nargout, out, nargin-1, in+1);
      break;
    case 199:
      gpmp2VehicleDynamicsFactorPose2Vector_collectorInsertAndMakeBase_199(nargout, out, nargin-1, in+1);
      break;
    case 200:
      gpmp2VehicleDynamicsFactorPose2Vector_upcastFromVoid_200(nargout, out, nargin-1, in+1);
      break;
    case 201:
      gpmp2VehicleDynamicsFactorPose2Vector_constructor_201(nargout, out, nargin-1, in+1);
      break;
    case 202:
      gpmp2VehicleDynamicsFactorPose2Vector_deconstructor_202(nargout, out, nargin-1, in+1);
      break;
    case 203:
      gpmp2VehicleDynamicsFactorVector_collectorInsertAndMakeBase_203(nargout, out, nargin-1, in+1);
      break;
    case 204:
      gpmp2VehicleDynamicsFactorVector_upcastFromVoid_204(nargout, out, nargin-1, in+1);
      break;
    case 205:
      gpmp2VehicleDynamicsFactorVector_constructor_205(nargout, out, nargin-1, in+1);
      break;
    case 206:
      gpmp2VehicleDynamicsFactorVector_deconstructor_206(nargout, out, nargin-1, in+1);
      break;
    case 207:
      gpmp2SignedDistanceField_collectorInsertAndMakeBase_207(nargout, out, nargin-1, in+1);
      break;
    case 208:
      gpmp2SignedDistanceField_constructor_208(nargout, out, nargin-1, in+1);
      break;
    case 209:
      gpmp2SignedDistanceField_constructor_209(nargout, out, nargin-1, in+1);
      break;
    case 210:
      gpmp2SignedDistanceField_deconstructor_210(nargout, out, nargin-1, in+1);
      break;
    case 211:
      gpmp2SignedDistanceField_getSignedDistance_211(nargout, out, nargin-1, in+1);
      break;
    case 212:
      gpmp2SignedDistanceField_initFieldData_212(nargout, out, nargin-1, in+1);
      break;
    case 213:
      gpmp2SignedDistanceField_loadSDF_213(nargout, out, nargin-1, in+1);
      break;
    case 214:
      gpmp2SignedDistanceField_print_214(nargout, out, nargin-1, in+1);
      break;
    case 215:
      gpmp2SignedDistanceField_saveSDF_215(nargout, out, nargin-1, in+1);
      break;
    case 216:
      gpmp2PlanarSDF_collectorInsertAndMakeBase_216(nargout, out, nargin-1, in+1);
      break;
    case 217:
      gpmp2PlanarSDF_constructor_217(nargout, out, nargin-1, in+1);
      break;
    case 218:
      gpmp2PlanarSDF_deconstructor_218(nargout, out, nargin-1, in+1);
      break;
    case 219:
      gpmp2PlanarSDF_getSignedDistance_219(nargout, out, nargin-1, in+1);
      break;
    case 220:
      gpmp2PlanarSDF_print_220(nargout, out, nargin-1, in+1);
      break;
    case 221:
      gpmp2ObstacleSDFFactorArm_collectorInsertAndMakeBase_221(nargout, out, nargin-1, in+1);
      break;
    case 222:
      gpmp2ObstacleSDFFactorArm_upcastFromVoid_222(nargout, out, nargin-1, in+1);
      break;
    case 223:
      gpmp2ObstacleSDFFactorArm_constructor_223(nargout, out, nargin-1, in+1);
      break;
    case 224:
      gpmp2ObstacleSDFFactorArm_deconstructor_224(nargout, out, nargin-1, in+1);
      break;
    case 225:
      gpmp2ObstacleSDFFactorArm_evaluateError_225(nargout, out, nargin-1, in+1);
      break;
    case 226:
      gpmp2ObstacleSDFFactorGPArm_collectorInsertAndMakeBase_226(nargout, out, nargin-1, in+1);
      break;
    case 227:
      gpmp2ObstacleSDFFactorGPArm_upcastFromVoid_227(nargout, out, nargin-1, in+1);
      break;
    case 228:
      gpmp2ObstacleSDFFactorGPArm_constructor_228(nargout, out, nargin-1, in+1);
      break;
    case 229:
      gpmp2ObstacleSDFFactorGPArm_deconstructor_229(nargout, out, nargin-1, in+1);
      break;
    case 230:
      gpmp2ObstaclePlanarSDFFactorArm_collectorInsertAndMakeBase_230(nargout, out, nargin-1, in+1);
      break;
    case 231:
      gpmp2ObstaclePlanarSDFFactorArm_upcastFromVoid_231(nargout, out, nargin-1, in+1);
      break;
    case 232:
      gpmp2ObstaclePlanarSDFFactorArm_constructor_232(nargout, out, nargin-1, in+1);
      break;
    case 233:
      gpmp2ObstaclePlanarSDFFactorArm_deconstructor_233(nargout, out, nargin-1, in+1);
      break;
    case 234:
      gpmp2ObstaclePlanarSDFFactorArm_evaluateError_234(nargout, out, nargin-1, in+1);
      break;
    case 235:
      gpmp2ObstaclePlanarSDFFactorArm_string_serialize_235(nargout, out, nargin-1, in+1);
      break;
    case 236:
      gpmp2ObstaclePlanarSDFFactorArm_string_deserialize_236(nargout, out, nargin-1, in+1);
      break;
    case 237:
      gpmp2ObstaclePlanarSDFFactorGPArm_collectorInsertAndMakeBase_237(nargout, out, nargin-1, in+1);
      break;
    case 238:
      gpmp2ObstaclePlanarSDFFactorGPArm_upcastFromVoid_238(nargout, out, nargin-1, in+1);
      break;
    case 239:
      gpmp2ObstaclePlanarSDFFactorGPArm_constructor_239(nargout, out, nargin-1, in+1);
      break;
    case 240:
      gpmp2ObstaclePlanarSDFFactorGPArm_deconstructor_240(nargout, out, nargin-1, in+1);
      break;
    case 241:
      gpmp2ObstaclePlanarSDFFactorPointRobot_collectorInsertAndMakeBase_241(nargout, out, nargin-1, in+1);
      break;
    case 242:
      gpmp2ObstaclePlanarSDFFactorPointRobot_upcastFromVoid_242(nargout, out, nargin-1, in+1);
      break;
    case 243:
      gpmp2ObstaclePlanarSDFFactorPointRobot_constructor_243(nargout, out, nargin-1, in+1);
      break;
    case 244:
      gpmp2ObstaclePlanarSDFFactorPointRobot_deconstructor_244(nargout, out, nargin-1, in+1);
      break;
    case 245:
      gpmp2ObstaclePlanarSDFFactorPointRobot_evaluateError_245(nargout, out, nargin-1, in+1);
      break;
    case 246:
      gpmp2ObstaclePlanarSDFFactorGPPointRobot_collectorInsertAndMakeBase_246(nargout, out, nargin-1, in+1);
      break;
    case 247:
      gpmp2ObstaclePlanarSDFFactorGPPointRobot_upcastFromVoid_247(nargout, out, nargin-1, in+1);
      break;
    case 248:
      gpmp2ObstaclePlanarSDFFactorGPPointRobot_constructor_248(nargout, out, nargin-1, in+1);
      break;
    case 249:
      gpmp2ObstaclePlanarSDFFactorGPPointRobot_deconstructor_249(nargout, out, nargin-1, in+1);
      break;
    case 250:
      gpmp2ObstaclePlanarSDFFactorPose2MobileBase_collectorInsertAndMakeBase_250(nargout, out, nargin-1, in+1);
      break;
    case 251:
      gpmp2ObstaclePlanarSDFFactorPose2MobileBase_upcastFromVoid_251(nargout, out, nargin-1, in+1);
      break;
    case 252:
      gpmp2ObstaclePlanarSDFFactorPose2MobileBase_constructor_252(nargout, out, nargin-1, in+1);
      break;
    case 253:
      gpmp2ObstaclePlanarSDFFactorPose2MobileBase_deconstructor_253(nargout, out, nargin-1, in+1);
      break;
    case 254:
      gpmp2ObstaclePlanarSDFFactorPose2MobileBase_evaluateError_254(nargout, out, nargin-1, in+1);
      break;
    case 255:
      gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase_collectorInsertAndMakeBase_255(nargout, out, nargin-1, in+1);
      break;
    case 256:
      gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase_upcastFromVoid_256(nargout, out, nargin-1, in+1);
      break;
    case 257:
      gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase_constructor_257(nargout, out, nargin-1, in+1);
      break;
    case 258:
      gpmp2ObstaclePlanarSDFFactorGPPose2MobileBase_deconstructor_258(nargout, out, nargin-1, in+1);
      break;
    case 259:
      gpmp2ObstaclePlanarSDFFactorPose2MobileArm_collectorInsertAndMakeBase_259(nargout, out, nargin-1, in+1);
      break;
    case 260:
      gpmp2ObstaclePlanarSDFFactorPose2MobileArm_upcastFromVoid_260(nargout, out, nargin-1, in+1);
      break;
    case 261:
      gpmp2ObstaclePlanarSDFFactorPose2MobileArm_constructor_261(nargout, out, nargin-1, in+1);
      break;
    case 262:
      gpmp2ObstaclePlanarSDFFactorPose2MobileArm_deconstructor_262(nargout, out, nargin-1, in+1);
      break;
    case 263:
      gpmp2ObstaclePlanarSDFFactorPose2MobileArm_evaluateError_263(nargout, out, nargin-1, in+1);
      break;
    case 264:
      gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm_collectorInsertAndMakeBase_264(nargout, out, nargin-1, in+1);
      break;
    case 265:
      gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm_upcastFromVoid_265(nargout, out, nargin-1, in+1);
      break;
    case 266:
      gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm_constructor_266(nargout, out, nargin-1, in+1);
      break;
    case 267:
      gpmp2ObstaclePlanarSDFFactorGPPose2MobileArm_deconstructor_267(nargout, out, nargin-1, in+1);
      break;
    case 268:
      gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms_collectorInsertAndMakeBase_268(nargout, out, nargin-1, in+1);
      break;
    case 269:
      gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms_upcastFromVoid_269(nargout, out, nargin-1, in+1);
      break;
    case 270:
      gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms_constructor_270(nargout, out, nargin-1, in+1);
      break;
    case 271:
      gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms_deconstructor_271(nargout, out, nargin-1, in+1);
      break;
    case 272:
      gpmp2ObstaclePlanarSDFFactorPose2Mobile2Arms_evaluateError_272(nargout, out, nargin-1, in+1);
      break;
    case 273:
      gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms_collectorInsertAndMakeBase_273(nargout, out, nargin-1, in+1);
      break;
    case 274:
      gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms_upcastFromVoid_274(nargout, out, nargin-1, in+1);
      break;
    case 275:
      gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms_constructor_275(nargout, out, nargin-1, in+1);
      break;
    case 276:
      gpmp2ObstaclePlanarSDFFactorGPPose2Mobile2Arms_deconstructor_276(nargout, out, nargin-1, in+1);
      break;
    case 277:
      gpmp2ObstacleSDFFactorPose2MobileBase_collectorInsertAndMakeBase_277(nargout, out, nargin-1, in+1);
      break;
    case 278:
      gpmp2ObstacleSDFFactorPose2MobileBase_upcastFromVoid_278(nargout, out, nargin-1, in+1);
      break;
    case 279:
      gpmp2ObstacleSDFFactorPose2MobileBase_constructor_279(nargout, out, nargin-1, in+1);
      break;
    case 280:
      gpmp2ObstacleSDFFactorPose2MobileBase_deconstructor_280(nargout, out, nargin-1, in+1);
      break;
    case 281:
      gpmp2ObstacleSDFFactorPose2MobileBase_evaluateError_281(nargout, out, nargin-1, in+1);
      break;
    case 282:
      gpmp2ObstacleSDFFactorGPPose2MobileBase_collectorInsertAndMakeBase_282(nargout, out, nargin-1, in+1);
      break;
    case 283:
      gpmp2ObstacleSDFFactorGPPose2MobileBase_upcastFromVoid_283(nargout, out, nargin-1, in+1);
      break;
    case 284:
      gpmp2ObstacleSDFFactorGPPose2MobileBase_constructor_284(nargout, out, nargin-1, in+1);
      break;
    case 285:
      gpmp2ObstacleSDFFactorGPPose2MobileBase_deconstructor_285(nargout, out, nargin-1, in+1);
      break;
    case 286:
      gpmp2ObstacleSDFFactorPose2MobileArm_collectorInsertAndMakeBase_286(nargout, out, nargin-1, in+1);
      break;
    case 287:
      gpmp2ObstacleSDFFactorPose2MobileArm_upcastFromVoid_287(nargout, out, nargin-1, in+1);
      break;
    case 288:
      gpmp2ObstacleSDFFactorPose2MobileArm_constructor_288(nargout, out, nargin-1, in+1);
      break;
    case 289:
      gpmp2ObstacleSDFFactorPose2MobileArm_deconstructor_289(nargout, out, nargin-1, in+1);
      break;
    case 290:
      gpmp2ObstacleSDFFactorPose2MobileArm_evaluateError_290(nargout, out, nargin-1, in+1);
      break;
    case 291:
      gpmp2ObstacleSDFFactorGPPose2MobileArm_collectorInsertAndMakeBase_291(nargout, out, nargin-1, in+1);
      break;
    case 292:
      gpmp2ObstacleSDFFactorGPPose2MobileArm_upcastFromVoid_292(nargout, out, nargin-1, in+1);
      break;
    case 293:
      gpmp2ObstacleSDFFactorGPPose2MobileArm_constructor_293(nargout, out, nargin-1, in+1);
      break;
    case 294:
      gpmp2ObstacleSDFFactorGPPose2MobileArm_deconstructor_294(nargout, out, nargin-1, in+1);
      break;
    case 295:
      gpmp2ObstacleSDFFactorPose2Mobile2Arms_collectorInsertAndMakeBase_295(nargout, out, nargin-1, in+1);
      break;
    case 296:
      gpmp2ObstacleSDFFactorPose2Mobile2Arms_upcastFromVoid_296(nargout, out, nargin-1, in+1);
      break;
    case 297:
      gpmp2ObstacleSDFFactorPose2Mobile2Arms_constructor_297(nargout, out, nargin-1, in+1);
      break;
    case 298:
      gpmp2ObstacleSDFFactorPose2Mobile2Arms_deconstructor_298(nargout, out, nargin-1, in+1);
      break;
    case 299:
      gpmp2ObstacleSDFFactorPose2Mobile2Arms_evaluateError_299(nargout, out, nargin-1, in+1);
      break;
    case 300:
      gpmp2ObstacleSDFFactorGPPose2Mobile2Arms_collectorInsertAndMakeBase_300(nargout, out, nargin-1, in+1);
      break;
    case 301:
      gpmp2ObstacleSDFFactorGPPose2Mobile2Arms_upcastFromVoid_301(nargout, out, nargin-1, in+1);
      break;
    case 302:
      gpmp2ObstacleSDFFactorGPPose2Mobile2Arms_constructor_302(nargout, out, nargin-1, in+1);
      break;
    case 303:
      gpmp2ObstacleSDFFactorGPPose2Mobile2Arms_deconstructor_303(nargout, out, nargin-1, in+1);
      break;
    case 304:
      gpmp2ObstacleSDFFactorPose2MobileVetLinArm_collectorInsertAndMakeBase_304(nargout, out, nargin-1, in+1);
      break;
    case 305:
      gpmp2ObstacleSDFFactorPose2MobileVetLinArm_upcastFromVoid_305(nargout, out, nargin-1, in+1);
      break;
    case 306:
      gpmp2ObstacleSDFFactorPose2MobileVetLinArm_constructor_306(nargout, out, nargin-1, in+1);
      break;
    case 307:
      gpmp2ObstacleSDFFactorPose2MobileVetLinArm_deconstructor_307(nargout, out, nargin-1, in+1);
      break;
    case 308:
      gpmp2ObstacleSDFFactorPose2MobileVetLinArm_evaluateError_308(nargout, out, nargin-1, in+1);
      break;
    case 309:
      gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm_collectorInsertAndMakeBase_309(nargout, out, nargin-1, in+1);
      break;
    case 310:
      gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm_upcastFromVoid_310(nargout, out, nargin-1, in+1);
      break;
    case 311:
      gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm_constructor_311(nargout, out, nargin-1, in+1);
      break;
    case 312:
      gpmp2ObstacleSDFFactorGPPose2MobileVetLinArm_deconstructor_312(nargout, out, nargin-1, in+1);
      break;
    case 313:
      gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms_collectorInsertAndMakeBase_313(nargout, out, nargin-1, in+1);
      break;
    case 314:
      gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms_upcastFromVoid_314(nargout, out, nargin-1, in+1);
      break;
    case 315:
      gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms_constructor_315(nargout, out, nargin-1, in+1);
      break;
    case 316:
      gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms_deconstructor_316(nargout, out, nargin-1, in+1);
      break;
    case 317:
      gpmp2ObstacleSDFFactorPose2MobileVetLin2Arms_evaluateError_317(nargout, out, nargin-1, in+1);
      break;
    case 318:
      gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms_collectorInsertAndMakeBase_318(nargout, out, nargin-1, in+1);
      break;
    case 319:
      gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms_upcastFromVoid_319(nargout, out, nargin-1, in+1);
      break;
    case 320:
      gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms_constructor_320(nargout, out, nargin-1, in+1);
      break;
    case 321:
      gpmp2ObstacleSDFFactorGPPose2MobileVetLin2Arms_deconstructor_321(nargout, out, nargin-1, in+1);
      break;
    case 322:
      gpmp2SelfCollisionArm_collectorInsertAndMakeBase_322(nargout, out, nargin-1, in+1);
      break;
    case 323:
      gpmp2SelfCollisionArm_upcastFromVoid_323(nargout, out, nargin-1, in+1);
      break;
    case 324:
      gpmp2SelfCollisionArm_constructor_324(nargout, out, nargin-1, in+1);
      break;
    case 325:
      gpmp2SelfCollisionArm_deconstructor_325(nargout, out, nargin-1, in+1);
      break;
    case 326:
      gpmp2SelfCollisionArm_evaluateError_326(nargout, out, nargin-1, in+1);
      break;
    case 327:
      gpmp2TrajOptimizerSetting_collectorInsertAndMakeBase_327(nargout, out, nargin-1, in+1);
      break;
    case 328:
      gpmp2TrajOptimizerSetting_constructor_328(nargout, out, nargin-1, in+1);
      break;
    case 329:
      gpmp2TrajOptimizerSetting_deconstructor_329(nargout, out, nargin-1, in+1);
      break;
    case 330:
      gpmp2TrajOptimizerSetting_setDogleg_330(nargout, out, nargin-1, in+1);
      break;
    case 331:
      gpmp2TrajOptimizerSetting_setGaussNewton_331(nargout, out, nargin-1, in+1);
      break;
    case 332:
      gpmp2TrajOptimizerSetting_setLM_332(nargout, out, nargin-1, in+1);
      break;
    case 333:
      gpmp2TrajOptimizerSetting_setOptimizationNoIncrase_333(nargout, out, nargin-1, in+1);
      break;
    case 334:
      gpmp2TrajOptimizerSetting_setVerbosityError_334(nargout, out, nargin-1, in+1);
      break;
    case 335:
      gpmp2TrajOptimizerSetting_setVerbosityNone_335(nargout, out, nargin-1, in+1);
      break;
    case 336:
      gpmp2TrajOptimizerSetting_set_Qc_model_336(nargout, out, nargin-1, in+1);
      break;
    case 337:
      gpmp2TrajOptimizerSetting_set_conf_prior_model_337(nargout, out, nargin-1, in+1);
      break;
    case 338:
      gpmp2TrajOptimizerSetting_set_cost_sigma_338(nargout, out, nargin-1, in+1);
      break;
    case 339:
      gpmp2TrajOptimizerSetting_set_epsilon_339(nargout, out, nargin-1, in+1);
      break;
    case 340:
      gpmp2TrajOptimizerSetting_set_flag_pos_limit_340(nargout, out, nargin-1, in+1);
      break;
    case 341:
      gpmp2TrajOptimizerSetting_set_flag_vel_limit_341(nargout, out, nargin-1, in+1);
      break;
    case 342:
      gpmp2TrajOptimizerSetting_set_joint_pos_limits_down_342(nargout, out, nargin-1, in+1);
      break;
    case 343:
      gpmp2TrajOptimizerSetting_set_joint_pos_limits_up_343(nargout, out, nargin-1, in+1);
      break;
    case 344:
      gpmp2TrajOptimizerSetting_set_max_iter_344(nargout, out, nargin-1, in+1);
      break;
    case 345:
      gpmp2TrajOptimizerSetting_set_obs_check_inter_345(nargout, out, nargin-1, in+1);
      break;
    case 346:
      gpmp2TrajOptimizerSetting_set_pos_limit_model_346(nargout, out, nargin-1, in+1);
      break;
    case 347:
      gpmp2TrajOptimizerSetting_set_pos_limit_thresh_347(nargout, out, nargin-1, in+1);
      break;
    case 348:
      gpmp2TrajOptimizerSetting_set_rel_thresh_348(nargout, out, nargin-1, in+1);
      break;
    case 349:
      gpmp2TrajOptimizerSetting_set_total_step_349(nargout, out, nargin-1, in+1);
      break;
    case 350:
      gpmp2TrajOptimizerSetting_set_total_time_350(nargout, out, nargin-1, in+1);
      break;
    case 351:
      gpmp2TrajOptimizerSetting_set_vel_limit_model_351(nargout, out, nargin-1, in+1);
      break;
    case 352:
      gpmp2TrajOptimizerSetting_set_vel_limit_thresh_352(nargout, out, nargin-1, in+1);
      break;
    case 353:
      gpmp2TrajOptimizerSetting_set_vel_limits_353(nargout, out, nargin-1, in+1);
      break;
    case 354:
      gpmp2TrajOptimizerSetting_set_vel_prior_model_354(nargout, out, nargin-1, in+1);
      break;
    case 355:
      gpmp2ISAM2TrajOptimizer2DArm_collectorInsertAndMakeBase_355(nargout, out, nargin-1, in+1);
      break;
    case 356:
      gpmp2ISAM2TrajOptimizer2DArm_constructor_356(nargout, out, nargin-1, in+1);
      break;
    case 357:
      gpmp2ISAM2TrajOptimizer2DArm_deconstructor_357(nargout, out, nargin-1, in+1);
      break;
    case 358:
      gpmp2ISAM2TrajOptimizer2DArm_addPoseEstimate_358(nargout, out, nargin-1, in+1);
      break;
    case 359:
      gpmp2ISAM2TrajOptimizer2DArm_addStateEstimate_359(nargout, out, nargin-1, in+1);
      break;
    case 360:
      gpmp2ISAM2TrajOptimizer2DArm_changeGoalConfigAndVel_360(nargout, out, nargin-1, in+1);
      break;
    case 361:
      gpmp2ISAM2TrajOptimizer2DArm_fixConfigAndVel_361(nargout, out, nargin-1, in+1);
      break;
    case 362:
      gpmp2ISAM2TrajOptimizer2DArm_initFactorGraph_362(nargout, out, nargin-1, in+1);
      break;
    case 363:
      gpmp2ISAM2TrajOptimizer2DArm_initValues_363(nargout, out, nargin-1, in+1);
      break;
    case 364:
      gpmp2ISAM2TrajOptimizer2DArm_removeGoalConfigAndVel_364(nargout, out, nargin-1, in+1);
      break;
    case 365:
      gpmp2ISAM2TrajOptimizer2DArm_update_365(nargout, out, nargin-1, in+1);
      break;
    case 366:
      gpmp2ISAM2TrajOptimizer2DArm_values_366(nargout, out, nargin-1, in+1);
      break;
    case 367:
      gpmp2ISAM2TrajOptimizer3DArm_collectorInsertAndMakeBase_367(nargout, out, nargin-1, in+1);
      break;
    case 368:
      gpmp2ISAM2TrajOptimizer3DArm_constructor_368(nargout, out, nargin-1, in+1);
      break;
    case 369:
      gpmp2ISAM2TrajOptimizer3DArm_deconstructor_369(nargout, out, nargin-1, in+1);
      break;
    case 370:
      gpmp2ISAM2TrajOptimizer3DArm_addPoseEstimate_370(nargout, out, nargin-1, in+1);
      break;
    case 371:
      gpmp2ISAM2TrajOptimizer3DArm_addStateEstimate_371(nargout, out, nargin-1, in+1);
      break;
    case 372:
      gpmp2ISAM2TrajOptimizer3DArm_changeGoalConfigAndVel_372(nargout, out, nargin-1, in+1);
      break;
    case 373:
      gpmp2ISAM2TrajOptimizer3DArm_fixConfigAndVel_373(nargout, out, nargin-1, in+1);
      break;
    case 374:
      gpmp2ISAM2TrajOptimizer3DArm_initFactorGraph_374(nargout, out, nargin-1, in+1);
      break;
    case 375:
      gpmp2ISAM2TrajOptimizer3DArm_initValues_375(nargout, out, nargin-1, in+1);
      break;
    case 376:
      gpmp2ISAM2TrajOptimizer3DArm_removeGoalConfigAndVel_376(nargout, out, nargin-1, in+1);
      break;
    case 377:
      gpmp2ISAM2TrajOptimizer3DArm_update_377(nargout, out, nargin-1, in+1);
      break;
    case 378:
      gpmp2ISAM2TrajOptimizer3DArm_values_378(nargout, out, nargin-1, in+1);
      break;
    case 379:
      gpmp2ISAM2TrajOptimizerPose2MobileArm2D_collectorInsertAndMakeBase_379(nargout, out, nargin-1, in+1);
      break;
    case 380:
      gpmp2ISAM2TrajOptimizerPose2MobileArm2D_constructor_380(nargout, out, nargin-1, in+1);
      break;
    case 381:
      gpmp2ISAM2TrajOptimizerPose2MobileArm2D_deconstructor_381(nargout, out, nargin-1, in+1);
      break;
    case 382:
      gpmp2ISAM2TrajOptimizerPose2MobileArm2D_addPoseEstimate_382(nargout, out, nargin-1, in+1);
      break;
    case 383:
      gpmp2ISAM2TrajOptimizerPose2MobileArm2D_addStateEstimate_383(nargout, out, nargin-1, in+1);
      break;
    case 384:
      gpmp2ISAM2TrajOptimizerPose2MobileArm2D_changeGoalConfigAndVel_384(nargout, out, nargin-1, in+1);
      break;
    case 385:
      gpmp2ISAM2TrajOptimizerPose2MobileArm2D_fixConfigAndVel_385(nargout, out, nargin-1, in+1);
      break;
    case 386:
      gpmp2ISAM2TrajOptimizerPose2MobileArm2D_initFactorGraph_386(nargout, out, nargin-1, in+1);
      break;
    case 387:
      gpmp2ISAM2TrajOptimizerPose2MobileArm2D_initValues_387(nargout, out, nargin-1, in+1);
      break;
    case 388:
      gpmp2ISAM2TrajOptimizerPose2MobileArm2D_removeGoalConfigAndVel_388(nargout, out, nargin-1, in+1);
      break;
    case 389:
      gpmp2ISAM2TrajOptimizerPose2MobileArm2D_update_389(nargout, out, nargin-1, in+1);
      break;
    case 390:
      gpmp2ISAM2TrajOptimizerPose2MobileArm2D_values_390(nargout, out, nargin-1, in+1);
      break;
    case 391:
      gpmp2ISAM2TrajOptimizerPose2MobileArm_collectorInsertAndMakeBase_391(nargout, out, nargin-1, in+1);
      break;
    case 392:
      gpmp2ISAM2TrajOptimizerPose2MobileArm_constructor_392(nargout, out, nargin-1, in+1);
      break;
    case 393:
      gpmp2ISAM2TrajOptimizerPose2MobileArm_deconstructor_393(nargout, out, nargin-1, in+1);
      break;
    case 394:
      gpmp2ISAM2TrajOptimizerPose2MobileArm_addPoseEstimate_394(nargout, out, nargin-1, in+1);
      break;
    case 395:
      gpmp2ISAM2TrajOptimizerPose2MobileArm_addStateEstimate_395(nargout, out, nargin-1, in+1);
      break;
    case 396:
      gpmp2ISAM2TrajOptimizerPose2MobileArm_changeGoalConfigAndVel_396(nargout, out, nargin-1, in+1);
      break;
    case 397:
      gpmp2ISAM2TrajOptimizerPose2MobileArm_fixConfigAndVel_397(nargout, out, nargin-1, in+1);
      break;
    case 398:
      gpmp2ISAM2TrajOptimizerPose2MobileArm_initFactorGraph_398(nargout, out, nargin-1, in+1);
      break;
    case 399:
      gpmp2ISAM2TrajOptimizerPose2MobileArm_initValues_399(nargout, out, nargin-1, in+1);
      break;
    case 400:
      gpmp2ISAM2TrajOptimizerPose2MobileArm_removeGoalConfigAndVel_400(nargout, out, nargin-1, in+1);
      break;
    case 401:
      gpmp2ISAM2TrajOptimizerPose2MobileArm_update_401(nargout, out, nargin-1, in+1);
      break;
    case 402:
      gpmp2ISAM2TrajOptimizerPose2MobileArm_values_402(nargout, out, nargin-1, in+1);
      break;
    case 403:
      gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_collectorInsertAndMakeBase_403(nargout, out, nargin-1, in+1);
      break;
    case 404:
      gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_constructor_404(nargout, out, nargin-1, in+1);
      break;
    case 405:
      gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_deconstructor_405(nargout, out, nargin-1, in+1);
      break;
    case 406:
      gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_addPoseEstimate_406(nargout, out, nargin-1, in+1);
      break;
    case 407:
      gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_addStateEstimate_407(nargout, out, nargin-1, in+1);
      break;
    case 408:
      gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_changeGoalConfigAndVel_408(nargout, out, nargin-1, in+1);
      break;
    case 409:
      gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_fixConfigAndVel_409(nargout, out, nargin-1, in+1);
      break;
    case 410:
      gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_initFactorGraph_410(nargout, out, nargin-1, in+1);
      break;
    case 411:
      gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_initValues_411(nargout, out, nargin-1, in+1);
      break;
    case 412:
      gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_removeGoalConfigAndVel_412(nargout, out, nargin-1, in+1);
      break;
    case 413:
      gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_update_413(nargout, out, nargin-1, in+1);
      break;
    case 414:
      gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms_values_414(nargout, out, nargin-1, in+1);
      break;
    case 415:
      gpmp2PriorFactorPose2Vector_collectorInsertAndMakeBase_415(nargout, out, nargin-1, in+1);
      break;
    case 416:
      gpmp2PriorFactorPose2Vector_upcastFromVoid_416(nargout, out, nargin-1, in+1);
      break;
    case 417:
      gpmp2PriorFactorPose2Vector_constructor_417(nargout, out, nargin-1, in+1);
      break;
    case 418:
      gpmp2PriorFactorPose2Vector_deconstructor_418(nargout, out, nargin-1, in+1);
      break;
    case 419:
      gpmp2BatchTrajOptimize2DArm_419(nargout, out, nargin-1, in+1);
      break;
    case 420:
      gpmp2BatchTrajOptimize3DArm_420(nargout, out, nargin-1, in+1);
      break;
    case 421:
      gpmp2BatchTrajOptimizePose2Mobile2Arms_421(nargout, out, nargin-1, in+1);
      break;
    case 422:
      gpmp2BatchTrajOptimizePose2MobileArm_422(nargout, out, nargin-1, in+1);
      break;
    case 423:
      gpmp2BatchTrajOptimizePose2MobileArm2D_423(nargout, out, nargin-1, in+1);
      break;
    case 424:
      gpmp2BatchTrajOptimizePose2MobileVetLin2Arms_424(nargout, out, nargin-1, in+1);
      break;
    case 425:
      gpmp2BatchTrajOptimizePose2MobileVetLinArm_425(nargout, out, nargin-1, in+1);
      break;
    case 426:
      gpmp2CollisionCost2DArm_426(nargout, out, nargin-1, in+1);
      break;
    case 427:
      gpmp2CollisionCost3DArm_427(nargout, out, nargin-1, in+1);
      break;
    case 428:
      gpmp2CollisionCostPose2Mobile2Arms_428(nargout, out, nargin-1, in+1);
      break;
    case 429:
      gpmp2CollisionCostPose2MobileArm_429(nargout, out, nargin-1, in+1);
      break;
    case 430:
      gpmp2CollisionCostPose2MobileArm2D_430(nargout, out, nargin-1, in+1);
      break;
    case 431:
      gpmp2CollisionCostPose2MobileBase_431(nargout, out, nargin-1, in+1);
      break;
    case 432:
      gpmp2CollisionCostPose2MobileBase2D_432(nargout, out, nargin-1, in+1);
      break;
    case 433:
      gpmp2CollisionCostPose2MobileVetLin2Arms_433(nargout, out, nargin-1, in+1);
      break;
    case 434:
      gpmp2CollisionCostPose2MobileVetLinArm_434(nargout, out, nargin-1, in+1);
      break;
    case 435:
      gpmp2atPose2VectorValues_435(nargout, out, nargin-1, in+1);
      break;
    case 436:
      gpmp2initArmTrajStraightLine_436(nargout, out, nargin-1, in+1);
      break;
    case 437:
      gpmp2initPose2TrajStraightLine_437(nargout, out, nargin-1, in+1);
      break;
    case 438:
      gpmp2initPose2VectorTrajStraightLine_438(nargout, out, nargin-1, in+1);
      break;
    case 439:
      gpmp2insertPose2VectorInValues_439(nargout, out, nargin-1, in+1);
      break;
    case 440:
      gpmp2interpolateArmTraj_440(nargout, out, nargin-1, in+1);
      break;
    case 441:
      gpmp2interpolateArmTraj_441(nargout, out, nargin-1, in+1);
      break;
    case 442:
      gpmp2interpolatePose2MobileArmTraj_442(nargout, out, nargin-1, in+1);
      break;
    case 443:
      gpmp2interpolatePose2Traj_443(nargout, out, nargin-1, in+1);
      break;
    case 444:
      gpmp2optimize_444(nargout, out, nargin-1, in+1);
      break;
    }
  } catch(const std::exception& e) {
    mexErrMsgTxt(("Exception from gtsam:\n" + std::string(e.what()) + "\n").c_str());
  }

  std::cout.rdbuf(outbuf);
}
