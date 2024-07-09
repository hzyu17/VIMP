%class ISAM2TrajOptimizerPose2MobileVetLin2Arms, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%ISAM2TrajOptimizerPose2MobileVetLin2Arms(Pose2MobileVetLin2ArmsModel marm, SignedDistanceField sdf, TrajOptimizerSetting setting)
%
%-------Methods-------
%addPoseEstimate(size_t state_idx, Pose2Vector pose, Matrix pose_cov) : returns void
%addStateEstimate(size_t state_idx, Pose2Vector pose, Matrix pose_cov, Vector vel, Matrix vel_cov) : returns void
%changeGoalConfigAndVel(Pose2Vector goal_conf, Vector goal_vel) : returns void
%fixConfigAndVel(size_t state_idx, Pose2Vector conf_fix, Vector vel_fix) : returns void
%initFactorGraph(Pose2Vector start_conf, Vector start_vel, Pose2Vector goal_conf, Vector goal_vel) : returns void
%initValues(Values init_values) : returns void
%removeGoalConfigAndVel() : returns void
%update() : returns void
%values() : returns gtsam::Values
%
classdef ISAM2TrajOptimizerPose2MobileVetLin2Arms < handle
  properties
    ptr_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms = 0
  end
  methods
    function obj = ISAM2TrajOptimizerPose2MobileVetLin2Arms(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gpmp2_wrapper(403, my_ptr);
      elseif nargin == 3 && isa(varargin{1},'gpmp2.Pose2MobileVetLin2ArmsModel') && isa(varargin{2},'gpmp2.SignedDistanceField') && isa(varargin{3},'gpmp2.TrajOptimizerSetting')
        my_ptr = gpmp2_wrapper(404, varargin{1}, varargin{2}, varargin{3});
      else
        error('Arguments do not match any overload of gpmp2.ISAM2TrajOptimizerPose2MobileVetLin2Arms constructor');
      end
      obj.ptr_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(405, obj.ptr_gpmp2ISAM2TrajOptimizerPose2MobileVetLin2Arms);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = addPoseEstimate(this, varargin)
      % ADDPOSEESTIMATE usage: addPoseEstimate(size_t state_idx, Pose2Vector pose, Matrix pose_cov) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 3 && isa(varargin{1},'numeric') && isa(varargin{2},'gpmp2.Pose2Vector') && isa(varargin{3},'double')
        gpmp2_wrapper(406, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.ISAM2TrajOptimizerPose2MobileVetLin2Arms.addPoseEstimate');
      end
    end

    function varargout = addStateEstimate(this, varargin)
      % ADDSTATEESTIMATE usage: addStateEstimate(size_t state_idx, Pose2Vector pose, Matrix pose_cov, Vector vel, Matrix vel_cov) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 5 && isa(varargin{1},'numeric') && isa(varargin{2},'gpmp2.Pose2Vector') && isa(varargin{3},'double') && isa(varargin{4},'double') && size(varargin{4},2)==1 && isa(varargin{5},'double')
        gpmp2_wrapper(407, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.ISAM2TrajOptimizerPose2MobileVetLin2Arms.addStateEstimate');
      end
    end

    function varargout = changeGoalConfigAndVel(this, varargin)
      % CHANGEGOALCONFIGANDVEL usage: changeGoalConfigAndVel(Pose2Vector goal_conf, Vector goal_vel) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gpmp2.Pose2Vector') && isa(varargin{2},'double') && size(varargin{2},2)==1
        gpmp2_wrapper(408, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.ISAM2TrajOptimizerPose2MobileVetLin2Arms.changeGoalConfigAndVel');
      end
    end

    function varargout = fixConfigAndVel(this, varargin)
      % FIXCONFIGANDVEL usage: fixConfigAndVel(size_t state_idx, Pose2Vector conf_fix, Vector vel_fix) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 3 && isa(varargin{1},'numeric') && isa(varargin{2},'gpmp2.Pose2Vector') && isa(varargin{3},'double') && size(varargin{3},2)==1
        gpmp2_wrapper(409, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.ISAM2TrajOptimizerPose2MobileVetLin2Arms.fixConfigAndVel');
      end
    end

    function varargout = initFactorGraph(this, varargin)
      % INITFACTORGRAPH usage: initFactorGraph(Pose2Vector start_conf, Vector start_vel, Pose2Vector goal_conf, Vector goal_vel) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 4 && isa(varargin{1},'gpmp2.Pose2Vector') && isa(varargin{2},'double') && size(varargin{2},2)==1 && isa(varargin{3},'gpmp2.Pose2Vector') && isa(varargin{4},'double') && size(varargin{4},2)==1
        gpmp2_wrapper(410, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.ISAM2TrajOptimizerPose2MobileVetLin2Arms.initFactorGraph');
      end
    end

    function varargout = initValues(this, varargin)
      % INITVALUES usage: initValues(Values init_values) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Values')
        gpmp2_wrapper(411, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.ISAM2TrajOptimizerPose2MobileVetLin2Arms.initValues');
      end
    end

    function varargout = removeGoalConfigAndVel(this, varargin)
      % REMOVEGOALCONFIGANDVEL usage: removeGoalConfigAndVel() : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(412, this, varargin{:});
    end

    function varargout = update(this, varargin)
      % UPDATE usage: update() : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(413, this, varargin{:});
    end

    function varargout = values(this, varargin)
      % VALUES usage: values() : returns gtsam::Values
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(414, this, varargin{:});
    end

  end

  methods(Static = true)
  end
end
