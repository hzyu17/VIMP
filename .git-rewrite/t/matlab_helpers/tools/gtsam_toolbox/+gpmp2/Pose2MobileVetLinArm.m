%class Pose2MobileVetLinArm, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%Pose2MobileVetLinArm(Arm arm)
%Pose2MobileVetLinArm(Arm arm, Pose3 base_T_torso, Pose3 torso_T_arm, bool reverse_linact)
%
%-------Methods-------
%arm() : returns gpmp2::Arm
%base_T_torso() : returns gtsam::Pose3
%dof() : returns size_t
%forwardKinematicsPose(Pose2Vector jp) : returns Matrix
%forwardKinematicsPosition(Pose2Vector jp) : returns Matrix
%forwardKinematicsVel(Pose2Vector jp, Vector jv) : returns Matrix
%nr_links() : returns size_t
%reverse_linact() : returns bool
%torso_T_arm() : returns gtsam::Pose3
%
classdef Pose2MobileVetLinArm < handle
  properties
    ptr_gpmp2Pose2MobileVetLinArm = 0
  end
  methods
    function obj = Pose2MobileVetLinArm(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gpmp2_wrapper(72, my_ptr);
      elseif nargin == 1 && isa(varargin{1},'gpmp2.Arm')
        my_ptr = gpmp2_wrapper(73, varargin{1});
      elseif nargin == 4 && isa(varargin{1},'gpmp2.Arm') && isa(varargin{2},'gtsam.Pose3') && isa(varargin{3},'gtsam.Pose3') && isa(varargin{4},'logical')
        my_ptr = gpmp2_wrapper(74, varargin{1}, varargin{2}, varargin{3}, varargin{4});
      else
        error('Arguments do not match any overload of gpmp2.Pose2MobileVetLinArm constructor');
      end
      obj.ptr_gpmp2Pose2MobileVetLinArm = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(75, obj.ptr_gpmp2Pose2MobileVetLinArm);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = arm(this, varargin)
      % ARM usage: arm() : returns gpmp2::Arm
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(76, this, varargin{:});
    end

    function varargout = base_T_torso(this, varargin)
      % BASE_T_TORSO usage: base_T_torso() : returns gtsam::Pose3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(77, this, varargin{:});
    end

    function varargout = dof(this, varargin)
      % DOF usage: dof() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(78, this, varargin{:});
    end

    function varargout = forwardKinematicsPose(this, varargin)
      % FORWARDKINEMATICSPOSE usage: forwardKinematicsPose(Pose2Vector jp) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gpmp2.Pose2Vector')
        varargout{1} = gpmp2_wrapper(79, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.Pose2MobileVetLinArm.forwardKinematicsPose');
      end
    end

    function varargout = forwardKinematicsPosition(this, varargin)
      % FORWARDKINEMATICSPOSITION usage: forwardKinematicsPosition(Pose2Vector jp) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gpmp2.Pose2Vector')
        varargout{1} = gpmp2_wrapper(80, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.Pose2MobileVetLinArm.forwardKinematicsPosition');
      end
    end

    function varargout = forwardKinematicsVel(this, varargin)
      % FORWARDKINEMATICSVEL usage: forwardKinematicsVel(Pose2Vector jp, Vector jv) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gpmp2.Pose2Vector') && isa(varargin{2},'double') && size(varargin{2},2)==1
        varargout{1} = gpmp2_wrapper(81, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.Pose2MobileVetLinArm.forwardKinematicsVel');
      end
    end

    function varargout = nr_links(this, varargin)
      % NR_LINKS usage: nr_links() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(82, this, varargin{:});
    end

    function varargout = reverse_linact(this, varargin)
      % REVERSE_LINACT usage: reverse_linact() : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(83, this, varargin{:});
    end

    function varargout = torso_T_arm(this, varargin)
      % TORSO_T_ARM usage: torso_T_arm() : returns gtsam::Pose3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(84, this, varargin{:});
    end

  end

  methods(Static = true)
  end
end
