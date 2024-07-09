%class Pose2MobileArm, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%Pose2MobileArm(Arm arm)
%Pose2MobileArm(Arm arm, Pose3 base_T_arm)
%
%-------Methods-------
%arm() : returns gpmp2::Arm
%base_T_arm() : returns gtsam::Pose3
%dof() : returns size_t
%forwardKinematicsPose(Pose2Vector jp) : returns Matrix
%forwardKinematicsPosition(Pose2Vector jp) : returns Matrix
%forwardKinematicsVel(Pose2Vector jp, Vector jv) : returns Matrix
%nr_links() : returns size_t
%
classdef Pose2MobileArm < handle
  properties
    ptr_gpmp2Pose2MobileArm = 0
  end
  methods
    function obj = Pose2MobileArm(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gpmp2_wrapper(48, my_ptr);
      elseif nargin == 1 && isa(varargin{1},'gpmp2.Arm')
        my_ptr = gpmp2_wrapper(49, varargin{1});
      elseif nargin == 2 && isa(varargin{1},'gpmp2.Arm') && isa(varargin{2},'gtsam.Pose3')
        my_ptr = gpmp2_wrapper(50, varargin{1}, varargin{2});
      else
        error('Arguments do not match any overload of gpmp2.Pose2MobileArm constructor');
      end
      obj.ptr_gpmp2Pose2MobileArm = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(51, obj.ptr_gpmp2Pose2MobileArm);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = arm(this, varargin)
      % ARM usage: arm() : returns gpmp2::Arm
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(52, this, varargin{:});
    end

    function varargout = base_T_arm(this, varargin)
      % BASE_T_ARM usage: base_T_arm() : returns gtsam::Pose3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(53, this, varargin{:});
    end

    function varargout = dof(this, varargin)
      % DOF usage: dof() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(54, this, varargin{:});
    end

    function varargout = forwardKinematicsPose(this, varargin)
      % FORWARDKINEMATICSPOSE usage: forwardKinematicsPose(Pose2Vector jp) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gpmp2.Pose2Vector')
        varargout{1} = gpmp2_wrapper(55, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.Pose2MobileArm.forwardKinematicsPose');
      end
    end

    function varargout = forwardKinematicsPosition(this, varargin)
      % FORWARDKINEMATICSPOSITION usage: forwardKinematicsPosition(Pose2Vector jp) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gpmp2.Pose2Vector')
        varargout{1} = gpmp2_wrapper(56, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.Pose2MobileArm.forwardKinematicsPosition');
      end
    end

    function varargout = forwardKinematicsVel(this, varargin)
      % FORWARDKINEMATICSVEL usage: forwardKinematicsVel(Pose2Vector jp, Vector jv) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gpmp2.Pose2Vector') && isa(varargin{2},'double') && size(varargin{2},2)==1
        varargout{1} = gpmp2_wrapper(57, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.Pose2MobileArm.forwardKinematicsVel');
      end
    end

    function varargout = nr_links(this, varargin)
      % NR_LINKS usage: nr_links() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(58, this, varargin{:});
    end

  end

  methods(Static = true)
  end
end
