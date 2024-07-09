%class Pose2Mobile2Arms, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%Pose2Mobile2Arms(Arm arm1, Arm arm2)
%Pose2Mobile2Arms(Arm arm1, Arm arm2, Pose3 base_T_arm1, Pose3 base_T_arm2)
%
%-------Methods-------
%arm1() : returns gpmp2::Arm
%arm2() : returns gpmp2::Arm
%base_T_arm1() : returns gtsam::Pose3
%base_T_arm2() : returns gtsam::Pose3
%dof() : returns size_t
%forwardKinematicsPose(Pose2Vector jp) : returns Matrix
%forwardKinematicsPosition(Pose2Vector jp) : returns Matrix
%forwardKinematicsVel(Pose2Vector jp, Vector jv) : returns Matrix
%nr_links() : returns size_t
%
classdef Pose2Mobile2Arms < handle
  properties
    ptr_gpmp2Pose2Mobile2Arms = 0
  end
  methods
    function obj = Pose2Mobile2Arms(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gpmp2_wrapper(59, my_ptr);
      elseif nargin == 2 && isa(varargin{1},'gpmp2.Arm') && isa(varargin{2},'gpmp2.Arm')
        my_ptr = gpmp2_wrapper(60, varargin{1}, varargin{2});
      elseif nargin == 4 && isa(varargin{1},'gpmp2.Arm') && isa(varargin{2},'gpmp2.Arm') && isa(varargin{3},'gtsam.Pose3') && isa(varargin{4},'gtsam.Pose3')
        my_ptr = gpmp2_wrapper(61, varargin{1}, varargin{2}, varargin{3}, varargin{4});
      else
        error('Arguments do not match any overload of gpmp2.Pose2Mobile2Arms constructor');
      end
      obj.ptr_gpmp2Pose2Mobile2Arms = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(62, obj.ptr_gpmp2Pose2Mobile2Arms);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = arm1(this, varargin)
      % ARM1 usage: arm1() : returns gpmp2::Arm
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(63, this, varargin{:});
    end

    function varargout = arm2(this, varargin)
      % ARM2 usage: arm2() : returns gpmp2::Arm
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(64, this, varargin{:});
    end

    function varargout = base_T_arm1(this, varargin)
      % BASE_T_ARM1 usage: base_T_arm1() : returns gtsam::Pose3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(65, this, varargin{:});
    end

    function varargout = base_T_arm2(this, varargin)
      % BASE_T_ARM2 usage: base_T_arm2() : returns gtsam::Pose3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(66, this, varargin{:});
    end

    function varargout = dof(this, varargin)
      % DOF usage: dof() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(67, this, varargin{:});
    end

    function varargout = forwardKinematicsPose(this, varargin)
      % FORWARDKINEMATICSPOSE usage: forwardKinematicsPose(Pose2Vector jp) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gpmp2.Pose2Vector')
        varargout{1} = gpmp2_wrapper(68, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.Pose2Mobile2Arms.forwardKinematicsPose');
      end
    end

    function varargout = forwardKinematicsPosition(this, varargin)
      % FORWARDKINEMATICSPOSITION usage: forwardKinematicsPosition(Pose2Vector jp) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gpmp2.Pose2Vector')
        varargout{1} = gpmp2_wrapper(69, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.Pose2Mobile2Arms.forwardKinematicsPosition');
      end
    end

    function varargout = forwardKinematicsVel(this, varargin)
      % FORWARDKINEMATICSVEL usage: forwardKinematicsVel(Pose2Vector jp, Vector jv) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gpmp2.Pose2Vector') && isa(varargin{2},'double') && size(varargin{2},2)==1
        varargout{1} = gpmp2_wrapper(70, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.Pose2Mobile2Arms.forwardKinematicsVel');
      end
    end

    function varargout = nr_links(this, varargin)
      % NR_LINKS usage: nr_links() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(71, this, varargin{:});
    end

  end

  methods(Static = true)
  end
end
