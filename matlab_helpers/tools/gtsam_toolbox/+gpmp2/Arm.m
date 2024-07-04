%class Arm, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%Arm(size_t dof, Vector a, Vector alpha, Vector d)
%Arm(size_t dof, Vector a, Vector alpha, Vector d, Pose3 base_pose)
%Arm(size_t dof, Vector a, Vector alpha, Vector d, Pose3 base_pose, Vector theta_bias)
%
%-------Methods-------
%a() : returns Vector
%alpha() : returns Vector
%base_pose() : returns gtsam::Pose3
%d() : returns Vector
%dof() : returns size_t
%forwardKinematicsPose(Vector jp) : returns Matrix
%forwardKinematicsPosition(Vector jp) : returns Matrix
%forwardKinematicsVel(Vector jp, Vector jv) : returns Matrix
%
classdef Arm < handle
  properties
    ptr_gpmp2Arm = 0
  end
  methods
    function obj = Arm(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gpmp2_wrapper(27, my_ptr);
      elseif nargin == 4 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'double') && isa(varargin{4},'double')
        my_ptr = gpmp2_wrapper(28, varargin{1}, varargin{2}, varargin{3}, varargin{4});
      elseif nargin == 5 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'double') && isa(varargin{4},'double') && isa(varargin{5},'gtsam.Pose3')
        my_ptr = gpmp2_wrapper(29, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5});
      elseif nargin == 6 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'double') && isa(varargin{4},'double') && isa(varargin{5},'gtsam.Pose3') && isa(varargin{6},'double')
        my_ptr = gpmp2_wrapper(30, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5}, varargin{6});
      else
        error('Arguments do not match any overload of gpmp2.Arm constructor');
      end
      obj.ptr_gpmp2Arm = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(31, obj.ptr_gpmp2Arm);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = a(this, varargin)
      % A usage: a() : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(32, this, varargin{:});
    end

    function varargout = alpha(this, varargin)
      % ALPHA usage: alpha() : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(33, this, varargin{:});
    end

    function varargout = base_pose(this, varargin)
      % BASE_POSE usage: base_pose() : returns gtsam::Pose3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(34, this, varargin{:});
    end

    function varargout = d(this, varargin)
      % D usage: d() : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(35, this, varargin{:});
    end

    function varargout = dof(this, varargin)
      % DOF usage: dof() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(36, this, varargin{:});
    end

    function varargout = forwardKinematicsPose(this, varargin)
      % FORWARDKINEMATICSPOSE usage: forwardKinematicsPose(Vector jp) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = gpmp2_wrapper(37, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.Arm.forwardKinematicsPose');
      end
    end

    function varargout = forwardKinematicsPosition(this, varargin)
      % FORWARDKINEMATICSPOSITION usage: forwardKinematicsPosition(Vector jp) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = gpmp2_wrapper(38, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.Arm.forwardKinematicsPosition');
      end
    end

    function varargout = forwardKinematicsVel(this, varargin)
      % FORWARDKINEMATICSVEL usage: forwardKinematicsVel(Vector jp, Vector jv) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'double') && size(varargin{1},2)==1 && isa(varargin{2},'double') && size(varargin{2},2)==1
        varargout{1} = gpmp2_wrapper(39, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.Arm.forwardKinematicsVel');
      end
    end

  end

  methods(Static = true)
  end
end
