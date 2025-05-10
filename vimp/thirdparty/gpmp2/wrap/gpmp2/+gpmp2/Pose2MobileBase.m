%class Pose2MobileBase, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%Pose2MobileBase()
%
%-------Methods-------
%dof() : returns size_t
%forwardKinematicsPose(Pose2 jp) : returns Matrix
%forwardKinematicsPosition(Pose2 jp) : returns Matrix
%forwardKinematicsVel(Pose2 jp, Vector jv) : returns Matrix
%nr_links() : returns size_t
%
classdef Pose2MobileBase < handle
  properties
    ptr_gpmp2Pose2MobileBase = 0
  end
  methods
    function obj = Pose2MobileBase(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gpmp2_wrapper(40, my_ptr);
      elseif nargin == 0
        my_ptr = gpmp2_wrapper(41);
      else
        error('Arguments do not match any overload of gpmp2.Pose2MobileBase constructor');
      end
      obj.ptr_gpmp2Pose2MobileBase = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(42, obj.ptr_gpmp2Pose2MobileBase);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = dof(this, varargin)
      % DOF usage: dof() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(43, this, varargin{:});
    end

    function varargout = forwardKinematicsPose(this, varargin)
      % FORWARDKINEMATICSPOSE usage: forwardKinematicsPose(Pose2 jp) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Pose2')
        varargout{1} = gpmp2_wrapper(44, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.Pose2MobileBase.forwardKinematicsPose');
      end
    end

    function varargout = forwardKinematicsPosition(this, varargin)
      % FORWARDKINEMATICSPOSITION usage: forwardKinematicsPosition(Pose2 jp) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Pose2')
        varargout{1} = gpmp2_wrapper(45, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.Pose2MobileBase.forwardKinematicsPosition');
      end
    end

    function varargout = forwardKinematicsVel(this, varargin)
      % FORWARDKINEMATICSVEL usage: forwardKinematicsVel(Pose2 jp, Vector jv) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gtsam.Pose2') && isa(varargin{2},'double') && size(varargin{2},2)==1
        varargout{1} = gpmp2_wrapper(46, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.Pose2MobileBase.forwardKinematicsVel');
      end
    end

    function varargout = nr_links(this, varargin)
      % NR_LINKS usage: nr_links() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(47, this, varargin{:});
    end

  end

  methods(Static = true)
  end
end
