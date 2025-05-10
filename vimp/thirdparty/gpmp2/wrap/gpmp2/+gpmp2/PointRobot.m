%class PointRobot, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%PointRobot(size_t dof, size_t nr_links)
%
%-------Methods-------
%dof() : returns size_t
%forwardKinematicsPose(Vector jp) : returns Matrix
%forwardKinematicsPosition(Vector jp) : returns Matrix
%forwardKinematicsVel(Vector jp, Vector jv) : returns Matrix
%nr_links() : returns size_t
%
classdef PointRobot < handle
  properties
    ptr_gpmp2PointRobot = 0
  end
  methods
    function obj = PointRobot(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gpmp2_wrapper(100, my_ptr);
      elseif nargin == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric')
        my_ptr = gpmp2_wrapper(101, varargin{1}, varargin{2});
      else
        error('Arguments do not match any overload of gpmp2.PointRobot constructor');
      end
      obj.ptr_gpmp2PointRobot = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(102, obj.ptr_gpmp2PointRobot);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = dof(this, varargin)
      % DOF usage: dof() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(103, this, varargin{:});
    end

    function varargout = forwardKinematicsPose(this, varargin)
      % FORWARDKINEMATICSPOSE usage: forwardKinematicsPose(Vector jp) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = gpmp2_wrapper(104, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.PointRobot.forwardKinematicsPose');
      end
    end

    function varargout = forwardKinematicsPosition(this, varargin)
      % FORWARDKINEMATICSPOSITION usage: forwardKinematicsPosition(Vector jp) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = gpmp2_wrapper(105, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.PointRobot.forwardKinematicsPosition');
      end
    end

    function varargout = forwardKinematicsVel(this, varargin)
      % FORWARDKINEMATICSVEL usage: forwardKinematicsVel(Vector jp, Vector jv) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'double') && size(varargin{1},2)==1 && isa(varargin{2},'double') && size(varargin{2},2)==1
        varargout{1} = gpmp2_wrapper(106, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.PointRobot.forwardKinematicsVel');
      end
    end

    function varargout = nr_links(this, varargin)
      % NR_LINKS usage: nr_links() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(107, this, varargin{:});
    end

  end

  methods(Static = true)
  end
end
