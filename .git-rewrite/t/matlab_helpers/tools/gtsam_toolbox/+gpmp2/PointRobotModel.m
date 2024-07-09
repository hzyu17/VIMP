%class PointRobotModel, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%PointRobotModel(PointRobot pR, BodySphereVector spheres)
%
%-------Methods-------
%dof() : returns size_t
%fk_model() : returns gpmp2::PointRobot
%nr_body_spheres() : returns size_t
%sphereCentersMat(Vector conf) : returns Matrix
%sphere_radius(size_t i) : returns double
%
classdef PointRobotModel < handle
  properties
    ptr_gpmp2PointRobotModel = 0
  end
  methods
    function obj = PointRobotModel(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gpmp2_wrapper(163, my_ptr);
      elseif nargin == 2 && isa(varargin{1},'gpmp2.PointRobot') && isa(varargin{2},'gpmp2.BodySphereVector')
        my_ptr = gpmp2_wrapper(164, varargin{1}, varargin{2});
      else
        error('Arguments do not match any overload of gpmp2.PointRobotModel constructor');
      end
      obj.ptr_gpmp2PointRobotModel = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(165, obj.ptr_gpmp2PointRobotModel);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = dof(this, varargin)
      % DOF usage: dof() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(166, this, varargin{:});
    end

    function varargout = fk_model(this, varargin)
      % FK_MODEL usage: fk_model() : returns gpmp2::PointRobot
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(167, this, varargin{:});
    end

    function varargout = nr_body_spheres(this, varargin)
      % NR_BODY_SPHERES usage: nr_body_spheres() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(168, this, varargin{:});
    end

    function varargout = sphereCentersMat(this, varargin)
      % SPHERECENTERSMAT usage: sphereCentersMat(Vector conf) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = gpmp2_wrapper(169, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.PointRobotModel.sphereCentersMat');
      end
    end

    function varargout = sphere_radius(this, varargin)
      % SPHERE_RADIUS usage: sphere_radius(size_t i) : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(170, this, varargin{:});
    end

  end

  methods(Static = true)
  end
end
