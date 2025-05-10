%class Pose2MobileArmModel, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%Pose2MobileArmModel(Pose2MobileArm r, BodySphereVector spheres)
%
%-------Methods-------
%dof() : returns size_t
%fk_model() : returns gpmp2::Pose2MobileArm
%nr_body_spheres() : returns size_t
%sphereCentersMat(Pose2Vector conf) : returns Matrix
%sphere_radius(size_t i) : returns double
%
classdef Pose2MobileArmModel < handle
  properties
    ptr_gpmp2Pose2MobileArmModel = 0
  end
  methods
    function obj = Pose2MobileArmModel(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gpmp2_wrapper(131, my_ptr);
      elseif nargin == 2 && isa(varargin{1},'gpmp2.Pose2MobileArm') && isa(varargin{2},'gpmp2.BodySphereVector')
        my_ptr = gpmp2_wrapper(132, varargin{1}, varargin{2});
      else
        error('Arguments do not match any overload of gpmp2.Pose2MobileArmModel constructor');
      end
      obj.ptr_gpmp2Pose2MobileArmModel = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(133, obj.ptr_gpmp2Pose2MobileArmModel);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = dof(this, varargin)
      % DOF usage: dof() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(134, this, varargin{:});
    end

    function varargout = fk_model(this, varargin)
      % FK_MODEL usage: fk_model() : returns gpmp2::Pose2MobileArm
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(135, this, varargin{:});
    end

    function varargout = nr_body_spheres(this, varargin)
      % NR_BODY_SPHERES usage: nr_body_spheres() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(136, this, varargin{:});
    end

    function varargout = sphereCentersMat(this, varargin)
      % SPHERECENTERSMAT usage: sphereCentersMat(Pose2Vector conf) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gpmp2.Pose2Vector')
        varargout{1} = gpmp2_wrapper(137, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.Pose2MobileArmModel.sphereCentersMat');
      end
    end

    function varargout = sphere_radius(this, varargin)
      % SPHERE_RADIUS usage: sphere_radius(size_t i) : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(138, this, varargin{:});
    end

  end

  methods(Static = true)
  end
end
