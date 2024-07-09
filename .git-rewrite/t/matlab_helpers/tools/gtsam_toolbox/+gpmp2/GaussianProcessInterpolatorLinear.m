%class GaussianProcessInterpolatorLinear, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%GaussianProcessInterpolatorLinear(Base Qc_model, double delta_t, double tau)
%
%-------Methods-------
%interpolatePose(Vector pose1, Vector vel1, Vector pose2, Vector vel2) : returns Vector
%interpolateVelocity(Vector pose1, Vector vel1, Vector pose2, Vector vel2) : returns Vector
%
classdef GaussianProcessInterpolatorLinear < handle
  properties
    ptr_gpmp2GaussianProcessInterpolatorLinear = 0
  end
  methods
    function obj = GaussianProcessInterpolatorLinear(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gpmp2_wrapper(22, my_ptr);
      elseif nargin == 3 && isa(varargin{1},'gtsam.noiseModel.Base') && isa(varargin{2},'double') && isa(varargin{3},'double')
        my_ptr = gpmp2_wrapper(23, varargin{1}, varargin{2}, varargin{3});
      else
        error('Arguments do not match any overload of gpmp2.GaussianProcessInterpolatorLinear constructor');
      end
      obj.ptr_gpmp2GaussianProcessInterpolatorLinear = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(24, obj.ptr_gpmp2GaussianProcessInterpolatorLinear);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = interpolatePose(this, varargin)
      % INTERPOLATEPOSE usage: interpolatePose(Vector pose1, Vector vel1, Vector pose2, Vector vel2) : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 4 && isa(varargin{1},'double') && size(varargin{1},2)==1 && isa(varargin{2},'double') && size(varargin{2},2)==1 && isa(varargin{3},'double') && size(varargin{3},2)==1 && isa(varargin{4},'double') && size(varargin{4},2)==1
        varargout{1} = gpmp2_wrapper(25, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.GaussianProcessInterpolatorLinear.interpolatePose');
      end
    end

    function varargout = interpolateVelocity(this, varargin)
      % INTERPOLATEVELOCITY usage: interpolateVelocity(Vector pose1, Vector vel1, Vector pose2, Vector vel2) : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 4 && isa(varargin{1},'double') && size(varargin{1},2)==1 && isa(varargin{2},'double') && size(varargin{2},2)==1 && isa(varargin{3},'double') && size(varargin{3},2)==1 && isa(varargin{4},'double') && size(varargin{4},2)==1
        varargout{1} = gpmp2_wrapper(26, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.GaussianProcessInterpolatorLinear.interpolateVelocity');
      end
    end

  end

  methods(Static = true)
  end
end
