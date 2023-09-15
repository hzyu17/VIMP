%class ObstacleSDFFactorPose2MobileBase, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%ObstacleSDFFactorPose2MobileBase(size_t posekey, Pose2MobileBaseModel robot, SignedDistanceField sdf, double cost_sigma, double epsilon)
%
%-------Methods-------
%evaluateError(Pose2 pose) : returns Vector
%
classdef ObstacleSDFFactorPose2MobileBase < gtsam.NoiseModelFactor
  properties
    ptr_gpmp2ObstacleSDFFactorPose2MobileBase = 0
  end
  methods
    function obj = ObstacleSDFFactorPose2MobileBase(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gpmp2_wrapper(278, varargin{2});
        end
        base_ptr = gpmp2_wrapper(277, my_ptr);
      elseif nargin == 5 && isa(varargin{1},'numeric') && isa(varargin{2},'gpmp2.Pose2MobileBaseModel') && isa(varargin{3},'gpmp2.SignedDistanceField') && isa(varargin{4},'double') && isa(varargin{5},'double')
        [ my_ptr, base_ptr ] = gpmp2_wrapper(279, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5});
      else
        error('Arguments do not match any overload of gpmp2.ObstacleSDFFactorPose2MobileBase constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gpmp2ObstacleSDFFactorPose2MobileBase = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(280, obj.ptr_gpmp2ObstacleSDFFactorPose2MobileBase);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = evaluateError(this, varargin)
      % EVALUATEERROR usage: evaluateError(Pose2 pose) : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Pose2')
        varargout{1} = gpmp2_wrapper(281, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.ObstacleSDFFactorPose2MobileBase.evaluateError');
      end
    end

  end

  methods(Static = true)
  end
end
