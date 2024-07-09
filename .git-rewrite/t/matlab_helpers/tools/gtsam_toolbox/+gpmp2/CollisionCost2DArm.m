function varargout = CollisionCost2DArm(varargin)
      if length(varargin) == 4 && isa(varargin{1},'gpmp2.ArmModel') && isa(varargin{2},'gpmp2.PlanarSDF') && isa(varargin{3},'gtsam.Values') && isa(varargin{4},'gpmp2.TrajOptimizerSetting')
        varargout{1} = gpmp2_wrapper(426, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.CollisionCost2DArm');
      end
