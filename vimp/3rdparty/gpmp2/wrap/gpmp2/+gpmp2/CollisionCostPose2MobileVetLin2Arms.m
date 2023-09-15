function varargout = CollisionCostPose2MobileVetLin2Arms(varargin)
      if length(varargin) == 4 && isa(varargin{1},'gpmp2.Pose2MobileVetLin2ArmsModel') && isa(varargin{2},'gpmp2.SignedDistanceField') && isa(varargin{3},'gtsam.Values') && isa(varargin{4},'gpmp2.TrajOptimizerSetting')
        varargout{1} = gpmp2_wrapper(433, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.CollisionCostPose2MobileVetLin2Arms');
      end
