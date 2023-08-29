function varargout = BatchTrajOptimizePose2MobileVetLin2Arms(varargin)
      if length(varargin) == 8 && isa(varargin{1},'gpmp2.Pose2MobileVetLin2ArmsModel') && isa(varargin{2},'gpmp2.SignedDistanceField') && isa(varargin{3},'gpmp2.Pose2Vector') && isa(varargin{4},'double') && size(varargin{4},2)==1 && isa(varargin{5},'gpmp2.Pose2Vector') && isa(varargin{6},'double') && size(varargin{6},2)==1 && isa(varargin{7},'gtsam.Values') && isa(varargin{8},'gpmp2.TrajOptimizerSetting')
        varargout{1} = gpmp2_wrapper(424, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.BatchTrajOptimizePose2MobileVetLin2Arms');
      end
