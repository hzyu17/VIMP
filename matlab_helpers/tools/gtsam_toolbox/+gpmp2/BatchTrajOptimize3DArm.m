function varargout = BatchTrajOptimize3DArm(varargin)
      if length(varargin) == 8 && isa(varargin{1},'gpmp2.ArmModel') && isa(varargin{2},'gpmp2.SignedDistanceField') && isa(varargin{3},'double') && size(varargin{3},2)==1 && isa(varargin{4},'double') && size(varargin{4},2)==1 && isa(varargin{5},'double') && size(varargin{5},2)==1 && isa(varargin{6},'double') && size(varargin{6},2)==1 && isa(varargin{7},'gtsam.Values') && isa(varargin{8},'gpmp2.TrajOptimizerSetting')
        varargout{1} = gpmp2_wrapper(420, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.BatchTrajOptimize3DArm');
      end
