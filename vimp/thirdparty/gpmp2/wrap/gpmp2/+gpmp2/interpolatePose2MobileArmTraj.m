function varargout = interpolatePose2MobileArmTraj(varargin)
      if length(varargin) == 6 && isa(varargin{1},'gtsam.Values') && isa(varargin{2},'gtsam.noiseModel.Base') && isa(varargin{3},'double') && isa(varargin{4},'numeric') && isa(varargin{5},'numeric') && isa(varargin{6},'numeric')
        varargout{1} = gpmp2_wrapper(442, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.interpolatePose2MobileArmTraj');
      end
