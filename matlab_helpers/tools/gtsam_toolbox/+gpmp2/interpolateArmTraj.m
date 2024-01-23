function varargout = interpolateArmTraj(varargin)
      if length(varargin) == 4 && isa(varargin{1},'gtsam.Values') && isa(varargin{2},'gtsam.noiseModel.Base') && isa(varargin{3},'double') && isa(varargin{4},'numeric')
        varargout{1} = gpmp2_wrapper(440, varargin{:});
      elseif length(varargin) == 6 && isa(varargin{1},'gtsam.Values') && isa(varargin{2},'gtsam.noiseModel.Base') && isa(varargin{3},'double') && isa(varargin{4},'numeric') && isa(varargin{5},'numeric') && isa(varargin{6},'numeric')
        varargout{1} = gpmp2_wrapper(441, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.interpolateArmTraj');
      end
