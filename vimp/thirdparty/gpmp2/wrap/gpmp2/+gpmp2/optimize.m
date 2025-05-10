function varargout = optimize(varargin)
      if length(varargin) == 4 && isa(varargin{1},'gtsam.NonlinearFactorGraph') && isa(varargin{2},'gtsam.Values') && isa(varargin{3},'gpmp2.TrajOptimizerSetting') && isa(varargin{4},'logical')
        varargout{1} = gpmp2_wrapper(444, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.optimize');
      end
