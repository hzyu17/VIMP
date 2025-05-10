function varargout = PrintKeyVector(varargin)
      if length(varargin) == 1 && isa(varargin{1},'gtsam.KeyVector')
        gtsam_wrapper(2838, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'gtsam.KeyVector') && isa(varargin{2},'char')
        gtsam_wrapper(2839, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.PrintKeyVector');
      end
