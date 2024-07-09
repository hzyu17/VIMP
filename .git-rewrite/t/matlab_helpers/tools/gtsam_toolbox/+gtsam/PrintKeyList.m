function varargout = PrintKeyList(varargin)
      if length(varargin) == 1 && isa(varargin{1},'gtsam.KeyList')
        gtsam_wrapper(2834, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'gtsam.KeyList') && isa(varargin{2},'char')
        gtsam_wrapper(2835, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.PrintKeyList');
      end
