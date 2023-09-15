function varargout = createKeyList(varargin)
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = gtsam_wrapper(2842, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'char') && isa(varargin{2},'double') && size(varargin{2},2)==1
        varargout{1} = gtsam_wrapper(2843, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.utilities.createKeyList');
      end
