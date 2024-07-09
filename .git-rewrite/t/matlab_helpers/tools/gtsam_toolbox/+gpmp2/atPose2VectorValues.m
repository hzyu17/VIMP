function varargout = atPose2VectorValues(varargin)
      if length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Values')
        varargout{1} = gpmp2_wrapper(435, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.atPose2VectorValues');
      end
