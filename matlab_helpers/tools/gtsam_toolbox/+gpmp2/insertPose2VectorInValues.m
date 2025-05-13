function varargout = insertPose2VectorInValues(varargin)
      if length(varargin) == 3 && isa(varargin{1},'numeric') && isa(varargin{2},'gpmp2.Pose2Vector') && isa(varargin{3},'gtsam.Values')
        gpmp2_wrapper(439, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.insertPose2VectorInValues');
      end
