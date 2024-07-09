function varargout = initPose2TrajStraightLine(varargin)
      if length(varargin) == 3 && isa(varargin{1},'gtsam.Pose2') && isa(varargin{2},'gtsam.Pose2') && isa(varargin{3},'numeric')
        varargout{1} = gpmp2_wrapper(437, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.initPose2TrajStraightLine');
      end
