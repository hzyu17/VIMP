function varargout = initPose2VectorTrajStraightLine(varargin)
      if length(varargin) == 5 && isa(varargin{1},'gtsam.Pose2') && isa(varargin{2},'double') && size(varargin{2},2)==1 && isa(varargin{3},'gtsam.Pose2') && isa(varargin{4},'double') && size(varargin{4},2)==1 && isa(varargin{5},'numeric')
        varargout{1} = gpmp2_wrapper(438, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.initPose2VectorTrajStraightLine');
      end
