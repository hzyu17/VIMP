function varargout = initArmTrajStraightLine(varargin)
      if length(varargin) == 3 && isa(varargin{1},'double') && size(varargin{1},2)==1 && isa(varargin{2},'double') && size(varargin{2},2)==1 && isa(varargin{3},'numeric')
        varargout{1} = gpmp2_wrapper(436, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.initArmTrajStraightLine');
      end
