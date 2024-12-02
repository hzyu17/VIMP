%class Pose2Vector, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%Pose2Vector()
%Pose2Vector(Pose2 pose, Vector c)
%
%-------Methods-------
%configuration() : returns Vector
%pose() : returns gtsam::Pose2
%print(string s) : returns void
%
classdef Pose2Vector < handle
  properties
    ptr_gpmp2Pose2Vector = 0
  end
  methods
    function obj = Pose2Vector(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gpmp2_wrapper(0, my_ptr);
      elseif nargin == 0
        my_ptr = gpmp2_wrapper(1);
      elseif nargin == 2 && isa(varargin{1},'gtsam.Pose2') && isa(varargin{2},'double')
        my_ptr = gpmp2_wrapper(2, varargin{1}, varargin{2});
      else
        error('Arguments do not match any overload of gpmp2.Pose2Vector constructor');
      end
      obj.ptr_gpmp2Pose2Vector = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(3, obj.ptr_gpmp2Pose2Vector);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = configuration(this, varargin)
      % CONFIGURATION usage: configuration() : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(4, this, varargin{:});
    end

    function varargout = pose(this, varargin)
      % POSE usage: pose() : returns gtsam::Pose2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gpmp2_wrapper(5, this, varargin{:});
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gpmp2_wrapper(6, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.Pose2Vector.print');
      end
    end

  end

  methods(Static = true)
  end
end
