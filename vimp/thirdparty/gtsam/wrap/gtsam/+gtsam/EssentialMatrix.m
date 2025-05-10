%class EssentialMatrix, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%EssentialMatrix(Rot3 aRb, Unit3 aTb)
%
%-------Methods-------
%dim() : returns size_t
%direction() : returns gtsam::Unit3
%equals(EssentialMatrix pose, double tol) : returns bool
%error(Vector vA, Vector vB) : returns double
%localCoordinates(EssentialMatrix s) : returns Vector
%matrix() : returns Matrix
%print(string s) : returns void
%retract(Vector v) : returns gtsam::EssentialMatrix
%rotation() : returns gtsam::Rot3
%
%-------Static Methods-------
%Dim() : returns size_t
%
classdef EssentialMatrix < handle
  properties
    ptr_gtsamEssentialMatrix = 0
  end
  methods
    function obj = EssentialMatrix(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gtsam_wrapper(333, my_ptr);
      elseif nargin == 2 && isa(varargin{1},'gtsam.Rot3') && isa(varargin{2},'gtsam.Unit3')
        my_ptr = gtsam_wrapper(334, varargin{1}, varargin{2});
      else
        error('Arguments do not match any overload of gtsam.EssentialMatrix constructor');
      end
      obj.ptr_gtsamEssentialMatrix = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(335, obj.ptr_gtsamEssentialMatrix);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = dim(this, varargin)
      % DIM usage: dim() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(336, this, varargin{:});
    end

    function varargout = direction(this, varargin)
      % DIRECTION usage: direction() : returns gtsam::Unit3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(337, this, varargin{:});
    end

    function varargout = equals(this, varargin)
      % EQUALS usage: equals(EssentialMatrix pose, double tol) : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gtsam.EssentialMatrix') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(338, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.EssentialMatrix.equals');
      end
    end

    function varargout = error(this, varargin)
      % ERROR usage: error(Vector vA, Vector vB) : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'double') && size(varargin{1},2)==1 && isa(varargin{2},'double') && size(varargin{2},2)==1
        varargout{1} = gtsam_wrapper(339, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.EssentialMatrix.error');
      end
    end

    function varargout = localCoordinates(this, varargin)
      % LOCALCOORDINATES usage: localCoordinates(EssentialMatrix s) : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.EssentialMatrix')
        varargout{1} = gtsam_wrapper(340, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.EssentialMatrix.localCoordinates');
      end
    end

    function varargout = matrix(this, varargin)
      % MATRIX usage: matrix() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(341, this, varargin{:});
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(342, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.EssentialMatrix.print');
      end
    end

    function varargout = retract(this, varargin)
      % RETRACT usage: retract(Vector v) : returns gtsam::EssentialMatrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = gtsam_wrapper(343, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.EssentialMatrix.retract');
      end
    end

    function varargout = rotation(this, varargin)
      % ROTATION usage: rotation() : returns gtsam::Rot3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(344, this, varargin{:});
    end

  end

  methods(Static = true)
    function varargout = Dim(varargin)
      % DIM usage: Dim() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(345, varargin{:});
    end

  end
end
