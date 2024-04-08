%class Point2Vector, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%Point2Vector()
%Point2Vector(Point2Vector v)
%
%-------Methods-------
%assign(size_t n, Point2 u) : returns void
%at(size_t n) : returns gtsam::Point2
%back() : returns gtsam::Point2
%capacity() : returns size_t
%empty() : returns bool
%front() : returns gtsam::Point2
%max_size() : returns size_t
%pop_back() : returns void
%push_back(Point2 x) : returns void
%reserve(size_t n) : returns void
%resize(size_t sz) : returns void
%size() : returns size_t
%
classdef Point2Vector < handle
  properties
    ptr_gtsamPoint2Vector = 0
  end
  methods
    function obj = Point2Vector(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gtsam_wrapper(128, my_ptr);
      elseif nargin == 0
        my_ptr = gtsam_wrapper(129);
      elseif nargin == 1 && isa(varargin{1},'gtsam.Point2Vector')
        my_ptr = gtsam_wrapper(130, varargin{1});
      else
        error('Arguments do not match any overload of gtsam.Point2Vector constructor');
      end
      obj.ptr_gtsamPoint2Vector = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(131, obj.ptr_gtsamPoint2Vector);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = assign(this, varargin)
      % ASSIGN usage: assign(size_t n, Point2 u) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Point2')
        gtsam_wrapper(132, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Point2Vector.assign');
      end
    end

    function varargout = at(this, varargin)
      % AT usage: at(size_t n) : returns gtsam::Point2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(133, this, varargin{:});
    end

    function varargout = back(this, varargin)
      % BACK usage: back() : returns gtsam::Point2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(134, this, varargin{:});
    end

    function varargout = capacity(this, varargin)
      % CAPACITY usage: capacity() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(135, this, varargin{:});
    end

    function varargout = empty(this, varargin)
      % EMPTY usage: empty() : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(136, this, varargin{:});
    end

    function varargout = front(this, varargin)
      % FRONT usage: front() : returns gtsam::Point2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(137, this, varargin{:});
    end

    function varargout = max_size(this, varargin)
      % MAX_SIZE usage: max_size() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(138, this, varargin{:});
    end

    function varargout = pop_back(this, varargin)
      % POP_BACK usage: pop_back() : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(139, this, varargin{:});
    end

    function varargout = push_back(this, varargin)
      % PUSH_BACK usage: push_back(Point2 x) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point2')
        gtsam_wrapper(140, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Point2Vector.push_back');
      end
    end

    function varargout = reserve(this, varargin)
      % RESERVE usage: reserve(size_t n) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(141, this, varargin{:});
    end

    function varargout = resize(this, varargin)
      % RESIZE usage: resize(size_t sz) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(142, this, varargin{:});
    end

    function varargout = size(this, varargin)
      % SIZE usage: size() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(143, this, varargin{:});
    end

  end

  methods(Static = true)
  end
end
