%class PlanarSDF, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%PlanarSDF(Point2 origin, double cell_size, Matrix data)
%
%-------Methods-------
%getSignedDistance(Point2 point) : returns double
%print(string s) : returns void
%
classdef PlanarSDF < handle
  properties
    ptr_gpmp2PlanarSDF = 0
  end
  methods
    function obj = PlanarSDF(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gpmp2_wrapper(216, my_ptr);
      elseif nargin == 3 && isa(varargin{1},'gtsam.Point2') && isa(varargin{2},'double') && isa(varargin{3},'double')
        my_ptr = gpmp2_wrapper(217, varargin{1}, varargin{2}, varargin{3});
      else
        error('Arguments do not match any overload of gpmp2.PlanarSDF constructor');
      end
      obj.ptr_gpmp2PlanarSDF = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(218, obj.ptr_gpmp2PlanarSDF);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = getSignedDistance(this, varargin)
      % GETSIGNEDDISTANCE usage: getSignedDistance(Point2 point) : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point2')
        varargout{1} = gpmp2_wrapper(219, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.PlanarSDF.getSignedDistance');
      end
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gpmp2_wrapper(220, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.PlanarSDF.print');
      end
    end

  end

  methods(Static = true)
  end
end
