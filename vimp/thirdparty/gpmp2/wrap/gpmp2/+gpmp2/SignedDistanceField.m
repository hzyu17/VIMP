%class SignedDistanceField, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%SignedDistanceField()
%SignedDistanceField(Point3 origin, double cell_size, size_t field_rows, size_t field_cols, size_t field_z)
%
%-------Methods-------
%getSignedDistance(Point3 point) : returns double
%initFieldData(size_t z_idx, Matrix field_layer) : returns void
%loadSDF(string filename) : returns void
%print(string s) : returns void
%saveSDF(string filename) : returns void
%
classdef SignedDistanceField < handle
  properties
    ptr_gpmp2SignedDistanceField = 0
  end
  methods
    function obj = SignedDistanceField(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gpmp2_wrapper(207, my_ptr);
      elseif nargin == 0
        my_ptr = gpmp2_wrapper(208);
      elseif nargin == 5 && isa(varargin{1},'gtsam.Point3') && isa(varargin{2},'double') && isa(varargin{3},'numeric') && isa(varargin{4},'numeric') && isa(varargin{5},'numeric')
        my_ptr = gpmp2_wrapper(209, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5});
      else
        error('Arguments do not match any overload of gpmp2.SignedDistanceField constructor');
      end
      obj.ptr_gpmp2SignedDistanceField = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(210, obj.ptr_gpmp2SignedDistanceField);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = getSignedDistance(this, varargin)
      % GETSIGNEDDISTANCE usage: getSignedDistance(Point3 point) : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Point3')
        varargout{1} = gpmp2_wrapper(211, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.SignedDistanceField.getSignedDistance');
      end
    end

    function varargout = initFieldData(this, varargin)
      % INITFIELDDATA usage: initFieldData(size_t z_idx, Matrix field_layer) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'double')
        gpmp2_wrapper(212, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.SignedDistanceField.initFieldData');
      end
    end

    function varargout = loadSDF(this, varargin)
      % LOADSDF usage: loadSDF(string filename) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gpmp2_wrapper(213, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.SignedDistanceField.loadSDF');
      end
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gpmp2_wrapper(214, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.SignedDistanceField.print');
      end
    end

    function varargout = saveSDF(this, varargin)
      % SAVESDF usage: saveSDF(string filename) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gpmp2_wrapper(215, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.SignedDistanceField.saveSDF');
      end
    end

  end

  methods(Static = true)
  end
end
