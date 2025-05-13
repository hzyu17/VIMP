%class Unit, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Methods-------
%R() : returns Matrix
%equals(Base expected, double tol) : returns bool
%print(string s) : returns void
%
%-------Static Methods-------
%Create(size_t dim) : returns gtsam::noiseModel::Unit
%
classdef Unit < gtsam.noiseModel.Isotropic
  properties
    ptr_gtsamnoiseModelUnit = 0
  end
  methods
    function obj = Unit(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_wrapper(691, varargin{2});
        end
        base_ptr = gtsam_wrapper(690, my_ptr);
      else
        error('Arguments do not match any overload of gtsam.noiseModel.Unit constructor');
      end
      obj = obj@gtsam.noiseModel.Isotropic(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsamnoiseModelUnit = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(692, obj.ptr_gtsamnoiseModelUnit);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = R(this, varargin)
      % R usage: R() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(693, this, varargin{:});
    end

    function varargout = equals(this, varargin)
      % EQUALS usage: equals(Base expected, double tol) : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gtsam.noiseModel.Base') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(694, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Unit.equals');
      end
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(695, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Unit.print');
      end
    end

  end

  methods(Static = true)
    function varargout = Create(varargin)
      % CREATE usage: Create(size_t dim) : returns gtsam::noiseModel::Unit
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(696, varargin{:});
    end

  end
end
