%class Isotropic, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Methods-------
%R() : returns Matrix
%equals(Base expected, double tol) : returns bool
%print(string s) : returns void
%
%-------Static Methods-------
%Precision(size_t dim, double precision) : returns gtsam::noiseModel::Isotropic
%Sigma(size_t dim, double sigma) : returns gtsam::noiseModel::Isotropic
%Variance(size_t dim, double varianace) : returns gtsam::noiseModel::Isotropic
%
classdef Isotropic < gtsam.noiseModel.Diagonal
  properties
    ptr_gtsamnoiseModelIsotropic = 0
  end
  methods
    function obj = Isotropic(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_wrapper(682, varargin{2});
        end
        base_ptr = gtsam_wrapper(681, my_ptr);
      else
        error('Arguments do not match any overload of gtsam.noiseModel.Isotropic constructor');
      end
      obj = obj@gtsam.noiseModel.Diagonal(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsamnoiseModelIsotropic = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(683, obj.ptr_gtsamnoiseModelIsotropic);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = R(this, varargin)
      % R usage: R() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(684, this, varargin{:});
    end

    function varargout = equals(this, varargin)
      % EQUALS usage: equals(Base expected, double tol) : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gtsam.noiseModel.Base') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(685, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Isotropic.equals');
      end
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(686, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Isotropic.print');
      end
    end

  end

  methods(Static = true)
    function varargout = Precision(varargin)
      % PRECISION usage: Precision(size_t dim, double precision) : returns gtsam::noiseModel::Isotropic
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(687, varargin{:});
    end

    function varargout = Sigma(varargin)
      % SIGMA usage: Sigma(size_t dim, double sigma) : returns gtsam::noiseModel::Isotropic
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(688, varargin{:});
    end

    function varargout = Variance(varargin)
      % VARIANCE usage: Variance(size_t dim, double varianace) : returns gtsam::noiseModel::Isotropic
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(689, varargin{:});
    end

  end
end
