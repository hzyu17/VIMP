%class Diagonal, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Methods-------
%R() : returns Matrix
%equals(Base expected, double tol) : returns bool
%print(string s) : returns void
%
%-------Static Methods-------
%Precisions(Vector precisions) : returns gtsam::noiseModel::Diagonal
%Sigmas(Vector sigmas) : returns gtsam::noiseModel::Diagonal
%Variances(Vector variances) : returns gtsam::noiseModel::Diagonal
%
classdef Diagonal < gtsam.noiseModel.Gaussian
  properties
    ptr_gtsamnoiseModelDiagonal = 0
  end
  methods
    function obj = Diagonal(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_wrapper(658, varargin{2});
        end
        base_ptr = gtsam_wrapper(657, my_ptr);
      else
        error('Arguments do not match any overload of gtsam.noiseModel.Diagonal constructor');
      end
      obj = obj@gtsam.noiseModel.Gaussian(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsamnoiseModelDiagonal = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(659, obj.ptr_gtsamnoiseModelDiagonal);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = R(this, varargin)
      % R usage: R() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(660, this, varargin{:});
    end

    function varargout = equals(this, varargin)
      % EQUALS usage: equals(Base expected, double tol) : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gtsam.noiseModel.Base') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(661, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Diagonal.equals');
      end
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(662, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Diagonal.print');
      end
    end

  end

  methods(Static = true)
    function varargout = Precisions(varargin)
      % PRECISIONS usage: Precisions(Vector precisions) : returns gtsam::noiseModel::Diagonal
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = gtsam_wrapper(663, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Diagonal.Precisions');
      end
    end

    function varargout = Sigmas(varargin)
      % SIGMAS usage: Sigmas(Vector sigmas) : returns gtsam::noiseModel::Diagonal
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = gtsam_wrapper(664, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Diagonal.Sigmas');
      end
    end

    function varargout = Variances(varargin)
      % VARIANCES usage: Variances(Vector variances) : returns gtsam::noiseModel::Diagonal
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = gtsam_wrapper(665, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Diagonal.Variances');
      end
    end

  end
end
