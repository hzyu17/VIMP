%class Constrained, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Methods-------
%R() : returns Matrix
%equals(Base expected, double tol) : returns bool
%print(string s) : returns void
%unit() : returns gtsam::noiseModel::Constrained
%
%-------Static Methods-------
%All(size_t dim) : returns gtsam::noiseModel::Constrained
%All(size_t dim, double mu) : returns gtsam::noiseModel::Constrained
%MixedPrecisions(Vector mu, Vector precisions) : returns gtsam::noiseModel::Constrained
%MixedPrecisions(Vector precisions) : returns gtsam::noiseModel::Constrained
%MixedSigmas(Vector mu, Vector sigmas) : returns gtsam::noiseModel::Constrained
%MixedSigmas(double m, Vector sigmas) : returns gtsam::noiseModel::Constrained
%MixedVariances(Vector mu, Vector variances) : returns gtsam::noiseModel::Constrained
%MixedVariances(Vector variances) : returns gtsam::noiseModel::Constrained
%
classdef Constrained < gtsam.noiseModel.Diagonal
  properties
    ptr_gtsamnoiseModelConstrained = 0
  end
  methods
    function obj = Constrained(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_wrapper(667, varargin{2});
        end
        base_ptr = gtsam_wrapper(666, my_ptr);
      else
        error('Arguments do not match any overload of gtsam.noiseModel.Constrained constructor');
      end
      obj = obj@gtsam.noiseModel.Diagonal(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsamnoiseModelConstrained = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(668, obj.ptr_gtsamnoiseModelConstrained);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = R(this, varargin)
      % R usage: R() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(669, this, varargin{:});
    end

    function varargout = equals(this, varargin)
      % EQUALS usage: equals(Base expected, double tol) : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gtsam.noiseModel.Base') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(670, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Constrained.equals');
      end
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(671, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Constrained.print');
      end
    end

    function varargout = unit(this, varargin)
      % UNIT usage: unit() : returns gtsam::noiseModel::Constrained
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(672, this, varargin{:});
    end

  end

  methods(Static = true)
    function varargout = All(varargin)
      % ALL usage: All(size_t dim), All(size_t dim, double mu) : returns gtsam::noiseModel::Constrained
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'numeric')
        varargout{1} = gtsam_wrapper(673, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(674, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Constrained.All');
      end
    end

    function varargout = MixedPrecisions(varargin)
      % MIXEDPRECISIONS usage: MixedPrecisions(Vector mu, Vector precisions), MixedPrecisions(Vector precisions) : returns gtsam::noiseModel::Constrained
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'double') && size(varargin{1},2)==1 && isa(varargin{2},'double') && size(varargin{2},2)==1
        varargout{1} = gtsam_wrapper(675, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = gtsam_wrapper(676, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Constrained.MixedPrecisions');
      end
    end

    function varargout = MixedSigmas(varargin)
      % MIXEDSIGMAS usage: MixedSigmas(Vector mu, Vector sigmas), MixedSigmas(double m, Vector sigmas) : returns gtsam::noiseModel::Constrained
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'double') && size(varargin{1},2)==1 && isa(varargin{2},'double') && size(varargin{2},2)==1
        varargout{1} = gtsam_wrapper(677, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'double') && isa(varargin{2},'double') && size(varargin{2},2)==1
        varargout{1} = gtsam_wrapper(678, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Constrained.MixedSigmas');
      end
    end

    function varargout = MixedVariances(varargin)
      % MIXEDVARIANCES usage: MixedVariances(Vector mu, Vector variances), MixedVariances(Vector variances) : returns gtsam::noiseModel::Constrained
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'double') && size(varargin{1},2)==1 && isa(varargin{2},'double') && size(varargin{2},2)==1
        varargout{1} = gtsam_wrapper(679, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = gtsam_wrapper(680, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.noiseModel.Constrained.MixedVariances');
      end
    end

  end
end
