%class Pose3AttitudeFactor, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%Pose3AttitudeFactor(size_t key, Unit3 nZ, Diagonal model, Unit3 bRef)
%Pose3AttitudeFactor(size_t key, Unit3 nZ, Diagonal model)
%Pose3AttitudeFactor()
%
%-------Methods-------
%active(Values c) : returns bool
%bRef() : returns gtsam::Unit3
%clone() : returns gtsam::NonlinearFactor
%dim() : returns size_t
%equals(NonlinearFactor expected, double tol) : returns bool
%error(Values c) : returns double
%keys() : returns gtsam::KeyVector
%linearize(Values c) : returns gtsam::GaussianFactor
%nZ() : returns gtsam::Unit3
%print(string s) : returns void
%printKeys(string s) : returns void
%size() : returns size_t
%
classdef Pose3AttitudeFactor < gtsam.NonlinearFactor
  properties
    ptr_gtsamPose3AttitudeFactor = 0
  end
  methods
    function obj = Pose3AttitudeFactor(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_wrapper(2349, varargin{2});
        end
        base_ptr = gtsam_wrapper(2348, my_ptr);
      elseif nargin == 4 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Unit3') && isa(varargin{3},'gtsam.noiseModel.Diagonal') && isa(varargin{4},'gtsam.Unit3')
        [ my_ptr, base_ptr ] = gtsam_wrapper(2350, varargin{1}, varargin{2}, varargin{3}, varargin{4});
      elseif nargin == 3 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Unit3') && isa(varargin{3},'gtsam.noiseModel.Diagonal')
        [ my_ptr, base_ptr ] = gtsam_wrapper(2351, varargin{1}, varargin{2}, varargin{3});
      elseif nargin == 0
        [ my_ptr, base_ptr ] = gtsam_wrapper(2352);
      else
        error('Arguments do not match any overload of gtsam.Pose3AttitudeFactor constructor');
      end
      obj = obj@gtsam.NonlinearFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsamPose3AttitudeFactor = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(2353, obj.ptr_gtsamPose3AttitudeFactor);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = active(this, varargin)
      % ACTIVE usage: active(Values c) : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Values')
        varargout{1} = gtsam_wrapper(2354, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Pose3AttitudeFactor.active');
      end
    end

    function varargout = bRef(this, varargin)
      % BREF usage: bRef() : returns gtsam::Unit3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(2355, this, varargin{:});
    end

    function varargout = clone(this, varargin)
      % CLONE usage: clone() : returns gtsam::NonlinearFactor
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(2356, this, varargin{:});
    end

    function varargout = dim(this, varargin)
      % DIM usage: dim() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(2357, this, varargin{:});
    end

    function varargout = equals(this, varargin)
      % EQUALS usage: equals(NonlinearFactor expected, double tol) : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gtsam.NonlinearFactor') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(2358, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Pose3AttitudeFactor.equals');
      end
    end

    function varargout = error(this, varargin)
      % ERROR usage: error(Values c) : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Values')
        varargout{1} = gtsam_wrapper(2359, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Pose3AttitudeFactor.error');
      end
    end

    function varargout = keys(this, varargin)
      % KEYS usage: keys() : returns gtsam::KeyVector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(2360, this, varargin{:});
    end

    function varargout = linearize(this, varargin)
      % LINEARIZE usage: linearize(Values c) : returns gtsam::GaussianFactor
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Values')
        varargout{1} = gtsam_wrapper(2361, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Pose3AttitudeFactor.linearize');
      end
    end

    function varargout = nZ(this, varargin)
      % NZ usage: nZ() : returns gtsam::Unit3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(2362, this, varargin{:});
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(2363, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Pose3AttitudeFactor.print');
      end
    end

    function varargout = printKeys(this, varargin)
      % PRINTKEYS usage: printKeys(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(2364, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Pose3AttitudeFactor.printKeys');
      end
    end

    function varargout = size(this, varargin)
      % SIZE usage: size() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(2365, this, varargin{:});
    end

  end

  methods(Static = true)
  end
end
