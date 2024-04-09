%class ISAM2DoglegParams, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%ISAM2DoglegParams()
%
%-------Methods-------
%getAdaptationMode() : returns string
%getInitialDelta() : returns double
%getWildfireThreshold() : returns double
%isVerbose() : returns bool
%print(string str) : returns void
%setAdaptationMode(string adaptationMode) : returns void
%setInitialDelta(double initialDelta) : returns void
%setVerbose(bool verbose) : returns void
%setWildfireThreshold(double wildfireThreshold) : returns void
%
classdef ISAM2DoglegParams < handle
  properties
    ptr_gtsamISAM2DoglegParams = 0
  end
  methods
    function obj = ISAM2DoglegParams(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gtsam_wrapper(1402, my_ptr);
      elseif nargin == 0
        my_ptr = gtsam_wrapper(1403);
      else
        error('Arguments do not match any overload of gtsam.ISAM2DoglegParams constructor');
      end
      obj.ptr_gtsamISAM2DoglegParams = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(1404, obj.ptr_gtsamISAM2DoglegParams);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = getAdaptationMode(this, varargin)
      % GETADAPTATIONMODE usage: getAdaptationMode() : returns string
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1405, this, varargin{:});
    end

    function varargout = getInitialDelta(this, varargin)
      % GETINITIALDELTA usage: getInitialDelta() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1406, this, varargin{:});
    end

    function varargout = getWildfireThreshold(this, varargin)
      % GETWILDFIRETHRESHOLD usage: getWildfireThreshold() : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1407, this, varargin{:});
    end

    function varargout = isVerbose(this, varargin)
      % ISVERBOSE usage: isVerbose() : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1408, this, varargin{:});
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string str) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(1409, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.ISAM2DoglegParams.print');
      end
    end

    function varargout = setAdaptationMode(this, varargin)
      % SETADAPTATIONMODE usage: setAdaptationMode(string adaptationMode) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(1410, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.ISAM2DoglegParams.setAdaptationMode');
      end
    end

    function varargout = setInitialDelta(this, varargin)
      % SETINITIALDELTA usage: setInitialDelta(double initialDelta) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(1411, this, varargin{:});
    end

    function varargout = setVerbose(this, varargin)
      % SETVERBOSE usage: setVerbose(bool verbose) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(1412, this, varargin{:});
    end

    function varargout = setWildfireThreshold(this, varargin)
      % SETWILDFIRETHRESHOLD usage: setWildfireThreshold(double wildfireThreshold) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(1413, this, varargin{:});
    end

  end

  methods(Static = true)
  end
end
