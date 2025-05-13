%class SelfCollisionArm, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%SelfCollisionArm(size_t poseKey, ArmModel arm, Matrix data)
%
%-------Methods-------
%evaluateError(Vector pose) : returns Vector
%
classdef SelfCollisionArm < gtsam.NoiseModelFactor
  properties
    ptr_gpmp2SelfCollisionArm = 0
  end
  methods
    function obj = SelfCollisionArm(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gpmp2_wrapper(323, varargin{2});
        end
        base_ptr = gpmp2_wrapper(322, my_ptr);
      elseif nargin == 3 && isa(varargin{1},'numeric') && isa(varargin{2},'gpmp2.ArmModel') && isa(varargin{3},'double')
        [ my_ptr, base_ptr ] = gpmp2_wrapper(324, varargin{1}, varargin{2}, varargin{3});
      else
        error('Arguments do not match any overload of gpmp2.SelfCollisionArm constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gpmp2SelfCollisionArm = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(325, obj.ptr_gpmp2SelfCollisionArm);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = evaluateError(this, varargin)
      % EVALUATEERROR usage: evaluateError(Vector pose) : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = gpmp2_wrapper(326, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.SelfCollisionArm.evaluateError');
      end
    end

  end

  methods(Static = true)
  end
end
