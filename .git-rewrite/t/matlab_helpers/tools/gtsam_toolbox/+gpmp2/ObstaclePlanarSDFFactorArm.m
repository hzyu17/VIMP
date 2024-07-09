%class ObstaclePlanarSDFFactorArm, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%ObstaclePlanarSDFFactorArm(size_t posekey, ArmModel arm, PlanarSDF sdf, double cost_sigma, double epsilon)
%
%-------Methods-------
%evaluateError(Vector pose) : returns Vector
%
%-------Serialization Interface-------
%string_serialize() : returns string
%string_deserialize(string serialized) : returns ObstaclePlanarSDFFactorArm
%
classdef ObstaclePlanarSDFFactorArm < gtsam.NoiseModelFactor
  properties
    ptr_gpmp2ObstaclePlanarSDFFactorArm = 0
  end
  methods
    function obj = ObstaclePlanarSDFFactorArm(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gpmp2_wrapper(231, varargin{2});
        end
        base_ptr = gpmp2_wrapper(230, my_ptr);
      elseif nargin == 5 && isa(varargin{1},'numeric') && isa(varargin{2},'gpmp2.ArmModel') && isa(varargin{3},'gpmp2.PlanarSDF') && isa(varargin{4},'double') && isa(varargin{5},'double')
        [ my_ptr, base_ptr ] = gpmp2_wrapper(232, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5});
      else
        error('Arguments do not match any overload of gpmp2.ObstaclePlanarSDFFactorArm constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gpmp2ObstaclePlanarSDFFactorArm = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(233, obj.ptr_gpmp2ObstaclePlanarSDFFactorArm);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = evaluateError(this, varargin)
      % EVALUATEERROR usage: evaluateError(Vector pose) : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        varargout{1} = gpmp2_wrapper(234, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.ObstaclePlanarSDFFactorArm.evaluateError');
      end
    end

    function varargout = string_serialize(this, varargin)
      % STRING_SERIALIZE usage: string_serialize() : returns string
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 0
        varargout{1} = gpmp2_wrapper(235, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.ObstaclePlanarSDFFactorArm.string_serialize');
      end
    end

    function sobj = saveobj(obj)
      % SAVEOBJ Saves the object to a matlab-readable format
      sobj = obj.string_serialize();
    end
  end

  methods(Static = true)
    function varargout = string_deserialize(varargin)
      % STRING_DESERIALIZE usage: string_deserialize() : returns gpmp2.ObstaclePlanarSDFFactorArm
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1
        varargout{1} = gpmp2_wrapper(236, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.ObstaclePlanarSDFFactorArm.string_deserialize');
      end
    end

    function obj = loadobj(sobj)
      % LOADOBJ Saves the object to a matlab-readable format
      obj = gpmp2.ObstaclePlanarSDFFactorArm.string_deserialize(sobj);
    end
  end
end
