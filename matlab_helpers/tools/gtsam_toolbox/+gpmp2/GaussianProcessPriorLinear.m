%class GaussianProcessPriorLinear, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%GaussianProcessPriorLinear(size_t key1, size_t key2, size_t key3, size_t key4, double delta, Base Qc_model)
%
%-------Methods-------
%evaluateError(Vector pose1, Vector vel1, Vector pose2, Vector vel2) : returns Vector
%
%-------Serialization Interface-------
%string_serialize() : returns string
%string_deserialize(string serialized) : returns GaussianProcessPriorLinear
%
classdef GaussianProcessPriorLinear < gtsam.NoiseModelFactor
  properties
    ptr_gpmp2GaussianProcessPriorLinear = 0
  end
  methods
    function obj = GaussianProcessPriorLinear(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gpmp2_wrapper(8, varargin{2});
        end
        base_ptr = gpmp2_wrapper(7, my_ptr);
      elseif nargin == 6 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'numeric') && isa(varargin{4},'numeric') && isa(varargin{5},'double') && isa(varargin{6},'gtsam.noiseModel.Base')
        [ my_ptr, base_ptr ] = gpmp2_wrapper(9, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5}, varargin{6});
      else
        error('Arguments do not match any overload of gpmp2.GaussianProcessPriorLinear constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gpmp2GaussianProcessPriorLinear = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(10, obj.ptr_gpmp2GaussianProcessPriorLinear);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = evaluateError(this, varargin)
      % EVALUATEERROR usage: evaluateError(Vector pose1, Vector vel1, Vector pose2, Vector vel2) : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 4 && isa(varargin{1},'double') && size(varargin{1},2)==1 && isa(varargin{2},'double') && size(varargin{2},2)==1 && isa(varargin{3},'double') && size(varargin{3},2)==1 && isa(varargin{4},'double') && size(varargin{4},2)==1
        varargout{1} = gpmp2_wrapper(11, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.GaussianProcessPriorLinear.evaluateError');
      end
    end

    function varargout = string_serialize(this, varargin)
      % STRING_SERIALIZE usage: string_serialize() : returns string
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 0
        varargout{1} = gpmp2_wrapper(12, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.GaussianProcessPriorLinear.string_serialize');
      end
    end

    function sobj = saveobj(obj)
      % SAVEOBJ Saves the object to a matlab-readable format
      sobj = obj.string_serialize();
    end
  end

  methods(Static = true)
    function varargout = string_deserialize(varargin)
      % STRING_DESERIALIZE usage: string_deserialize() : returns gpmp2.GaussianProcessPriorLinear
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1
        varargout{1} = gpmp2_wrapper(13, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.GaussianProcessPriorLinear.string_deserialize');
      end
    end

    function obj = loadobj(sobj)
      % LOADOBJ Saves the object to a matlab-readable format
      obj = gpmp2.GaussianProcessPriorLinear.string_deserialize(sobj);
    end
  end
end
