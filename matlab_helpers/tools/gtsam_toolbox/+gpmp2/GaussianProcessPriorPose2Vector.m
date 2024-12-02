%class GaussianProcessPriorPose2Vector, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%GaussianProcessPriorPose2Vector(size_t key1, size_t key2, size_t key3, size_t key4, double delta, Base Qc_model)
%
classdef GaussianProcessPriorPose2Vector < gtsam.NoiseModelFactor
  properties
    ptr_gpmp2GaussianProcessPriorPose2Vector = 0
  end
  methods
    function obj = GaussianProcessPriorPose2Vector(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gpmp2_wrapper(19, varargin{2});
        end
        base_ptr = gpmp2_wrapper(18, my_ptr);
      elseif nargin == 6 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'numeric') && isa(varargin{4},'numeric') && isa(varargin{5},'double') && isa(varargin{6},'gtsam.noiseModel.Base')
        [ my_ptr, base_ptr ] = gpmp2_wrapper(20, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5}, varargin{6});
      else
        error('Arguments do not match any overload of gpmp2.GaussianProcessPriorPose2Vector constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gpmp2GaussianProcessPriorPose2Vector = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(21, obj.ptr_gpmp2GaussianProcessPriorPose2Vector);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
