%class VehicleDynamicsFactorPose2Vector, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%VehicleDynamicsFactorPose2Vector(size_t poseKey, size_t velKey, double cost_sigma)
%
classdef VehicleDynamicsFactorPose2Vector < gtsam.NoiseModelFactor
  properties
    ptr_gpmp2VehicleDynamicsFactorPose2Vector = 0
  end
  methods
    function obj = VehicleDynamicsFactorPose2Vector(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gpmp2_wrapper(200, varargin{2});
        end
        base_ptr = gpmp2_wrapper(199, my_ptr);
      elseif nargin == 3 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'double')
        [ my_ptr, base_ptr ] = gpmp2_wrapper(201, varargin{1}, varargin{2}, varargin{3});
      else
        error('Arguments do not match any overload of gpmp2.VehicleDynamicsFactorPose2Vector constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gpmp2VehicleDynamicsFactorPose2Vector = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(202, obj.ptr_gpmp2VehicleDynamicsFactorPose2Vector);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
