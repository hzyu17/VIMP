%class VehicleDynamicsFactorPose2, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%VehicleDynamicsFactorPose2(size_t poseKey, size_t velKey, double cost_sigma)
%
classdef VehicleDynamicsFactorPose2 < gtsam.NoiseModelFactor
  properties
    ptr_gpmp2VehicleDynamicsFactorPose2 = 0
  end
  methods
    function obj = VehicleDynamicsFactorPose2(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gpmp2_wrapper(196, varargin{2});
        end
        base_ptr = gpmp2_wrapper(195, my_ptr);
      elseif nargin == 3 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'double')
        [ my_ptr, base_ptr ] = gpmp2_wrapper(197, varargin{1}, varargin{2}, varargin{3});
      else
        error('Arguments do not match any overload of gpmp2.VehicleDynamicsFactorPose2 constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gpmp2VehicleDynamicsFactorPose2 = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(198, obj.ptr_gpmp2VehicleDynamicsFactorPose2);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
