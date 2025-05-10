%class VehicleDynamicsFactorVector, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%VehicleDynamicsFactorVector(size_t poseKey, size_t velKey, double cost_sigma)
%
classdef VehicleDynamicsFactorVector < gtsam.NoiseModelFactor
  properties
    ptr_gpmp2VehicleDynamicsFactorVector = 0
  end
  methods
    function obj = VehicleDynamicsFactorVector(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gpmp2_wrapper(204, varargin{2});
        end
        base_ptr = gpmp2_wrapper(203, my_ptr);
      elseif nargin == 3 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'double')
        [ my_ptr, base_ptr ] = gpmp2_wrapper(205, varargin{1}, varargin{2}, varargin{3});
      else
        error('Arguments do not match any overload of gpmp2.VehicleDynamicsFactorVector constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gpmp2VehicleDynamicsFactorVector = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(206, obj.ptr_gpmp2VehicleDynamicsFactorVector);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
