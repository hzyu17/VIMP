%class PriorFactorPose2Vector, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%PriorFactorPose2Vector(size_t poseKey, Pose2Vector value, Base model)
%
classdef PriorFactorPose2Vector < gtsam.NoiseModelFactor
  properties
    ptr_gpmp2PriorFactorPose2Vector = 0
  end
  methods
    function obj = PriorFactorPose2Vector(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gpmp2_wrapper(416, varargin{2});
        end
        base_ptr = gpmp2_wrapper(415, my_ptr);
      elseif nargin == 3 && isa(varargin{1},'numeric') && isa(varargin{2},'gpmp2.Pose2Vector') && isa(varargin{3},'gtsam.noiseModel.Base')
        [ my_ptr, base_ptr ] = gpmp2_wrapper(417, varargin{1}, varargin{2}, varargin{3});
      else
        error('Arguments do not match any overload of gpmp2.PriorFactorPose2Vector constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gpmp2PriorFactorPose2Vector = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(418, obj.ptr_gpmp2PriorFactorPose2Vector);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
