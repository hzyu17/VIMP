%class GoalFactorArm, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%GoalFactorArm(size_t poseKey, Base cost_model, Arm arm, Point3 dest_point)
%
classdef GoalFactorArm < gtsam.NoiseModelFactor
  properties
    ptr_gpmp2GoalFactorArm = 0
  end
  methods
    function obj = GoalFactorArm(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gpmp2_wrapper(172, varargin{2});
        end
        base_ptr = gpmp2_wrapper(171, my_ptr);
      elseif nargin == 4 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.noiseModel.Base') && isa(varargin{3},'gpmp2.Arm') && isa(varargin{4},'gtsam.Point3')
        [ my_ptr, base_ptr ] = gpmp2_wrapper(173, varargin{1}, varargin{2}, varargin{3}, varargin{4});
      else
        error('Arguments do not match any overload of gpmp2.GoalFactorArm constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gpmp2GoalFactorArm = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(174, obj.ptr_gpmp2GoalFactorArm);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
