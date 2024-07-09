%class GaussianPriorWorkspacePoseArm, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%GaussianPriorWorkspacePoseArm(size_t poseKey, ArmModel arm, int joint, Pose3 des_pose, Base cost_model)
%
classdef GaussianPriorWorkspacePoseArm < gtsam.NoiseModelFactor
  properties
    ptr_gpmp2GaussianPriorWorkspacePoseArm = 0
  end
  methods
    function obj = GaussianPriorWorkspacePoseArm(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gpmp2_wrapper(192, varargin{2});
        end
        base_ptr = gpmp2_wrapper(191, my_ptr);
      elseif nargin == 5 && isa(varargin{1},'numeric') && isa(varargin{2},'gpmp2.ArmModel') && isa(varargin{3},'numeric') && isa(varargin{4},'gtsam.Pose3') && isa(varargin{5},'gtsam.noiseModel.Base')
        [ my_ptr, base_ptr ] = gpmp2_wrapper(193, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5});
      else
        error('Arguments do not match any overload of gpmp2.GaussianPriorWorkspacePoseArm constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gpmp2GaussianPriorWorkspacePoseArm = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(194, obj.ptr_gpmp2GaussianPriorWorkspacePoseArm);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
