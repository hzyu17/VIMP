%class GaussianPriorWorkspacePositionArm, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%GaussianPriorWorkspacePositionArm(size_t poseKey, ArmModel arm, int joint, Point3 des_position, Base cost_model)
%
classdef GaussianPriorWorkspacePositionArm < gtsam.NoiseModelFactor
  properties
    ptr_gpmp2GaussianPriorWorkspacePositionArm = 0
  end
  methods
    function obj = GaussianPriorWorkspacePositionArm(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gpmp2_wrapper(184, varargin{2});
        end
        base_ptr = gpmp2_wrapper(183, my_ptr);
      elseif nargin == 5 && isa(varargin{1},'numeric') && isa(varargin{2},'gpmp2.ArmModel') && isa(varargin{3},'numeric') && isa(varargin{4},'gtsam.Point3') && isa(varargin{5},'gtsam.noiseModel.Base')
        [ my_ptr, base_ptr ] = gpmp2_wrapper(185, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5});
      else
        error('Arguments do not match any overload of gpmp2.GaussianPriorWorkspacePositionArm constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gpmp2GaussianPriorWorkspacePositionArm = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(186, obj.ptr_gpmp2GaussianPriorWorkspacePositionArm);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
