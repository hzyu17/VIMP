%class JointLimitFactorVector, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%JointLimitFactorVector(size_t key, Base cost_model, Vector down_limit, Vector up_limit, Vector limit_thresh)
%
classdef JointLimitFactorVector < gtsam.NoiseModelFactor
  properties
    ptr_gpmp2JointLimitFactorVector = 0
  end
  methods
    function obj = JointLimitFactorVector(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gpmp2_wrapper(176, varargin{2});
        end
        base_ptr = gpmp2_wrapper(175, my_ptr);
      elseif nargin == 5 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.noiseModel.Base') && isa(varargin{3},'double') && isa(varargin{4},'double') && isa(varargin{5},'double')
        [ my_ptr, base_ptr ] = gpmp2_wrapper(177, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5});
      else
        error('Arguments do not match any overload of gpmp2.JointLimitFactorVector constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gpmp2JointLimitFactorVector = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(178, obj.ptr_gpmp2JointLimitFactorVector);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
