%class VelocityLimitFactorVector, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%VelocityLimitFactorVector(size_t key, Base cost_model, Vector vel_limit, Vector limit_thresh)
%
classdef VelocityLimitFactorVector < gtsam.NoiseModelFactor
  properties
    ptr_gpmp2VelocityLimitFactorVector = 0
  end
  methods
    function obj = VelocityLimitFactorVector(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gpmp2_wrapper(180, varargin{2});
        end
        base_ptr = gpmp2_wrapper(179, my_ptr);
      elseif nargin == 4 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.noiseModel.Base') && isa(varargin{3},'double') && isa(varargin{4},'double')
        [ my_ptr, base_ptr ] = gpmp2_wrapper(181, varargin{1}, varargin{2}, varargin{3}, varargin{4});
      else
        error('Arguments do not match any overload of gpmp2.VelocityLimitFactorVector constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gpmp2VelocityLimitFactorVector = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(182, obj.ptr_gpmp2VelocityLimitFactorVector);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
