%class TrajOptimizerSetting, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%TrajOptimizerSetting(size_t dof)
%
%-------Methods-------
%setDogleg() : returns void
%setGaussNewton() : returns void
%setLM() : returns void
%setOptimizationNoIncrase(bool flag) : returns void
%setVerbosityError() : returns void
%setVerbosityNone() : returns void
%set_Qc_model(Matrix Qc) : returns void
%set_conf_prior_model(double sigma) : returns void
%set_cost_sigma(double sigma) : returns void
%set_epsilon(double eps) : returns void
%set_flag_pos_limit(bool flag) : returns void
%set_flag_vel_limit(bool flag) : returns void
%set_joint_pos_limits_down(Vector v) : returns void
%set_joint_pos_limits_up(Vector v) : returns void
%set_max_iter(size_t iter) : returns void
%set_obs_check_inter(size_t inter) : returns void
%set_pos_limit_model(Vector v) : returns void
%set_pos_limit_thresh(Vector v) : returns void
%set_rel_thresh(double thresh) : returns void
%set_total_step(size_t step) : returns void
%set_total_time(double time) : returns void
%set_vel_limit_model(Vector v) : returns void
%set_vel_limit_thresh(Vector v) : returns void
%set_vel_limits(Vector v) : returns void
%set_vel_prior_model(double sigma) : returns void
%
classdef TrajOptimizerSetting < handle
  properties
    ptr_gpmp2TrajOptimizerSetting = 0
  end
  methods
    function obj = TrajOptimizerSetting(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gpmp2_wrapper(327, my_ptr);
      elseif nargin == 1 && isa(varargin{1},'numeric')
        my_ptr = gpmp2_wrapper(328, varargin{1});
      else
        error('Arguments do not match any overload of gpmp2.TrajOptimizerSetting constructor');
      end
      obj.ptr_gpmp2TrajOptimizerSetting = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(329, obj.ptr_gpmp2TrajOptimizerSetting);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = setDogleg(this, varargin)
      % SETDOGLEG usage: setDogleg() : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(330, this, varargin{:});
    end

    function varargout = setGaussNewton(this, varargin)
      % SETGAUSSNEWTON usage: setGaussNewton() : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(331, this, varargin{:});
    end

    function varargout = setLM(this, varargin)
      % SETLM usage: setLM() : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(332, this, varargin{:});
    end

    function varargout = setOptimizationNoIncrase(this, varargin)
      % SETOPTIMIZATIONNOINCRASE usage: setOptimizationNoIncrase(bool flag) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(333, this, varargin{:});
    end

    function varargout = setVerbosityError(this, varargin)
      % SETVERBOSITYERROR usage: setVerbosityError() : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(334, this, varargin{:});
    end

    function varargout = setVerbosityNone(this, varargin)
      % SETVERBOSITYNONE usage: setVerbosityNone() : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(335, this, varargin{:});
    end

    function varargout = set_Qc_model(this, varargin)
      % SET_QC_MODEL usage: set_Qc_model(Matrix Qc) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double')
        gpmp2_wrapper(336, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.TrajOptimizerSetting.set_Qc_model');
      end
    end

    function varargout = set_conf_prior_model(this, varargin)
      % SET_CONF_PRIOR_MODEL usage: set_conf_prior_model(double sigma) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(337, this, varargin{:});
    end

    function varargout = set_cost_sigma(this, varargin)
      % SET_COST_SIGMA usage: set_cost_sigma(double sigma) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(338, this, varargin{:});
    end

    function varargout = set_epsilon(this, varargin)
      % SET_EPSILON usage: set_epsilon(double eps) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(339, this, varargin{:});
    end

    function varargout = set_flag_pos_limit(this, varargin)
      % SET_FLAG_POS_LIMIT usage: set_flag_pos_limit(bool flag) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(340, this, varargin{:});
    end

    function varargout = set_flag_vel_limit(this, varargin)
      % SET_FLAG_VEL_LIMIT usage: set_flag_vel_limit(bool flag) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(341, this, varargin{:});
    end

    function varargout = set_joint_pos_limits_down(this, varargin)
      % SET_JOINT_POS_LIMITS_DOWN usage: set_joint_pos_limits_down(Vector v) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        gpmp2_wrapper(342, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.TrajOptimizerSetting.set_joint_pos_limits_down');
      end
    end

    function varargout = set_joint_pos_limits_up(this, varargin)
      % SET_JOINT_POS_LIMITS_UP usage: set_joint_pos_limits_up(Vector v) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        gpmp2_wrapper(343, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.TrajOptimizerSetting.set_joint_pos_limits_up');
      end
    end

    function varargout = set_max_iter(this, varargin)
      % SET_MAX_ITER usage: set_max_iter(size_t iter) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(344, this, varargin{:});
    end

    function varargout = set_obs_check_inter(this, varargin)
      % SET_OBS_CHECK_INTER usage: set_obs_check_inter(size_t inter) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(345, this, varargin{:});
    end

    function varargout = set_pos_limit_model(this, varargin)
      % SET_POS_LIMIT_MODEL usage: set_pos_limit_model(Vector v) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        gpmp2_wrapper(346, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.TrajOptimizerSetting.set_pos_limit_model');
      end
    end

    function varargout = set_pos_limit_thresh(this, varargin)
      % SET_POS_LIMIT_THRESH usage: set_pos_limit_thresh(Vector v) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        gpmp2_wrapper(347, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.TrajOptimizerSetting.set_pos_limit_thresh');
      end
    end

    function varargout = set_rel_thresh(this, varargin)
      % SET_REL_THRESH usage: set_rel_thresh(double thresh) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(348, this, varargin{:});
    end

    function varargout = set_total_step(this, varargin)
      % SET_TOTAL_STEP usage: set_total_step(size_t step) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(349, this, varargin{:});
    end

    function varargout = set_total_time(this, varargin)
      % SET_TOTAL_TIME usage: set_total_time(double time) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(350, this, varargin{:});
    end

    function varargout = set_vel_limit_model(this, varargin)
      % SET_VEL_LIMIT_MODEL usage: set_vel_limit_model(Vector v) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        gpmp2_wrapper(351, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.TrajOptimizerSetting.set_vel_limit_model');
      end
    end

    function varargout = set_vel_limit_thresh(this, varargin)
      % SET_VEL_LIMIT_THRESH usage: set_vel_limit_thresh(Vector v) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        gpmp2_wrapper(352, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.TrajOptimizerSetting.set_vel_limit_thresh');
      end
    end

    function varargout = set_vel_limits(this, varargin)
      % SET_VEL_LIMITS usage: set_vel_limits(Vector v) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        gpmp2_wrapper(353, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.TrajOptimizerSetting.set_vel_limits');
      end
    end

    function varargout = set_vel_prior_model(this, varargin)
      % SET_VEL_PRIOR_MODEL usage: set_vel_prior_model(double sigma) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gpmp2_wrapper(354, this, varargin{:});
    end

  end

  methods(Static = true)
  end
end
