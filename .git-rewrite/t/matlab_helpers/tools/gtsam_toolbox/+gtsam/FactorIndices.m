%class FactorIndices, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
classdef FactorIndices < handle
  properties
    ptr_gtsamFactorIndices = 0
  end
  methods
    function obj = FactorIndices(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gtsam_wrapper(1460, my_ptr);
      else
        error('Arguments do not match any overload of gtsam.FactorIndices constructor');
      end
      obj.ptr_gtsamFactorIndices = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(1461, obj.ptr_gtsamFactorIndices);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
