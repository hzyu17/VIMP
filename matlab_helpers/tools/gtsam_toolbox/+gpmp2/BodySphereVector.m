%class BodySphereVector, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%BodySphereVector()
%
%-------Methods-------
%push_back(BodySphere sphere) : returns void
%
classdef BodySphereVector < handle
  properties
    ptr_gpmp2BodySphereVector = 0
  end
  methods
    function obj = BodySphereVector(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gpmp2_wrapper(111, my_ptr);
      elseif nargin == 0
        my_ptr = gpmp2_wrapper(112);
      else
        error('Arguments do not match any overload of gpmp2.BodySphereVector constructor');
      end
      obj.ptr_gpmp2BodySphereVector = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(113, obj.ptr_gpmp2BodySphereVector);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = push_back(this, varargin)
      % PUSH_BACK usage: push_back(BodySphere sphere) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gpmp2.BodySphere')
        gpmp2_wrapper(114, this, varargin{:});
      else
        error('Arguments do not match any overload of function gpmp2.BodySphereVector.push_back');
      end
    end

  end

  methods(Static = true)
  end
end
