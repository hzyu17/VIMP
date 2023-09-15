%class BodySphere, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%BodySphere(size_t id, double r, Point3 c)
%
classdef BodySphere < handle
  properties
    ptr_gpmp2BodySphere = 0
  end
  methods
    function obj = BodySphere(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gpmp2_wrapper(108, my_ptr);
      elseif nargin == 3 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'gtsam.Point3')
        my_ptr = gpmp2_wrapper(109, varargin{1}, varargin{2}, varargin{3});
      else
        error('Arguments do not match any overload of gpmp2.BodySphere constructor');
      end
      obj.ptr_gpmp2BodySphere = my_ptr;
    end

    function delete(obj)
      gpmp2_wrapper(110, obj.ptr_gpmp2BodySphere);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
