%class JacobianFactor, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%JacobianFactor()
%JacobianFactor(GaussianFactor factor)
%JacobianFactor(Vector b_in)
%JacobianFactor(size_t i1, Matrix A1, Vector b, Diagonal model)
%JacobianFactor(size_t i1, Matrix A1, size_t i2, Matrix A2, Vector b, Diagonal model)
%JacobianFactor(size_t i1, Matrix A1, size_t i2, Matrix A2, size_t i3, Matrix A3, Vector b, Diagonal model)
%JacobianFactor(GaussianFactorGraph graph)
%
%-------Methods-------
%augmentedInformation() : returns Matrix
%augmentedJacobian() : returns Matrix
%augmentedJacobianUnweighted() : returns Matrix
%clone() : returns gtsam::GaussianFactor
%cols() : returns size_t
%eliminate(Ordering keys) : returns pair< gtsam::GaussianConditional, gtsam::JacobianFactor >
%empty() : returns bool
%equals(GaussianFactor lf, double tol) : returns bool
%error(VectorValues c) : returns double
%error_vector(VectorValues c) : returns Vector
%getA() : returns Matrix
%get_model() : returns gtsam::noiseModel::Diagonal
%getb() : returns Vector
%information() : returns Matrix
%isConstrained() : returns bool
%jacobian() : returns pair< Matrix, Vector >
%jacobianUnweighted() : returns pair< Matrix, Vector >
%keys() : returns gtsam::KeyVector
%negate() : returns gtsam::GaussianFactor
%print(string s) : returns void
%printKeys(string s) : returns void
%rows() : returns size_t
%setModel(bool anyConstrained, Vector sigmas) : returns void
%size() : returns size_t
%transposeMultiplyAdd(double alpha, Vector e, VectorValues x) : returns void
%unweighted_error(VectorValues c) : returns Vector
%whiten() : returns gtsam::JacobianFactor
%
%-------Serialization Interface-------
%string_serialize() : returns string
%string_deserialize(string serialized) : returns JacobianFactor
%
classdef JacobianFactor < gtsam.GaussianFactor
  properties
    ptr_gtsamJacobianFactor = 0
  end
  methods
    function obj = JacobianFactor(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_wrapper(782, varargin{2});
        end
        base_ptr = gtsam_wrapper(781, my_ptr);
      elseif nargin == 0
        [ my_ptr, base_ptr ] = gtsam_wrapper(783);
      elseif nargin == 1 && isa(varargin{1},'gtsam.GaussianFactor')
        [ my_ptr, base_ptr ] = gtsam_wrapper(784, varargin{1});
      elseif nargin == 1 && isa(varargin{1},'double')
        [ my_ptr, base_ptr ] = gtsam_wrapper(785, varargin{1});
      elseif nargin == 4 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'double') && isa(varargin{4},'gtsam.noiseModel.Diagonal')
        [ my_ptr, base_ptr ] = gtsam_wrapper(786, varargin{1}, varargin{2}, varargin{3}, varargin{4});
      elseif nargin == 6 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'numeric') && isa(varargin{4},'double') && isa(varargin{5},'double') && isa(varargin{6},'gtsam.noiseModel.Diagonal')
        [ my_ptr, base_ptr ] = gtsam_wrapper(787, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5}, varargin{6});
      elseif nargin == 8 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'numeric') && isa(varargin{4},'double') && isa(varargin{5},'numeric') && isa(varargin{6},'double') && isa(varargin{7},'double') && isa(varargin{8},'gtsam.noiseModel.Diagonal')
        [ my_ptr, base_ptr ] = gtsam_wrapper(788, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5}, varargin{6}, varargin{7}, varargin{8});
      elseif nargin == 1 && isa(varargin{1},'gtsam.GaussianFactorGraph')
        [ my_ptr, base_ptr ] = gtsam_wrapper(789, varargin{1});
      else
        error('Arguments do not match any overload of gtsam.JacobianFactor constructor');
      end
      obj = obj@gtsam.GaussianFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsamJacobianFactor = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(790, obj.ptr_gtsamJacobianFactor);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = augmentedInformation(this, varargin)
      % AUGMENTEDINFORMATION usage: augmentedInformation() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(791, this, varargin{:});
    end

    function varargout = augmentedJacobian(this, varargin)
      % AUGMENTEDJACOBIAN usage: augmentedJacobian() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(792, this, varargin{:});
    end

    function varargout = augmentedJacobianUnweighted(this, varargin)
      % AUGMENTEDJACOBIANUNWEIGHTED usage: augmentedJacobianUnweighted() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(793, this, varargin{:});
    end

    function varargout = clone(this, varargin)
      % CLONE usage: clone() : returns gtsam::GaussianFactor
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(794, this, varargin{:});
    end

    function varargout = cols(this, varargin)
      % COLS usage: cols() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(795, this, varargin{:});
    end

    function varargout = eliminate(this, varargin)
      % ELIMINATE usage: eliminate(Ordering keys) : returns pair< gtsam::GaussianConditional, gtsam::JacobianFactor >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Ordering')
        [ varargout{1} varargout{2} ] = gtsam_wrapper(796, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.JacobianFactor.eliminate');
      end
    end

    function varargout = empty(this, varargin)
      % EMPTY usage: empty() : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(797, this, varargin{:});
    end

    function varargout = equals(this, varargin)
      % EQUALS usage: equals(GaussianFactor lf, double tol) : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gtsam.GaussianFactor') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(798, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.JacobianFactor.equals');
      end
    end

    function varargout = error(this, varargin)
      % ERROR usage: error(VectorValues c) : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.VectorValues')
        varargout{1} = gtsam_wrapper(799, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.JacobianFactor.error');
      end
    end

    function varargout = error_vector(this, varargin)
      % ERROR_VECTOR usage: error_vector(VectorValues c) : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.VectorValues')
        varargout{1} = gtsam_wrapper(800, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.JacobianFactor.error_vector');
      end
    end

    function varargout = getA(this, varargin)
      % GETA usage: getA() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(801, this, varargin{:});
    end

    function varargout = get_model(this, varargin)
      % GET_MODEL usage: get_model() : returns gtsam::noiseModel::Diagonal
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(802, this, varargin{:});
    end

    function varargout = getb(this, varargin)
      % GETB usage: getb() : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(803, this, varargin{:});
    end

    function varargout = information(this, varargin)
      % INFORMATION usage: information() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(804, this, varargin{:});
    end

    function varargout = isConstrained(this, varargin)
      % ISCONSTRAINED usage: isConstrained() : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(805, this, varargin{:});
    end

    function varargout = jacobian(this, varargin)
      % JACOBIAN usage: jacobian() : returns pair< Matrix, Vector >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      [ varargout{1} varargout{2} ] = gtsam_wrapper(806, this, varargin{:});
    end

    function varargout = jacobianUnweighted(this, varargin)
      % JACOBIANUNWEIGHTED usage: jacobianUnweighted() : returns pair< Matrix, Vector >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      [ varargout{1} varargout{2} ] = gtsam_wrapper(807, this, varargin{:});
    end

    function varargout = keys(this, varargin)
      % KEYS usage: keys() : returns gtsam::KeyVector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(808, this, varargin{:});
    end

    function varargout = negate(this, varargin)
      % NEGATE usage: negate() : returns gtsam::GaussianFactor
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(809, this, varargin{:});
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(810, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.JacobianFactor.print');
      end
    end

    function varargout = printKeys(this, varargin)
      % PRINTKEYS usage: printKeys(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(811, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.JacobianFactor.printKeys');
      end
    end

    function varargout = rows(this, varargin)
      % ROWS usage: rows() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(812, this, varargin{:});
    end

    function varargout = setModel(this, varargin)
      % SETMODEL usage: setModel(bool anyConstrained, Vector sigmas) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'logical') && isa(varargin{2},'double') && size(varargin{2},2)==1
        gtsam_wrapper(813, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.JacobianFactor.setModel');
      end
    end

    function varargout = size(this, varargin)
      % SIZE usage: size() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(814, this, varargin{:});
    end

    function varargout = transposeMultiplyAdd(this, varargin)
      % TRANSPOSEMULTIPLYADD usage: transposeMultiplyAdd(double alpha, Vector e, VectorValues x) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 3 && isa(varargin{1},'double') && isa(varargin{2},'double') && size(varargin{2},2)==1 && isa(varargin{3},'gtsam.VectorValues')
        gtsam_wrapper(815, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.JacobianFactor.transposeMultiplyAdd');
      end
    end

    function varargout = unweighted_error(this, varargin)
      % UNWEIGHTED_ERROR usage: unweighted_error(VectorValues c) : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.VectorValues')
        varargout{1} = gtsam_wrapper(816, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.JacobianFactor.unweighted_error');
      end
    end

    function varargout = whiten(this, varargin)
      % WHITEN usage: whiten() : returns gtsam::JacobianFactor
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(817, this, varargin{:});
    end

    function varargout = string_serialize(this, varargin)
      % STRING_SERIALIZE usage: string_serialize() : returns string
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 0
        varargout{1} = gtsam_wrapper(818, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.JacobianFactor.string_serialize');
      end
    end

    function sobj = saveobj(obj)
      % SAVEOBJ Saves the object to a matlab-readable format
      sobj = obj.string_serialize();
    end
  end

  methods(Static = true)
    function varargout = string_deserialize(varargin)
      % STRING_DESERIALIZE usage: string_deserialize() : returns gtsam.JacobianFactor
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1
        varargout{1} = gtsam_wrapper(819, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.JacobianFactor.string_deserialize');
      end
    end

    function obj = loadobj(sobj)
      % LOADOBJ Saves the object to a matlab-readable format
      obj = gtsam.JacobianFactor.string_deserialize(sobj);
    end
  end
end
