%class GaussianFactorGraph, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%GaussianFactorGraph()
%GaussianFactorGraph(GaussianBayesNet bayesNet)
%GaussianFactorGraph(GaussianBayesTree bayesTree)
%
%-------Methods-------
%add(GaussianFactor factor) : returns void
%add(Vector b) : returns void
%add(size_t key1, Matrix A1, Vector b, Diagonal model) : returns void
%add(size_t key1, Matrix A1, size_t key2, Matrix A2, Vector b, Diagonal model) : returns void
%add(size_t key1, Matrix A1, size_t key2, Matrix A2, size_t key3, Matrix A3, Vector b, Diagonal model) : returns void
%at(size_t idx) : returns gtsam::GaussianFactor
%augmentedHessian() : returns Matrix
%augmentedHessian(Ordering ordering) : returns Matrix
%augmentedJacobian() : returns Matrix
%augmentedJacobian(Ordering ordering) : returns Matrix
%clone() : returns gtsam::GaussianFactorGraph
%eliminateMultifrontal() : returns gtsam::GaussianBayesTree
%eliminateMultifrontal(Ordering ordering) : returns gtsam::GaussianBayesTree
%eliminatePartialMultifrontal(Ordering ordering) : returns pair< gtsam::GaussianBayesTree, gtsam::GaussianFactorGraph >
%eliminatePartialMultifrontal(KeyVector keys) : returns pair< gtsam::GaussianBayesTree, gtsam::GaussianFactorGraph >
%eliminatePartialSequential(Ordering ordering) : returns pair< gtsam::GaussianBayesNet, gtsam::GaussianFactorGraph >
%eliminatePartialSequential(KeyVector keys) : returns pair< gtsam::GaussianBayesNet, gtsam::GaussianFactorGraph >
%eliminateSequential() : returns gtsam::GaussianBayesNet
%eliminateSequential(Ordering ordering) : returns gtsam::GaussianBayesNet
%equals(GaussianFactorGraph lfgraph, double tol) : returns bool
%error(VectorValues c) : returns double
%exists(size_t idx) : returns bool
%gradient(VectorValues x0) : returns gtsam::VectorValues
%gradientAtZero() : returns gtsam::VectorValues
%hessian() : returns pair< Matrix, Vector >
%hessian(Ordering ordering) : returns pair< Matrix, Vector >
%jacobian() : returns pair< Matrix, Vector >
%jacobian(Ordering ordering) : returns pair< Matrix, Vector >
%keyVector() : returns gtsam::KeyVector
%keys() : returns gtsam::KeySet
%marginal(KeyVector key_vector) : returns gtsam::GaussianFactorGraph
%marginalMultifrontalBayesNet(Ordering ordering) : returns gtsam::GaussianBayesNet
%marginalMultifrontalBayesNet(KeyVector key_vector) : returns gtsam::GaussianBayesNet
%marginalMultifrontalBayesNet(Ordering ordering, Ordering marginalizedVariableOrdering) : returns gtsam::GaussianBayesNet
%marginalMultifrontalBayesNet(KeyVector key_vector, Ordering marginalizedVariableOrdering) : returns gtsam::GaussianBayesNet
%negate() : returns gtsam::GaussianFactorGraph
%optimize() : returns gtsam::VectorValues
%optimize(Ordering ordering) : returns gtsam::VectorValues
%optimizeGradientSearch() : returns gtsam::VectorValues
%print(string s) : returns void
%probPrime(VectorValues c) : returns double
%push_back(GaussianFactor factor) : returns void
%push_back(GaussianConditional conditional) : returns void
%push_back(GaussianFactorGraph graph) : returns void
%push_back(GaussianBayesNet bayesNet) : returns void
%push_back(GaussianBayesTree bayesTree) : returns void
%size() : returns size_t
%sparseJacobian_() : returns Matrix
%
%-------Serialization Interface-------
%string_serialize() : returns string
%string_deserialize(string serialized) : returns GaussianFactorGraph
%
classdef GaussianFactorGraph < handle
  properties
    ptr_gtsamGaussianFactorGraph = 0
  end
  methods
    function obj = GaussianFactorGraph(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gtsam_wrapper(848, my_ptr);
      elseif nargin == 0
        my_ptr = gtsam_wrapper(849);
      elseif nargin == 1 && isa(varargin{1},'gtsam.GaussianBayesNet')
        my_ptr = gtsam_wrapper(850, varargin{1});
      elseif nargin == 1 && isa(varargin{1},'gtsam.GaussianBayesTree')
        my_ptr = gtsam_wrapper(851, varargin{1});
      else
        error('Arguments do not match any overload of gtsam.GaussianFactorGraph constructor');
      end
      obj.ptr_gtsamGaussianFactorGraph = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(852, obj.ptr_gtsamGaussianFactorGraph);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = add(this, varargin)
      % ADD usage: add(GaussianFactor factor), add(Vector b), add(size_t key1, Matrix A1, Vector b, Diagonal model), add(size_t key1, Matrix A1, size_t key2, Matrix A2, Vector b, Diagonal model), add(size_t key1, Matrix A1, size_t key2, Matrix A2, size_t key3, Matrix A3, Vector b, Diagonal model) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.GaussianFactor')
        gtsam_wrapper(853, this, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'double') && size(varargin{1},2)==1
        gtsam_wrapper(854, this, varargin{:});
      elseif length(varargin) == 4 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'double') && size(varargin{3},2)==1 && isa(varargin{4},'gtsam.noiseModel.Diagonal')
        gtsam_wrapper(855, this, varargin{:});
      elseif length(varargin) == 6 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'numeric') && isa(varargin{4},'double') && isa(varargin{5},'double') && size(varargin{5},2)==1 && isa(varargin{6},'gtsam.noiseModel.Diagonal')
        gtsam_wrapper(856, this, varargin{:});
      elseif length(varargin) == 8 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && isa(varargin{3},'numeric') && isa(varargin{4},'double') && isa(varargin{5},'numeric') && isa(varargin{6},'double') && isa(varargin{7},'double') && size(varargin{7},2)==1 && isa(varargin{8},'gtsam.noiseModel.Diagonal')
        gtsam_wrapper(857, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.add');
      end
    end

    function varargout = at(this, varargin)
      % AT usage: at(size_t idx) : returns gtsam::GaussianFactor
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(858, this, varargin{:});
    end

    function varargout = augmentedHessian(this, varargin)
      % AUGMENTEDHESSIAN usage: augmentedHessian(), augmentedHessian(Ordering ordering) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 0
        varargout{1} = gtsam_wrapper(859, this, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'gtsam.Ordering')
        varargout{1} = gtsam_wrapper(860, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.augmentedHessian');
      end
    end

    function varargout = augmentedJacobian(this, varargin)
      % AUGMENTEDJACOBIAN usage: augmentedJacobian(), augmentedJacobian(Ordering ordering) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 0
        varargout{1} = gtsam_wrapper(861, this, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'gtsam.Ordering')
        varargout{1} = gtsam_wrapper(862, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.augmentedJacobian');
      end
    end

    function varargout = clone(this, varargin)
      % CLONE usage: clone() : returns gtsam::GaussianFactorGraph
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(863, this, varargin{:});
    end

    function varargout = eliminateMultifrontal(this, varargin)
      % ELIMINATEMULTIFRONTAL usage: eliminateMultifrontal(), eliminateMultifrontal(Ordering ordering) : returns gtsam::GaussianBayesTree
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 0
        varargout{1} = gtsam_wrapper(864, this, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'gtsam.Ordering')
        varargout{1} = gtsam_wrapper(865, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.eliminateMultifrontal');
      end
    end

    function varargout = eliminatePartialMultifrontal(this, varargin)
      % ELIMINATEPARTIALMULTIFRONTAL usage: eliminatePartialMultifrontal(Ordering ordering), eliminatePartialMultifrontal(KeyVector keys) : returns pair< gtsam::GaussianBayesTree, gtsam::GaussianFactorGraph >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Ordering')
        [ varargout{1} varargout{2} ] = gtsam_wrapper(866, this, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'gtsam.KeyVector')
        [ varargout{1} varargout{2} ] = gtsam_wrapper(867, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.eliminatePartialMultifrontal');
      end
    end

    function varargout = eliminatePartialSequential(this, varargin)
      % ELIMINATEPARTIALSEQUENTIAL usage: eliminatePartialSequential(Ordering ordering), eliminatePartialSequential(KeyVector keys) : returns pair< gtsam::GaussianBayesNet, gtsam::GaussianFactorGraph >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Ordering')
        [ varargout{1} varargout{2} ] = gtsam_wrapper(868, this, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'gtsam.KeyVector')
        [ varargout{1} varargout{2} ] = gtsam_wrapper(869, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.eliminatePartialSequential');
      end
    end

    function varargout = eliminateSequential(this, varargin)
      % ELIMINATESEQUENTIAL usage: eliminateSequential(), eliminateSequential(Ordering ordering) : returns gtsam::GaussianBayesNet
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 0
        varargout{1} = gtsam_wrapper(870, this, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'gtsam.Ordering')
        varargout{1} = gtsam_wrapper(871, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.eliminateSequential');
      end
    end

    function varargout = equals(this, varargin)
      % EQUALS usage: equals(GaussianFactorGraph lfgraph, double tol) : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gtsam.GaussianFactorGraph') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(872, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.equals');
      end
    end

    function varargout = error(this, varargin)
      % ERROR usage: error(VectorValues c) : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.VectorValues')
        varargout{1} = gtsam_wrapper(873, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.error');
      end
    end

    function varargout = exists(this, varargin)
      % EXISTS usage: exists(size_t idx) : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(874, this, varargin{:});
    end

    function varargout = gradient(this, varargin)
      % GRADIENT usage: gradient(VectorValues x0) : returns gtsam::VectorValues
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.VectorValues')
        varargout{1} = gtsam_wrapper(875, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.gradient');
      end
    end

    function varargout = gradientAtZero(this, varargin)
      % GRADIENTATZERO usage: gradientAtZero() : returns gtsam::VectorValues
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(876, this, varargin{:});
    end

    function varargout = hessian(this, varargin)
      % HESSIAN usage: hessian(), hessian(Ordering ordering) : returns pair< Matrix, Vector >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 0
        [ varargout{1} varargout{2} ] = gtsam_wrapper(877, this, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'gtsam.Ordering')
        [ varargout{1} varargout{2} ] = gtsam_wrapper(878, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.hessian');
      end
    end

    function varargout = jacobian(this, varargin)
      % JACOBIAN usage: jacobian(), jacobian(Ordering ordering) : returns pair< Matrix, Vector >
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 0
        [ varargout{1} varargout{2} ] = gtsam_wrapper(879, this, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'gtsam.Ordering')
        [ varargout{1} varargout{2} ] = gtsam_wrapper(880, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.jacobian');
      end
    end

    function varargout = keyVector(this, varargin)
      % KEYVECTOR usage: keyVector() : returns gtsam::KeyVector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(881, this, varargin{:});
    end

    function varargout = keys(this, varargin)
      % KEYS usage: keys() : returns gtsam::KeySet
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(882, this, varargin{:});
    end

    function varargout = marginal(this, varargin)
      % MARGINAL usage: marginal(KeyVector key_vector) : returns gtsam::GaussianFactorGraph
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.KeyVector')
        varargout{1} = gtsam_wrapper(883, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.marginal');
      end
    end

    function varargout = marginalMultifrontalBayesNet(this, varargin)
      % MARGINALMULTIFRONTALBAYESNET usage: marginalMultifrontalBayesNet(Ordering ordering), marginalMultifrontalBayesNet(KeyVector key_vector), marginalMultifrontalBayesNet(Ordering ordering, Ordering marginalizedVariableOrdering), marginalMultifrontalBayesNet(KeyVector key_vector, Ordering marginalizedVariableOrdering) : returns gtsam::GaussianBayesNet
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Ordering')
        varargout{1} = gtsam_wrapper(884, this, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'gtsam.KeyVector')
        varargout{1} = gtsam_wrapper(885, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'gtsam.Ordering') && isa(varargin{2},'gtsam.Ordering')
        varargout{1} = gtsam_wrapper(886, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'gtsam.KeyVector') && isa(varargin{2},'gtsam.Ordering')
        varargout{1} = gtsam_wrapper(887, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.marginalMultifrontalBayesNet');
      end
    end

    function varargout = negate(this, varargin)
      % NEGATE usage: negate() : returns gtsam::GaussianFactorGraph
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(888, this, varargin{:});
    end

    function varargout = optimize(this, varargin)
      % OPTIMIZE usage: optimize(), optimize(Ordering ordering) : returns gtsam::VectorValues
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 0
        varargout{1} = gtsam_wrapper(889, this, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'gtsam.Ordering')
        varargout{1} = gtsam_wrapper(890, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.optimize');
      end
    end

    function varargout = optimizeGradientSearch(this, varargin)
      % OPTIMIZEGRADIENTSEARCH usage: optimizeGradientSearch() : returns gtsam::VectorValues
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(891, this, varargin{:});
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(892, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.print');
      end
    end

    function varargout = probPrime(this, varargin)
      % PROBPRIME usage: probPrime(VectorValues c) : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.VectorValues')
        varargout{1} = gtsam_wrapper(893, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.probPrime');
      end
    end

    function varargout = push_back(this, varargin)
      % PUSH_BACK usage: push_back(GaussianFactor factor), push_back(GaussianConditional conditional), push_back(GaussianFactorGraph graph), push_back(GaussianBayesNet bayesNet), push_back(GaussianBayesTree bayesTree) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.GaussianFactor')
        gtsam_wrapper(894, this, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'gtsam.GaussianConditional')
        gtsam_wrapper(895, this, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'gtsam.GaussianFactorGraph')
        gtsam_wrapper(896, this, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'gtsam.GaussianBayesNet')
        gtsam_wrapper(897, this, varargin{:});
      elseif length(varargin) == 1 && isa(varargin{1},'gtsam.GaussianBayesTree')
        gtsam_wrapper(898, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.push_back');
      end
    end

    function varargout = size(this, varargin)
      % SIZE usage: size() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(899, this, varargin{:});
    end

    function varargout = sparseJacobian_(this, varargin)
      % SPARSEJACOBIAN_ usage: sparseJacobian_() : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(900, this, varargin{:});
    end

    function varargout = string_serialize(this, varargin)
      % STRING_SERIALIZE usage: string_serialize() : returns string
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 0
        varargout{1} = gtsam_wrapper(901, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.string_serialize');
      end
    end

    function sobj = saveobj(obj)
      % SAVEOBJ Saves the object to a matlab-readable format
      sobj = obj.string_serialize();
    end
  end

  methods(Static = true)
    function varargout = string_deserialize(varargin)
      % STRING_DESERIALIZE usage: string_deserialize() : returns gtsam.GaussianFactorGraph
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1
        varargout{1} = gtsam_wrapper(902, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.GaussianFactorGraph.string_deserialize');
      end
    end

    function obj = loadobj(sobj)
      % LOADOBJ Saves the object to a matlab-readable format
      obj = gtsam.GaussianFactorGraph.string_deserialize(sobj);
    end
  end
end
