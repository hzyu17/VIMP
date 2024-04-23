%class Values, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%Values()
%Values(Values other)
%
%-------Methods-------
%atCal3Bundler(size_t j) : returns gtsam::Cal3Bundler
%atCal3DS2(size_t j) : returns gtsam::Cal3DS2
%atCal3_S2(size_t j) : returns gtsam::Cal3_S2
%atConstantBias(size_t j) : returns gtsam::imuBias::ConstantBias
%atDouble(size_t j) : returns double
%atEssentialMatrix(size_t j) : returns gtsam::EssentialMatrix
%atMatrix(size_t j) : returns Matrix
%atPoint2(size_t j) : returns gtsam::Point2
%atPoint3(size_t j) : returns gtsam::Point3
%atPose2(size_t j) : returns gtsam::Pose2
%atPose3(size_t j) : returns gtsam::Pose3
%atRot2(size_t j) : returns gtsam::Rot2
%atRot3(size_t j) : returns gtsam::Rot3
%atVector(size_t j) : returns Vector
%clear() : returns void
%dim() : returns size_t
%empty() : returns bool
%equals(Values other, double tol) : returns bool
%erase(size_t j) : returns void
%exists(size_t j) : returns bool
%insert(Values values) : returns void
%insert(size_t j, Point2 point2) : returns void
%insert(size_t j, Point3 point3) : returns void
%insert(size_t j, Rot2 rot2) : returns void
%insert(size_t j, Pose2 pose2) : returns void
%insert(size_t j, Rot3 rot3) : returns void
%insert(size_t j, Pose3 pose3) : returns void
%insert(size_t j, Cal3_S2 cal3_s2) : returns void
%insert(size_t j, Cal3DS2 cal3ds2) : returns void
%insert(size_t j, Cal3Bundler cal3bundler) : returns void
%insert(size_t j, EssentialMatrix essential_matrix) : returns void
%insert(size_t j, SimpleCamera simpel_camera) : returns void
%insert(size_t j, ConstantBias constant_bias) : returns void
%insert(size_t j, Vector vector) : returns void
%insert(size_t j, Matrix matrix) : returns void
%insertDouble(size_t j, double c) : returns void
%keys() : returns gtsam::KeyVector
%localCoordinates(Values cp) : returns gtsam::VectorValues
%print(string s) : returns void
%retract(VectorValues delta) : returns gtsam::Values
%size() : returns size_t
%swap(Values values) : returns void
%update(Values values) : returns void
%update(size_t j, Point2 point2) : returns void
%update(size_t j, Point3 point3) : returns void
%update(size_t j, Rot2 rot2) : returns void
%update(size_t j, Pose2 pose2) : returns void
%update(size_t j, Rot3 rot3) : returns void
%update(size_t j, Pose3 pose3) : returns void
%update(size_t j, Cal3_S2 cal3_s2) : returns void
%update(size_t j, Cal3DS2 cal3ds2) : returns void
%update(size_t j, Cal3Bundler cal3bundler) : returns void
%update(size_t j, EssentialMatrix essential_matrix) : returns void
%update(size_t j, ConstantBias constant_bias) : returns void
%update(size_t j, Vector vector) : returns void
%update(size_t j, Matrix matrix) : returns void
%zeroVectors() : returns gtsam::VectorValues
%
%-------Serialization Interface-------
%string_serialize() : returns string
%string_deserialize(string serialized) : returns Values
%
classdef Values < handle
  properties
    ptr_gtsamValues = 0
  end
  methods
    function obj = Values(varargin)
      if nargin == 2 && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        my_ptr = varargin{2};
        gtsam_wrapper(1149, my_ptr);
      elseif nargin == 0
        my_ptr = gtsam_wrapper(1150);
      elseif nargin == 1 && isa(varargin{1},'gtsam.Values')
        my_ptr = gtsam_wrapper(1151, varargin{1});
      else
        error('Arguments do not match any overload of gtsam.Values constructor');
      end
      obj.ptr_gtsamValues = my_ptr;
    end

    function delete(obj)
      gtsam_wrapper(1152, obj.ptr_gtsamValues);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
    function varargout = atCal3Bundler(this, varargin)
      % ATCAL3BUNDLER usage: atCal3Bundler(size_t j) : returns gtsam::Cal3Bundler
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1153, this, varargin{:});
    end

    function varargout = atCal3DS2(this, varargin)
      % ATCAL3DS2 usage: atCal3DS2(size_t j) : returns gtsam::Cal3DS2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1154, this, varargin{:});
    end

    function varargout = atCal3_S2(this, varargin)
      % ATCAL3_S2 usage: atCal3_S2(size_t j) : returns gtsam::Cal3_S2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1155, this, varargin{:});
    end

    function varargout = atConstantBias(this, varargin)
      % ATCONSTANTBIAS usage: atConstantBias(size_t j) : returns gtsam::imuBias::ConstantBias
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1156, this, varargin{:});
    end

    function varargout = atDouble(this, varargin)
      % ATDOUBLE usage: atDouble(size_t j) : returns double
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1157, this, varargin{:});
    end

    function varargout = atEssentialMatrix(this, varargin)
      % ATESSENTIALMATRIX usage: atEssentialMatrix(size_t j) : returns gtsam::EssentialMatrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1158, this, varargin{:});
    end

    function varargout = atMatrix(this, varargin)
      % ATMATRIX usage: atMatrix(size_t j) : returns Matrix
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1159, this, varargin{:});
    end

    function varargout = atPoint2(this, varargin)
      % ATPOINT2 usage: atPoint2(size_t j) : returns gtsam::Point2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1160, this, varargin{:});
    end

    function varargout = atPoint3(this, varargin)
      % ATPOINT3 usage: atPoint3(size_t j) : returns gtsam::Point3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1161, this, varargin{:});
    end

    function varargout = atPose2(this, varargin)
      % ATPOSE2 usage: atPose2(size_t j) : returns gtsam::Pose2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1162, this, varargin{:});
    end

    function varargout = atPose3(this, varargin)
      % ATPOSE3 usage: atPose3(size_t j) : returns gtsam::Pose3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1163, this, varargin{:});
    end

    function varargout = atRot2(this, varargin)
      % ATROT2 usage: atRot2(size_t j) : returns gtsam::Rot2
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1164, this, varargin{:});
    end

    function varargout = atRot3(this, varargin)
      % ATROT3 usage: atRot3(size_t j) : returns gtsam::Rot3
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1165, this, varargin{:});
    end

    function varargout = atVector(this, varargin)
      % ATVECTOR usage: atVector(size_t j) : returns Vector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1166, this, varargin{:});
    end

    function varargout = clear(this, varargin)
      % CLEAR usage: clear() : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(1167, this, varargin{:});
    end

    function varargout = dim(this, varargin)
      % DIM usage: dim() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1168, this, varargin{:});
    end

    function varargout = empty(this, varargin)
      % EMPTY usage: empty() : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1169, this, varargin{:});
    end

    function varargout = equals(this, varargin)
      % EQUALS usage: equals(Values other, double tol) : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 2 && isa(varargin{1},'gtsam.Values') && isa(varargin{2},'double')
        varargout{1} = gtsam_wrapper(1170, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Values.equals');
      end
    end

    function varargout = erase(this, varargin)
      % ERASE usage: erase(size_t j) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(1171, this, varargin{:});
    end

    function varargout = exists(this, varargin)
      % EXISTS usage: exists(size_t j) : returns bool
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1172, this, varargin{:});
    end

    function varargout = insert(this, varargin)
      % INSERT usage: insert(Values values), insert(size_t j, Point2 point2), insert(size_t j, Point3 point3), insert(size_t j, Rot2 rot2), insert(size_t j, Pose2 pose2), insert(size_t j, Rot3 rot3), insert(size_t j, Pose3 pose3), insert(size_t j, Cal3_S2 cal3_s2), insert(size_t j, Cal3DS2 cal3ds2), insert(size_t j, Cal3Bundler cal3bundler), insert(size_t j, EssentialMatrix essential_matrix), insert(size_t j, SimpleCamera simpel_camera), insert(size_t j, ConstantBias constant_bias), insert(size_t j, Vector vector), insert(size_t j, Matrix matrix) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Values')
        gtsam_wrapper(1173, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Point2')
        gtsam_wrapper(1174, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Point3')
        gtsam_wrapper(1175, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Rot2')
        gtsam_wrapper(1176, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Pose2')
        gtsam_wrapper(1177, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Rot3')
        gtsam_wrapper(1178, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Pose3')
        gtsam_wrapper(1179, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Cal3_S2')
        gtsam_wrapper(1180, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Cal3DS2')
        gtsam_wrapper(1181, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Cal3Bundler')
        gtsam_wrapper(1182, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.EssentialMatrix')
        gtsam_wrapper(1183, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.SimpleCamera')
        gtsam_wrapper(1184, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.imuBias.ConstantBias')
        gtsam_wrapper(1185, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && size(varargin{2},2)==1
        gtsam_wrapper(1186, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'double')
        gtsam_wrapper(1187, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Values.insert');
      end
    end

    function varargout = insertDouble(this, varargin)
      % INSERTDOUBLE usage: insertDouble(size_t j, double c) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      gtsam_wrapper(1188, this, varargin{:});
    end

    function varargout = keys(this, varargin)
      % KEYS usage: keys() : returns gtsam::KeyVector
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1189, this, varargin{:});
    end

    function varargout = localCoordinates(this, varargin)
      % LOCALCOORDINATES usage: localCoordinates(Values cp) : returns gtsam::VectorValues
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Values')
        varargout{1} = gtsam_wrapper(1190, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Values.localCoordinates');
      end
    end

    function varargout = print(this, varargin)
      % PRINT usage: print(string s) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'char')
        gtsam_wrapper(1191, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Values.print');
      end
    end

    function varargout = retract(this, varargin)
      % RETRACT usage: retract(VectorValues delta) : returns gtsam::Values
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.VectorValues')
        varargout{1} = gtsam_wrapper(1192, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Values.retract');
      end
    end

    function varargout = size(this, varargin)
      % SIZE usage: size() : returns size_t
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1193, this, varargin{:});
    end

    function varargout = swap(this, varargin)
      % SWAP usage: swap(Values values) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Values')
        gtsam_wrapper(1194, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Values.swap');
      end
    end

    function varargout = update(this, varargin)
      % UPDATE usage: update(Values values), update(size_t j, Point2 point2), update(size_t j, Point3 point3), update(size_t j, Rot2 rot2), update(size_t j, Pose2 pose2), update(size_t j, Rot3 rot3), update(size_t j, Pose3 pose3), update(size_t j, Cal3_S2 cal3_s2), update(size_t j, Cal3DS2 cal3ds2), update(size_t j, Cal3Bundler cal3bundler), update(size_t j, EssentialMatrix essential_matrix), update(size_t j, ConstantBias constant_bias), update(size_t j, Vector vector), update(size_t j, Matrix matrix) : returns void
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1 && isa(varargin{1},'gtsam.Values')
        gtsam_wrapper(1195, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Point2')
        gtsam_wrapper(1196, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Point3')
        gtsam_wrapper(1197, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Rot2')
        gtsam_wrapper(1198, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Pose2')
        gtsam_wrapper(1199, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Rot3')
        gtsam_wrapper(1200, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Pose3')
        gtsam_wrapper(1201, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Cal3_S2')
        gtsam_wrapper(1202, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Cal3DS2')
        gtsam_wrapper(1203, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.Cal3Bundler')
        gtsam_wrapper(1204, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.EssentialMatrix')
        gtsam_wrapper(1205, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'gtsam.imuBias.ConstantBias')
        gtsam_wrapper(1206, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'double') && size(varargin{2},2)==1
        gtsam_wrapper(1207, this, varargin{:});
      elseif length(varargin) == 2 && isa(varargin{1},'numeric') && isa(varargin{2},'double')
        gtsam_wrapper(1208, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Values.update');
      end
    end

    function varargout = zeroVectors(this, varargin)
      % ZEROVECTORS usage: zeroVectors() : returns gtsam::VectorValues
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      varargout{1} = gtsam_wrapper(1209, this, varargin{:});
    end

    function varargout = string_serialize(this, varargin)
      % STRING_SERIALIZE usage: string_serialize() : returns string
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 0
        varargout{1} = gtsam_wrapper(1210, this, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Values.string_serialize');
      end
    end

    function sobj = saveobj(obj)
      % SAVEOBJ Saves the object to a matlab-readable format
      sobj = obj.string_serialize();
    end
  end

  methods(Static = true)
    function varargout = string_deserialize(varargin)
      % STRING_DESERIALIZE usage: string_deserialize() : returns gtsam.Values
      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
      if length(varargin) == 1
        varargout{1} = gtsam_wrapper(1211, varargin{:});
      else
        error('Arguments do not match any overload of function gtsam.Values.string_deserialize');
      end
    end

    function obj = loadobj(sobj)
      % LOADOBJ Saves the object to a matlab-readable format
      obj = gtsam.Values.string_deserialize(sobj);
    end
  end
end
