MATLAB_ROOT=/usr/local/MATLAB/R2024a
EIGEN_PATH=/usr/include/eigen3

CXX=g++
CXXFLAGS=-fPIC -shared -std=c++17
MEXFLAGS=-I$(MATLAB_ROOT)/extern/include -I$(EIGEN_PATH) -I. -I../GaussianVI -I../GaussianVI/include/cereal/include

LDFLAGS=-L$(MATLAB_ROOT)/bin/glnxa64 -lmex -lmx -lmat

TARGET=SignedDistanceField_mex.mexa64
SRC=SignedDistanceField_mex.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(MEXFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET)
