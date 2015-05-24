#
# Choose compiler
#
CPP = g++

#
# MATLAB
#
MATLABDIR=/Applications/MATLAB_R2014b.app
MATLAB_INCLUDE=-I$(MATLABDIR)/extern/include
MATLAB_LIB=-L$(MATLABDIR)/bin/maci64

#
# Compiler flags
#
CFLAGS= -O3 $(MATLAB_INCLUDE)

#
#  MYO framework (SDK)
#
FRAMEWORK_MYO = -F/Users/JNGApura/Documents/MATLAB -framework myo

myoDemo: myoDemo.cpp
	$(CPP) $(CFLAGS) $(MATLAB_LIB) $(FRAMEWORK_MYO) -rpath @loader_path -leng -lmx -lm -lmat -lut -lstdc++ myoDemo.cpp -o myoDemo