INCLUDES = -I/usr/local/include/opencv4 -I. \
			-I./include -I.
LIBS = -L./lib \
	-lopencv_core \
	-lopencv_imgproc \
	-lopencv_highgui \
	-lopencv_imgcodecs \
	-lopencv_aruco \
	-lopencv_stitching \


CXXFLAGS = -g -Wall -o0
OUTPUT = ./bin/main
HEADERS = 

SRCS = $(wildcard ./src/*.cpp)                                                                         
OBJS = $(SRCS:.cpp = .o)
CXX = g++

all:$(OUTPUT)
$(OUTPUT) : $(OBJS)
	$(CXX) -std=c++11 $^ -o $@ $(INCLUDES) $(LIBS)

%.o : %.cpp $(HEADERS)
	$(CXX) -std=c++11 -c $< $(CXXFLAGS)

.PHONY:clean
clean:
	rm -rf *.out *.oprocess
