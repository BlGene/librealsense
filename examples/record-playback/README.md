# install libopencv-core-dev, libglfw3-dev
g++  rs-record-minimal.cpp  -lrealsense2  -lstdc++fs -lopencv_core -lopencv_imgcodecs -lpthread -o rs-record-minimal

# The following line compiles is for the gui version
#g++ ../../third-party/imgui/*.cpp rs-record-minimal.cpp -I.. -I../../third-party -I../../third-party/imgui -lGL -lglfw -lGLU -lrealsense2 -lm -lstdc++fs -lopencv_core -lopencv_imgcodecs -lpthread -o rs-record-minimal

# The following line compiles the minimal example, can un-stick driver
#gcc main.c -lrealsense2 -lm -o main
