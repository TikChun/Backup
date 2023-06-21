CXX            = g++
VISP_BUILD_DIR = ${VISP_WS}/visp-build
VISP_CFLAGS    = `/home/tikchuntong/visp-ws/visp-build/bin/visp-config --cflags`
VISP_LDFLAGS   = `/home/tikchuntong/visp-ws/visp-build/bin/visp-config --libs`

realsense_streaming: realsense_streaming.cpp
	$(CXX) -g $(VISP_CFLAGS) -o realsense_streaming realsense_streaming.cpp $(VISP_LDFLAGS) $(pkg-config --cflags --libs opencv4)  -lrealsense2

clean:
	rm -f *~ realsense_streaming