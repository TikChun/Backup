CXX            = g++
VISP_BUILD_DIR = ${VISP_WS}/visp-build
VISP_CFLAGS    = `/home/tikchuntong/visp-ws/visp-build/bin/visp-config --cflags`
VISP_LDFLAGS   = `/home/tikchuntong/visp-ws/visp-build/bin/visp-config --libs`

tutorial-grabber-realsense: tutorial-grabber-realsense.cpp
	$(CXX) -g $(VISP_CFLAGS) -o tutorial-grabber-realsense tutorial-grabber-realsense.cpp $(VISP_LDFLAGS) -lpthread -pthread

clean:
	rm -f *~ tutorial-grabber-realsense