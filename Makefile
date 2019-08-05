CPPFLAGS=-g -Wall -I. -DDEBUG
LDFLAGS=-g
LDLIBS=-lopencv_core -lopencv_calib3d -lopencv_highgui -lopencv_imgproc -lopencv_stitching -lopencv_video

main: main.o videostab.o
	g++ $(LDFLAGS) -o main main.o videostab.o $(LDLIBS)

main.o: main.cpp
	g++ $(CPPFLAGS) -c main.cpp

videostab.o: videostab.cpp videostab.h
	g++ $(CPPFLAGS) -c videostab.cpp

clean:
	rm main main.o videostab.o
