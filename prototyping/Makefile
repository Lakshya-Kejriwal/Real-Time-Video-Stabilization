#CPPFLAGS=-g -Wall -I. -DDEBUG
#LDFLAGS=-g
CXXFLAGS+=-Wall -Wextra -O0 -g -D_GLIBCXX_DEBUG
#LDFLAGS+=-g -lutil -lboost_iostreams -lboost_system -lboost_filesystem 
LDLIBS+=-lutil -lboost_iostreams -lboost_system -lboost_filesystem -lopencv_core -lopencv_calib3d -lopencv_highgui -lopencv_imgproc -lopencv_stitching -lopencv_video

main: main.o AffineKalmanLP.o
	g++ $(LDFLAGS) -o main main.o AffineKalmanLP.o $(LDLIBS)

main.o: main.cpp main.h
	g++ $(CPPFLAGS) -c main.cpp

AffineKalmanLP.o: AffineKalmanLP.cpp AffineKalmanLP.h
	g++ $(CPPFLAGS) -c AffineKalmanLP.cpp

clean:
	rm main main.o AffineKalmanLP.o
