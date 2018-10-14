#include <iostream>
#include <videostab.h>

using namespace std;

int main()
{
    //Create a object of stabilization class
    VideoStab stab;

    //Initialize the VideoCapture object
    VideoCapture cap(0);

    Mat frame_2, frame2;
    Mat frame_1, frame1;

    cap >> frame_1;

    Mat smoothedMat(2,3,CV_64F);

    VideoWriter outputVideo;
    outputVideo.open("com.avi" , CV_FOURCC('X' , 'V' , 'I' , 'D'), 30 , frame_1.size());

    while(true)
    {
        cap >> frame_2;

        if(frame_2.data == NULL)
        {
            break;
        }

        cvtColor(frame_2, frame2, COLOR_BGR2GRAY);

        Mat smoothedFrame;

        smoothedFrame = stab.stabilize(frame_1 , frame_2);

        outputVideo.write(smoothedFrame);

        if(waitKey(10) == 27)
            break;

        frame_1 = frame_2.clone();
        frame2.copyTo(frame1);
    }

    return 0;
}


