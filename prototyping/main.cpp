#include "main.h"
#include "AffineKalmanLP.h"
#include "cvErrorRedirector.h"
#include <vector>
#include <cmath>
#include <boost/tuple/tuple.hpp>

#include "gnuplot-iostream.h"

#define movement_detect         1
//#define INPUTFILE               "GOPR4290_SC.MP4"
//#define INPUTFILE               "GOPR4291_SC.MP4"
//#define INPUTFILE               "GOPR4288.MP4"
//#define INPUTFILE               "GOPR4292_SC.MP4"
//#define INPUTFILE               "GOPR4294_SC.MP4"
//#define INPUTFILE               "IMG_2730.mp4"
#define INPUTFILE               "IMG_2731_SC_2.mp4"
#define OUTPUTFILE              "stabilized_output.avi"

const int HORIZONTAL_BORDER_CROP = 30;

int main(int argc, char **argv)
{
    cvErrorRedirector redir;
    
    Gnuplot gp;
    // For debugging or manual editing of commands:
    //Gnuplot gp(std::fopen("plot.gnu"));
    // or
    //Gnuplot gp("tee plot.gnu | gnuplot -persist");

    std::vector<std::pair<double, double> > xy_pts_A;
    gp << "set autoscale xfix\nset autoscale yfix\n";
    //gp << "plot '-' with lines title 'translation'\n";
    gp << "plot" << gp.file1d(xy_pts_A) << "with lines title 'cubic'\n";
    
    //Create a object of stabilization class
    AKLP_VideoStab *stab = new AKLP_VideoStab();

    //Initialize the VideoCapture object
    cv::VideoCapture cap(INPUTFILE);

    cv::Mat frame_2, frame2;
    cv::Mat frame_1, frame1;

    cap >> frame_1;
    cv::cvtColor(frame_1, frame1, cv::COLOR_BGR2GRAY);

    cv::Mat smoothedMat(2,3,CV_64F);

    #ifdef OUTPUTFILE
        cv::VideoWriter outputVideo;
        bool isVideoWriterInit = false;
    #endif

        static int tryes = 10;
    while(true)
    {
        static int num_of_frames = 0;
        try {
            std::cout << "Start: " << ++num_of_frames << std::endl;
            
            if(num_of_frames == 17){
                num_of_frames = 17;
            }
            
            std::cout << "CP: 1" << std::endl;
            cap >> frame_2;

            std::cout << "CP: 2" << std::endl;
            if(frame_2.data == NULL)
            {
                //delete stab;
                // It's a good idea to reset path parameters when there's an exception.
                // Unless you want to put something graceful
                //stab = new AKLP_VideoStab();
                cap >> frame_1;
                cv::cvtColor(frame_1, frame1, cv::COLOR_BGR2GRAY);
                
                if(tryes == 0)
                    break;
                else {
                    tryes--;
                    continue;
                }
            }

            std::cout << "CP: 3" << std::endl;
            cv::cvtColor(frame_2, frame2, cv::COLOR_BGR2GRAY);

            std::cout << "CP: 4" << std::endl;
            cv::Mat smoothedFrame;

            std::cout << "CP: 5" << std::endl;
            smoothedFrame = stab->stabilize(frame_1 , frame_2);
            
            

            std::cout << "CP: 6" << std::endl;
            
                
            std::cout << "CP: 7" << std::endl;
            cv::Mat resized_smoothedFrame;
            
            std::cout << "CP: 8" << std::endl;
            cv::resize(smoothedFrame, resized_smoothedFrame,  cv::Size(1440, 1024));
            
            static float p_diff_x_acc = 0;
            static float p_diff_y_acc = 0;
            p_diff_x_acc += stab->p_diff_x;
            p_diff_y_acc += stab->p_diff_y;
            xy_pts_A.push_back(std::make_pair(-p_diff_x_acc, p_diff_y_acc));
            
            gp.send1d(xy_pts_A);
            gp << "plot" << gp.file1d(xy_pts_A) << "with lines title 'cubic'\n";
            //gp << "reread\n";
            
            std::ostringstream stringStream;
            stringStream << "transx: " + std::to_string(p_diff_x_acc) + ", transy: " + std::to_string(p_diff_y_acc);
            std::string copyOfStr = stringStream.str();
            //std::string("transx: " + std::to_string(stab->transX) + ", transy: " + std::to_string(stab->transY)),
            if(movement_detect) {
            cv::putText(resized_smoothedFrame, 
                copyOfStr,
                cv::Point(5, 40), // Coordinates
                cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                1.0, // Scale. 2.0 = 2x bigger
                cv::Scalar(255,255,255), // BGR Color
                1, // Line Thickness (Optional)
                8,
                false); // Anti-alias (Optional)
            }
            
            std::cout << "CP: 9" << std::endl;
            cv::imshow("Stabilized Video" , resized_smoothedFrame);
            
            #ifdef OUTPUTFILE
                if(isVideoWriterInit == false) {
                    isVideoWriterInit = true;
                    outputVideo.open(OUTPUTFILE , CV_FOURCC('X' , 'V' , 'I' , 'D'), 30 , frame_1.size());
                }
                outputVideo.write(resized_smoothedFrame);
            #endif

            std::cout << "CP: 10" << std::endl;
            cv::waitKey(10);

            std::cout << "CP: 11" << std::endl;
            frame_1 = frame_2.clone();
            
            std::cout << "CP: 12" << std::endl;
            frame2.copyTo(frame1);
            
            std::cout << "End: " << num_of_frames << std::endl;
        } catch (cv::Exception& e) {
            delete stab;
            // It's a good idea to reset path parameters when there's an exception.
            // Unless you want to put something graceful
            stab = new AKLP_VideoStab();
            cap >> frame_1;
            cv::cvtColor(frame_1, frame1, cv::COLOR_BGR2GRAY);
        }

    }

    return 0;
}
