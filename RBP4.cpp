#include <opencv2/opencv.hpp>
#include <raspicam_cv.h> // RBP cam
#include <iostream>
#include <chrono>
#include <ctime>
#include <wiringPi.h>

using namespace std;
using namespace cv;
using namespace raspicam;

/*	Global Variables and Matrices	*/
// 	frame: Stores the original captured frame from the camera
//	framePers, frameGray, frameThresh, frameEdge, frameFinal: Intermediate images/matrices for different stages of image processing
// 	ROILane, ROILaneEnd: Regions of interest for lane detection
//	LeftLanePos, RightLanePos, frameCenter, laneCenter, Result, laneEnd: Variables to store lane detection results and other measurements
//	Camera: The RaspiCam_Cv object used to interface with the Raspberry Pi camera module
// 	histrogramLane, histrogramLaneEnd: Vectors to store the histograms of lane regions for analysis
//	Source[]: An array of four points representing the source points for perspective transformation
//	Destination[]: An array of four points representing the destination points for perspective transformation
Mat frame, framePers, frameGray, frameThresh, frameEdge, frameFinal;
Mat ROILane, ROILaneEnd;
int LeftLanePos, RightLanePos, frameCenter, laneCenter, Result, laneEnd;

RaspiCam_Cv Camera;
stringstream ss;

vector<int> histrogramLane(320);
vector<int> histrogramLaneEnd(320);

Point2f Source[] = {Point2f(26, 100), Point2f(294, 100), Point2f(0, 140), Point2f(320, 140)};
Point2f Destination[] = {Point2f(0, 0), Point2f(320, 0), Point2f(0, 240), Point2f(320, 240)};

void Setup(int argc, char **argv, RaspiCam_Cv &Camera)
{
    // Configures the camera settings using Camera.set() functions
    Camera.set(CAP_PROP_FRAME_WIDTH, 320);
    Camera.set(CAP_PROP_FRAME_HEIGHT, 240);
    Camera.set(CAP_PROP_BRIGHTNESS, 50);
    Camera.set(CAP_PROP_CONTRAST, 50);
    Camera.set(CAP_PROP_SATURATION, 50);
    Camera.set(CAP_PROP_GAIN, 50);
    Camera.set(CAP_PROP_FPS, 0);
}

void Capture()
{
    //	Grabs a frame from the camera and retrieves it into the frame matrix
    Camera.grab();
    Camera.retrieve(frame);
}

void Perspective()
{
    //	Applies perspective transformation to the frame matrix using the source and destination points defined earlier
    line(frame, Source[0], Source[1], Scalar(0, 0, 255), 2);
    line(frame, Source[1], Source[3], Scalar(0, 0, 255), 2);
    line(frame, Source[3], Source[2], Scalar(0, 0, 255), 2);
    line(frame, Source[2], Source[0], Scalar(0, 0, 255), 2);

    Mat Matrix = getPerspectiveTransform(Source, Destination);
    warpPerspective(frame, framePers, Matrix, Size(320, 240));
}

void Threshold()
{
    //	Threshold(): Performs image thresholding and edge detection on the perspective-transformed frame to obtain a binary image with lane markings
    cvtColor(framePers, frameGray, COLOR_BGR2GRAY);
    inRange(frameGray, 230, 255, frameThresh);
    Canny(frameGray, frameEdge, 900, 900, 3, false);
    add(frameThresh, frameEdge, frameFinal);
    cvtColor(frameFinal, frameFinal, COLOR_GRAY2BGR);
}

void Histrogram()
{
    //	Computes the histogram of lane regions in the binary image to determine the lane positions
    for (int i = 0; i < 320; i++)
    {
        ROILane = frameFinal(Rect(i, 120, 1, 80));
        divide(255, ROILane, ROILane);
        histrogramLane[i] = static_cast<int>(sum(ROILane)[0]);
    }

    for (int i = 0; i < 320; i++)
    {
        ROILaneEnd = frameFinal(Rect(i, 0, 1, 240));
        divide(255, ROILaneEnd, ROILaneEnd);
        histrogramLaneEnd[i] = static_cast<int>(sum(ROILaneEnd)[0]);
    }

    laneEnd = sum(histrogramLaneEnd)[0];
    cout << "Lane END = " << laneEnd << endl;
}

void LaneFinder()
{
    //	Determines the lane positions and calculates the lane center and its deviation from the frame center
    LeftLanePos = distance(histrogramLane.begin(), max_element(histrogramLane.begin(), histrogramLane.begin() + 150));
    RightLanePos = distance(histrogramLane.begin() + 150, max_element(histrogramLane.begin() + 150, histrogramLane.end())) + 150;

    laneCenter = (RightLanePos - LeftLanePos) / 2 + LeftLanePos;
    frameCenter = 160;

    line(frameFinal, Point2f(LeftLanePos, 0), Point2f(LeftLanePos, 240), Scalar(0, 255, 0), 2);
    line(frameFinal, Point2f(RightLanePos, 0), Point2f(RightLanePos, 240), Scalar(0, 255, 0), 2);

    line(frameFinal, Point2f(laneCenter, 0), Point2f(laneCenter, 240), Scalar(0, 255, 0), 3);
    line(frameFinal, Point2f(frameCenter, 0), Point2f(frameCenter, 240), Scalar(255, 0, 0), 3);
    Result = laneCenter - frameCenter;
}

int main(int argc, char **argv)
{
    wiringPiSetup();

    Setup(argc, argv, Camera);
    cout << "Connecting to camera" << endl;

    if (!Camera.open())
    {
        cout << "Error opening the camera" << endl;
        return -1;
    }

    cout << "Camera ID = " << Camera.getId() << endl;

    namedWindow("Original", WINDOW_KEEPRATIO);
    namedWindow("Perspective", WINDOW_KEEPRATIO);
    namedWindow("Final", WINDOW_KEEPRATIO);
    moveWindow("Original", 0, 100);
    moveWindow("Perspective", 320, 100);
    moveWindow("Final", 640, 100);
    resizeWindow("Original", 320, 240);
    resizeWindow("Perspective", 320, 240);
    resizeWindow("Final", 320, 240);

    /*	DIO */
    pinMode(13, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(14, OUTPUT);
    pinMode(10, OUTPUT);

    while (true)
    {	
        auto start = chrono::system_clock::now();

        Capture();
        Perspective();
        Threshold();
        Histrogram();
        LaneFinder();

        cout << "Lane Center Position = " << laneCenter << endl;
        cout << "Lane Position from center = " << Result << endl;

        auto end = chrono::system_clock::now();
        chrono::duration<double> elapsed_seconds = end - start;
        float t = elapsed_seconds.count();

        int FPS = 1.0 / t;

        cout << "FPS = " << FPS << endl;

        if (Result > -15 and Result < 15)
        {
            ss.str(" ");
            ss.clear();
            ss << "Result = Move Forward";
            putText(frame, ss.str(), Point2f(1, 50), 0, 1, Scalar(0, 0, 255), 2);

            digitalWrite(13, 0);
            digitalWrite(6, 0); // decimal = 0
            digitalWrite(14, 0);
            digitalWrite(10, 0);
        }

        else if (Result > 15)
        {
            ss.str(" ");
            ss.clear();
            ss << "Result = Move Right";
            putText(frame, ss.str(), Point2f(1, 50), 0, 1, Scalar(0, 0, 255), 2);

            digitalWrite(13, 1);
            digitalWrite(6, 0); // decimal = 1
            digitalWrite(14, 0);
            digitalWrite(10, 0);
        }

        else if (Result < -15)
        {
            ss.str(" ");
            ss.clear();
            ss << "Result = Move Left";
			
            digitalWrite(13, 0);
            digitalWrite(6, 1); // decimal = 2
            digitalWrite(14, 0);
            digitalWrite(10, 0);
            putText(frame, ss.str(), Point2f(1, 50), 0, 1, Scalar(0, 0, 255), 2);
        }
		cout<<Result;
		
        imshow("Original", frame);
        imshow("Perspective", framePers);
        imshow("Final", frameFinal);

        if (waitKey(1) == 27)
        {
            break;
        }
		
	}

    return 0;
}