// OpenCVCam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

// These three header files required by OpenCV 
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "time.h"
#include "iostream"
// Header for the PSEye
#include "CLEyeMulticam.h"

#define FRAME_RATE		60
#define FRAME_SIZE		CLEYE_VGA
#define FRAME_FORMAT	CLEYE_COLOR_RAW

using namespace std;
using namespace cv;

typedef vector<Point2f> Point2fVector;

int H_MIN = 0;
int H_MAX = 255;
int S_MIN = 0;
int S_MAX = 255;
int V_MIN = 0;
int V_MAX = 255;
int BLUR = 0;

typedef struct{
	CLEyeCameraInstance CameraInstance;
	PBYTE FramePointer;
}CAMERA_AND_FRAME;

static DWORD WINAPI CaptureThread(LPVOID ThreadPointer);

void MousCallback(int mEvent, int x, int y, int flags, void* param)
{
	Point2fVector* pPointVec = (Point2fVector*)param;
	if (mEvent == CV_EVENT_LBUTTONDOWN)
	{
		pPointVec->push_back(Point2f(float(x), float(y)));
	}
}
void on_trackbar(int, void*)
{

}
void createTrackbars()
{
	//create window for trackbars
	namedWindow("Trackbars", 0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf(TrackbarName, "H_MIN", H_MIN);
	sprintf(TrackbarName, "H_MAX", H_MAX);
	sprintf(TrackbarName, "S_MIN", S_MIN);
	sprintf(TrackbarName, "S_MAX", S_MAX);
	sprintf(TrackbarName, "V_MIN", V_MIN);
	sprintf(TrackbarName, "V_MAX", V_MAX);
	sprintf(TrackbarName, "BLUR", BLUR);
	//create trackbars and insert them into window to change H,S,V values

	createTrackbar("H_MIN", "Trackbars", &H_MIN, H_MAX, on_trackbar);
	createTrackbar("H_MAX", "Trackbars", &H_MAX, H_MAX, on_trackbar);
	createTrackbar("S_MIN", "Trackbars", &S_MIN, S_MAX, on_trackbar);
	createTrackbar("S_MAX", "Trackbars", &S_MAX, S_MAX, on_trackbar);
	createTrackbar("V_MIN", "Trackbars", &V_MIN, V_MAX, on_trackbar);
	createTrackbar("V_MAX", "Trackbars", &V_MAX, V_MAX, on_trackbar);
	createTrackbar("BLUR", "Trackbars", &BLUR, 24, on_trackbar);
}

int _tmain(int argc, _TCHAR* argv[])
{

	///////MY VARS////////
	PBYTE FramePointer=NULL;
	int width,height,CameraCount,FramerCounter=0;
	CLEyeCameraInstance EyeCamera=NULL;
	GUID CameraID;
	IplImage *frame;
	clock_t StartTime,EndTime;
	CAMERA_AND_FRAME ThreadPointer;
	HANDLE _hThread;
	//////////////////////

	//Check for presence of EYE
	CameraCount=CLEyeGetCameraCount();
	if(CameraCount>0) printf("Number of EYE's detected: %d\n\n",CameraCount);
	else{
		printf("No camera detected, press any key to exit...");
		getchar();
		return 0;
	}
	// Get ID of first PSEYE
	CameraID = CLEyeGetCameraUUID(0);
	// Get connection to camera and send it running parameters
	EyeCamera = CLEyeCreateCamera(CameraID,FRAME_FORMAT,FRAME_SIZE,FRAME_RATE);
	//Couldn't Connect to camera
	if(EyeCamera == NULL){
		printf("Couldn't connect to camera, press any key to exit...");
		getchar();
		return 0;
	}
	// Set some camera parameters;
	CLEyeSetCameraParameter(EyeCamera, CLEYE_EXPOSURE, 511);
	CLEyeSetCameraParameter(EyeCamera, CLEYE_GAIN, 0);
	// Get camera frame dimensions;
	CLEyeCameraGetFrameDimensions(EyeCamera, width, height);
	// Create a window in which the captured images will be presented
	cvNamedWindow( "Camera", CV_WINDOW_AUTOSIZE );
	//Make a image to hold the frames captured from the camera
	frame=cvCreateImage(cvSize(width ,height),IPL_DEPTH_8U, 4);
	// GetPointer To Image Data For frame
	cvGetImageRawData(frame,&FramePointer);
	//Start the eye camera
	CLEyeCameraStart(EyeCamera);	

	//Need to copy vars into one var to launch the second thread
	ThreadPointer.CameraInstance=EyeCamera;
	ThreadPointer.FramePointer=FramePointer;
	//Launch thread and confirm its running
	_hThread = CreateThread(NULL, 0, &CaptureThread, &ThreadPointer, 0, 0);
	if(_hThread == NULL)
	{
		printf("failed to create thread...");
		getchar();
		return false;
	}

	Mat camImage;
	camImage = frame;
	//camImage = cvLoadImage("camera.jpg");

	//Take a couple of pics to warm up
	printf("Press Enter to start");
	getchar();

	imshow("Camera Input", camImage);

	MessageBoxA(NULL, "Please click four circle centers of the air hockey table.\n"
		"Click the left down corner first and counter-clockwise for the rest.",
		"Click", MB_OK);

	Point2fVector points;

	cvSetMouseCallback("Camera Input", MousCallback, &points);

	while (1)
	{
		// wait for mouse clicks
		waitKey(10);
		if (points.size() == 4)
			break;
	}

	// HERE I ASSUME EACH PIXEL WILL BE 2 mm
	double scale = 2.0;
	Point2fVector points2;
	points2.push_back(Point2f(0.0, 1390.0 / scale));
	points2.push_back(Point2f(680.0 / scale, 1390.0 / scale));
	points2.push_back(Point2f(680.0 / scale, 0.0));
	points2.push_back(Point2f(0.0, 0.0));

	Mat_<double> H = findHomography(Mat(points), Mat(points2));

	cout << "The transformation Matrix is :" << endl;
	//printMatrix(H);
	cout << endl;

	Mat unwarpImage, hsvImage, filtered, blurred;
	Scalar red(0, 0, 255);
	Scalar blue(255, 0, 0);
	Scalar green(0, 255, 0);
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	string filename;
	vector<vector<Point>> contours;
	vector<Point> lastPoints;
	int pointIndex = 0, allowedDelta = 50, zeroBuffer = 2, lefty = 0, righty = 0;

	createTrackbars();

	while( 1 ) {
		//Display the captured frame
		cvShowImage( "Camera", frame );
		// Do vision here?
		try
		{
			// Take/Load image
			//filename = (string)"C:\\Temp\\capture" + to_string(i) + ".bmp";
			camImage = frame;
			// Warp perspective
			warpPerspective(camImage, unwarpImage, H, Size(680 / scale, 1390 / scale));
			// Convert to HSV
			cvtColor(unwarpImage, hsvImage, COLOR_BGR2HSV);
			// Blur to remove noise
			medianBlur(hsvImage, blurred, (3));
			// Threshold by green
			//inRange(blurred, Scalar(70, 38, 93), Scalar(93, 255, 146), filtered);
			inRange(blurred, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), filtered);

			// Erode to reduce noise
			erode(filtered, filtered, element);
			imshow("Thresholded Image", filtered);
			// Find Puck
			findContours(filtered, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			// For each contor, make sure the area is appropriate
			// //Maybe, can't get this to work/really slow
			//string msg = "Found " + to_string(contours.size()) + " contours";
			//printf(msg.c_str());
			if (contours.size() < 1)
			{
				//throw exception("No contours found");
			}
			else
			{
				// Calculate center of mass 
				Moments m = moments(contours[0], true);
				Point p(m.m10 / m.m00, m.m01 / m.m00);

				int pointsSize = lastPoints.size();

				if (pointsSize != 0 && ((p.x > lastPoints[pointsSize - 1].x + allowedDelta) || (p.x < lastPoints[pointsSize - 1].x - allowedDelta) ||
					(p.y > lastPoints[pointsSize - 1].y + allowedDelta) || (p.y > lastPoints[pointsSize - 1].y + allowedDelta)))
				{
					printf("Throwing out point %f, %f\n", p.x, p.y);
				}
				else
				{
					lastPoints.push_back(p);
					//cout << "added point";
					//cout << endl;
					if (pointsSize > 30)
						lastPoints.erase(lastPoints.begin());
					if (pointsSize > 5)
					{
						Vec4f lines;

						fitLine(Mat(lastPoints), lines, 2, 0, 0.01, 0.01);

						lefty = (-lines[2] * lines[1] / lines[0]) + lines[3];
						righty = ((unwarpImage.cols - lines[2])*lines[1] / lines[0]) + lines[3];
					}


					for (int j = 0; j < pointsSize; j++)
						circle(unwarpImage, lastPoints[j], 3, green, -1);

					line(unwarpImage, Point(unwarpImage.cols - 1, righty), Point(0, lefty), Scalar(255, 0, 0), 2);
					drawContours(unwarpImage, contours, -1, blue, 3);
					imshow("Drawn Image", unwarpImage);

					if (FramerCounter == 0) StartTime = clock();
					FramerCounter++;
					EndTime = clock();
					if ((EndTime - StartTime) / CLOCKS_PER_SEC >= 1)
					{
						printf("Processed FPS: %d\n", FramerCounter);
						FramerCounter = 0;
					}
				}
			}
		}
		catch (Exception E)
		{
			string errorMessage = (string)"Exception in main: \n " + E.msg;
			MessageBoxA(NULL, errorMessage.c_str(), "Exception", MB_OK);
		}
		catch (exception E)
		{
			string errorMessage = (string)"Exception in main: \n " + E.what();
			MessageBoxA(NULL, errorMessage.c_str(), "Exception", MB_OK);
		}

		//If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
		//remove higher bits using AND operator
		if( (cvWaitKey(1) & 255) == 27 ) break;
		//}

	}


	// reset the callback function
	cvSetMouseCallback("Camera Input", NULL, NULL);
	
	CLEyeCameraStop(EyeCamera);
	CLEyeDestroyCamera(EyeCamera);
	EyeCamera = NULL;
	cvDestroyWindow( "Camera" );

	return 0;
}

//static void VisionProcessing(Mat frame, Mat H)
//{
//	Mat unwarpImage, hsvImage, filtered, blurred, camImage;
//	try
//	{
//		// Take/Load image
//		//filename = (string)"C:\\Temp\\capture" + to_string(i) + ".bmp";
//		camImage = frame;
//		// Warp perspective
//		warpPerspective(camImage, unwarpImage, H, Size(680 / scale, 1390 / scale));
//		// Convert to HSV
//		cvtColor(unwarpImage, hsvImage, COLOR_BGR2HSV);
//		// Blur to remove noise
//		medianBlur(hsvImage, blurred, (3));
//		// Threshold by green
//		//inRange(blurred, Scalar(70, 38, 93), Scalar(93, 255, 146), filtered);
//		inRange(blurred, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), filtered);
//
//		// Erode to reduce noise
//		erode(filtered, filtered, element);
//		imshow("Thresholded Image", filtered);
//		// Find Puck
//		findContours(filtered, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
//		// For each contor, make sure the area is appropriate
//		// //Maybe, can't get this to work/really slow
//		//string msg = "Found " + to_string(contours.size()) + " contours";
//		//printf(msg.c_str());
//		if (contours.size() < 1)
//		{
//			//throw exception("No contours found");
//		}
//		else
//		{
//			// Calculate center of mass 
//			Moments m = moments(contours[0], true);
//			Point p(m.m10 / m.m00, m.m01 / m.m00);
//
//			int pointsSize = lastPoints.size();
//
//			if (pointsSize != 0 && ((p.x > lastPoints[pointsSize - 1].x + allowedDelta) || (p.x < lastPoints[pointsSize - 1].x - allowedDelta) ||
//				(p.y > lastPoints[pointsSize - 1].y + allowedDelta) || (p.y > lastPoints[pointsSize - 1].y + allowedDelta)))
//			{
//				printf("Throwing out point %f, %f\n", p.x, p.y);
//			}
//			else
//			{
//				lastPoints.push_back(p);
//				//cout << "added point";
//				//cout << endl;
//				if (pointsSize > 30)
//					lastPoints.erase(lastPoints.begin());
//				if (pointsSize > 5)
//				{
//					Vec4f lines;
//
//					fitLine(Mat(lastPoints), lines, 2, 0, 0.01, 0.01);
//
//					lefty = (-lines[2] * lines[1] / lines[0]) + lines[3];
//					righty = ((unwarpImage.cols - lines[2])*lines[1] / lines[0]) + lines[3];
//				}
//
//
//				for (int j = 0; j < pointsSize; j++)
//					circle(unwarpImage, lastPoints[j], 3, green, -1);
//
//				line(unwarpImage, Point(unwarpImage.cols - 1, righty), Point(0, lefty), Scalar(255, 0, 0), 2);
//				drawContours(unwarpImage, contours, -1, blue, 3);
//				imshow("Drawn Image", unwarpImage);
//
//				if (FramerCounter == 0) StartTime = clock();
//				FramerCounter++;
//				EndTime = clock();
//				if ((EndTime - StartTime) / CLOCKS_PER_SEC >= 1)
//				{
//					printf("Processed FPS: %d\n", FramerCounter);
//					FramerCounter = 0;
//				}
//			}
//		}
//	}
//	catch (Exception E)
//	{
//		string errorMessage = (string)"Exception in main: \n " + E.msg;
//		MessageBoxA(NULL, errorMessage.c_str(), "Exception", MB_OK);
//	}
//	catch (exception E)
//	{
//		string errorMessage = (string)"Exception in main: \n " + E.what();
//		MessageBoxA(NULL, errorMessage.c_str(), "Exception", MB_OK);
//	}
//}

static DWORD WINAPI CaptureThread(LPVOID ThreadPointer){
	CAMERA_AND_FRAME *Instance=(CAMERA_AND_FRAME*)ThreadPointer;
	CLEyeCameraInstance Camera=Instance->CameraInstance;
	PBYTE FramePtr= Instance->FramePointer;
	int FramerCounter=0;
	clock_t StartTime,EndTime;
	

	while(1){
		//Get Frame From Camera
		CLEyeCameraGetFrame(Camera,FramePtr);

		// put your vision code here

		// Track FPS
		if(FramerCounter==0) StartTime=clock();
		FramerCounter++;
		EndTime=clock();
		if((EndTime-StartTime)/CLOCKS_PER_SEC>=1)
		{
			printf("FPS: %d\n",FramerCounter);
			FramerCounter=0;
		}
	}
	return 0;
}

