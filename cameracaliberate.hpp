// This is start of the header guard.  ADD_H can be any unique name.  By convention, we use the name of the header file.
#ifndef CAMERA_CALIBERATION_H
#define CAMERA_CALIBERATION_H

#include <opencv2/core/core.hpp>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <pylon/PylonIncludes.h>


// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using GenApi objects
using namespace GenApi;

// Namespace for using opencv objects.
using namespace cv;

// Namespace for using cout.
using namespace std;

// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 20;


class CameraCaliberation
{
	public:
		//cameracaliberation(unsigned short int);
		void GetImages(unsigned short int);
		bool Caliberate();
		bool Caliberate2();
		void GetTransformation(unsigned short int, char*, char*, char*, char*, char*, char*);
		void StressTest(unsigned short int, unsigned short int, char*, char*, char*, char*, char*, char*, char*, char*);
		void ShowUndistorted(char*, char*);

	private:

		unsigned int BoardWidth, BoardHeight, NumberofBoards;
		float SizeofSquare;
		cv::Size ImageSize;
		char FileName[100];

		std::vector< std::vector< cv::Point2f> > ImagePoints;
		std::vector< std::vector< cv::Point3f> > ObjectPoints;

		std::vector< cv::Mat> RotationVector, TranslationVector;
		cv::Mat IntrinsicMatrix;
		cv::Mat DistortionCoefficient;
		void ExtractPointsFromImageFiles();
		void DoCaliberation();
		double ComputeError();
		void SaveCaliberationParameters();

};

void CameraCaliberation::ShowUndistorted(char* intrinsicfilename, char* distortionfilename)
{
	unsigned short int i, j;

	std::fstream intrinsicfile, distortionfile;

	char line[100];
	intrinsicfile.open(intrinsicfilename, std::fstream::in);
	if(!intrinsicfile.is_open())
	{
		std::cout<<" [ERR] Intrinsic Parameter file not found"<<std::endl;
		std::exit(0);
	}
	else
	{
		IntrinsicMatrix.create(3, 3, CV_64F);
		for(i=0; i<IntrinsicMatrix.rows; ++i)
		{
			for(j=0; j<IntrinsicMatrix.cols; ++j)
			{
				intrinsicfile>>line;
				IntrinsicMatrix.at<double>(i,j) = std::atof(line);
			}
		}
		intrinsicfile.close();
	}

	distortionfile.open(distortionfilename, std::fstream::in);
	if(!distortionfile.is_open())
	{
		std::cout<<" [ERR] Distortion Parameter file not found"<<std::endl;
		std::exit(0);
	}
	else
	{
		DistortionCoefficient.create(5, 1, CV_64F);
		for(i=0; i<DistortionCoefficient.rows; ++i)
		{
			for(j=0; j<DistortionCoefficient.cols; ++j)
			{
				distortionfile>>line;
				DistortionCoefficient.at<double>(i,j) = std::atof(line);
			}
		}
		distortionfile.close();
	}

	std::cout<<std::endl<<IntrinsicMatrix<<std::endl;
	std::cout<<std::endl<<DistortionCoefficient<<std::endl;


	cv::VideoCapture cam[1];
	cv::Mat img[1];
	cv::Mat imgU[1];
	cam[0].open(0);
	if(!cam[0].isOpened())
	{
		std::cout<<" [ERR] Camera not opened "<<std::endl;
		std::exit(0);
	}
	char key = 'r';
	for(;;)
	{
		cam[0]>>img[0];

		cv::undistort(img[0], imgU[0], IntrinsicMatrix, DistortionCoefficient);
		cv::imshow("OG", img[0]);
		cv::imshow("UD", imgU[0]);
		key = cv::waitKey(5);
		if(key == 'q')
			break;
	}
	img[0].release();
	imgU[0].release();
	cam[0].release();
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     	if(event == cv::EVENT_RBUTTONDOWN)
     	{
     		std::cout << " [INF] mouse is R clicked - position (" << x << ", " << y << ")" << std::endl;
     		((std::vector<cv::Point2f>*)userdata)->push_back(cv::Point2f(x,y));
     	}
}

bool status;


void CallBackFunc1(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == cv::EVENT_RBUTTONDOWN )
     {
    	 std::cout << "0 (" << x << ", " << y << ")" << std::endl;
    	 ((double*)userdata)[0] = x;
    	 ((double*)userdata)[1] = y;
     }
}

void CallBackFunc2(int event, int x, int y, int flags, void* userdata)
{
	if  ( event == cv::EVENT_RBUTTONDOWN )
    {
    	 std::cout << "1 (" << x << ", " << y << ")" << std::endl;
    	 ((double*)userdata)[0] = x;
    	 ((double*)userdata)[1] = y;
    }
}

void CameraCaliberation::StressTest(unsigned short int camid1, unsigned short int camid2,
										char* intrinsicfilename1, char* intrinsicfilename2,
										char* distortionfilename1,char* distortionfilename2,
										char* rotationfilename1, char* rotationfilename2,
										char* translationfilename1, char* translationfilename2)
{
    Pylon::PylonAutoInitTerm autoInitTerm;
          PylonInitialize();
          static const size_t c_maxCamerasToUse = 2;
         try
         {
              CTlFactory& tlFactory = CTlFactory::GetInstance();
              // Get all attached devices and exit application if no device is found.
              DeviceInfoList_t devices;
              if ( tlFactory.EnumerateDevices(devices) == 0 )
              {
                  throw RUNTIME_EXCEPTION( "No camera present.");
              }
              // Create an array of instant cameras for the found devices and avoid exceeding a maximum number of devices.
              CInstantCameraArray camera( min( devices.size(), c_maxCamerasToUse));
              // Create and attach all Pylon Devices.
              for ( size_t i = 0; i < camera.GetSize(); ++i)
              {
                  camera[ i ].Attach( tlFactory.CreateDevice( devices[ i ]));
                  // Print the model name of the camera.
                  cout << "Using device " << camera[ i ].GetDeviceInfo().GetModelName() << endl;
              }
//               CInstantCamera camera1( CTlFactory::GetInstance().CreateFirstDevice());
//             CInstantCamera camera2( CTlFactory::GetInstance().CreateFirstDevice());
//          cout << "Using device " << camera1.GetDeviceInfo().GetModelName() << endl;
//          cout << "Using device " << camera2.GetDeviceInfo().GetModelName() << endl;

          camera[0].MaxNumBuffer = 10;
          camera[1].MaxNumBuffer = 10;

      // create pylon image format converter and pylon image
      CImageFormatConverter formatConverter1;
      formatConverter1.OutputPixelFormat= PixelType_BGR8packed;
      CPylonImage pylonImage1;
      CImageFormatConverter formatConverter2;
      formatConverter2.OutputPixelFormat= PixelType_BGR8packed;
      CPylonImage pylonImage2;

          // Start the grabbing of c_countOfImagesToGrab images.
          // The camera device is parameterized with a default configuration which
          // sets up free-running continuous acquisition.
          camera[0].StartGrabbing(GrabStrategy_LatestImageOnly);
          camera[1].StartGrabbing(GrabStrategy_LatestImageOnly);
          cv::Mat image1,image2, gray;

//	cv::VideoCapture cam[2];
//	cv::Mat img[2];
//	cam[0].open(camid1);
//	cam[1].open(camid2);
//	status = false;
//	if(!cam[0].isOpened() || !cam[1].isOpened())
//	{
//		std::cout<<" [ERR] Camera not opened "<<std::endl;
//		std::exit(0);
//	}





        double k1[5], k2[5];

        double pt1[2], pt2[2];
        cv::Mat pt1cv, pt2cv;
        cv::Mat pt1Ucv, pt2Ucv;
        cv::Mat Pt3d(4,1,CV_64FC1);

        cv::Mat P(3,4,CV_64FC1);
        pt1cv.create(1, 1, CV_64FC2);
        pt2cv.create(1, 1, CV_64FC2);
        pt1Ucv.create(1, 1, CV_64FC2);
        pt2Ucv.create(1, 1, CV_64FC2);

        unsigned short int i, j;
        cv::Mat intrinsicMatrix1, intrinsicMatrix2;
        cv::Mat distortionCoefficient1, distortionCoefficient2;
        cv::Mat fundamentalMatrix1, fundamentalMatrix2;
        cv::Mat projectionMatrix1, projectionMatrix2;

        std::fstream file, file2;
        char line[100];
        file.open(intrinsicfilename1, std::fstream::in);
        if(!file.is_open())
        {
                std::cout<<" [ERR] Intrinsic1 Parameter file not found"<<std::endl;
                std::exit(0);
        }
        else
        {
                intrinsicMatrix1.create(3, 3, CV_64FC1);
                for(i=0; i<intrinsicMatrix1.rows; ++i)
                {
                        for(j=0; j<intrinsicMatrix1.cols; ++j)
                        {
                                file>>line;
                                intrinsicMatrix1.at<double>(i,j) = std::atof(line);
                        }
                }
                file.close();
        }

        file.open(intrinsicfilename2, std::fstream::in);
        if(!file.is_open())
        {
                std::cout<<" [ERR] Intrinsic2 Parameter file not found"<<std::endl;
                std::exit(0);
        }
        else
        {
                intrinsicMatrix2.create(3, 3, CV_64FC1);
                for(i=0; i<intrinsicMatrix2.rows; ++i)
                {
                        for(j=0; j<intrinsicMatrix2.cols; ++j)
                        {
                                file>>line;
                                intrinsicMatrix2.at<double>(i,j) = std::atof(line);
                        }
                }
                file.close();
        }

        file.open(distortionfilename1, std::fstream::in);
        if(!file.is_open())
        {
                std::cout<<" [ERR] Distortion1 Parameter file not found"<<std::endl;
                std::exit(0);
        }
        else
        {
                distortionCoefficient1.create(5, 1, CV_64FC1);
                for(i=0; i<distortionCoefficient1.rows; ++i)
                {
                        for(j=0; j<distortionCoefficient1.cols; ++j)
                        {
                                file>>line;
                                distortionCoefficient1.at<double>(i,j) = std::atof(line);
                        }
                        k1[i] = std::atof(line);
                }
                file.close();
        }

        file.open(distortionfilename2, std::fstream::in);
        if(!file.is_open())
        {
                std::cout<<" [ERR] Distortion2 Parameter file not found"<<std::endl;
                std::exit(0);
        }
        else
        {
                distortionCoefficient2.create(5, 1, CV_64FC1);
                for(i=0; i<distortionCoefficient2.rows; ++i)
                {
                        for(j=0; j<distortionCoefficient2.cols; ++j)
                        {
                                file>>line;
                                distortionCoefficient2.at<double>(i,j) = std::atof(line);
                        }
                        k2[i] = std::atof(line);
                }
                file.close();
        }

        file.open(rotationfilename1, std::fstream::in);
        file2.open(translationfilename1, std::fstream::in);
        if(!file.is_open() || !file2.is_open())
        {
                std::cout<<" [ERR] fun1 Parameter file not found"<<std::endl;
                std::exit(0);
        }
        else
        {
                fundamentalMatrix1.create(3, 4, CV_64FC1);
                for(i=0; i<fundamentalMatrix1.rows; ++i)
                {
                        for(j=0; j<fundamentalMatrix1.cols-1; ++j)
                        {
                                file>>line;
                                fundamentalMatrix1.at<double>(i,j) = std::atof(line);
                        }
                        file2>>line;
                        fundamentalMatrix1.at<double>(i,j) = std::atof(line);

                }
                file.close();
                file2.close();
        }

        file.open(rotationfilename2, std::fstream::in);
        file2.open(translationfilename2, std::fstream::in);
        if(!file.is_open() || !file2.is_open())
        {
                std::cout<<" [ERR] fun2 Parameter file not found"<<std::endl;
                std::exit(0);
        }
        else
        {
                fundamentalMatrix2.create(3, 4, CV_64FC1);
                for(i=0; i<fundamentalMatrix2.rows; ++i)
                {
                        for(j=0; j<fundamentalMatrix2.cols-1; ++j)
                        {
                                file>>line;
                                fundamentalMatrix2.at<double>(i,j) = std::atof(line);
                        }
                        file2>>line;
                        std::cout<<"    -   "<<line<<std::endl;
                        fundamentalMatrix2.at<double>(i,j) = std::atof(line);
                }
                file.close();
        }

        projectionMatrix1 = intrinsicMatrix1*fundamentalMatrix1;
        projectionMatrix2 = intrinsicMatrix2*fundamentalMatrix2;

        //P.at<double>(0,0)

        std::cout<<std::endl<<intrinsicMatrix1<<std::endl;
        std::cout<<std::endl<<intrinsicMatrix2<<std::endl;
//	std::cout<<std::endl<<distortionCoefficient1<<std::endl;
//	std::cout<<std::endl<<distortionCoefficient2<<std::endl;
//	std::cout<<std::endl<<fundamentalMatrix1<<std::endl;
//	std::cout<<std::endl<<fundamentalMatrix2<<std::endl;
//	std::cout<<std::endl<<projectionMatrix1<<std::endl;
//	std::cout<<std::endl<<projectionMatrix2<<std::endl;



        cv::namedWindow("0", 1);
        cv::namedWindow("1", 1);
        cv::setMouseCallback("0", CallBackFunc1, (void*)pt1);
        cv::setMouseCallback("1", CallBackFunc2, (void*)pt2);
        pt1[0] = pt2[0] = 0;

        cv::Mat A(6, 4, CV_64FC1);
        cv::Mat U(6, 4, CV_64FC1);
        cv::Mat W(4, 1, CV_64FC1);
        cv::Mat V(4, 4, CV_64FC1);

        double deltaY, deltaX, r, icdist;

        char key = 'r';
        for(;;)
        {
//		cam[0]>>img[0];
//		cam[1]>>img[1];

            // This smart pointer will receive the grab result data.
            CGrabResultPtr ptrGrabResult1;
            CGrabResultPtr ptrGrabResult2;

            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera[0].RetrieveResult(50, ptrGrabResult1, TimeoutHandling_ThrowException);
            camera[1].RetrieveResult(50, ptrGrabResult2, TimeoutHandling_ThrowException);

                const uint8_t *pImageBuffer1 = (uint8_t *) ptrGrabResult1->GetBuffer();
                const uint8_t *pImageBuffer2 = (uint8_t *) ptrGrabResult2->GetBuffer();

        // Convert the grabbed buffer to pylon imag
        formatConverter1.Convert(pylonImage1, ptrGrabResult1);
        formatConverter2.Convert(pylonImage2, ptrGrabResult2);

        // Create an OpenCV image out of pylon image
        image1= cv::Mat(ptrGrabResult1->GetHeight(), ptrGrabResult1->GetWidth(), CV_8UC3, (uint8_t *) pylonImage1.GetBuffer());
        image2= cv::Mat(ptrGrabResult2->GetHeight(), ptrGrabResult2->GetWidth(), CV_8UC3, (uint8_t *) pylonImage2.GetBuffer());


        if(pt1[0] != 0 && pt2[0] != 0)
                {
                        //std::cout<<pt1[0]<<" "<<pt1[1]<<std::endl;
                        //std::cout<<pt2[0]<<" "<<pt2[1]<<std::endl;
                        //pt2[0] = 490.035;
                        //pt2[1] = 144.5;
                        //pt1[0] = 514.716;
                        //pt1  [1] = 143.296;

                        pt1cv.at<cv::Vec2d>(0, 0)[0] = pt1[0];
                        pt1cv.at<cv::Vec2d>(0, 0)[1] = pt1[1];

                        pt2cv.at<cv::Vec2d>(0, 0)[0] = pt2[0];
                        pt2cv.at<cv::Vec2d>(0, 0)[1] = pt2[1];

//			pt1[0] = (pt1[0]-intrinsicMatrix1.at<double>(0, 2))/intrinsicMatrix1.at<double>(0, 0);
//			pt1[1] = (pt1[1]-intrinsicMatrix1.at<double>(1, 2))/intrinsicMatrix1.at<double>(1, 1);
//
//			r = pt1[0]*pt1[0] + pt1[1]*pt1[1];
//			icdist = 1/(1 + ((k1[4]*r + k1[1])*r + k1[0])*r);
//			deltaX = 2*k1[2]*pt1[0]*pt1[1] + k1[3]*(r + 2*pt1[0]*pt1[0]);
//			deltaY = k1[2]*(r + 2*pt1[1]*pt1[1]) + 2*k1[3]*pt1[0]*pt1[1];
//			pt1[0] = (pt1[0] - deltaX)*icdist;
//			pt1[1] = (pt1[1] - deltaY)*icdist;
//
//			pt2[0] = (pt2[0]-intrinsicMatrix2.at<double>(0, 2))/intrinsicMatrix2.at<double>(0, 0);
//			pt2[1] = (pt2[1]-intrinsicMatrix2.at<double>(1, 2))/intrinsicMatrix2.at<double>(1, 1);
//
//			r = pt2[0]*pt2[0] + pt2[1]*pt2[1];
//			icdist = 1/(1 + ((k2[4]*r + k2[1])*r + k2[0])*r);
//			deltaX = 2*k2[2]*pt2[0]*pt2[1] + k2[3]*(r + 2*pt2[0]*pt2[0]);
//			deltaY = k2[2]*(r + 2*pt2[1]*pt2[1]) + 2*k2[3]*pt2[0]*pt2[1];
//			pt2[0] = (pt2[0] - deltaX)*icdist;
//			pt2[1] = (pt2[1] - deltaY)*icdist;

                        //std::cout<<intrinsicMatrix1<<std::endl;
                        cv::undistortPoints(pt1cv, pt1Ucv, intrinsicMatrix1, distortionCoefficient1);
                        std::cout<<" "<<pt1Ucv.at<cv::Vec2d>(0,0)[0]<<" "<<pt1[0]<<std::endl;
                        std::cout<<" "<<pt1Ucv.at<cv::Vec2d>(0,0)[1]<<" "<<pt1[1]<<std::endl;

                        cv::undistortPoints(pt2cv, pt2Ucv, intrinsicMatrix2, distortionCoefficient2);
                        std::cout<<" "<<pt2Ucv.at<cv::Vec2d>(0,0)[0]<<" "<<pt2[0]<<std::endl;
                        std::cout<<" "<<pt2Ucv.at<cv::Vec2d>(0,0)[1]<<" "<<pt2[1]<<std::endl;

                        //pt1Ucv.at<cv::Vec2d>(0,0)[0] = pt1[0];
                        //pt1Ucv.at<cv::Vec2d>(0,0)[1] = pt1[1];
                        //pt2Ucv.at<cv::Vec2d>(0,0)[0] = pt2[0];
                        //pt2Ucv.at<cv::Vec2d>(0,0)[1] = pt2[1];

//			A.at<double>(0, 0) = pt1[0]*projectionMatrix1.at<double>(2,0) - projectionMatrix1.at<double>(0,0);
//			A.at<double>(1, 0) = pt1[1]*projectionMatrix1.at<double>(2,0) - projectionMatrix1.at<double>(1,0);
//			A.at<double>(2, 0) = pt1[0]*projectionMatrix1.at<double>(1,0) - pt1[1]*projectionMatrix1.at<double>(0,0);
//
//			A.at<double>(0, 1) = pt1[0]*projectionMatrix1.at<double>(2,1) - projectionMatrix1.at<double>(0,1);
//			A.at<double>(1, 1) = pt1[1]*projectionMatrix1.at<double>(2,1) - projectionMatrix1.at<double>(1,1);
//			A.at<double>(2, 0) = pt1[0]*projectionMatrix1.at<double>(1,1) - pt1[1]*projectionMatrix1.at<double>(0,1);
//
//			A.at<double>(0, 2) = pt1[0]*projectionMatrix1.at<double>(2,2) - projectionMatrix1.at<double>(0,2);
//			A.at<double>(1, 2) = pt1[1]*projectionMatrix1.at<double>(2,2) - projectionMatrix1.at<double>(1,2);
//			A.at<double>(2, 0) = pt1[0]*projectionMatrix1.at<double>(1,2) - pt1[1]*projectionMatrix1.at<double>(0,2);
//
//			A.at<double>(0, 3) = pt1[0]*projectionMatrix1.at<double>(2,3) - projectionMatrix1.at<double>(0,3);
//			A.at<double>(1, 3) = pt1[1]*projectionMatrix1.at<double>(2,3) - projectionMatrix1.at<double>(1,3);
//			A.at<double>(2, 0) = pt1[0]*projectionMatrix1.at<double>(1,2) - pt1[1]*projectionMatrix1.at<double>(0,3);
//
//			A.at<double>(3, 0) = pt2[0]*projectionMatrix2.at<double>(2,0) - projectionMatrix2.at<double>(0,0);
//			A.at<double>(4, 0) = pt2[1]*projectionMatrix2.at<double>(2,0) - projectionMatrix2.at<double>(1,0);
//			A.at<double>(5, 0) = pt1[0]*projectionMatrix1.at<double>(1,0) - pt2[1]*projectionMatrix1.at<double>(0,0);
//
//			A.at<double>(3, 1) = pt2[0]*projectionMatrix2.at<double>(2,1) - projectionMatrix2.at<double>(0,1);
//			A.at<double>(4, 1) = pt2[1]*projectionMatrix2.at<double>(2,1) - projectionMatrix2.at<double>(1,1);
//			A.at<double>(5, 0) = pt1[0]*projectionMatrix1.at<double>(1,1) - pt2[1]*projectionMatrix1.at<double>(0,1);
//
//			A.at<double>(3, 2) = pt2[0]*projectionMatrix2.at<double>(2,2) - projectionMatrix2.at<double>(0,2);
//			A.at<double>(4, 2) = pt2[1]*projectionMatrix2.at<double>(2,2) - projectionMatrix2.at<double>(1,2);
//			A.at<double>(5, 0) = pt1[0]*projectionMatrix1.at<double>(1,2) - pt2[1]*projectionMatrix1.at<double>(0,2);
//
//			A.at<double>(3, 3) = pt2[0]*projectionMatrix2.at<double>(2,3) - projectionMatrix2.at<double>(0,3);
//			A.at<double>(4, 3) = pt2[1]*projectionMatrix2.at<double>(2,3) - projectionMatrix2.at<double>(1,3);
//			A.at<double>(5, 0) = pt1[0]*projectionMatrix1.at<double>(1,3) - pt2[1]*projectionMatrix1.at<double>(0,3);
//
//			cv::SVD::compute(A, W, U, V);


                        cv::triangulatePoints(fundamentalMatrix1,fundamentalMatrix2,pt1Ucv,pt2Ucv,Pt3d);

                        //std::cout<<Pt3d<<std::endl;
                        std::cout<<Pt3d.at<double>(0,0)/Pt3d.at<double>(3,0)<<" "<<Pt3d.at<double>(1,0)/Pt3d.at<double>(3,0)<<" "<<Pt3d.at<double>(2,0)/Pt3d.at<double>(3,0)<<" "<<std::endl;
//			std::cout<<V.at<double>(3,0)/V.at<double>(3,3)<<" "<<V.at<double>(3,1)/V.at<double>(3,3)<<" "<<V.at<double>(3,2)/V.at<double>(3,3)<<" "<<std::endl;


                        pt1[0] = pt2[0] =  0;
                }

                //float ptx = pt1Ucv.at<cv::Vec2f>(0,0)[0]*intrinsicMatrix1.at<double>(0, 0) + intrinsicMatrix1.at<double>(0, 2);
                //float pty = pt1Ucv.at<cv::Vec2f>(0,0)[1]*intrinsicMatrix1.at<double>(1, 1) + intrinsicMatrix1.at<double>(1, 2);

                //std::cout<< ptx<<" "<<pty<<std::endl;
                //cv::circle( img[0], cv::Point2f(ptx, pty), 2, cv::Scalar( 255,0,0 ), 8, 0 );
                //cv::circle( img[1], cv::Point2f(pt2Ucv.at<cv::Vec2f>(0,0)[0], pt2Ucv.at<cv::Vec2f>(0,0)[1]), 2, cv::Scalar( 255,0,0 ), 8, 0 );

                cv::imshow("0", image1);
                cv::imshow("1", image2);

                key = cv::waitKey(10000);
                if(key == 'q')
                        break;
        }
//	ima.release();
//	img[1].release();
//	cam[0].release();
//	cam[1].release();
       }
          catch (GenICam::GenericException &e)
          {
              // Error handling.
              cerr << "An exception occurred." << endl
              << e.GetDescription() << endl;
      //            exitCode = 1;
          }

      //        return exitCode;
          PylonTerminate();


}








using namespace std;
using namespace cv;
void CameraCaliberation::GetTransformation(unsigned short int camid, char* intrinsicfilename, char* distortionfilename,
												char* rotationfilename, char* translationfilename,
												char* pixelfile, char* worldfile)
{
//
//vector<Point2f> points1, points2;
//
////First point's set
//points1.push_back(Point2f(450.0f,  400.0f));
//points1.push_back(Point2f(454.0f, 375.0f));
//points1.push_back(Point2f(410.0f, 392.0f));
//points1.push_back(Point2f(416.0f, 358.0f));
//
////Second point's set
//points2.push_back(Point2f(427.64938f, 158.77661f));
//points2.push_back(Point2f(428.79471f, 131.60953f));
//points2.push_back(Point2f(454.04532f, 134.97353f));
//points2.push_back(Point2f(452.23096f, 162.13156f));
//
////Real object point's set
//vector<Point3f> object;
//object.push_back(Point3f(0.0f,0.0f,0));
//object.push_back(Point3f(100.0f,0.0f,0));
//object.push_back(Point3f(0.0f,100.0f,0));
//object.push_back(Point3f(100.0f,100.0f,0));
//
////Camera matrix
//Mat cam_matrix = Mat(3,3,CV_32FC1,Scalar::all(0));
//cam_matrix.at<float>(0,0) = 520.0f;
//cam_matrix.at<float>(0,2) = 320.0f;
//cam_matrix.at<float>(1,1) = 520.0f;
//cam_matrix.at<float>(1,2) = 240.0f;
//cam_matrix.at<float>(2,2) = 1.0f;
//
////PnP
//Mat rvec1i,rvec2i,tvec1i,tvec2i;
//Mat rvec1p,rvec2p,tvec1p,tvec2p;
//solvePnP(Mat(object),Mat(points1),cam_matrix,Mat(),rvec1i,tvec1i,false,CV_ITERATIVE);
//solvePnP(Mat(object),Mat(points2),cam_matrix,Mat(),rvec2i,tvec2i,false,CV_ITERATIVE);
//solvePnP(Mat(object),Mat(points1),cam_matrix,Mat(),rvec1p,tvec1p,false,CV_P3P);
//solvePnP(Mat(object),Mat(points2),cam_matrix,Mat(),rvec2p,tvec2p,false,CV_P3P);
//
////Print result
//cout<<"Iterative: "<<endl;
//cout<<" rvec1 "<<endl<<" "<<rvec1i<<endl<<endl;
//cout<<" rvec2 "<<endl<<" "<<rvec2i<<endl<<endl;
//cout<<" tvec1 "<<endl<<" "<<tvec1i<<endl<<endl;
//cout<<" tvec1 "<<endl<<" "<<tvec2i<<endl<<endl;
//
//cout<<"P3P: "<<endl;
//cout<<" rvec1 "<<endl<<" "<<rvec1p<<endl<<endl;
//cout<<" rvec2 "<<endl<<" "<<rvec2p<<endl<<endl;
//cout<<" tvec1 "<<endl<<" "<<tvec1p<<endl<<endl;
//cout<<" tvec1 "<<endl<<" "<<tvec2p<<endl<<endl;
//int ii;
//
//Mat J;
//vector<Point2f> p;
//projectPoints(object, rvec1i, tvec1i, cam_matrix, Mat(), p, J);
//for(ii = 0; ii < p.size(); ++ii)
//	std::cout<<"Image point: ("<<points1[ii].x<<", "<<points1[ii].y<<")"<<" Projected to ("<<p[ii].x<<", "<<p[ii].y<<")"<< std::endl;
//J.release();
//Mat RR;
//cv::Rodrigues(rvec1i,RR , J); // R is 3x3
//cout<<RR<<endl;
//


	/*
	unsigned short int i, j, k;
	std::cout<<" [INF] Right click on points in same order as they are entered in file "<<std::endl;
	cv::VideoCapture camera;
	cv::Mat image;
	camera.open(camid);

	if(!camera.isOpened())
	{
		std::cout<<" [ERR] Opening camera"<<std::endl;
		std::exit(0);
	}
	std::vector<cv::Point2f> imagecoordinates;
	std::vector<cv::Point3f> worldcoordinates;
	cv::Point3f coordi;
	cv::Mat rotR, R, T, J;

	cv::namedWindow("camera");
	cv::setMouseCallback("camera", CallBackFunc, &imagecoordinates);
	std::cout<<" [INF] Statrt clicking "<<std::endl;
	char key;
	for(;;)
	{
		camera>>image;
		cv::imshow("camera", image);
		key = cv::waitKey(10);
		if(key == 'q')
			break;
	}
	char line[100];
	std::fstream outputfile;
	outputfile.open("data/camera/caliberation/input/3dcoodi.txt", std::fstream::in);
	if(!outputfile.is_open())
	{
		std::cout<<" [ERR] World coordinate files 3dcoodi.txt not found"<<std::endl;
		std::exit(0);
	}



	for(i=0; i<imagecoordinates.size() && i<16; ++i)
	{
		outputfile>>line;
		coordi.x = std::atof(line);
		outputfile>>line;
		coordi.y = std::atof(line);
		outputfile>>line;
		coordi.z = std::atof(line);
		worldcoordinates.push_back(coordi);
	}

	std::cout<<" [INF] Got file coordinates "<<std::endl;
	std::fstream intrinsicfile, distortionfile;
	intrinsicfile.open(intrinsicfilename, std::fstream::in);
	if(!intrinsicfile.is_open())
	{
		std::cout<<" [ERR] Intrinsic Parameter file not found"<<std::endl;
		std::exit(0);
	}
	else
	{
		IntrinsicMatrix.create(3, 3, CV_64F);
		for(i=0; i<IntrinsicMatrix.rows; ++i)
		{
			for(j=0; j<IntrinsicMatrix.cols; ++j)
			{
				intrinsicfile>>line;
				IntrinsicMatrix.at<double>(i,j) = std::atof(line);
			}
		}
		intrinsicfile.close();
	}

	distortionfile.open(distortionfilename, std::fstream::in);
	if(!distortionfile.is_open())
	{
		std::cout<<" [ERR] Distortion Parameter file not found"<<std::endl;
		std::exit(0);
	}
	else
	{
		DistortionCoefficient.create(8, 1, CV_64F);
		for(i=0; i<IntrinsicMatrix.rows; ++i)
		{
			for(j=0; j<IntrinsicMatrix.cols; ++j)
			{
				distortionfile>>line;
				DistortionCoefficient.at<double>(i,j) = std::atof(line);
			}
		}
		distortionfile.close();
	}

	//std::cout<<imagecoordinates<<std::endl;
	//std::cout<<worldcoordinates<<std::endl;
	//std::cout<<IntrinsicMatrix<<std::endl;
	//std::cout<<DistortionCoefficient<<std::endl;


	//cv::solvePnP(cv::Mat(worldcoordinates), cv::Mat(imagecoordinates), IntrinsicMatrix, DistortionCoefficient, RotationVector[0], TranslationVector[0], false, CV_EPNP);
	solvePnP(Mat(worldcoordinates),Mat(imagecoordinates), IntrinsicMatrix, DistortionCoefficient, rotR, T, false, CV_ITERATIVE);
	std::cout<<"hi"<<std::endl;
	J.release();
	R.release();
	cv::Rodrigues(rotR, R, J); // R is 3x3

	//R = R.t();  // rotation of inverse
	//T = tvec1i; // translation of inverse

	outputfile.close();
	outputfile.open("data/camera/caliberation/results/camerarotation.txt", std::fstream::out);
	for(j=0; j<R.rows; ++j)
	{
		for(k=0; k<R.cols; ++k)
			outputfile<<R.at< double >(j,k)<<" ";

		outputfile<<std::endl;
	}

	outputfile.close();

	outputfile.open("data/camera/caliberation/results/cameratranslation.txt", std::fstream::out);
	for(j=0; j<T.rows; ++j)
	{
		for(k=0; k<T.cols; ++k)
			outputfile<<T.at< double >(j,k);

		outputfile<<std::endl;
	}

	outputfile.close();

	*/

    Pylon::PylonAutoInitTerm autoInitTerm;
          PylonInitialize();
         try
         {
             CInstantCamera camera( CTlFactory::GetInstance().CreateFirstDevice());
          cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;

          camera.MaxNumBuffer = 10;

      // create pylon image format converter and pylon image
      CImageFormatConverter formatConverter;
      formatConverter.OutputPixelFormat= PixelType_BGR8packed;
      CPylonImage pylonImage;

          // Start the grabbing of c_countOfImagesToGrab images.
          // The camera device is parameterized with a default configuration which
          // sets up free-running continuous acquisition.
          camera.StartGrabbing(GrabStrategy_LatestImageOnly);
          cv::Mat image, gray;

          unsigned short int i, j, k;
          char line[100];
          std::cout<<" [INF]Opened camera "<<std::endl;

//	if(!camera.isOpened())
//	{
//		std::cout<<" [ERR] Opening camera"<<std::endl;
//		std::exit(0);
//	}

//	camera>>image;
//	camera>>image;
//	camera>>image;

    NumberofBoards = 0;
    //for(;;)
    //{



//		camera>>image;
//        cv::imshow("image", image);
//        char key = 'r';
//        key = cv::waitKey(10);

	char key = 'r';

	while(key != 'q')
	{
        // This smart pointer will receive the grab result data.
        CGrabResultPtr ptrGrabResult;

        // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
        camera.RetrieveResult(50, ptrGrabResult, TimeoutHandling_ThrowException);

            const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
    // Convert the grabbed buffer to pylon imag
    formatConverter.Convert(pylonImage, ptrGrabResult);

    // Create an OpenCV image out of pylon image
    image= cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());
        //camera>>image;
                key = cv::waitKey(50);
		imshow("test",image);
	}
	cv::Size ImageSize;
	ImageSize = cv::Size(image.cols, image.rows);
	cv::cvtColor(image, gray, CV_RGB2GRAY);
	char str[100];
    sprintf(str, "/home/mudit/pylon-calibration/build/results/final1/final%d.jpg",camid);
	cv::imwrite(str, image);

	std::vector< cv::Point2f> corners;
	std::vector< cv::Point2f> v_tImgPT;
	std::vector< cv::Point3f> v_tObjPT;
	cv::Point2f tImgPT;
	cv::Point3f tObjPT;

	corners.clear();
    bool sCorner = cv::findChessboardCorners(gray, cv::Size(7, 4), corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

	if(sCorner)
	{

		cv::cornerSubPix(gray, corners, cv::Size(10,10), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));
        cv::drawChessboardCorners(image, cv::Size(7, 4), cv::Mat(corners), sCorner);
		std::cout<<corners.size()<<std::endl;

        if(corners.size() == 7*4)
		{
			v_tImgPT.clear();
			v_tObjPT.clear();

			for(unsigned int j=0; j< corners.size(); ++j)
			{
                std::cout<<" AAAA GYI RE "<<std::endl;

				tImgPT.x = corners[j].x;
				tImgPT.y = corners[j].y;
				//instead of zero put xyz from table corner to chess board black side top corner
                tObjPT.y = j%7*100 ;//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
                tObjPT.x = j/7*100;//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
                tObjPT.z = 0 ; // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

				v_tImgPT.push_back(tImgPT);
				v_tObjPT.push_back(tObjPT);
			}

		}
	}

    // This smart pointer will receive the grab result data.
    CGrabResultPtr ptrGrabResult;

    // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
    camera.RetrieveResult(50, ptrGrabResult, TimeoutHandling_ThrowException);

        const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
// Convert the grabbed buffer to pylon imag
formatConverter.Convert(pylonImage, ptrGrabResult);

// Create an OpenCV image out of pylon image
image= cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

    cv::Scalar color = Scalar( 255,0,0 );
	for(unsigned short int i=0; i<v_tImgPT.size(); i++)
	{
		std::cout<<v_tImgPT[i]<<std::endl;
		cv::circle( image, v_tImgPT[i], 2, color, 8, 0 );
		cv::imshow("hi",image);
		cv::waitKey(50);
	}


//	for(unsigned short int i=0; i<v_tObjPT.size(); i++)
//	{
//		std::cout<<v_tObjPT[i]<<std::endl;
//		std::cout<<v_tImgPT[i]<<std::endl<<std::endl;
//	}


	std::fstream intrinsicfile, distortionfile;
	intrinsicfile.open(intrinsicfilename, std::fstream::in);
	if(!intrinsicfile.is_open())
	{
		std::cout<<" [ERR] Intrinsic Parameter file not found"<<std::endl;
		std::exit(0);
	}
	else
	{
		IntrinsicMatrix.create(3, 3, CV_64F);
		for(i=0; i<IntrinsicMatrix.rows; ++i)
		{
			for(j=0; j<IntrinsicMatrix.cols; ++j)
			{
				intrinsicfile>>line;
				IntrinsicMatrix.at<double>(i,j) = std::atof(line);
			}
		}
		intrinsicfile.close();
	}

	distortionfile.open(distortionfilename, std::fstream::in);
	if(!distortionfile.is_open())
	{
		std::cout<<" [ERR] Distortion Parameter file not found"<<std::endl;
		std::exit(0);
	}
	else
	{
		DistortionCoefficient.create(5, 1, CV_64F);
		for(i=0; i<DistortionCoefficient.rows; ++i)
		{
			for(j=0; j<DistortionCoefficient.cols; ++j)
			{
				distortionfile>>line;
				DistortionCoefficient.at<double>(i,j) = std::atof(line);
			}
		}
		distortionfile.close();
	}
	//DistortionCoefficient = cv::Mat::zeros(DistortionCoefficient.size(),DistortionCoefficient.type());

	cv::Mat rotR, R, T, J;

	cv::solvePnP(Mat(v_tObjPT),Mat(v_tImgPT), IntrinsicMatrix, DistortionCoefficient, rotR, T, false, CV_ITERATIVE);

	J.release();
	R.release();
	cv::Rodrigues(rotR, R, J); // R is 3x3

J.release();
	  std::vector<cv::Point2f> projectedPoints;

	 // DistortionCoefficient = cv::Mat::zeros(DistortionCoefficient.size(),DistortionCoefficient.type());


	  cv::projectPoints(Mat(v_tObjPT), rotR, T, IntrinsicMatrix, DistortionCoefficient, projectedPoints, J);
//
	  for(unsigned int i = 0; i < projectedPoints.size(); ++i)
	    {
	    std::cout << "Image point: " << v_tImgPT[i].x<<" "<<v_tImgPT[i].y << "  to " << projectedPoints[i].x<<" "<< projectedPoints[i].y<< std::endl;

			cv::circle( image, projectedPoints[i], 2, cv::Scalar(0,0,255), 8, 0 );
			cv::imshow("hi",image);
			cv::waitKey(50);
	    }

	//R = R.t();  // rotation of inverse
	//T = tvec1i; // translation of inverse
	std::fstream outputfile;
	outputfile.open(rotationfilename, std::fstream::out);
	for(j=0; j<R.rows; ++j)
	{
		for(k=0; k<R.cols; ++k)
			outputfile<<R.at< double >(j,k)<<" ";

		outputfile<<std::endl;
	}

	outputfile.close();

	outputfile.open(translationfilename, std::fstream::out);
	for(j=0; j<T.rows; ++j)
	{
		for(k=0; k<T.cols; ++k)
			outputfile<<T.at< double >(j,k);

		outputfile<<std::endl;
	}

	outputfile.close();

	outputfile.open(pixelfile, std::fstream::out);
	for(j=0; j<v_tImgPT.size(); ++j)
	{
			outputfile<<v_tImgPT[j].x<<" "<<v_tImgPT[j].y<<std::endl;
	}

	outputfile.close();

	outputfile.open(worldfile, std::fstream::out);
	for(j=0; j<v_tObjPT.size(); ++j)
	{
			outputfile<<v_tObjPT[j].x<<" "<<v_tObjPT[j].y<<" "<<v_tObjPT[j].z<<std::endl;
	}
	cv::waitKey(50000);
	outputfile.close();
    }
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred." << endl
        << e.GetDescription() << endl;
//            exitCode = 1;
    }

//        return exitCode;
    PylonTerminate();


}

void CameraCaliberation::GetImages(unsigned short int camid)
{
    Pylon::PylonAutoInitTerm autoInitTerm;
          PylonInitialize();
         try
         {
             CInstantCamera camera( CTlFactory::GetInstance().CreateFirstDevice());
          cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;

          camera.MaxNumBuffer = 10;

      // create pylon image format converter and pylon image
      CImageFormatConverter formatConverter;
      formatConverter.OutputPixelFormat= PixelType_BGR8packed;
      CPylonImage pylonImage;

          // Start the grabbing of c_countOfImagesToGrab images.
          // The camera device is parameterized with a default configuration which
          // sets up free-running continuous acquisition.
          camera.StartGrabbing(GrabStrategy_LatestImageOnly);
          cv::Mat image;


      char imagename[100];
      std::cout<<" [ASK] Input Filename "<<std::endl;
      std::cin>>FileName;
  //	camera.open(camid);
  //	if(!camera.isOpened())
  //	{
  //		std::cout<<" [ERR] Opening camera"<<std::endl;
  //		std::exit(0);
  //	}


      NumberofBoards = 0;
      for(;;)
      {
          // This smart pointer will receive the grab result data.
          CGrabResultPtr ptrGrabResult;

          // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
          camera.RetrieveResult(50, ptrGrabResult, TimeoutHandling_ThrowException);

              const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
      // Convert the grabbed buffer to pylon imag
      formatConverter.Convert(pylonImage, ptrGrabResult);

      // Create an OpenCV image out of pylon image
      image= cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());


  //		camera>>image;
          cv::imshow("image", image);
          char key = 'r';
          key = cv::waitKey(10);
          if(key == 's')
          {
              sprintf(imagename, "temp/cam726/%s%d.jpg", FileName, ++NumberofBoards);
              cv::imwrite(imagename, image);
          }
          else if(key == 'q')
              break;
      }
      std::cout<<" - "<<NumberofBoards<<std::endl;
      ImageSize = cv::Size(image.cols, image.rows);
  //	camera.release();
      image.release();
      }
          catch (GenICam::GenericException &e)
          {
              // Error handling.
              cerr << "An exception occurred." << endl
              << e.GetDescription() << endl;
  //            exitCode = 1;
          }

  //        return exitCode;
          PylonTerminate();
   }


bool CameraCaliberation::Caliberate()
{
	std::cout<<" [INF] Starting Caliberation"<<std::endl;

	BoardWidth = ImageSize.width;
	BoardHeight = ImageSize.height;

	std::cout<<" [ASK] How many cross points of Width direction? ";
	std::cin>>BoardWidth;
	std::cout<<" [ASK] How many cross points of Height direction? ";
	std::cin>>BoardHeight;
	std::cout<<" [ASK] How many board? ";
	std::cin>>NumberofBoards;
	std::cout<<" [ASK] What mm? ";
	std::cin>>SizeofSquare;
	std::cout<<" [ASK] Give File name. "<<std::endl;
	std::cin>>FileName;

	std::cout<<" [INF] W = "<<BoardWidth<<" H = "<<BoardHeight<<" N = "<<NumberofBoards<<" S = "<<SizeofSquare<<std::endl;

	ExtractPointsFromImageFiles();
	DoCaliberation();
	std::cout<<" [INF] Caliberation error "<<ComputeError()<<std::endl;
	SaveCaliberationParameters();
}

void CameraCaliberation::ExtractPointsFromImageFiles()
{
	char str[100];
	cv::Mat img;
	cv::Mat gray;

	std::vector< cv::Point2f> corners;
	std::vector< cv::Point2f> v_tImgPT;
	std::vector< cv::Point3f> v_tObjPT;

	cv::Point2f tImgPT;
	cv::Point3f tObjPT;


	for(unsigned short int i=0; i< NumberofBoards; ++i)
	{
        //sprintf(str,"/home/mudit/pylon-calibration/build/temp", i);
        sprintf(str, "temp/cam726/%s%d.jpg", FileName, i+1);

		img = cv::imread(str);
		if(img.empty())
		{
			std::cout<<" [ERR] Cannot open file "<<str<<std::endl;
			std::exit(0);
		}
		else
		{
			std::cout<<" [INF] Loaded image "<<str<<std::endl;
		}

		ImageSize = cv::Size(img.cols, img.rows);
		cv::cvtColor(img, gray, CV_RGB2GRAY);

		corners.clear();
		bool sCorner = cv::findChessboardCorners(gray, cv::Size(BoardWidth, BoardHeight), corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

		if(sCorner)
		{
			cv::cornerSubPix(gray, corners, cv::Size(10,10), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));
			cv::drawChessboardCorners(img, cv::Size(BoardWidth, BoardHeight), cv::Mat(corners), sCorner);

			if(corners.size() == BoardWidth*BoardHeight)
			{
				v_tImgPT.clear();
				v_tObjPT.clear();

				for(unsigned int j=0; j< corners.size(); ++j)
				{
					tImgPT.x = corners[j].x;
					tImgPT.y = corners[j].y;

					tObjPT.x = j%BoardWidth*SizeofSquare;
					tObjPT.y = j/BoardWidth*SizeofSquare;
					tObjPT.z = 0;

					v_tImgPT.push_back(tImgPT);
					v_tObjPT.push_back(tObjPT);
				}
			    ImagePoints.push_back(v_tImgPT);
			    ObjectPoints.push_back(v_tObjPT);
			}
		}


        sprintf(str, "temp/detect726/detected%d.jpg",i+1);
		cv::imwrite(str, img);
		cv::imshow("pattern", img);
		cv::waitKey(10);
	}
}

void CameraCaliberation::DoCaliberation()
{
	IntrinsicMatrix.create(3, 3, CV_64F);
	DistortionCoefficient.create(8, 1, CV_64F);

	std::cout<<ObjectPoints.size()<<" "<<ImagePoints.size()<<" "<<ImageSize<<std::endl;

	cv::calibrateCamera(ObjectPoints, ImagePoints, ImageSize, IntrinsicMatrix, DistortionCoefficient, RotationVector, TranslationVector);
	std::cout<<IntrinsicMatrix<<std::endl;
	std::cout<<DistortionCoefficient<<std::endl;
}

double CameraCaliberation::ComputeError()
{
	std::vector<cv::Point2f> imagePoints2;
	unsigned int i, totalPoints = 0;
	double totalErr = 0, err;
	std::vector< float > perViewErrors(ObjectPoints.size());

	for( i = 0; i < (int)ObjectPoints.size(); ++i )
	{
		cv::projectPoints( cv::Mat(ObjectPoints[i]), RotationVector[i], TranslationVector[i], IntrinsicMatrix,  DistortionCoefficient, imagePoints2);
		err = cv::norm(cv::Mat(ImagePoints[i]), cv::Mat(imagePoints2), CV_L2);

		int n = (int)ObjectPoints[i].size();
		perViewErrors[i] = (float) std::sqrt(err*err/n);
		totalErr        += err*err;
		totalPoints     += n;
	}

	return std::sqrt(totalErr/totalPoints);
}

void CameraCaliberation::SaveCaliberationParameters()
{
	std::ofstream outputfile;
    outputfile.open("results/cam726/intrinsic1.txt");

	unsigned short int i, j, k;

	for(i=0; i<IntrinsicMatrix.rows; ++i)
	{
		for(j=0; j<IntrinsicMatrix.cols; ++j)
			outputfile<<IntrinsicMatrix.at<double>(i,j)<<" ";
		outputfile<<std::endl;
	}
	outputfile.close();

    outputfile.open("results/cam726/distortion_coeffs1.txt");
	for(i=0; i<DistortionCoefficient.rows; ++i)
	{
		for(j=0; j<DistortionCoefficient.cols; ++j)
			outputfile<<DistortionCoefficient.at<double>(i,j)<<" ";
		outputfile<<std::endl;
	}
	outputfile.close();

    outputfile.open("results/cam726/rotation1.txt");
	for(i=0; i<RotationVector.size(); ++i)
	{
		for(j=0; j<RotationVector[i].rows; ++j)
		{
			for(k=0; k<RotationVector[i].cols; ++k)
				outputfile<<RotationVector[i].at< double >(j,k);

			outputfile<<std::endl;
		}
		outputfile<<std::endl;
	}
	outputfile.close();

    outputfile.open("results/cam726/translation1.txt");
	for(i=0; i<TranslationVector.size(); ++i)
	{
		for(j=0; j<TranslationVector[i].rows; ++j)
		{
			for(k=0; k<TranslationVector[i].cols; ++k)
				outputfile<<TranslationVector[i].at< double >(j,k);

			outputfile<<std::endl;
		}
		outputfile<<std::endl;
	}
	outputfile.close();

    outputfile.open("results/cam726/objectpt1.txt");
	for(i=0; i<ObjectPoints.size(); ++i)
	{
		for(j=0; j<ObjectPoints[i].size(); ++j)
			outputfile<<ObjectPoints[i][j].x<<" "<<ObjectPoints[i][j].y<<" "<<ObjectPoints[i][j].z;
		outputfile<<std::endl;
	}
	outputfile.close();

    outputfile.open("results/cam726/imagept1.txt");
	for(int i=0; i<ImagePoints.size(); ++i)
	{
		for(int j=0; j<ImagePoints[i].size(); ++j)
				outputfile<<ImagePoints[i][j].x<<" "<<ImagePoints[i][j].y;
		outputfile<<std::endl;
	}
	outputfile.close();

//	 fprintMatrix(intrinsic_Matrix, "intrinsic.txt");
//	 fprintMatrix(distortion_coeffs, "distortion_coeffs.txt");
//
//	 fprintfVectorMat(rvecs, "rotation.txt");
//	 fprintfVectorMat(tvecs, "translation.txt");
//
//	 fprintf3Point(objectPoints, "objectpt.txt");
//	 fprintf2Point(imagePoints, "imagept.txt");
//
//	 FILE* fp=fopen("ptSize.txt","w");
//	 fprintf(fp,"%d %d\n", board_w, board_h);
//	 fclose(fp);
}

#endif

