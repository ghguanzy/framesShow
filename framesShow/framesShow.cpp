// framesShow.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "FreeImage.h"
#include "export.h"
#include "Deform2D.h"

using namespace std;
using namespace cv;

IplImage*  gif2ipl(const char* filename)
{
	FreeImage_Initialise();         //load the FreeImage function lib  
	FREE_IMAGE_FORMAT fif = FIF_GIF;
	FIBITMAP* fiBmp = FreeImage_Load(fif, filename, GIF_DEFAULT);
	FIMULTIBITMAP * pGIF = FreeImage_OpenMultiBitmap(fif, filename, 0, 1, 0, GIF_PLAYBACK);
	//  FIBITMAPINFO fiBmpInfo = getfiBmpInfo(fiBmp);  
	int gifImgCnt = FreeImage_GetPageCount(pGIF);
	FIBITMAP * pFrame;
	int width, height;
	width = FreeImage_GetWidth(fiBmp);
	height = FreeImage_GetHeight(fiBmp);
	IplImage * iplImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	iplImg->origin = 1;//should set to 1-top-left structure(Windows bitmap style)  
	RGBQUAD* ptrPalette = new RGBQUAD; // = FreeImage_GetPalette(fiBmp);  
	BYTE intens;
	BYTE* pIntensity = &intens;
	cvNamedWindow("gif", 0);
	printf("gifImgCnt %d \n", gifImgCnt);
	for (int curFrame = 0; curFrame<gifImgCnt; curFrame++)
	{
		pFrame = FreeImage_LockPage(pGIF, curFrame);
		//ptrPalette = FreeImage_GetPalette(pFrame);  
		char * ptrImgDataPerLine;
		for (int i = 0; i<height; i++)
		{
			ptrImgDataPerLine = iplImg->imageData + i * iplImg->widthStep;
			for (int j = 0; j<width; j++)
			{
				//get the pixel index   
				//FreeImage_GetPixelIndex(pFrame,j,i,pIntensity);    
				FreeImage_GetPixelColor(pFrame, j, i, ptrPalette);
				ptrImgDataPerLine[3 * j] = ptrPalette->rgbBlue;
				ptrImgDataPerLine[3 * j + 1] = ptrPalette->rgbGreen;
				ptrImgDataPerLine[3 * j + 2] = ptrPalette->rgbRed;
				//ptrImgDataPerLine[3*j] = ptrPalette[intens].rgbBlue;  
				//ptrImgDataPerLine[3*j+1] = ptrPalette[intens].rgbGreen;  
				//ptrImgDataPerLine[3*j+2] = ptrPalette[intens].rgbRed;  
			}
		}

		printf("convert curFrame end %d \n", curFrame);
		cvShowImage("gif", iplImg);
		cvWaitKey(30);
		FreeImage_UnlockPage(pGIF, pFrame, 1);
	}
	FreeImage_Unload(fiBmp);
	FreeImage_DeInitialise();
	return iplImg;
}

int cvAdd4cMat_q(cv::Mat &dst, cv::Mat &scr, double scale)
{
	if (dst.channels() != 3 || scr.channels() != 4)
	{
		return true;
	}
	if (scale < 0.01)
		return false;
	std::vector<cv::Mat>scr_channels;
	std::vector<cv::Mat>dstt_channels;
	split(scr, scr_channels);
	split(dst, dstt_channels);
	CV_Assert(scr_channels.size() == 4 && dstt_channels.size() == 3);

	if (scale < 1)
	{
		scr_channels[3] *= scale;
		scale = 1;
	}
	for (int i = 0; i < 3; i++)
	{
		dstt_channels[i] = dstt_channels[i].mul(255.0 / scale - scr_channels[3], scale / 255.0);
		dstt_channels[i] += scr_channels[i].mul(scr_channels[3], scale / 255.0);
	}
	merge(dstt_channels, dst);
	return true;
}

const int eyeNum = 14;
const int mouseNum = 22;

void test_copy()
{
	Mat bg = imread("image.jpg");
	Mat eye, mouse, dst;
	String filename;
	int sCenterX, sCenterY, dCenterX, dCenterY;
	int index;

	dCenterX = bg.cols / 2;
	dCenterY = bg.rows / 2;

	int maxnum = max(eyeNum, mouseNum);

	for (int i = 0; i < mouseNum; i++)
	{
		dst = bg.clone();

		filename = "./eye/" + to_string(i % eyeNum + 1) + ".png";
		eye = imread(filename.c_str(), -1);
		resize(eye, eye, eye.size()*3/4);
		sCenterX = eye.cols / 2;
		sCenterY = eye.rows / 2;

		Mat temp(dst, cvRect(dCenterX - sCenterX, 0, eye.cols, eye.rows));
		cvAdd4cMat_q(temp, eye, 1);

		filename = "./mouse/" + to_string(i + 1) + ".png";
		mouse = imread(filename.c_str(), -1);
		resize(mouse, mouse, mouse.size() / 2);
		sCenterX = mouse.cols / 2;
		sCenterY = mouse.rows / 2;

		Mat temp2(dst, cvRect(dCenterX - sCenterX, dCenterY*3/2 - sCenterY, mouse.cols, mouse.rows));
		cvAdd4cMat_q(temp2, mouse, 1);

		imshow("img", dst);
		cvWaitKey(50);
	}
}

void test_contour()
{
	IplImage* src = cvLoadImage("./eye/1.png", 0);
	CvMemStorage *storage = cvCreateMemStorage(0);
	CvSeq * contour = 0;
	cvThreshold(src, src, 50, 255, CV_THRESH_BINARY);
	cvFindContours(src, storage, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	IplImage* dst = cvCreateImage(cvGetSize(src), 8, 3);

	for (; contour != 0; contour = contour->h_next)
	{
		cvDrawContours(dst, contour, CvScalar(255,255,255), CvScalar(255, 255, 255), -1, CV_FILLED, 8);
	}

	cvShowImage("img", dst);
}

void test_add()
{
	Mat img = imread("bottle.jpg");
	int size = img.rows*img.step;

	addSenseOrgan((const char *)img.data, img.cols, img.rows);
}

int main(int argc, char ** argv)
{
	//Mat img = imread("wdj0.jpg");
	//imshow("window", img);
	//gif2ipl("picaco.gif");
	//showFrames("image.jpg");
	//test_add();
	//waitKey(0);
	deform2D(argc, argv);
    return 0;
}

