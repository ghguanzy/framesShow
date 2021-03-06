// export.cpp: 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include "export.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "FreeImage.h"

using namespace std;
using namespace cv;

const int FRAMES = 14;

IplImage * parseImgData(const char* data, int width, int height)
{
	IplImage * iplImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	char * ptrImgDataPerLine;
	for (int i = 0; i<height; i++)
	{
		ptrImgDataPerLine = iplImg->imageData + i * iplImg->widthStep;
		for (int j = 0; j<width; j++)
		{
			ptrImgDataPerLine[3 * j] = data[i * iplImg->widthStep + 3 * j];
			ptrImgDataPerLine[3 * j + 1] = data[i * iplImg->widthStep + 3 * j + 1];
			ptrImgDataPerLine[3 * j + 2] = data[i * iplImg->widthStep + 3 * j + 2];
		}
	}
	return iplImg;
}

Mat parseMatData(const char* data, int width, int height)
{
	int step = width * 3;
	Mat img(height, width, CV_8UC3, (void *)data, step);

	return img;
}

void mergeImg(IplImage *src, IplImage *dst)
{
	int sCenterX = src->width / 2;
	int sCenterY = src->height / 2;
	int dCenterX = dst->width / 2;
	int dCenterY = dst->height / 2;
	
	CvRect rect;
	rect.x = dCenterX - sCenterX;
	rect.y = dCenterY - sCenterY;
	rect.height = src->height;
	rect.width = src->width;

	cvSetImageROI(dst, rect);
	src->origin = 1;
	cvCopy(src, dst);
	cvResetImageROI(dst);
}

void proFrames(const char* backImgFile)
{
	String filename;
	IplImage * backImg = cvLoadImage(backImgFile);
	IplImage * srcImg;

	for (int i = 0; i < 14; i++)
	{
		filename = "./eye/" + to_string(i + 1) + ".png";
		srcImg = cvLoadImage(filename.c_str());
		//cvScale(srcImg, srcImg, 0.5);
		mergeImg(srcImg, backImg);
		cvShowImage("gif", backImg);
		cvWaitKey(50);
	}

}

IplImage*  gif2ipl(const char* filename, int w, int h, const char *backImgFile)
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
	double scale = 1, scaleW = 1, scaleH = 1;
	cvNamedWindow("gif", 1);
	//cvResizeWindow("gif", width, height);

	IplImage * backImg = cvLoadImage(backImgFile);

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
		if (iplImg->width > w)
			scaleW = double(w) / (double)iplImg->width;
		if (iplImg->height > h)
			scaleH = double(h) / (double)iplImg->height;
		scale = min(scaleW, scaleH);
		//IplImage * dstImg = cvCreateImage(cvSize(width*scale, height*scale), IPL_DEPTH_8U, 3);
		//cvScale(iplImg, dstImg, scale);
		mergeImg(iplImg, backImg);
		cvShowImage("gif", backImg);
		cvWaitKey(30);
		FreeImage_UnlockPage(pGIF, pFrame, 1);
	}
	FreeImage_Unload(fiBmp);
	FreeImage_DeInitialise();
	return iplImg;
}

void showFrames(const char *filename)
{
	Mat img = imread(filename);
	int w = img.cols;
	int h = img.rows;

	//gif2ipl("picaco.gif", w, h, filename);
	proFrames(filename);
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

void addSenseOrgan(const char *data, int width, int height)
{
	const int eyeNum = 14, mouseNum = 22, frames = 100;

	//String fileName;
	//IplImage *oriIplimg = parseImgData(data, width, height);
	//IplImage * srcImg, *cloneImg;

	//for (int i = 0; i < FRAMES; i++)
	//{
	//	cloneImg = cvCloneImage(oriIplimg);
	//	fileName = "./eye/" + to_string(i + 1) + ".png";
	//	srcImg = cvLoadImage(fileName.c_str());

	//}
	//
	//Mat bg = imread("image.jpg");
	Mat bg = parseMatData(data, width, height);
	Mat eye, mouse, dst;
	String filename;
	int sCenterX, sCenterY, dCenterX, dCenterY;
	int index;

	dCenterX = bg.cols / 2;
	dCenterY = bg.rows / 2;

	int maxnum = max(eyeNum, mouseNum);

	for (int i = 0; i < frames; i++)
	{
		dst = bg.clone();

		filename = "./eye/" + to_string(i % eyeNum + 1) + ".png";
		eye = imread(filename.c_str(), -1);
		resize(eye, eye, eye.size() * 3 / 4);
		sCenterX = eye.cols / 2;
		sCenterY = eye.rows / 2;

		Mat temp(dst, cvRect(dCenterX - sCenterX, dCenterY - sCenterY, eye.cols, eye.rows));
		cvAdd4cMat_q(temp, eye, 1);

		filename = "./mouse/" + to_string(i % mouseNum + 1) + ".png";
		mouse = imread(filename.c_str(), -1);
		resize(mouse, mouse, mouse.size()* 3 / 4);
		sCenterX = mouse.cols / 2;
		sCenterY = mouse.rows / 2;

		Mat temp2(dst, cvRect(dCenterX - sCenterX, dCenterY * 3 / 2 - sCenterY, mouse.cols, mouse.rows));
		cvAdd4cMat_q(temp2, mouse, 1);

		imshow("img", dst);
		cvWaitKey(50);
	}

}