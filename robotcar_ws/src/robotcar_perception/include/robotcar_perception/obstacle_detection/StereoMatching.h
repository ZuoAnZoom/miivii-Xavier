/**
@file StereoMatcing.h
@date 2017/09/03
@author tkwoo(wtk1101@gmail.com).
@brief create disparity map
*/

#pragma once
 
#include <opencv2/opencv.hpp>
#include <iostream>

#if CV_MAJOR_VERSION==4
//#include "opencv2\ximgproc.hpp"
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#endif

#include <robotcar_perception/obstacle_detection/DefStruct.h>

using namespace std;
using namespace cv;


#if CV_MAJOR_VERSION==4
using namespace cv::ximgproc;
#endif

typedef enum error { NO_PROB, IMGSIZE_ERR, IMGSCALE_ERR } MATCHING_ERROR;//typedef enum自定义一个枚举类型
//其声明的变量MATCHING_ERROR只能设置为枚举中的值
/**
	@class CStereoMatching
	@brief disparity map
*/
class CStereoMatching{
private:
	//----------------input-------------------矫正后的左右影像
	Mat m_imgLeftInput;		///< rectified image
	Mat m_imgRightInput;	///< rectified image



	//----------------param-------------------定义立体匹配模块bm以及wls滤波器和双目相机参数
#if CV_MAJOR_VERSION==4  //如果opencv是第三版本
	Ptr<StereoBM> bm = StereoBM::create(0, 0); ///立体匹配模块　默认构造
	Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 0, 0);
//create(numDisparities,blockSize)numDisparities：即最大视差值与最小视差值之差, 窗口大小必须是16的整数倍，int 型 
//视差值在立体匹配过程中求得
//blockSize：匹配的块大小。它必须是> = 1的奇数。通常情况下，它应该在3--11的范围内。这里设置为大于11也可以，但必须为奇数。

	Ptr<DisparityWLSFilter> wls_filter;//基于加权最小二乘法的视差图过滤器

	Rect m_rectFilterROI;//rect对象用来存储一个矩形框的左上角坐标、宽度和高度
#endif

#if CV_MAJOR_VERSION == 2
	StereoBM bm;
#endif

	StereoCamParam_t m_objStereoParam;//构造参数结构体，存储视差数，窗口大小，基线长度和最远距离参数

	//-------------member image---------------

	//---------------function------------------双目参数设定函数，构造视差图函数，处理视差图函数
	void SetParamOCVStereo(); ///< Stereo parameter setting
	MATCHING_ERROR MakeDisparity();
	MATCHING_ERROR ImproveDisparity(Mat& imgDisp8);

public:

	//----------------output-------------------获取视差图
	Mat m_matDisp16;
	Mat m_imgDisp8;

	//---------------function------------------
	CStereoMatching(StereoCamParam_t& objStereoParam);
	CStereoMatching(Mat& imgLeftInput, Mat& imgRightInput, StereoCamParam_t& objStereoParam);
	
	//Set param　
	void SetParamOCVStereo(StereoCamParam_t& objStereoParam);

	//make disparity
	MATCHING_ERROR SetImage(Mat& imgLeft, Mat& imgRight);
	MATCHING_ERROR MakeDisparity(Mat& imgLeft, Mat& imgRight, bool flgUseWLSFilter=true);
	MATCHING_ERROR MakeDisparity(Mat& imgLeft, Mat& imgRight, Mat& matDisp16);
	MATCHING_ERROR ImproveDisparity_Naive(Mat& imgDisp8);
	MATCHING_ERROR ImproveDisparity_WLSFilter(Mat& imgDisp8); ///< OCV310 new disparity postprocess

};