#include <opencv2/highgui/highgui_c.h>
#include <robotcar_perception/obstacle_detection/StereoVisionForADAS.h>

StereoCamParam_t CStereoVisionForADAS::InitStereoParam(int nDatasetName)
{
	int c;
	StereoCamParam_t objStereoCamParam;

	if (nDatasetName == Daimler)
	{
		objStereoCamParam.m_dBaseLine = 0.25;
		objStereoCamParam.m_dMaxDist = 70;
		objStereoCamParam.m_nNumberOfDisp = 48;
		objStereoCamParam.m_nWindowSize = 9;
		objStereoCamParam.objCamParam.m_dCameraHeight = 1.17;
		objStereoCamParam.objCamParam.m_dFocalLength = 1200.;
		objStereoCamParam.objCamParam.m_dPitchDeg = -1.89;
		objStereoCamParam.objCamParam.m_sizeSrc = cv::Size(640, 480);
	}
	else if (nDatasetName == KITTI)
	{
		objStereoCamParam.m_dBaseLine = 0.54;
		objStereoCamParam.m_dMaxDist = 60.;
		objStereoCamParam.m_nNumberOfDisp = 80;
		objStereoCamParam.m_nWindowSize = 11;
		objStereoCamParam.objCamParam.m_dCameraHeight = 1.65;			//TBD..
		objStereoCamParam.objCamParam.m_dFocalLength = 722;
		objStereoCamParam.objCamParam.m_dPitchDeg = -1.;	///< ������ ���׷��� ����� ������ �ݴ�� ��. ���׷��� ����
		objStereoCamParam.objCamParam.m_dYawDeg = -1.2;
		objStereoCamParam.objCamParam.m_dOx = 610;
		objStereoCamParam.objCamParam.m_dOy = 173;
		objStereoCamParam.objCamParam.m_sizeSrc = cv::Size(1241, 376);

	}
	else if (nDatasetName == benben)
	{
		objStereoCamParam.m_dBaseLine = 0.1;
		objStereoCamParam.m_dMaxDist = 30.;
		objStereoCamParam.m_nNumberOfDisp = 32;
		objStereoCamParam.m_nWindowSize = 9;
		objStereoCamParam.objCamParam.m_dCameraHeight = 1.165;			//TBD..
		objStereoCamParam.objCamParam.m_dFocalLength = 1312.961;
		objStereoCamParam.objCamParam.m_dPitchDeg = 0;	///< ������ ���׷��� ����� ������ �ݴ�� ��. ���׷��� ����
		objStereoCamParam.objCamParam.m_dYawDeg =0;
		objStereoCamParam.objCamParam.m_dOx = 610;
		objStereoCamParam.objCamParam.m_dOy = 173;
		objStereoCamParam.objCamParam.m_sizeSrc = cv::Size(1280, 720);
		//c = objStereoCamParam.objCamParam.m_sizeSrc.width ;
	}

	else if (nDatasetName == benben2)
	{
		objStereoCamParam.m_dBaseLine = 0.4086;
		objStereoCamParam.m_dMaxDist = 100.;
		objStereoCamParam.m_nNumberOfDisp = 64;
		objStereoCamParam.m_nWindowSize = 13;
		objStereoCamParam.objCamParam.m_dCameraHeight = 1.165;			//TBD..
		objStereoCamParam.objCamParam.m_dFocalLength = 1356.582;
		objStereoCamParam.objCamParam.m_dPitchDeg = -1;	///< ������ ���׷��� ����� ������ �ݴ�� ��. ���׷��� ����
		objStereoCamParam.objCamParam.m_dYawDeg =-1.2;
		objStereoCamParam.objCamParam.m_dOx = 583.011;
		objStereoCamParam.objCamParam.m_dOy = 263.072;
		objStereoCamParam.objCamParam.m_sizeSrc = cv::Size(1280/2, 720/2);
		//c = objStereoCamParam.objCamParam.m_sizeSrc.width ;
	}

	else if (nDatasetName == benben3)
	{
		objStereoCamParam.m_dBaseLine = 0.4021;
		objStereoCamParam.m_dMaxDist = 70.;
		objStereoCamParam.m_nNumberOfDisp = 96;
		objStereoCamParam.m_nWindowSize = 13;
		objStereoCamParam.objCamParam.m_dCameraHeight = 1.165;			//TBD..
		objStereoCamParam.objCamParam.m_dFocalLength = 858.651454;
		objStereoCamParam.objCamParam.m_dPitchDeg = -1;	///< ������ ���׷��� ����� ������ �ݴ�� ��. ���׷��� ����
		objStereoCamParam.objCamParam.m_dYawDeg = 0;
		objStereoCamParam.objCamParam.m_dOx = 577.745;
		objStereoCamParam.objCamParam.m_dOy = 277.903;
		objStereoCamParam.objCamParam.m_sizeSrc = cv::Size(1280 / 2, 720 / 2);
		//c = objStereoCamParam.objCamParam.m_sizeSrc.width ;
	}

	else if (nDatasetName == CityScape)
	{
		//printf("CityScape DB is not availabe yet. sorry");
		objStereoCamParam.m_dBaseLine = 0.21;
		objStereoCamParam.m_dMaxDist = 70.;
		objStereoCamParam.m_nNumberOfDisp = 80;
		objStereoCamParam.m_nWindowSize = 11;
		objStereoCamParam.objCamParam.m_dCameraHeight = 1.22;			
		objStereoCamParam.objCamParam.m_dFocalLength = 2263.5/2;
		objStereoCamParam.objCamParam.m_dPitchDeg = -2.18;
		objStereoCamParam.objCamParam.m_dYawDeg = -1.2;
		objStereoCamParam.objCamParam.m_dOx = 610;
		objStereoCamParam.objCamParam.m_dOy = 173;
		//objStereoCamParam.objCamParam.m_sizeSrc = cv::Size(2048 * fScale, 1024 * fScale);
		objStereoCamParam.objCamParam.m_sizeSrc = cv::Size(1024, 512);
	}
	else
		printf("This DB is not availabe. sorry");
	PitchDegToVanishingLine(objStereoCamParam);
	//cout << objStereoCamParam.objCamParam.m_sizeSrc << endl;
	
	return objStereoCamParam;
}
int CStereoVisionForADAS::PitchDegToVanishingLine(StereoCamParam_t& objStereoParam)
{
	objStereoParam.objCamParam.m_nVanishingY = (int)(objStereoParam.objCamParam.m_dFocalLength*tan(objStereoParam.objCamParam.m_dPitchDeg*PI / 180)) + objStereoParam.objCamParam.m_sizeSrc.height / 2;
	return 0;
}
void CStereoVisionForADAS::MakePseudoColorLUT()
{
	int b = 125;
	int g = 0;
	int r = 0;

	int idx = 0;

	int mode = 0;
	// mode = 0 : increasing 'b'
	// mode = 1 : increasing 'g'
	// mode = 2 : decreasing 'b'
	// mode = 3 : increasing 'r'
	// mode = 4 : decreasing 'g'
	// mode = 5 : decreasing 'r'

	while (1)
	{
		m_pseudoColorLUT[idx][0] = b;
		m_pseudoColorLUT[idx][1] = g;
		m_pseudoColorLUT[idx][2] = r;

		if (b == 255 && g == 0 && r == 0)
			mode = 1;
		else if (b == 255 && g == 255 && r == 0)
			mode = 2;
		else if (b == 0 && g == 255 && r == 0)
			mode = 3;
		else if (b == 0 && g == 255 && r == 255)
			mode = 4;
		else if (b == 0 && g == 0 && r == 255)
			mode = 5;

		switch (mode)
		{
		case 0: b += 5; break;
		case 1: g += 5; break;
		case 2: b -= 5; break;
		case 3: r += 5; break;
		case 4: g -= 5; break;
		case 5: r -= 5; break;
		default: break;
		}

		if (idx == 255)
			break;

		idx++;
	}
}
void CStereoVisionForADAS::cvtPseudoColorImage(Mat& srcGray, Mat& dstColor)//输入 :灰度图, 输出:彩色图
{
	for (int i = 0; i<srcGray.rows; i++)
	{
		for (int j = 0; j<srcGray.cols; j++)
		{
			unsigned char val = srcGray.data[i*srcGray.cols + j];
			if (val == 0) continue;
			dstColor.data[(i*srcGray.cols + j) * 3 + 0] = m_pseudoColorLUT[val][0];
			dstColor.data[(i*srcGray.cols + j) * 3 + 1] = m_pseudoColorLUT[val][1];
			dstColor.data[(i*srcGray.cols + j) * 3 + 2] = m_pseudoColorLUT[val][2];
		}
	}
	
}

CStereoVisionForADAS::CStereoVisionForADAS(StereoCamParam_t& objStereoParam)
	:m_objStereoMatching(objStereoParam), m_objStixelEstimation(objStereoParam), m_objStixelSegmentation(objStereoParam)
{
	//cout << objStereoParam.objCamParam.m_sizeSrc << endl;
	m_objStereoParam = objStereoParam;
	MakePseudoColorLUT();
	m_imgColorDisp = Mat(objStereoParam.objCamParam.m_sizeSrc, CV_8UC3);
	m_imgColorDisp = Scalar(0);
	m_imgGround = Mat(objStereoParam.objCamParam.m_sizeSrc, CV_8UC1);//CV_8UC1代表颜色通道只有一个，这里是建立m_sizeSrc大小的块来存储ground
	m_imgStixelGray = Mat(objStereoParam.objCamParam.m_sizeSrc, CV_8UC1);
	m_imgStixelGray = Scalar(0);
}
int CStereoVisionForADAS::Objectness(Mat& imgLeft, Mat& imgRight)
{
	m_vecobjBB.clear(); //vector of object boundingbox
	m_vecobjStixelInROI.clear();//< stixel output in 3D ROI
	m_vecobjStixels.clear();//< stixel output

	if (imgLeft.channels() == 3){
		cvtColor(imgLeft, m_imgLeftInput, COLOR_BGR2GRAY);
		cvtColor(imgRight, m_imgRightInput, COLOR_BGR2GRAY);
		//return NO_PROB;
	}
	else
	{
		m_imgLeftInput = imgLeft;
		m_imgRightInput = imgRight;
	}
	
#if CV_MAJOR_VERSION==2
	m_objStereoMatching.MakeDisparity(m_imgLeftInput, m_imgRightInput,false);
#else
	m_objStereoMatching.MakeDisparity(m_imgLeftInput, m_imgRightInput,false); // wls filter = false
#endif
	m_matDisp16 = m_objStereoMatching.m_matDisp16;
	m_imgDisp8 = m_objStereoMatching.m_imgDisp8;
	
	//imshow("disparity", m_imgDisp8);

	m_imgGround = Scalar(0);
	m_objStixelEstimation.EstimateStixels(m_matDisp16, m_imgDisp8);//, false);
	m_imgGround = m_objStixelEstimation.m_imgGround;
	m_vecobjStixelInROI = m_objStixelEstimation.m_vecobjStixelInROI;
	m_vecobjStixels = m_objStixelEstimation.m_vecobjStixels;

	m_objStixelSegmentation.SegmentStixel(m_vecobjStixels);
	m_vecobjBB = m_objStixelSegmentation.m_vecobjBB;

	return 0;
}
int CStereoVisionForADAS::Objectness(Mat& imgLeft, Mat& imgRight, Mat& imgDisp8) //不进入
{
	m_vecobjBB.clear();  //segmentation output
	m_vecobjStixelInROI.clear(); //stixel output in 3D ROI
	m_vecobjStixels.clear(); //stixel output

	if (imgLeft.channels() == 3){
		cvtColor(imgLeft, m_imgLeftInput, COLOR_BGR2GRAY);
		cvtColor(imgRight, m_imgRightInput, COLOR_BGR2GRAY);
	}  //从一个颜色空间转换到另一个颜色空间的转换
	else
	{
		m_imgLeftInput = imgLeft;
		m_imgRightInput = imgRight;
	}
	// m_matDisp16 = m_objStereoMatching.m_matDisp16;
	m_imgDisp8 = imgDisp8;
	
	m_imgGround = Scalar(0); //scalar是将图像设置成单一灰度和颜色
	m_objStixelEstimation.EstimateStixels_only8bitDisp(m_imgDisp8);//, false);
	m_imgGround = m_objStixelEstimation.m_imgGround;
	m_vecobjStixelInROI = m_objStixelEstimation.m_vecobjStixelInROI;
	m_vecobjStixels = m_objStixelEstimation.m_vecobjStixels;

	
// #ifdef _DEBUG
// 	m_objStixelSegmentation.SetDebugImg(m_imgLeftInput);
// #endif
	m_objStixelSegmentation.SegmentStixel(m_vecobjStixels);
	m_vecobjBB = m_objStixelSegmentation.m_vecobjBB;
// #ifdef _DEBUG
// 	cout << "stixel size : " << m_vecobjStixelInROI.size() << endl;
// 	cout << "objectness size : " << m_vecobjBB.size() << endl;
// #endif

	return 0;
}
int CStereoVisionForADAS::RectToDisp(Rect& rectBox, Mat& matRect)//不进入
{
	if (m_matDisp16.empty()) { cout << "There is no disparity image" << endl; return -1; }
	if (rectBox.x < 0 || rectBox.x + rectBox.width > m_matDisp16.cols || rectBox.y < 0 || rectBox.y + rectBox.height > m_matDisp16.rows)
	{
		cout << "out of range" << endl;
		return -1;
	}
	Mat imgDisp8;
	m_matDisp16.convertTo(imgDisp8, CV_8U, 1/16.);
	matRect = imgDisp8(rectBox).clone();
	return 0;
}
int CStereoVisionForADAS::Disp16ToDepth(const uchar nDisparity, float& fDistanceMeter)//不进入
{
	fDistanceMeter = (float)m_objStereoParam.m_dBaseLine*(float)m_objStereoParam.objCamParam.m_dFocalLength / (float)nDisparity;
	return 0;
}

void CStereoVisionForADAS::Display() //不进入
{
	TopViewStixel(m_vecobjStixelInROI);
	//imshow("topview", m_imgTopView);
}
void CStereoVisionForADAS::Display(Mat& imgDisplay) //不进入
{
#ifndef _DEBUG
	Display();
#endif
	for (unsigned int i = 0; i < m_vecobjBB.size(); i++){
		rectangle(imgDisplay, m_vecobjBB[i].rectBB, Scalar::all(255), 2, 8);
	}
}
void CStereoVisionForADAS::Display(Mat& imgDisplay, Mat& imgStixelResult) 
{
	//if (imgDisplay.channels() == 3) cvtColor(imgDisplay, imgDisplay, COLOR_BGR2GRAY);
	if (imgStixelResult.channels() == 3) cvtColor(imgStixelResult, imgStixelResult, COLOR_BGR2GRAY);

	float fBrightness = 30;

	//Display(imgDisplay);
	for (unsigned int i = 0; i < m_vecobjBB.size(); i++){
		rectangle(imgDisplay, m_vecobjBB[i].rectBB, Scalar::all(255 - m_vecobjBB[i].dZ / fBrightness * 255), 2, 8);
	}

	/*Mat imgStixelTemp = m_imgColorDisp.clone();
	DrawStixel(imgStixelTemp, m_vecobjStixelInROI);
	imshow("gg", imgStixelTemp);*/
	m_imgStixelGray = Scalar(0);

	DrawGround(m_imgColorDisp, m_imgGround);
	line(m_imgColorDisp, Point(0, m_objStereoParam.objCamParam.m_nVanishingY), Point(m_imgColorDisp.cols, m_objStereoParam.objCamParam.m_nVanishingY), Scalar(255, 255, 255), 3);
	DrawStixel(m_imgColorDisp, m_vecobjStixels);
	DrawLane(m_imgColorDisp, m_objStereoParam);
	DrawStixel(m_imgColorDisp, m_vecobjStixelInROI);

	cvtColor(m_imgLeftInput, imgStixelResult, COLOR_GRAY2BGR);
	addWeighted(imgStixelResult, 0.4, m_imgColorDisp, 0.6, 0., imgStixelResult);//addWeighted（）函数是将两张相同大小，相同类型的图片融合的函数。
	//    第一个数组    第一个数组的权重  第二个数组  第二个数组权重  两个数组作和后添加的数值 输出图片
	for (unsigned int i = 0; i < m_vecobjBB.size(); i++){
		rectangle(imgStixelResult, m_vecobjBB[i].rectBB, Scalar::all(255 - m_vecobjBB[i].dZ / fBrightness * 250), 2, 8);
		char temp1[20];
		char temp2[20];
		char temp3[20];
		string temp;
		sprintf_s(temp1, sizeof(temp1), "%.2fm", m_vecobjBB[i].dZ);//sprintf_s是一个函数，其函数功能是将数据格式化输出到字符串
		sprintf_s(temp2, sizeof(temp2), "%.2fm", m_vecobjBB[i].dX);
		sprintf_s(temp3, sizeof(temp3), "%.2fm", m_vecobjBB[i].dY);
		printf("z: %s\t,x: %s\t,y: %s\n", temp1,temp2,temp3);
		//temp = strcat(temp1,temp2);
		putText(imgStixelResult, temp1, m_vecobjBB[i].rectBB.br() - Point(m_vecobjBB[i].rectBB.width, 0), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255 - m_vecobjBB[i].dZ / fBrightness * 250), 2);
	}
	//br是the bottom-right corner 右下角角点坐标

}
void CStereoVisionForADAS::DrawLane(Mat& imgResult, StereoCamParam_t& objStereoParam)
{
	if (objStereoParam.objCamParam.m_dCameraHeight == 1.17){
		///////////////////////////////temp///////////////// daimler
		line(imgResult,
			Point(340, 216),//(int)(31 / 40 * 340) - 47),
			Point(640, 449),//(int)(31 / 40 * 640) - 47),
			Scalar(0, 255, 255), 5
			);
		line(imgResult,
			Point(300, 216),
			Point(0, 449),
			Scalar(0, 255, 255), 5
			);
	}
}
void CStereoVisionForADAS::DrawStixel(Mat& imgResult, vector<stixel_t>& vecobjStixels)
{
	for (unsigned int u = 0; u < vecobjStixels.size(); u++){
		line(m_imgStixelGray,
			Point(vecobjStixels[u].nCol, vecobjStixels[u].nGround),
			Point(vecobjStixels[u].nCol, vecobjStixels[u].nHeight),
			Scalar(vecobjStixels[u].chDisparity));
	}
	cvtPseudoColorImage(m_imgStixelGray, imgResult);
	m_imgStixelGray.setTo(0);
}
void CStereoVisionForADAS::DrawGround(Mat& imgResult, Mat& imgGround)
{
	threshold(imgGround, imgGround, 0, 255, CV_THRESH_BINARY);
	cvtColor(imgGround, imgResult, COLOR_GRAY2BGR);
	Mat imgTemp = imgResult.clone();
	imgTemp = Scalar(0, 75, 150);
	imgResult = imgResult&imgTemp;
	/*imshow("temp", imgResult);
	waitKey();*/
}
void CStereoVisionForADAS::TopViewStixel(vector<stixel_t>& objStixelInROI)//不进入
{
	Mat imgTopView(500, m_imgLeftInput.cols, CV_8UC3, Scalar::all(0));

	int nScale = 10; // nScale^-1 meter

	TopViewLane(imgTopView, 3., imgTopView.cols /2);

	// 5m ���� ���̵� �� ǥ��
	for (int i = 0; i < 500; i += 5 * nScale)
	{
		line(imgTopView,
			Point(0, i),
			Point(imgTopView.cols, i),
			Scalar(100, 100, 100));
	}

	//Top view drawing
	//cout << m_vecobjStixels.size() << endl;
	for (unsigned int u = 0; u < m_vecobjStixels.size(); u++){
		if (m_vecobjStixels[u].chDisparity == 0) continue;
		//int nDistance = (int)(m_nFocalLength*m_dBaseLine*nScale / ((double)m_vecobjStixels[u].chDisparity*m_nNumberOfDisp/255));
		int nDistance = (int)m_vecobjStixels[u].dZ * nScale;
		if (nDistance < 500) imgTopView.at<Vec3b>(500 - nDistance, m_vecobjStixels[u].nCol) = Vec3b(0, 0, 255);//Vec3b(0, 0, 10*(double)nDistance*m_vecobjStixels[u].nHeight/(double)m_nFocalLength);
	}
	for (unsigned int u = 0; u < objStixelInROI.size(); u++){
		int nDistance = (int)objStixelInROI[u].dZ * nScale;
		if (nDistance < 500) imgTopView.at<Vec3b>(500 - nDistance, objStixelInROI[u].nCol) = Vec3b(255, 255, 0);//Vec3b(0, 0, 10*(double)nDistance*m_vecobjStixels[u].nHeight/(double)m_nFocalLength);
		//imshow("topview", imgTopView);
		//waitKey(20);
	}

	imgTopView.copyTo(m_imgTopView);
 	//imshow("topview", imgTopView);

}
void CStereoVisionForADAS::TopViewLane(Mat& imgTopView, float fLaneInterval, int nCenterPointX)//不进入
{
	float fLaneIntervalPixel = 0;
	for (int i = 1; i < imgTopView.rows; i++)
	{
		fLaneIntervalPixel = (float)(m_objStereoParam.objCamParam.m_dFocalLength)*(float)fLaneInterval / ((float)(500 - i) / 10);
		if (fLaneIntervalPixel < m_imgLeftInput.cols)
		{
			imgTopView.at<Vec3b>(i, nCenterPointX - (int)fLaneIntervalPixel / 2) = Vec3b(0, 255, 255);
			imgTopView.at<Vec3b>(i, nCenterPointX + (int)fLaneIntervalPixel / 2) = Vec3b(0, 255, 255);
		}
	}

}
