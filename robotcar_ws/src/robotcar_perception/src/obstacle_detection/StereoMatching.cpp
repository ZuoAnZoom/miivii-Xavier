#include <robotcar_perception/obstacle_detection/StereoMatching.h>

CStereoMatching::CStereoMatching(StereoCamParam_t& objStereoParam)
{
	SetParamOCVStereo(objStereoParam);
}
CStereoMatching::CStereoMatching(Mat& imgLeftInput, Mat& imgRightInput, StereoCamParam_t& objStereoParam)
{
	SetParamOCVStereo(objStereoParam);
	m_imgLeftInput = imgLeftInput;
	m_imgRightInput = imgRightInput;
}
void CStereoMatching::SetParamOCVStereo(StereoCamParam_t& objStereoParam)
{
	m_objStereoParam = objStereoParam;
	

#if CV_MAJOR_VERSION==4
/*	bm->create(m_objStereoParam.m_nNumberOfDisp, m_objStereoParam.m_nWindowSize);
	//                            视差数　　　　　　　　　　　　匹配块
	bm->setPreFilterCap(31);
	bm->setBlockSize(m_objStereoParam.m_nWindowSize > 0 ? m_objStereoParam.m_nWindowSize : 9);//SAD窗口大小，设为奇数
	bm->setMinDisparity(0);//确定匹配搜索从哪里开始  默认值是0
	bm->setNumDisparities(m_objStereoParam.m_nNumberOfDisp);//在该数值确定的视差范围内进行搜索,视差窗口，即最大视差值与最小视差值之差, 大小必须是16的整数倍
	bm->setTextureThreshold(10);		/// SAD window response threshold : default=12
	bm->setUniquenessRatio(5);			/// >(match_val - min_match)/min_match　使用匹配功能模式 
	bm->setSpeckleWindowSize(100);		//25;//9;检查视差连通区域变化度的窗口大小, 值为0时取消 speckle 检查  
	bm->setSpeckleRange(32);			//4;　视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零  
	//bm->setSmallerBlockSize(9);
	//bm->setDisp12MaxDiff(1); */

	enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };
    int numberOfDisparities = (m_objStereoParam.m_nNumberOfDisp);//((imgSize.width / 8) + 15) & -16;
    sgbm->create(0, m_objStereoParam.m_nNumberOfDisp, m_objStereoParam.m_nWindowSize);
    sgbm->setPreFilterCap(63);
    int SADWindowSize = 13;
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);  //越小视差图的噪声越大；越大视差图的误匹配增多，空洞增多
    int cn =1;// imgLeftInput.channels();
    sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
	int alg = STEREO_SGBM;
    if (alg == STEREO_HH)
        sgbm->setMode(cv::StereoSGBM::MODE_HH);
    else if (alg == STEREO_SGBM)
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
    else if (alg == STEREO_3WAY)
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

#else
	bm.state->preFilterCap = 31;
	bm.state->SADWindowSize = m_objStereoParam.m_nWindowSize > 0 ? m_objStereoParam.m_nWindowSize : 9;
	bm.state->minDisparity = 0;
	bm.state->numberOfDisparities = m_objStereoParam.m_nNumberOfDisp;
	bm.state->textureThreshold = 10;		//SAD window response threshold : default=12
	bm.state->uniquenessRatio = 15;		// > (match_val - min_match)/min_match
	bm.state->speckleWindowSize = 100;//25;//9;
	bm.state->speckleRange = 32;//4;
	//bm.state->disp12MaxDiff = 1;
#endif

}

//判断输入图像的格式是否合适
MATCHING_ERROR CStereoMatching::SetImage(Mat& imgLeft, Mat& imgRight){
	if (imgLeft.size() != imgRight.size()) return IMGSCALE_ERR;
	if (imgLeft.size() != m_objStereoParam.objCamParam.m_sizeSrc)
	{
		cout << imgLeft.size() << " " << m_objStereoParam.objCamParam.m_sizeSrc << endl;
		return IMGSCALE_ERR;
	}
	if (imgLeft.channels() == 3){
		cvtColor(imgLeft, m_imgLeftInput, COLOR_BGR2GRAY);
		cvtColor(imgRight, m_imgRightInput, COLOR_BGR2GRAY);
		return NO_PROB;
	}
	m_imgLeftInput = imgLeft;
	m_imgRightInput = imgRight;
	
	return NO_PROB;
}

MATCHING_ERROR CStereoMatching::MakeDisparity()
{
	MakeDisparity(m_imgLeftInput, m_imgRightInput, m_matDisp16);
	m_matDisp16.convertTo(m_imgDisp8, CV_8U, 255.0 / (m_objStereoParam.m_nNumberOfDisp*16.));
	//将深度为１６的影像转换为深度为８的
	//m_imgDisp8_ori = m_imgDisp8.clone();
  	// imshow("disp", m_imgDisp8);

	return NO_PROB;
}
MATCHING_ERROR CStereoMatching::MakeDisparity(Mat& imgLeft, Mat& imgRight, bool flgUseWLSFilter)//main
{
#if CV_MAJOR_VERSION==2
	flgUseWLSFilter = false;
#endif
	MATCHING_ERROR Error;
	Error = SetImage(imgLeft, imgRight); 
	//cout << Error << endl;
	if (flgUseWLSFilter == false){
		MakeDisparity();
		ImproveDisparity_Naive(m_imgDisp8);
		
	}
}

//计算左右影像差异
MATCHING_ERROR CStereoMatching::MakeDisparity(Mat& imgLeft, Mat& imgRight, Mat& matDisp16){
#if CV_MAJOR_VERSION==4
	// cout << "SADWindowSize : " << bm->getBlockSize() << endl;
	// cout << "NumOfDisparity : " << bm->getNumDisparities() << endl;
	//bm->compute(imgLeft, imgRight, matDisp16);//计算差异
    sgbm->compute(imgLeft, imgRight, matDisp16);
#else
	sgbm(imgLeft, imgRight, matDisp16, CV_16S);
#endif
	m_matDisp16 = matDisp16;
	return NO_PROB;
}

//初步处理视差图
MATCHING_ERROR CStereoMatching::ImproveDisparity_Naive(Mat& imgDisp8) //用周围的视差去填充空洞
{
	uchar chTempCur = 0;
	uchar chTempPrev = 0;
	
	int cnt = 1;
	for (int v = 0; v < imgDisp8.rows; v++){
		for (int u = m_objStereoParam.m_nNumberOfDisp; u < imgDisp8.cols; u++){
			chTempCur = imgDisp8.at<uchar>(v, u);//v行u列的这个像素
			//shTempCur = m_matDisp16.at<short>(v, u);
			if (chTempCur == 0) {
				imgDisp8.at<uchar>(v, u) = chTempPrev;
				//m_matDisp16.at<short>(v, u) = shTempPrev;
			}
			else {
				chTempPrev = chTempCur;
				//shTempPrev = shTempCur;
			}
		}
	}
	Size size(7, 7);
	Mat matKernel = getStructuringElement(MORPH_RECT, size);//getStructuringElement函数会返回指定形状和尺寸的结构元素
	//MORPH_RECT是矩形

	morphologyEx(imgDisp8, imgDisp8, MORPH_OPEN, matKernel, Point(-1, -1), 2);//morphologyEx这个函数可以方便的对图像进行一系列的膨胀腐蚀组合
	//(源图像；目标图像；先腐蚀后膨胀；用于膨胀操作的结构元素，如果取值为Mat(),那么默认使用一个3 x 3 的方形结构元素；锚点的位置；迭代使用 dilate() 的次数)
	return NO_PROB;
}

//对视差图进行后处理，滤波  //没用
MATCHING_ERROR CStereoMatching::ImproveDisparity_WLSFilter(Mat& imgDisp8)
{
 #if CV_MAJOR_VERSION==4
	Mat matDispLeft16;
	Mat matDispRight16;
	/*Mat conf_map = Mat(m_imgLeftInput.rows, m_imgLeftInput.cols, CV_8U);
	conf_map = Scalar(255);*/

	wls_filter = createDisparityWLSFilter(sgbm);//该方法创建DisparityWLSFilter的实例，并根据匹配器实例自动设置所有相关的过滤器参数。
	Ptr<StereoMatcher> right_sgbm = createRightMatcher(sgbm);//设置匹配器以计算右视图视差图的便利方法
	
	wls_filter->setLambda(8000.);
	wls_filter->setSigmaColor(1.5);

	sgbm->compute(m_imgLeftInput, m_imgRightInput, matDispLeft16);
	right_sgbm->compute(m_imgRightInput, m_imgLeftInput, matDispRight16);
	wls_filter->filter(matDispLeft16, m_imgLeftInput, m_matDisp16, matDispRight16);
	/*conf_map = wls_filter->getConfidenceMap();

	m_rectFilterROI = wls_filter->getROI();*/

	m_matDisp16.convertTo(imgDisp8, CV_8U, 255 / (m_objStereoParam.m_nNumberOfDisp*16.));
 #endif
	return NO_PROB;
}