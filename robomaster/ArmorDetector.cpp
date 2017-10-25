#include "ArmorDetector.hpp"
#include <vector>

#ifndef SHOW_DEBUG_IMG
//#define SHOW_DEBUG_IMG
#endif

#ifndef COUT_LOG
//#define COUT_LOG
#endif

#ifndef USE_NEON    // ARMv7 NEON options of optimization
//#define USE_NEON
#endif

ArmorDetector::ArmorDetector(const ArmorParam & para = ArmorParam())
{
    this->pitch_angle = 0;
    this->m_solver = NULL;
    this->_para = para;
    this->_res_last = cv::RotatedRect();
    this->_dect_rect = cv::Rect();
    this->_is_small_armor = false;
    this->_lost_cnt = 0;
    this->_is_lost = true;

}

void ArmorDetector::reset()
{
    this->_res_last = cv::RotatedRect();
    this->_dect_rect = cv::Rect();
    this->_is_small_armor = false;
    this->_lost_cnt = 0;
    this->_is_lost = true;
}

cv::RotatedRect ArmorDetector::getTargetArea(const cv::Mat & src)
{
	setImage(src);
    cv::Mat contrast_left, contrast_right;
    
    std::vector<std::vector<cv::Point2i> > contours_left;
    std::vector<std::vector<cv::Point2i> > contours_right;

    findContourInEnemyColor(contrast_left, contrast_right, contours_left, contours_right);
    
    std::vector<cv::RotatedRect> rects;
    std::vector<double> score;
    findTargetInContours(contours_left, contours_right, rects, score);
    cv::RotatedRect final_rect = chooseTarget(rects, score);

    //cout << "w:" << final_rect.size.width << ", h:" << final_rect.size.height << ", w/h:" << (double)final_rect.size.width/final_rect.size.height << endl;
#ifdef SHOW_DEBUG_IMG
    Mat rect_show;
    _src.copyTo(rect_show);
	for (size_t i = 0; i < rects.size(); i++)	{
		Scalar color(rand() & 255, rand() & 255, rand() & 255);
		Point2f vertices[4];
		rects[i].points(vertices);
		for (int i = 0; i < 4; i++)
			line(rect_show, vertices[i], vertices[(i + 1) % 4], color);
	}

	Point2f vertices[4];
	final_rect.points(vertices);
	for (int i = 0; i < 4; i++)
		line(rect_show, vertices[i], vertices[(i + 1) % 4], CV_RGB(255, 0, 0));
	imshow("5.rect", rect_show);
#endif

//    for (size_t i = 0; i < rects.size(); i++)	{
//        //Point offset = Point(_dect_rect.x, _dect_rect);
//        Scalar color(rand() & 255, rand() & 255, rand() & 255);
//        Point2f vertices[4];
//        rects[i].points(vertices);
//        for (int i = 0; i < 4; i++)
//            line(_src, vertices[i], vertices[(i + 1) % 4], color);
//    }
//    imshow("5.rect", _src);

    if(final_rect.size.width != 0){
        final_rect.center.x += _dect_rect.x;
        final_rect.center.y += _dect_rect.y;
        _res_last = final_rect;
        _lost_cnt = 0;
    }
    else{
        ++_lost_cnt;

        if (_lost_cnt < 3)
            _res_last.size = cv::Size2f(_res_last.size.width * 2, _res_last.size.height * 1.5);
        else if(_lost_cnt == 6)
            _res_last.size = cv::Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
        else if(_lost_cnt == 12)
            _res_last.size = cv::Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
        else if(_lost_cnt == 18)
            _res_last.size = cv::Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
        else if (_lost_cnt > 60 )
            _res_last = cv::RotatedRect();
    }
	return final_rect;
}


