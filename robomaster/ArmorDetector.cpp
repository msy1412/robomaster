#include "ArmorDetector.hpp"
#include <vector>
#include <opencv2/imgproc.hpp>

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

void ArmorDetector::setImage(const cv::Mat &src)
{
    this->_size = src.size();  //size of image
    const cv::Point &last_result = this->_res_last.center;  //record the center of last result

    if (last_result.x == 0 || last_result.y == 0)  //if last rect is none search source Image
    {
        this->_src = src;
        this->_dect_rect = cv::Rect(0, 0, src.cols, src.rows); //detect range of the src
    }
    else
    {  //roi erct , settings of params
        cv::Rect rect = this->_res_last.boundingRect();
        int max_half_w = this->_para.max_light_delta_h * 1.3;  //亮度的最大差值
        int max_half_h = 300;  //高度的最大值的一半
        double scale = 1.8;  

        int exp_half_w = std::min(max_half_w / 2, int(rect.width * scale));
        int exp_half_h = std::min(max_half_h / 2, int(rect.height * scale));

        int w = std::min(max_half_w, exp_half_w);
        int h = std::min(max_half_h, exp_half_h);

        cv::Point center = last_result;
        int x = std::max(center.x - w, 0);
        int y = std::max(center.y - h, 0);
        cv::Point lu = cv::Point(x, y);

        x = std::min(center.x + w, src.cols);
        y = std::min(center.y + h, src.rows);
        cv::Point rd = cv::Point(x, y);

        _dect_rect = cv::Rect(lu, rd);  // detect rect area where maybe has armor

        if (makeRectSafe(_dect_rect, src.size()) == false){ // if no-vaild area
            _res_last = cv::RotatedRect();
            _dect_rect = cv::Rect(0, 0, src.cols, src.rows);
            _src = src;
        }
        else
        {
            src(_dect_rect).copyTo(_src); // copy-to Matrix
        }  
    }

    int total_pixel = this->_src.cols * this->_src.rows;
    const uchar *ptr_src = this->_src.data;
    const uchar *ptr_src_end = this->_src.data + total_pixel * 3;

    this->_g.create(this->_src.size(), CV_8UC1);
    this->_ec.create(this->_src.size(), CV_8UC1);
    this->_max_color = cv::Mat(this->_src.size(), CV_8UC1, cv::Scalar(0));

    uchar *ptr_g = this->_g.data, *ptr_ec = this->_ec.data, *ptr_max_color = this->_max_color.data;

    if (this->_para.enemy_color == RED)
    {       // 对方是红色的  下面的循环是遍历矩阵的元素，以此来初始化max_color _g _ec 这三个矩阵
        for ( ; ptr_src != ptr_src_end; ++ptr_src, ++ptr_g, ++ptr_max_color, ++ptr_ec)
        {
            //这种遍历方式是有问题的吗？需要判断是否连续存储么？
            uchar b = *ptr_src;
            uchar g = *(++ptr_src);
            uchar r = *(++ptr_src);
            *ptr_g = g;
            *ptr_ec = r;
            if (r > this->_para.min_light_gray)
            {
                *ptr_max_color = 255;
            }
        }

    }
    else
    {       //非红
        for (; ptr_src != ptr_src_end; ++ptr_src, ++ptr_g, ++ptr_max_color, ++ptr_ec)
        {
            uchar b = *ptr_src;
            uchar g = *(++ptr_src);
            uchar r = *(++ptr_src);
            *ptr_g = g;
            *ptr_ec = r;
            if (b > this->_para.min_light_gray)
            {
                *ptr_max_color = 255;
            }
        }
    }
    //调试
#ifdef SHOW_DEBUG_IMG 
	cv::imshow("g", _g);
	cv::imshow("_max_color", _max_color);
#endif

}

void ArmorDetector::findContourInEnemyColor(  //在敌人的通道图像中找到边界
    cv::Mat & left, cv::Mat & right,
    std::vector<std::vector<cv::Point2i> > &contours_left,
    std::vector<std::vector<cv::Point2i> > &contours_right)
{
    std::vector<std::vector<cv::Point2i> > contours_br;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(this->_max_color, contours_br, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point2i> > ::const_iterator ite = contours_br.begin();


    left = cv::Mat::zeros(this->_max_color.size(), CV_8UC1);
    right = cv::Mat::zeros(this->_max_color.size(), CV_8UC1);
    const int margin_l = 1, margin_r = 10; //左右边缘
    const int margin_h = 3;  //高度边缘是个什么东西呢？

    while(ite != contours_br.end())   //for each sub contour
    {
        cv::Rect rect = cv::boundingRect(*ite);  //获取外包的最小矩形区域
        if (rect.height < this->_para.min_light_height)
        {  //利用先验知识判断该矩形是否为合适的矩形
            ++ite;
            continue;
        }
        int max_i = rect.x + rect.width;
        max_i = std::min(_max_color.cols, max_i + margin_r);  // safe range of max_i 
        
		int half_j = (margin_h >> 1), max_j = rect.y + rect.height, min_j = rect.y;
		max_j = std::min(_max_color.rows - 1, max_j + half_j);  // safe range of max_j 
		min_j = std::max(min_j, half_j);
		int count_left = 0, count_right = 0;

        const uchar * ptr_gray_base = _g.data;
        for (size_t j = min_j; j < max_j; ++j)	// looping of range of j 
        {
            const uchar * ptr_gray = ptr_gray_base + j * _g.cols;
            for (size_t i = rect.x; i < max_i; ++i)	  // looping of range of i 
            {
                if (*(ptr_gray + i) < _para.min_light_gray)
                    continue;

                float block0 = 0, block1 = 0, block_1 = 0;
                // do margin protection
                if (i >= margin_r)
                {
                    for (int m = -half_j; m <= half_j; ++m)	
                    {
                        const uchar * ptr = ptr_gray + m * _g.cols + i;
                        // for common 'o' of template
                        for (int k = 0; k < margin_l; k++)
                            block0 += *(ptr + k);

                        // for 'x' of left template
                        for (int k = margin_l; k < margin_r; k++)
                            block1 += *(ptr + k);

                        // for 'x' of right template
                        for (int k = -margin_l; k > -margin_r; --k)
                            block_1 += *(ptr + k);
                    }
                    block0 /= margin_h * margin_l;
                    block1 /= margin_h * (margin_r - margin_l);
                    block_1 /= margin_h * (margin_r - margin_l);
                    int avgdist = block0 - block1;
                    left.at<uchar>(j, i) = avgdist > _para.avg_contrast_threshold ? (++count_left, 255) : 0;
                    avgdist = block0 - block_1;
                    right.at<uchar>(j, i) = avgdist > _para.avg_contrast_threshold ? (++count_right, 255) : 0;
                }
                else {
                    for (int m = -half_j; m <= half_j; ++m)	{
                        const uchar * ptr = ptr_gray + m * _g.cols + i;

                        // for 'o' of left template
                        for (int k = 0; k < margin_l; k++)
                            block0 += *(ptr + k);

                        // for 'x' of left template
                        for (int k = margin_l; k < margin_r; k++)
                            block1 += *(ptr + k);;
                    }
                    block0 /= margin_h * margin_l;
                    block1 /= margin_h * (margin_r - margin_l);
                    int avgdist = block0 - block1;
                    left.at<uchar>(j, i) = avgdist > _para.avg_contrast_threshold ? (++count_left, 255) : 0;
                }
            }
        }

        // find the lamp contours
		if (count_left > 10){  //右侧灯柱
            std::vector<std::vector<cv::Point2i> > contour;
			std::vector<cv::Vec4i> hierarchy;
			findContours(left(rect), contour, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point2i(rect.x, rect.y));
			contours_left.insert(contours_left.end(), contour.begin(), contour.end());
		}

		if (count_right > 10){  //左侧灯柱
            std::vector<std::vector<cv::Point2i> > contour;
			std::vector<cv::Vec4i> hierarchy;
			findContours(right(rect), contour, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point2i(rect.x, rect.y));
			contours_right.insert(contours_right.end(), contour.begin(), contour.end());
		}
		++ite;
    }

}

void ArmorDetector::findTargetInContours(const std::vector<std::vector<cv::Point> > &contour_left,
                                        const std::vector<std::vector<cv::Point> > &contour_right,
                                        std::vector<cv::RotatedRect> &rects,
                                        std::vector<double> &score)
{
    
}



