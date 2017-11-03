#include "ArmorDetector.hpp"
#include <vector>
#include <opencv2/imgproc.hpp>

#define SafeRect(rect, max_size) {if (makeRectSafe(rect, max_size) == false) continue;}
#ifndef SHOW_DEBUG_IMG
//#define SHOW_DEBUG_IMG
#endif

#ifndef COUT_LOG
//#define COUT_LOG
#endif

#ifndef USE_NEON    // ARMv7 NEON options of optimization
//#define USE_NEON
#endif

ArmorDetector::ArmorDetector(const ArmorParam &para = ArmorParam())
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

void ArmorDetector::initTemplate(const cv::Mat & _template, const cv::Mat & _template_small)
{
    std::vector<cv::Mat> bgr;
    bgr.resize(3);
    split(_template, bgr);

    cv::Mat temp_green;
    resize(bgr[1], temp_green, cv::Size(100,25));
    cv::threshold(temp_green, _binary_template, 128, 255, cv::THRESH_OTSU);

    split(_template_small, bgr);
    resize(bgr[1], temp_green, cv::Size(100,25));
    cv::threshold(temp_green, _binary_template_small, 128, 255, cv::THRESH_OTSU);
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
    cv::Mat rect_show;
    this->_src.copyTo(rect_show);
	for (size_t i = 0; i < rects.size(); i++)	{
		Scalar color(rand() & 255, rand() & 255, rand() & 255);
		cv::Point2f vertices[4];
		rects[i].points(vertices);
		for (int i = 0; i < 4; i++)
			line(rect_show, vertices[i], vertices[(i + 1) % 4], color);
	}

	cv::Point2f vertices[4];
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
//            line(this->_src, vertices[i], vertices[(i + 1) % 4], color);
//    }
//    imshow("5.rect", this->_src);

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
            this->_src = src;
        }
        else
        {
            src(_dect_rect).copyTo(this->_src); // copy-to Matrix
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
			cv::findContours(left(rect), contour, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point2i(rect.x, rect.y));
			contours_left.insert(contours_left.end(), contour.begin(), contour.end());
		}

		if (count_right > 10){  //左侧灯柱
            std::vector<std::vector<cv::Point2i> > contour;
			std::vector<cv::Vec4i> hierarchy;
			cv::findContours(right(rect), contour, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point2i(rect.x, rect.y));
			contours_right.insert(contours_right.end(), contour.begin(), contour.end());
		}
		++ite;
    }

}

void ArmorDetector::findTargetInContours(const std::vector<std::vector<cv::Point> > &contours_left,
                                        const std::vector<std::vector<cv::Point> > &contours_right,
                                        std::vector<cv::RotatedRect> &rects,
                                        std::vector<double> &score)
{
// Fit the outline with a straight line to find the contour that matches the slope range
std::vector<cv::RotatedRect> final_contour_rect_left, final_contour_rect_right;
std::vector<double> score_left, score_right;

#ifdef SHOW_DEBUG_IMG
    cv::Mat contours_show_left, contours_show_right;
    this->_src.copyTo(contours_show_left);
    this->_src.copyTo(contours_show_right);
#endif

for (size_t i = 0; i < contours_left.size(); ++i)  // for_each contours_left
{
    // fit the lamp contour as a eclipse
    cv::RotatedRect rrect = cv::minAreaRect(contours_left[i]);
    rrect = adjustRRect(rrect);
    double angle = rrect.angle;
    angle = 90 - angle;
    angle = angle < 0 ? angle + 180 : angle;

#ifdef SHOW_DEBUG_IMG
    //cout << "(angle)\t(" << angle << ")\n";
    cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
    char slope_str[15];
    sprintf(slope_str, "%.1f", angle);
    cv::putText(contours_show_left, slope_str, contours_left[i][0], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.5, color, 1);
    cv::drawContours(contours_show_left, contours_left, i, color, CV_FILLED, 8);
#endif
    // the contour must be near-vertical
    float delta_angle = std::abs(angle - 90);
    if (delta_angle < _para.light_slope_offset)
    {
        final_contour_rect_left.push_back(rrect); 
        score_left.push_back(delta_angle);
    }
}

for (size_t i = 0; i < contours_right.size(); ++i) // for_each contours_right
{
    // fit the lamp contour as a eclipse
    cv::RotatedRect rrect = minAreaRect(contours_right[i]);
    rrect = adjustRRect(rrect);
    double angle = rrect.angle;
    angle = 90 - angle;
    angle = angle < 0 ? angle + 180 : angle;

#ifdef SHOW_DEBUG_IMG
    //cout << "(angle)\t(" << angle << ")\n";
    cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
    char slope_str[15];
    sprintf(slope_str, "%.1f", angle);
    cv::putText(contours_show_right, slope_str, contours_right[i][0], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.5, color, 1);
    cv::drawContours(contours_show_right, contours_right, i, color, CV_FILLED, 8);
#endif
    // the contour must be near-vertical
    float delta_angle = std::abs(angle - 90);
    if (delta_angle < _para.light_slope_offset)
    {
        final_contour_rect_right.push_back(rrect);
        score_right.push_back(delta_angle);
    }
}

// using all the left edge and right edge to make up rectangles
for (size_t i = 0; i < final_contour_rect_left.size(); ++i) 
{
    const cv::RotatedRect & rect_i = final_contour_rect_left[i];
    const cv::Point & center_i = rect_i.center;
    float xi = center_i.x;
    float yi = center_i.y;

    for (size_t j = 0; j < final_contour_rect_right.size(); j++) 
    {
        const cv::RotatedRect & rect_j = final_contour_rect_right[j];
        const cv::Point & center_j = rect_j.center;
        float xj = center_j.x;
        float yj = center_j.y;
        float delta_h = xj - xi;
        float delta_angle = std::abs(rect_j.angle - rect_i.angle);

        // if rectangle is match condition, put it in candidate vector
        if (delta_h > _para.min_light_delta_h && 
            delta_h < _para.max_light_delta_h &&
            std::abs(yi - yj) < _para.max_light_delta_v &&        // ֱϵĸ߶Ȳ
            delta_angle < _para.max_light_delta_angle) 
        {
            cv::RotatedRect rect = boundingRRect(rect_i, rect_j); // Create New Armor Area Detect
            rects.push_back(rect);
            score.push_back((score_right[j] + score_left[i]) / 6.0 + delta_angle);  // Has a expr to calu the score
        }
    }
}
#ifdef SHOW_DEBUG_IMG
    cv::imshow("4.contours_l", contours_show_left);
    cv::imshow("4.contours_r", contours_show_right);
#endif
}

cv::RotatedRect ArmorDetector::chooseTarget(const std::vector<cv::RotatedRect> & rects, 
                const std::vector<double> & score) 
{
    if (rects.size() < 1)
    {
        this->_is_lost = true;
        return cv::RotatedRect();
    }

    int ret_idx = -1;
    double avg_score = 0.0;
    for(int i = 0; i < score.size(); ++i)
    {
        avg_score += score[i];
    }
    avg_score /= score.size();

    const double small_armor_wh_threshold = 3.6;
    const double avg_slope = 4.0;
    const double exp_weight_scale = 15.0;
    const double degree2rad_scale = 3.1415926 / 180.0;

    double percent_large_grad_threshold = 0.25;
    double template_dist_threshold = 0.20;
    double max_wh_ratio = 5.2, min_wh_ratio = 1.25;

    if( _is_lost == false)
    {
        template_dist_threshold = 0.4;
        percent_large_grad_threshold = 0.5;
        max_wh_ratio += 0.5;
        min_wh_ratio -= 0.2;
    }
    double weight = template_dist_threshold * percent_large_grad_threshold / exp(-(avg_slope + avg_score) * degree2rad_scale * exp_weight_scale);
    bool is_small = false;



    for (size_t i = 0; i < rects.size(); ++i)  // for_each rects
    {
        const cv::RotatedRect & rect = rects[i];

        // the ratio of width and height must be matched
        double w = rect.size.width;
        double h = rect.size.height;
        double wh_ratio = w / h;
        if (wh_ratio > max_wh_ratio || wh_ratio < min_wh_ratio)
        continue;

        AngleSolver * slover = m_solver;

        if (wh_ratio < small_armor_wh_threshold)
        is_small = true;
        else
        is_small = false;

        if (slover != NULL && _is_lost) 
        {
            is_small == true ? slover->setTargetSize(12.4, 5.4) : slover->setTargetSize(21.6, 5.4);
            double angle_y = 0.0, angle_x = 0.0;
            if (false == slover->getAngle(rect, angle_x, angle_y, 0., 0., cv::Point(_dect_rect.x, _dect_rect.y)))
            continue;
            cv::Mat xyz_in_ptz = slover->position_in_ptz;
            double d = sqrt(xyz_in_ptz.at<double>(1) * xyz_in_ptz.at<double>(1) + xyz_in_ptz.at<double>(2) * xyz_in_ptz.at<double>(2));
            double t_offset = sin((pitch_angle -  angle_y) * 3.1415926 / 180.0) * d;
            if(t_offset < -40.0 || t_offset > 35.0){
            #ifdef COUT_LOG
            cout << "refused : target out of range: " <<
            "\n\tcurrent ptz angle: " << pitch_angle <<
            "\n\tcurrent target position: "  << xyz_in_ptz.t() <<
            "\n\tcurrent target offet set: " << t_offset << "\n";
            #endif
            continue;
            }
            if(t_offset > 5.0)
            is_small = false;
            else
            is_small = true;
        }

        // width must close to the last result
        const cv::Size2f size_last = _res_last.size;
        if(_is_lost == false && size_last.width > _para.min_light_delta_h)
        {
            double percent = 0.50 * size_last.width;
            if (std::abs(w - size_last.width) > percent)
            {
            //cout << "refused 0 : size_last.width: " << size_last.width << "\tcur width: "  << w << endl;
            continue;
            }
        }

        // rotate the area
        int lamp_width = std::max((int)w / (is_small ? 12 : 20), 1);
        cv::Rect bounding_roi = rect.boundingRect();
        bounding_roi.x -= w / 8;
        bounding_roi.width += w / 4;
        SafeRect(bounding_roi, this->_src.size());

        cv::Point2f new_center = rect.center - cv::Point2f(bounding_roi.x, bounding_roi.y);
        cv::Mat roi_src = this->_src(bounding_roi);
        cv::Mat rotation = getRotationMatrix2D(new_center, rect.angle, 1);
        cv::Mat rectify_target;
        cv::warpAffine(roi_src, rectify_target, rotation, bounding_roi.size());

        // get the black board of the armor
        cv::Point ul = cv::Point(std::max(int(new_center.x - (w / 2.0)) + 1, 0), std::max((int)(new_center.y - h / 2.0), 0));
        cv::Point dr = cv::Point(new_center.x + w / 2.0, new_center.y + h / 2.0);
        cv::Rect roi_black = cv::Rect(cv::Point(ul.x, ul.y), cv::Point(dr.x, dr.y));
        // get the left lamp and right lamp of the armor
        cv::Rect roi_left_lamp = cv::Rect(cv::Point(std::max(0, ul.x - lamp_width), ul.y), cv::Point(std::max(rectify_target.cols, ul.x), dr.y));
        cv::Rect roi_right_lamp = cv::Rect(cv::Point(dr.x , ul.y), cv::Point(std::min(dr.x + lamp_width, rectify_target.cols), dr.y));

        SafeRect(roi_left_lamp, rectify_target.size());
        SafeRect(roi_right_lamp, rectify_target.size());
        SafeRect(roi_black, rectify_target.size());

        // valid the gray value of black area
        cv::Mat black_part;
        rectify_target(roi_black).copyTo(black_part);
        int black_side = std::min(_para.min_light_delta_h / 2, 4);
        cv::Mat gray_mid_black(cv::Size(roi_black.width - black_side * 2, roi_black.height), CV_8UC1);
        const uchar * ptr = black_part.data;
        uchar * ptr_gray = gray_mid_black.data;
        int avg_green_mid = 0;
        int avg_red_side = 0;
        int avg_blue_side = 0;
        int avg_green_side = 0;
        int cf = black_part.cols - black_side;
        for(int j = 0; j < black_part.rows; ++j) // ɫ
        {
            for (int k = 0; k < black_side; ++k, ++ptr)
            {
                uchar b = *ptr;
                uchar g = *(++ptr);
                uchar r = *(++ptr);
                avg_red_side += r;
                avg_blue_side += b;
                avg_green_side += g;
            }
            for (int k = black_side; k < cf; ++k, ++ptr, ++ptr_gray)
            {
                uchar b = *ptr;
                uchar g = *(++ptr);
                uchar r = *(++ptr);
                avg_green_mid += g;
                *ptr_gray = (uchar)((r * 38 + g * 75 + b * 15) >> 7);
            }
            for (int k = cf; k < black_part.cols; ++k, ++ptr)
            {
                uchar b = *ptr;
                uchar g = *(++ptr);
                uchar r = *(++ptr);
                avg_red_side += r;
                avg_blue_side += b;
                avg_green_side += g;
            }
        }
        avg_green_mid /= (gray_mid_black.cols * gray_mid_black.rows);
        int side_total = black_side * 2 * gray_mid_black.rows;
        avg_green_side /= side_total;
        if (avg_green_mid > _para.avg_board_gray_threshold)
        {
#ifdef COUT_LOG
        cout << "refused 1 : avg_green: " << avg_green_mid << "\tavg_board_gray_threshold: "  << (int)_para.avg_board_gray_threshold << endl;
#endif
            continue;
        }

        if (_para.enemy_color == RED && avg_red_side - 10 < avg_blue_side)
        {
#ifdef COUT_LOG
            cout << "refused 1.1 : red < blue:  red:" << avg_red_side/side_total << "\tblue: "  << avg_blue_side/side_total << endl;
#endif
            continue;
        }
        else if (_para.enemy_color == BLUE && avg_blue_side - 10 < avg_red_side)
        {
#ifdef COUT_LOG
            cout << "refused 1.2 : red > blue:  red:" << avg_red_side/side_total << "\tblue: "  << avg_blue_side/side_total << endl;
#endif
            continue;
        }

        // valid the gradient of the black area
        cv::Mat gradX, gradY;
        cv::Sobel(gray_mid_black, gradX, CV_16S, 1, 0);
        cv::Sobel(gray_mid_black, gradY, CV_16S, 0, 1);

        int y_grad = 0, x_grad = 0;
        int large_grad_count = 0;
        int side_width = gray_mid_black.cols * 10.0 / 100; // jump over the side
        short * ptr_x = (short *)gradX.data;
        short * ptr_y = (short *)gradY.data;
        for (size_t j = 0; j < gradX.rows; ++j)
        {
        // jump over left side part
            ptr_x+= side_width;
            ptr_y+= side_width;

            // compute the middle part
            int up_b = gradX.cols - side_width;
            for (size_t k = side_width; k < up_b; ++k, ++ptr_x, ++ptr_y)
            {
                int x = abs(*ptr_x);
                int y = abs(*ptr_y);
                x_grad += x;
                y_grad += y;
                large_grad_count += y / _para.grad_threshold;
            }
            // jump over right side part
            ptr_x+= gradX.cols-up_b;
            ptr_y+= gradX.cols-up_b;
        }

        // kick out the area of large gradients
        int total_pixel = (gradX.cols - (side_width << 1)) * gradX.rows;
        double large_grad_percent =(double)large_grad_count/total_pixel;
        if(large_grad_percent > percent_large_grad_threshold)
        {
        #ifdef COUT_LOG
        cout << "refused 2: large_grad_percent: " << large_grad_percent <<  endl;
        #endif
        continue;
        }

        // valid the average gradient of the black area
        double avg_x = (double)x_grad / total_pixel;
        double avg_y = (double)y_grad / total_pixel;
        if (avg_x < _para.avg_board_grad_threshold + 30 && avg_y < _para.avg_board_grad_threshold ){
        cv::Point p1(roi_left_lamp.x, roi_left_lamp.y);
        cv::Point p2(roi_right_lamp.x+roi_right_lamp.width, roi_right_lamp.y+roi_right_lamp.height);
        // get the whole area of armor
        cv::Rect armor_rect(p1, p2);
        SafeRect(armor_rect, rectify_target.size());
        cv::Mat armor = rectify_target(armor_rect);

        // compute the distance of template
        cv::Size cur_size;
        if (is_small)
        cur_size = _binary_template_small.size();
        else
        cur_size = _binary_template.size();
        cv::resize(armor, armor, cur_size);
        double dist = templateDist(armor, is_small);
        dist = dist / (cur_size.width * cur_size.height);
        if(dist >  template_dist_threshold)
        {
        #ifdef COUT_LOG
        cout << "refused 3: dist: " << dist << "\tdist threshold:" << (cur_size.width * cur_size.height) / 4 << endl;
        #endif
        continue;
        }

        // choose the best rectangle with minimum (large_grad_percent * dist)
        dist = std::max(dist, 0.001);
        large_grad_percent = std::max(large_grad_percent, 0.001);
        double cur_weight = large_grad_percent * dist / std::exp(-(std::abs(rect.angle) + score[i]) * degree2rad_scale * exp_weight_scale);
        if(cur_weight < weight)
        {
            weight = cur_weight;
            ret_idx = i;
            this->_is_small_armor = is_small;
            //cout << "red:" << avg_red_side/side_total << "\tblue: "  << avg_blue_side/side_total << endl;
            //imwrite("armor_org.bmp", rectify_target(armor_rect));
        }
        #ifdef COUT_LOG
        else
        cout << "refused 4: cur_weight: " << cur_weight << "\tweight threshold:" << weight << endl;
        #endif
        }
        #ifdef COUT_LOG
        else
        cout << "refused 3: (x_grad, y_grad): (" << avg_x << ", " << avg_y << ")\t avg_grad_threshold: " <<  (int)_para.avg_board_grad_threshold << endl;
        #endif
    }
    //return ret_idx == -1 ? RotatedRect() : rects[ret_idx];
    if (ret_idx == -1)
    {
        this->_is_lost = true;
        return cv::RotatedRect();
    }
    this->_is_lost = false;
    // broaden the height of target
    cv::RotatedRect ret_rect = rects[ret_idx];
    cv::Rect ret_rect1 = ret_rect.boundingRect();
    if (broadenRect(ret_rect1, 3, 3, this->_src.size()) == false)
    return ret_rect;
    //rectangle(this->_src, ret_rect1, CV_RGB(255,128,128),2);
    //imshow("this->_src", this->_src);

    cv::Mat s_b = _ec(ret_rect1);
    cv::threshold(s_b, s_b, 128, 255, CV_THRESH_OTSU);
    std::vector<std::vector<cv::Point2i> > contours_br;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(s_b, contours_br, hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE);
    for (int k = 0; k < contours_br.size(); ++ k)
    {
        cv::Rect r = cv::boundingRect(contours_br[k]);
        if (r.height > ret_rect.size.height)
        {
            ret_rect.size.height = r.height;
            if(ret_rect.size.width / ret_rect.size.height < small_armor_wh_threshold)
            this->_is_small_armor = true;
        }
    }
    return ret_rect;
}

cv::RotatedRect ArmorDetector::adjustRRect(const cv::RotatedRect &rect)
{
    const cv::Size2f &s = rect.size;
    if (s.width < s.height)
    {
        return rect;
    }
    return cv::RotatedRect(rect.center, cv::Size2f(s.height, s.width), rect.angle + 90.0);
}

cv::RotatedRect ArmorDetector::boundingRRect(const cv::RotatedRect &left, const cv::RotatedRect &right)
{
    const cv::Point &pl = left.center, &pr = right.center;
    cv::Point2f center = (pl + pr) / 2.0;
    cv::Size2f wh_l = left.size;
    cv::Size2f wh_r = right.size;
    double width = PointDist(pl, pr) - (wh_l.width + wh_r.width) / 2.0;
    double height = std::max(wh_l.height, wh_r.height);
    double angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
    return cv::RotatedRect(center, cv::Size2f(width, height), angle * 180 / CV_PI);
}

double ArmorDetector::PointDist(const cv::Point &p1, const cv::Point &p2)
{
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

int ArmorDetector::templateDist(const cv::Mat &img, bool is_small)
{
    int dist = 0;
    const uchar threshold_value = this->_para.min_light_gray - 15;
    int total_pixel = img.rows * img.cols;
    const uchar *p1 = is_small ? this->_binary_template_small.data : this->_binary_template.data;
    const uchar *p2 = img.data;

    for (int i = 0; i < total_pixel; ++i, p1 += 1, p2 += 3)
    {
        uchar v = (*p2 + 1) > threshold_value ? 255 : 0;
        dist += (*p1) == v ? 0 : 1;
    }
    return dist;
}

bool ArmorDetector::broadenRect(cv::Rect &rect, int width_added, int height_added, cv::Size size)
{
    rect.x -= width_added;
    rect.width += width_added * 2;
    rect.y -= height_added;
    rect.height += height_added * 2;
    return this->makeRectSafe(rect, size);
}

bool ArmorDetector::makeRectSafe(cv::Rect &rect, cv::Size size)
{
    if (rect.x < 0)
    {
        rect.x = 0;
    }
    if (rect.x + rect.width > size.width)
    {
        rect.width = size.width - rect.x;
    }
    if (rect.y < 0)
    {
        rect.y = 0;
    }
    if (rect.y + rect.height > size.height)
    {
        rect.height = size.height - rect.y;
    }
    if (rect.width <= 0 || rect.height <= 0)
    {
        return false;
    }
    return true;

}





