#include "ImageConsProd.hpp"
#include "Predictor.hpp"
#include "ArmorDetector.hpp"
#include "RuneDetector.hpp"
#include "ResFilter.hpp"
#include "AngleSolver.hpp"
#include "AngleSolverFactory.hpp"
#include "Serial.hpp" 
#include "RMVideoCapture.hpp"
#include <opencv2/imgcodecs.hpp>

#ifndef _SHOW_DETECT_
//#define _SHOW_DETECT_
#endif

#define VIDEO_WIDTH  640
#define VIDEO_HEIGHT 480
#define BUFFER_SIZE 1
volatile unsigned int prdIdx = 0;  //属于全局变量，如果未初始化的话，默认为0,但是局部变量的初始值是随机的
volatile unsigned int csmIdx = 0;


struct ImageData {
	cv::Mat img;             // data come from camera
	unsigned int frame;  // speed of img
};

ImageData capturedata[BUFFER_SIZE];   // Buffer of capture

void ImageConsProd::ImageProducer() 
{ 									
	RMVideoCapture cap("/dev/video0", 3); 
	cap.setVideoFormat(VIDEO_WIDTH, VIDEO_HEIGHT, 1);
	cap.startStream();
	cap.info();
	while (true) {
		while (prdIdx - csmIdx >= BUFFER_SIZE);
		cap >> capturedata[prdIdx % BUFFER_SIZE].img;
		capturedata[prdIdx % BUFFER_SIZE].frame = cap.getFrameCount();
		++prdIdx;
	}
}

void ImageConsProd::ImageConsumer()
{
    //load calibration parameters
    cv::FileStorage fs("/home/cam_intrinsic.xml", cv::FileStorage::READ);
    if (!fs.isOpened)
    {
        return;
    }
    cv::Mat cam_matrix, distortion_coeff;  //相机矩阵 和 倾斜系数  歪曲系数
    int delay_time;
    fs["Camera_Matrix"] >> cam_matrix;
    fs["Distortion Coefficients"] >> distortion_coeff;
    fs["Delay_Time"] >> delay_time;

    AngleSolver angle_solver(cam_matrix, distortion_coeff, 21.6, 5.4, _settings->_scale_z_480, _settings->_min_detect_distance, _settings->_max_detect_distance);

    cv::Point2f image_center = cv::Point2f(cam_matrix.at<double>(0, 2), cam_matrix.at<double>(1, 2));  //获取中心zuobiao

    //parameters of PTZ and barrel
    const double overlap_dist = 100000.0;
    const double barrel_ptz_offset_y = 3.3;
    const double ptz_camera_y = 0.8;
    const double ptz_camera_z = -20.5;
    
    double theta = -atan((ptz_camera_y + barrel_ptz_offset_y)/overlap_dist);
    double r_data[] = {1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta)};
    double t_data[] = {0, ptz_camera_y, ptz_camera_z}; //ptz org position in camera coodinate system
    cv::Mat t_camera_ptz(3, 1, CV_64FC1, t_data);
    cv::Mat r_camera_ptz(3, 3, CV_64FC1, r_data);

    angle_solver.setRelationPoseCameraPTZ(r_camera_ptz, t_camera_ptz, barrel_ptz_offset_y);
    AngleSolverFactory angle_solver_factory;
    angle_solver_factory.setTargetSize(21.6, 5.4, AngleSolverFactory::TARGET_ARMOR);
    angle_solver_factory.setTargetSize(12.4, 5.4, AngleSolverFactory::TARGET_SMALL_ARMOR);
    angle_slover_factory.setTargetSize(28.0, 16.0, AngleSolverFactory::TARGET_RUNE);

    Predictor predictor;
    //load armor detector template;
    ArmorDetector armor_detector(this->_settings->_armor);
    cv::Mat template_img = cv::imread(this->_settings->_template_image_file);
    cv::Mat small_template_img =  cv::imread(this->_settings->_small_template_image_file);

    armor_detector.initTemplate(template_img, small_template_img);
    armor_detector.setPnPSolver(&angle_solver);
    ArmorFilter armor_filter(3);

    //rune detection
    const RuneParam &runeparm = this->_settings->_rune;
    RuneDector rune_detector(runeparm.sudoku_cell_width, runeparm.sudoku_cell_height, true,RuneDetector::RUNE_CANNY);
    RuneResFilter filter(runeparm.shoot_filter_size, runeparm.shoot_time_gap);


    //process loop
    const double offset_angle_x = 0;
    const double offset_angle_y = 0;

    cv::Mat frame;
    int frame_num = 30; //帧率

    int miss_detection_cnt = 0;
    int last_rune_idx = -1;
    int last_rune_timestamp = 0;
    bool flash_flag = false;

    cv::RotatedRect rect; //armor rect;
    std::pair<int, int> rune_result; //rune result;

    double send_data[3] = { 0 };
    double angle_x = 0.0, angle_y = 0.0;

    while(true)
    {
        while(prdIdx - csmIdx == 0);
        capturedata[csmIdx % BUFFER_SIZE].img.copyTo(frame);
        frame_num = capturedata[csmIdx % BUFFER_SIZE].frame;
        ++csmIdx;

        if (this->_settings->_mode == ARMOR_MODE)  //ARMOR detect mode
        {
            armor_detector.setPara(this->_settings->_armor);
            angle_slover_factory.setSolver(&angle_solver);
            if(frame.rows == 720)
            {
                this->_settings->_armor.max_light_delta_h = 700;
                this->_settings->_armor.min_light_height = 5;
                this->_settings->_armor.min_light_delta_h = 20;
                this->_settings->_armor.avg_contrast_threshold = 90;
            }
            armor_detector.setPitchAngle(this->_pitch_param->angle_pitch);
            //检测出来了矩形
            rect = armor_detector.getTargetArea(frame);

            //找到矩形中心并将数据转换成car可以直接使用的坐标
            bool is_small = armor_detector.isSmallArmor();
            AngleSolverFactory::TargetType type = is_small ? AngleSolverFactory::TARGET_SAMLL_ATMOR : AngleSolverFactory::TARGET_ARMOR;
            if (angle_slover_factory.getAngle(rect, type, angle_x, angle_y, _settings->_bullet_speed, _pitch_param->angle_pitch) == true) 
            {
                
            miss_detection_cnt = 0;
            // using history data to predict the motion
            predictor.setRecord(angle_x, frame_num);
            double z = angle_slover_factory.getSolver().position_in_camera.at<double>(2,0);
            double angle_x_predict = predictor.predict(frame_num + 1.0);

            send_data[0] = (angle_x_predict + offset_angle_x) * 100;   // send_X
            send_data[1] = (angle_y + offset_angle_y) * 100;           // send_y
            send_data[2] = 1;                                          // send_z

            // send data to car
            Serial::sendXYZ(_fd2car, send_data); // [anglex  angleY  1]
//测试阶段
#ifdef _SHOW_DETECT_
            std::cout << "Armor Type: " << (type == AngleSolverFactory::TARGET_ARMOR ? "Large\t" : "Small\t") << "Armor Size: " << rect.size << "\t";
            std::cout << "barrels angle:" << send_data[0] << ", " << send_data[1] << ", "  << send_data[2] << std::endl;
            Point2f vertices[4];
            rect.points(vertices);
            for (int i = 0; i < 4; i++)
                line(frame, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0));
#endif
            }
            else 
            {
                ++miss_detection_cnt;
                send_data[2] = 0;
                Serial::sendXYZ(_fd2car, send_data);  // [anglex  angleY  0]
            }
              // end armor detection mode
        }
        else if (this->_settings->_mode == RUNE_MODE)
        {
            //rune detect process
            angle_solver_factory.setSolver(&angle_solver);

            rune_result = rune_detector.getTarget(frame);  //pair 返回类型的数据

            //上面检测出来了结果
            if (filter.setRecord(rune_result.second) && filter.getResult())	
			{
                rect = rune_detector.getRect(rune_result.first);
				if (angle_slover_factory.getAngle(rect, AngleSolverFactory::TARGET_RUNE, angle_x, angle_y, _settings->_bullet_speed, _pitch_param->angle_pitch) == true) {
                    send_data[0] = (angle_x + offset_anlge_x) * 100;
                    send_data[1] = (angle_y + offset_anlge_y) * 100;
                    send_data[2] = rune_result.second + 1;

                    if (last_rune_idx != send_data[2]){
                        int cur_timestamp = cv::getTickCount();
                        if ((cur_timestamp - last_rune_timestamp)* 1000.0 / cv::getTickFrequency() < 500.0)  // More than 0.5 seconds to hit the next Rune
                            flash_flag = true;
                        else
                            flash_flag = false;
                        last_rune_timestamp = cur_timestamp;
                        last_rune_idx = send_data[2];
                    }
                    if(flash_flag == false)
                        Serial::sendXYZ(_fd2car, send_data); // [angleX  angleY  Index]
#ifdef _SHOW_DETECT_
					std::cout << "barrels angle:(" << send_data[0] << ", " << send_data[1] << ") - index: " << send_data[2] << std::endl;
					Point2f vertices[4];
					rect.points(vertices);
					for (int i = 0; i < 4; i++)
						cv::line(frame, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0));
#endif
                }
            }
            // count out of view range
            if (rune_result.first >=0){
                miss_detection_cnt = 0;
            }
            else {
                ++miss_detection_cnt;
                if (miss_detection_cnt >= 5)
                {
					send_data[2] = 0;
                    Serial::sendXYZ(_fd2car, send_data);  // [angleX  angleY 0] 
                }
            }
        } // end rune system

        }
        else
        {  //如果给出的指示都不是两种模式，那就进行各个指标的更改。
            miss_detection_cnt = 0;
            last_rune_idx = -1;
            last_rune_timestamp = 0;
            flash_flag = false;
            filter.clear();
            predictor.clear();
            armor_detector.reset();
        }

//测试程序
#ifdef _SHOW_DETECT_
        try
        {
            cv::imshow("frame", frame);
            cv::waitKey(delay_time);
        }
        catch(cv::Excepthins e)
        {
            std::cout << "error" << std::endl;
        }
#endif

    }

}