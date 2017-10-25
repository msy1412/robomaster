#include <opencv2/core.hpp>

class RectPnPSolver
{
public:
    RectPnPSolver(const cv::Mat &camera_matrix, const cv::Mat &dist_coeff,
                  double target_width = 0, double target_height = 0)
                  {
                     camera_matrix.copyTo(this->cam_matrix);
                     dist_coeff.copyTo(this->distortion_coeff);
                     this->width_target = target_width;
                     this->height_target = target_height; 
                  }
    void setTargetSize(double width, double height)
    {
        this->width_target = width;
        this->height_target = height;
    }

    void setCameraParam(const cv::Mat & camera_matrix, const cv::Mat & dist_coeff){ 
        camera_matrix.copyTo(this->cam_matrix);
        dist_coeff.copyTo(this->distortion_coeff);
    }

    /**
     * @brief solvePnP
     * @param vector<Point_2D> : left_up, right_up, left_down, right_down
     * @param rot rotation between camera and target center
     * @param trans tanslation between camera and target center
     */
    void solvePnP4Points(const std::vector<cv::Point2f> & points2d, cv::Mat & rot, cv::Mat & trans);
public:
    cv::Mat cam_matrix;
    cv::Mat distortion_coeff;
    double width_target;
    double height_target;
};

class AngleSolver : public RectPnPSolver
{
public:
    AngleSolver(const cv::Mat & camera_matrix, const cv::Mat & dist_coeff,
        double target_width = 0, double target_height = 0, double z_scale = 1.0,
        double min_dist = 50.0, double max_dist = 600.0);
    void setScaleZ(double scale) {this->scale_z = scale;}
    void setRelationPoseCameraPTZ(const cv::Mat & rot_camera_ptz, const cv::Mat & trans_camera_ptz, double y_offset_barrel_ptz);
    void getTarget2dPoinstion(const cv::RotatedRect & rect, std::vector<cv::Point2f> & target2d, const cv::Point2f & offset);
    bool getAngle(const cv::RotatedRect &rect, double &angle_x, double & angle_y, double bullet_speed = 0, double current_ptz_angle = 0.0, const cv::Point2f & offset = cv::Point2f());
    void tranformationCamera2PTZ(const cv::Mat & pos, cv::Mat & transed_pos);
    void adjustPTZ2Barrel(const cv::Mat & pos_in_ptz, double & angle_x, double & angle_y, double bullet_speed = 0.0, double current_ptz_angle = 0.0);

public:
    cv::Mat position_in_camera;
    cv::Mat position_in_ptz;

private:
    cv::Mat trans_camera2ptz;
    cv::Mat rot_camera2ptz;

    // offset between barrel and ptz on y axis (cm)
    double offset_y_barrel_ptz;

    // scope of detection distance (cm)
    double min_distance;
    double max_distance;
    double scale_z;
};