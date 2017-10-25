#include <opencv2/core.hpp>
#include "AngleSolver.hpp"

class AngleSolverFactory
{
public:
    typedef enum {TARGET_RUNE, TARGET_ARMOR, TARGET_SAMLL_ATMOR} TargetType;
    AngleSolverFactory(AngleSolver * angle_solver = NULL): solver(angle_solver){}
    void setSolver(AngleSolver * angle_slover)
    {
        this->solver = angle_slover;
    }

    AngleSolver & getSolver()
    {
        return *this->solver;
    }

    void setTargetSize(double width, double height, TargetType type);

    void adjustRect2FixedRatio(cv::RotatedRect & rect, double wh_ratio)
    {
        rect.size.height = rect.size.width / wh_ratio;
    }

    bool getAngle(const cv::RotatedRect & rect, TargetType type, double & angle_x, double & angle_y, double bullet_speed, double current_ptz_angle, const cv::Point2f & offset = cv::Point2f());
private:
    double armor_width;
    double armor_height;
    double small_armor_width;
    double small_armor_height;
    double rune_width;
    double rune_height;
    AngleSolver * solver;
};