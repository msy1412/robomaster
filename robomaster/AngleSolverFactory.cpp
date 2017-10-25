#include "AngleSolverFactory.hpp"
#include <iostream>
void AngleSolverFactory::setTargetSize(double width, double height, TargetType type)
{
    if(type == TARGET_RUNE){
        rune_width = width;
        rune_height = height;
    }
    else if(type == TARGET_ARMOR){
        armor_width = width;
        armor_height = height;
    }
    else if(type == TARGET_SAMLL_ATMOR){
        small_armor_width = width;
        small_armor_height = height;
    }
}

bool AngleSolverFactory::getAngle(const cv::RotatedRect & rect, TargetType type, double & angle_x, double & angle_y, double bullet_speed, double current_ptz_angle, const cv::Point2f & offset)
{
    if(solver == NULL){
        std::cerr << "slover not set\n";
        return false;
    }

    double width = 0.0, height = 0.0;

    if(type == TARGET_RUNE){
        width = rune_width;
        height = rune_height;
    }
    else if(type == TARGET_ARMOR){
        width = armor_width;
        height = armor_height;
    }
    else if(type == TARGET_SAMLL_ATMOR){
        width = small_armor_width;
        height = small_armor_height;
    }
    cv::RotatedRect rect_rectifid = rect;  // target of rotated_erct
    AngleSolverFactory::adjustRect2FixedRatio(rect_rectifid, width/height);
    solver->setTargetSize(width, height);

    return solver->getAngle(rect_rectifid, angle_x, angle_y, bullet_speed, current_ptz_angle, offset);
}
