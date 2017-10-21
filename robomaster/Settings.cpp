#include "Settings.hpp"

Settings::Settings(const std::string & filename)
{
    _scale_z_480 = 1.0;
    cv::FileStorage setting_fs(filename, cv::FileStorage::READ);
    read(setting_fs);
    setting_fs.release();
}

void Settings::read(const cv::FileStorage& fs) {

    // for rune system
    fs["sudoku_cell_width"] >> _rune.sudoku_cell_width;
    fs["sudoku_cell_height"] >> _rune.sudoku_cell_height;
    fs["shoot_time_gap"] >> _rune.shoot_time_gap;
    fs["shoot_filter_size"] >> _rune.shoot_filter_size;

    // for armor system
    fs["min_light_gray"] >> _armor.min_light_gray;
    fs["min_light_height"] >> _armor.min_light_height;
    fs["avg_contrast_threshold"] >> _armor.avg_contrast_threshold;
    fs["light_slope_offset"] >> _armor.light_slope_offset;
    fs["max_light_delta_h"] >> _armor.max_light_delta_h;
    fs["min_light_delta_h"] >> _armor.min_light_delta_h;
    fs["max_light_delta_v"] >> _armor.max_light_delta_v;
    fs["max_light_delta_angle"] >> _armor.max_light_delta_angle;
    fs["avg_board_gray_threshold"] >> _armor.avg_board_gray_threshold;
    fs["avg_board_grad_threshold"] >> _armor.avg_board_grad_threshold;
    fs["grad_threshold"] >> _armor.grad_threshold;
    fs["br_threshold"] >> _armor.br_threshold;
    fs["enemy_color"] >> _armor.enemy_color;

    fs["min_detect_distance"] >> _min_detect_distance;
    fs["max_detect_distance"] >> _max_detect_distance;

    // for armor template
    fs["template_image_file"] >> _template_image_file;
    fs["small_template_image_file"] >> _small_template_image_file;

    // for system mode
    fs["mode"] >> _mode;

    fs["bullet_speed"] >> _bullet_speed;
    fs["scale_z_480"] >> _scale_z_480;
    check();
}

void Settings::check(){
    ArmorParam default_armor;
    if (_armor.min_light_gray < 5)
        _armor.min_light_gray = default_armor.min_light_gray;
    if (_armor.min_light_height < 5)
        _armor.min_light_height = default_armor.min_light_height;
    if (_armor.avg_contrast_threshold < 5)
        _armor.avg_contrast_threshold = default_armor.avg_contrast_threshold;
    if (_armor.light_slope_offset < 5)
        _armor.light_slope_offset = default_armor.light_slope_offset;
    if (_armor.max_light_delta_h < 5)
        _armor.max_light_delta_h = default_armor.max_light_delta_h;
    if (_armor.min_light_delta_h < 5)
        _armor.min_light_delta_h = default_armor.min_light_delta_h;
    if (_armor.max_light_delta_v < 5)
        _armor.max_light_delta_v = default_armor.max_light_delta_v;
    if (_armor.max_light_delta_angle < 5)
        _armor.max_light_delta_angle = default_armor.max_light_delta_angle;
    if (_armor.avg_board_gray_threshold < 5)
        _armor.avg_board_gray_threshold = default_armor.avg_board_gray_threshold;
    if (_armor.avg_board_grad_threshold < 5)
        _armor.avg_board_grad_threshold = default_armor.avg_board_grad_threshold;
    if (_armor.grad_threshold < 5)
        _armor.grad_threshold = default_armor.grad_threshold;
    if (_armor.br_threshold < 5)
        _armor.br_threshold = default_armor.br_threshold;

    if(_min_detect_distance < 10e-5)
        _min_detect_distance = 50.0;
    if(_max_detect_distance < 10e-5)
        _max_detect_distance = 600.0;

    RuneParam default_rune;
    if (_rune.sudoku_cell_width < 5)
        _rune.sudoku_cell_width = default_rune.sudoku_cell_width;
    if (_rune.sudoku_cell_height < 5)
        _rune.sudoku_cell_height = default_rune.sudoku_cell_height;
    if (_rune.shoot_time_gap < 5)
        _rune.shoot_time_gap = default_rune.shoot_time_gap;
    if (_rune.shoot_filter_size < 5)
        _rune.shoot_filter_size = default_rune.shoot_filter_size;

    if (_template_image_file.size() < 1)
        _template_image_file = "template.bmp";
    if (_small_template_image_file.size() < 1)
        _small_template_image_file = "small_template.bmp";

    if(_mode < 0)
        _mode = ARMOR_MODE;

    if(_bullet_speed < 10)
        _bullet_speed = 10;

    if(_scale_z_480 < 0.1)
        _scale_z_480 = 1.0;
}