#include "ParametersSet.hpp"
#include <opencv2/core.hpp>
#include <string>

#define ARMOR_MODE 0 
#define RUNE_MODE 1
#define STOP_MODE 2
class Settings { 
public:
    Settings(const std::string & filename);
    void read(const cv::FileStorage& fs);
    void check();

    
public:
	RuneParam _rune;
	ArmorParam _armor;
    int _mode;
    std::string _template_image_file; 
    std::string _small_template_image_file;
    double _min_detect_distance;
    double _max_detect_distance;
    double _bullet_speed;
    double _scale_z_480;    // has issue
};
