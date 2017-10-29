#ifndef ImageConsProd_H
#define ImageConsProd_H
#include "Settings.hpp"
class ImageConsProd {
public:
    ImageConsProd(Settings * settings, Angle_Pitch * pitch_param, int fd_car){
        _settings = settings;
        _pitch_param = pitch_param;
        _fd2car = fd_car;
    }
    void ImageProducer();
    void ImageConsumer();

public:
    Settings * _settings;
    Angle_Pitch * _pitch_param;
    int _fd2car;
};

#endif