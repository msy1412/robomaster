#include "Settings.hpp"
#include "Serial.hpp"
#include "ImageConsProd.hpp"
#include <thread>
#include <unistd.h>
#include <iostream>

int main(int argc, char * argv[])
{
    
        const char * config_file_name = ".//param_config.xml"; 
        if (argc > 1)
            config_file_name = argv[1];
    
        Settings setting(config_file_name);
        Angle_Pitch pitch_param;
        
        int fd2car = Serial::openPort("/dev/ttyrobomaster");
        Serial::configurePort(fd2car);
     
        ImageConsProd image_cons_prod(&setting, &pitch_param, fd2car);
        std::thread task0(&ImageConsProd::ImageProducer, image_cons_prod); 
        std::thread task1(&ImageConsProd::ImageConsumer, image_cons_prod);
    
        //RemoteController controller(&setting, &pitch_param, fd2car);
        //std::thread task2(&RemoteController::paraReceiver, controller);
    
        task0.join();
        task1.join();
        //task2.join();  
        close(fd2car);
    
        return EXIT_SUCCESS;
    }