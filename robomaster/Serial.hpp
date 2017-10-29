#ifndef Serial_H
#define Serial_H
class Serial
{
public:
    enum DATATYPE { IMAGE, INFO};
    static int openPort(const char * dev_name);
    static int configurePort(int fd);
    static bool sendXYZ(int fd, double * xyz);
    static bool sendData(int fd, char * data, int size, DATATYPE data_tpye);
};
#endif