#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <string>

class timeStamp
{
public:

    explicit timeStamp();
    //~timeStamp();
    std::string getTimestamp();
    void run(const cv::Mat &color,const cv::Mat &depth);
    
protected:
    
    void image_save(const cv::Mat &color,const cv::Mat &depth);
    void generateText(std::string filename,std::string txtname);

    struct timeval t;
    std::string m_timestamp;
};

#endif