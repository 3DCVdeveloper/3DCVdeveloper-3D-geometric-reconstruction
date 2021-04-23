#include "timestamp.h"

timeStamp::timeStamp(){

}
//返回时间戳
std::string timeStamp::getTimestamp(){
    gettimeofday(&t,NULL);
    double result = static_cast<double>(t.tv_sec)+static_cast<double>(t.tv_usec)/1000000;
    this->m_timestamp = std::to_string(result);
    return std::to_string(result);
}

/**color：要保存的rgb图
 * depth：要保存的深度图
 */
void timeStamp::image_save(const cv::Mat &color,const cv::Mat &depth){
    std::string color_filename = "rgb/"+this->m_timestamp+".png";
    std::string depth_filename = "depth/"+this->m_timestamp+".png";

    cv::imwrite(color_filename,color);
    cv::imwrite(depth_filename,depth);

    generateText(color_filename,"rgb.txt");
    generateText(depth_filename,"depth.txt");
}
/**filename：将要保存的rgb图或深度图的路径
 * txtname ：要写入的文本名，如rgb.txt  depth.txt
 */
void timeStamp::generateText(std::string filename,std::string txtname){
    std::string row = this->m_timestamp+" "+filename;
    std::ofstream fout;
    fout.open(txtname,std::ios::app);
    fout<<row<<std::endl;
    fout.close();
}

void timeStamp::run(const cv::Mat &color,const cv::Mat &depth){
    //创建两个目录rgb和depth,若没有就创建，若存在则不创建
    int isRgb = mkdir("rgb", S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    int isDepth = mkdir("depth", S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    this->getTimestamp();
    this->image_save(color,depth);
}