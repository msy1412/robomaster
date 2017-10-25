#include "AngleSolver.hpp"
#include <opencv2/highgui.hpp>
#include "ParametersSet.hpp"

class ArmorDetector
{
public:
    ArmorDetector(const ArmorParam & para = ArmorParam());

    void setPara(const ArmorParam &param) { this->_para = param;}

    void setPnPSolver(AngleSolver *solver){ this->m_solver = solver; }

    void setPitchAngle(double angle){this->pitch_angle = angle;}

    void setLastResult(const cv::RotatedRect &rect){ this->_res_last = rect; }

    void initTemplate(const cv::Mat &_template, const cv::Mat &_templatee_small);

    void reset();

    bool isSmallArmor(){return this->_is_small_armor;}

    const cv::RotatedRect &getLastResult() const { return this->_res_last; }

    cv::RotatedRect getTargetArea(const cv::Mat &src);

private:
    void setImage(const cv::Mat &src);

    int templateDist(const cv::Mat &img, bool is_small);

    void findContourInEnemyColor(
        cv::Mat & left, cv::Mat & right,
        std::vector<std::vector<cv::Point2i> > &contours_left,
        std::vector<std::vector<cv::Point2i> > &contours_right);

    void findTargetInContours(
        const std::vector<std::vector<cv::Point> > & contours_left,
        const std::vector<std::vector<cv::Point> > & contours_right,
        std::vector<cv::RotatedRect> & rects,
        std::vector<double> & score);    

        cv::RotatedRect chooseTarget(const std::vector<cv::RotatedRect> & rects, const std::vector<double> & score);

        cv::RotatedRect boundingRRect(const cv::RotatedRect & left, const cv::RotatedRect & right);

        cv::RotatedRect adjustRRect(const cv::RotatedRect & rect);

        bool makeRectSafe(cv::Rect & rect, cv::Size size);
private:
    AngleSolver *m_solver;
    double pitch_angle;
    bool _is_lost;
    int _lost_cnt;
    bool _is_small_armor;
    cv::RotatedRect _res_last;
    cv::Rect _dect_rect;
    ArmorParam _para;
    cv::Mat _binary_template;
    cv::Mat _binary_template_small;
    cv::Mat _src;
    cv::Mat _g;
    cv::Mat _ec;
    cv::Mat _max_color;
    cv::Size _size;
};
