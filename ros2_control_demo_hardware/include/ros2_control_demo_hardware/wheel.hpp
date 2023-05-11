#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <string>


class Wheel
{
public:
    std::string name = "";  // joint 名称
    int enc = 0;     // 码数
    double cmd = 0;  // 命令
    
    double pos = 0;  // ???位置
    double vel = 0;  // 速度

    double eff = 0;  // ???力矩
    double velSetPt = 0;
    double rads_per_count = 0;  // 每刻度的弧度数

    Wheel() = default;
    Wheel(const std::string &wheel_name, int counts_per_rev);

    void setup(const std::string &wheel_name, int counts_per_rev = 1);  // counts_per_rev 每转刻度数

    double calcEncAngle();
};

#endif // WHEEL_HPP