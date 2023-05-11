#ifndef ORIGINBOTCOMMS_HPP
#define ORIGINBOTCOMMS_HPP

//#include <serial/serial.h>
#include <string>

class OriginBotComms
{
public:
    OriginBotComms();
    OriginBotComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
    ~OriginBotComms();

    void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);

    bool connected();

    void volocity_control(float x_linear, float z_angular);              // 速度控制
    bool buzzer_control(bool on);                                        // 蜂鸣器控制
    bool led_control(bool on);                                           // led灯控制
    bool left_pid_control(short motor_p, short motor_i, short motor_d);  // 左轮 pid 控制
    bool right_pid_control(short motor_p, short motor_i, short motor_d); // 右轮 pid 控制

    void get_volocity(float &left_vel, float &right_vel);  // 读取左轮速度，右轮速度

private:
    void readRawData();  //读取原始数据
    void calSumCheck(); // 计算校验和
private:
    //serial::Serial serial_conn_;
};

#endif // !ORIGINBOTCOMMS_HPP