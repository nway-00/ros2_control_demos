#include "originbot_comms.hpp"

OriginBotComms::OriginBotComms()
{
    //TODO
}

OriginBotComms::OriginBotComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{

}

OriginBotComms::~OriginBotComms()
{

}

void OriginBotComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
    //TODO
    //serial_conn_.setPort(serial_device);
    //serial_conn_.setBaudrate(baud_rate);
    //serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    //serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    //serial_conn_.open();
    //// serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));

}

bool OriginBotComms::connected()
{
    //TODO
    return false;
}

void OriginBotComms::volocity_control(float x_linear, float z_angular)
{
    //TODO
}

bool OriginBotComms::buzzer_control(bool on)
{
    //TODO

}

bool OriginBotComms::led_control(bool on)
{
    //TODO

}

bool OriginBotComms::left_pid_control(short motor_p, short motor_i, short motor_d)
{
    //TODO

}

bool OriginBotComms::right_pid_control(short motor_p, short motor_i, short motor_d)
{
    //TODO

}

void OriginBotComms::get_volocity(float &left_vel, float &right_vel)
{
    //TODO
}
