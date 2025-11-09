#include "PID.h"

PID::PID(float kp, float ki, float kd, float sampling_time, float min_limit, float max_limit)
{
    this->_kp = kp;
    this->_ki = ki;
    this->_kd = kd;
    this->_T = sampling_time;

    this->_min_pid = min_limit;
    this->_max_pid = max_limit;
}

PID::~PID()
{
}

float PID::pid_out(float error)
{
    float pid_out = _pid_out_1prev + (_kp + (_kd/_T)) * error
                    + (-_kp + _ki*_T - 2 * (_kd/_T)) * _error_1prev + (_kd/_T) * _error_2prev;
    _pid_out_1prev = pid_out;
    _error_1prev = error;
    _error_2prev = _error_1prev;

    // saturation
    if (pid_out > this->_max_pid)
    {
        pid_out = this->_max_pid;
    } else if (pid_out < this->_min_pid)
    {
        pid_out = this->_min_pid;
    }

    return pid_out;
}

void PID::reset()
{
    this->_error_1prev = 0.0f;
    this->_error_2prev = 0.0f;
    this->_error_current = 0.0f;
    this->_pid_out_1prev = 0.0f;
}
