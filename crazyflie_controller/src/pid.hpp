#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/Float32MultiArray.h>

class PID
{
public:
    PID(
        float kp,
        float kd,
        float ki,
        float minOutput,
        float maxOutput,
        float integratorMin,
        float integratorMax,
        const std::string& name)
        : m_kp(kp)
        , m_kd(kd)
        , m_ki(ki)
        , m_minOutput(minOutput)
        , m_maxOutput(maxOutput)
        , m_integratorMin(integratorMin)
        , m_integratorMax(integratorMax)
        , m_integral(0)
        , m_previousError(0)
        , m_previousTime(ros::Time::now())
    {
        // Init handlers and publishers for PID, error and output:
        ros::NodeHandle nh;
        std::string pid_name = "PID_debug_"+name+"_error";
        std::cout << ""+pid_name+"initiated\n";
        pid_debug_publisher = nh.advertise<std_msgs::Float32MultiArray>(pid_name, 10);   // build publisher name by its object name!!! (e.g if object name is pidX, the publisher is pidX_debug
                                                                                            // print pid vector format {output, error, p, d, i}
    }

    void reset()
    {
        m_integral = 0;
        m_previousError = 0;
        m_previousTime = ros::Time::now();
    }

    void setIntegral(float integral)
    {
        m_integral = integral;
    }

    float ki() const
    {
        return m_ki;
    }

    float update(float value, float targetValue)
    {
        ros::Time time = ros::Time::now();
        float dt = time.toSec() - m_previousTime.toSec();
        float error = targetValue - value;
        m_integral += error * dt;
        m_integral = std::max(std::min(m_integral, m_integratorMax), m_integratorMin);
        float p = m_kp * error;
        float d = 0;

        // PID topic preparation
        //std_msgs::Float32 output_message; // single Float32 ros topic message
        std_msgs::Float32MultiArray output_message_array;   // MulltiArray of type float32
        //Clear array
		output_message_array.data.clear();


        if (dt > 0)
        {
            // update 'd' only if positive, to prevent "dancing" around the desired value.
            d = m_kd * (error - m_previousError) / dt;
        }

        float i = m_ki * m_integral;
        float output = p + d + i;
        m_previousError = error;
        m_previousTime = time;

        // ros_topic all PID's data per axe
        //output_message_array.data.push_back(output);
        output_message_array.data.push_back(error);
        //output_message_array.data.push_back(p);
        //output_message_array.data.push_back(d);
        //output_message_array.data.push_back(i);

        pid_debug_publisher.publish(output_message_array);
//         self.pubError.publish(error)
//         self.pubP.publish(p)
//         self.pubD.publish(d)
//         self.pubI.publish(i)
        return std::max(std::min(output, m_maxOutput), m_minOutput);
    }

private:
    float m_kp;
    float m_kd;
    float m_ki;
    float m_minOutput;
    float m_maxOutput;
    float m_integratorMin;
    float m_integratorMax;
    float m_integral;
    float m_previousError;
    ros::Time m_previousTime;
    ros::Publisher pid_debug_publisher;
};
