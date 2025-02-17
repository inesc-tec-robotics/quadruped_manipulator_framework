#pragma once

#include <sys/socket.h>
#include <linux/can.h>
#include <string>
#include <iostream>
#include <net/if.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <thread>
#include <cmath>

#define _USE_MATH_DEFINES

namespace igus_arm_driver{

    class CanInterface{
      private:
        int socket_num;
        bool is_connected;
        std::string can_port;
        int read_timeout;
        const int read_tries = 4;

      public:
        CanInterface(std::string interface_name);
        ~CanInterface();

        void connect();
        void disconnect();
        void writeMessage(const struct can_frame&);
        can_frame readMessage();
        bool isConnected();

        //cyclical messages
        void velocityMsg(canid_t id, int vel, int message_conter); 

        //non-cyclical messages
        void resetErrors(canid_t id);
        void enableMotor(canid_t id);
        void disableMotor(canid_t id);
        void pingModule(canid_t id);

    };
}