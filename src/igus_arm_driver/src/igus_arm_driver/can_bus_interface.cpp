#include "igus_arm_driver/can_bus_interface.h"

namespace igus_arm_driver
{
  CanInterface::CanInterface(std::string interface_name) : can_port(interface_name){
    socket_num = -1;
    is_connected = false;
    read_timeout = 50;
  }

  CanInterface::~CanInterface(){
  }

  void CanInterface::connect(){
    socket_num = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (socket_num < 0) {
        throw "CANBUS: Failed to create socket.";
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_port.c_str());
    int errcode = ioctl(socket_num, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    errcode = bind(socket_num, (struct sockaddr*) & addr, sizeof(addr));
    if (errcode == -1)
    {
      throw "CANBUS: Failed to bind to the socket.";
    }

    is_connected = true;
  }

  void CanInterface::disconnect(){
    if (socket_num == -1){
      throw "CANBUS: Attempting to disconnect non-existent socket.";
    }

    int errcode = close(socket_num);
    if (errcode == -1){
      throw "CANBUS: Failed to close socket.";
    }

    socket_num = -1;
    is_connected = false;

  }

  void CanInterface::writeMessage(const struct can_frame& frame){
    int nbytes = write(socket_num, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        throw "Failed to write CAN message.";
    }

    std::this_thread::sleep_for (std::chrono::milliseconds(5));
  }

  can_frame CanInterface::readMessage(){
    struct can_frame rcv_frame;

    int bytes_read = read(socket_num, &rcv_frame, sizeof(struct can_frame));

    if(bytes_read >= 0){
      return rcv_frame;
    }
    else{
      throw strerror(errno);
    }
    
  }

  bool CanInterface::isConnected(){
    return is_connected;
  }

  void CanInterface::velocityMsg(canid_t id, int vel, int message_conter){
    struct can_frame frame;

    frame.can_id = id;
    frame.can_dlc = 4;
    frame.data[0] = 0x25;
    frame.data[1] = (uint8_t)((vel >> 8) & 0xFF);
    frame.data[2] = (uint8_t)(vel & 0xFF);
    frame.data[3] = (uint8_t)(message_conter & 0xFF);
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    this->writeMessage(frame);
  }

  void CanInterface::resetErrors(canid_t id){
    try{
      struct can_frame frame, rcv_msg;
      frame.can_id = id;
      frame.can_dlc = 2;
      frame.data[0] = 0x01;
      frame.data[1] = 0x06;
      frame.data[2] = 0x00;
      frame.data[3] = 0x00;
      frame.data[4] = 0x00;
      frame.data[5] = 0x00;
      frame.data[6] = 0x00;
      frame.data[7] = 0x00;
      this->writeMessage(frame);
    }
    catch(const char *msg){
      throw msg;
    }
  }

  void CanInterface::enableMotor(canid_t id){
    try{
      struct can_frame frame, rcv_msg;
      frame.can_id = id;
      frame.can_dlc = 2;
      frame.data[0] = 0x01;
      frame.data[1] = 0x09;
      frame.data[2] = 0x00;
      frame.data[3] = 0x00;
      frame.data[4] = 0x00;
      frame.data[5] = 0x00;
      frame.data[6] = 0x00;
      frame.data[7] = 0x00;
      this->writeMessage(frame);
    }
    catch(const char *msg){
      throw msg;
    }
  }

  void CanInterface::disableMotor(canid_t id){
    try{
      struct can_frame frame, rcv_msg;
      frame.can_id = id;
      frame.can_dlc = 2;
      frame.data[0] = 0x01;
      frame.data[1] = 0x0a;
      frame.data[2] = 0x00;
      frame.data[3] = 0x00;
      frame.data[4] = 0x00;
      frame.data[5] = 0x00;
      frame.data[6] = 0x00;
      frame.data[7] = 0x00;
      this->writeMessage(frame);
    }
    catch(const char *msg){
      throw msg;
    }
    
  }

  void CanInterface::pingModule(canid_t id){
    try{
      struct can_frame frame, rcv_msg;
      frame.can_id = id;
      frame.can_dlc = 2;
      frame.data[0] = 0x01;
      frame.data[1] = 0xcc;
      frame.data[2] = 0x00;
      frame.data[3] = 0x00;
      frame.data[4] = 0x00;
      frame.data[5] = 0x00;
      frame.data[6] = 0x00;
      frame.data[7] = 0x00;
      this->writeMessage(frame);

      rcv_msg = readMessage();

    }
    catch(const char *msg){
      throw msg;
    }
  }  
} // namespace arm_hardware
