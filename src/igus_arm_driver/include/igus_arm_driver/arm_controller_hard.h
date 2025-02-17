#include "igus_arm_driver/can_bus_interface.h"
#include "igus_arm_driver/arm_controller.h"
#include <bitset>

namespace igus_arm_driver{

  enum class states{
    kInit,
    kError,
    kEnabling,
    kEnabled,
    kDisabling,
    kDisabled,
  };

  struct joint_info{
    uint8_t can_id;
    int message_counter;
    int error_state;
    states state;
    int gr; 
    int tpr;
    float offset;
    float orientation;
  };

  
  class HardArmController : public ArmController{
    private:
      ros::NodeHandle nh_;
      CanInterface can_bus_;
      std::thread read_thread_, write_thread_;
      std::vector<joint_info> joint_params_;
      ros::Publisher joint_states_pub_;
      std::mutex mutex;
      bool check_joint_status = false;
      std::array<bool,3> enable_joint_ = {true, true, true};
      std::array<bool,3> disable_joint_ = {true, true, true};

      const int timeout = 2;

      void errorHandling(int error_state, int joint);
      float ticks2rad(int32_t ticks, int joint);
      double convert2JointSpeed(double speed_rads, int joint_id);
      void timerWatchdog(const ros::TimerEvent& event);
      void checkArmStatus();

    public:
      HardArmController(ros::NodeHandle &nh);
      ~HardArmController();

      void getParameters();
      void publishJointStates();
      void updateJointStatus(bool enabled) override;
      void writeThread();
      void readThread();
      
  };
}