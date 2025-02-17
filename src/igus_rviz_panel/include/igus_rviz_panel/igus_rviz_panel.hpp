#ifndef rviz_panel_H_
#define rviz_panel_H_

#include <ros/ros.h>
#include <rviz/panel.h>

/** 
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */
#include <ui_igus_rviz_panel.h>

// Other ROS dependencies
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <actionlib/client/simple_action_client.h>
#include "igus_arm_driver/IgusCommandAction.h" 

typedef actionlib::SimpleActionClient<igus_arm_driver::IgusCommandAction> IgusArmClient;

namespace igus_rviz_panel
{   
    enum class motionType{
        kMoveJ,
        kMoveJJoints,
        kMoveL,
        kMoveLJoints
    };
    /**
     *  Here we declare our new subclass of rviz::Panel. Every panel which
     *  can be added via the Panels/Add_New_Panel menu is a subclass of
     *  rviz::Panel.
     */

    class IgusRvizPanel : public rviz::Panel
    {
        /**
         * This class uses Qt slots and is a subclass of QObject, so it needs
         * the Q_OBJECT macro.
         */
        Q_OBJECT

        public:
            /**
             *  QWidget subclass constructors usually take a parent widget
             *  parameter (which usually defaults to 0).  At the same time,
             *  pluginlib::ClassLoader creates instances by calling the default
             *  constructor (with no arguments). Taking the parameter and giving
             *  a default of 0 lets the default constructor work and also lets
             *  someone using the class for something else to pass in a parent
             *  widget as they normally would with Qt.
             */
            IgusRvizPanel(QWidget * parent = 0);

        /**
         *  Next come a couple of public Qt Slots.
         */
        public Q_SLOTS:

        /**
         *  Here we declare some internal slots.
         */
        private Q_SLOTS:
            void executeMove();
            motionType motionTypeToString(std::string move_text);
            
        /**
         *  Finally, we close up with protected member variables
         */
        protected:
            // UI pointer
            std::shared_ptr<Ui::igus_rviz_panel> ui_;
            // ROS declaration
            ros::NodeHandle nh_;
            IgusArmClient igus_arm_client_;

    };
} // namespace rviz_panel

#endif