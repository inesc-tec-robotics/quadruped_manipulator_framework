#include "igus_rviz_panel/igus_rviz_panel.hpp"
#include <pluginlib/class_list_macros.hpp>


PLUGINLIB_EXPORT_CLASS(igus_rviz_panel::IgusRvizPanel, rviz::Panel)

namespace igus_rviz_panel
{
    IgusRvizPanel::IgusRvizPanel(QWidget * parent)
    :   rviz::Panel(parent),
        ui_(std::make_shared<Ui::igus_rviz_panel>()),
        igus_arm_client_("igus_arm_driver_skill", true)
    {
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);

        connect(ui_->button_go, SIGNAL(clicked()), this, SLOT(executeMove()));

        igus_arm_client_.waitForServer();
    }

    void IgusRvizPanel::executeMove(){
        std::stringstream command;
        geometry_msgs::Point goal;
        std::vector<double> goal_joints;
        double speed, acel;
        motionType move_type;

        if (ui_->text_goal1->text().isEmpty() ||
            ui_->text_goal2->text().isEmpty() ||
            ui_->text_goal3->text().isEmpty() ||
            ui_->text_speed->text().isEmpty() ||
            ui_->text_acel->text().isEmpty()) 
        {
            ROS_WARN("Input Error: All input fields must be filled.");
            return;
        }

        move_type = motionTypeToString(ui_->select_move_type->currentText().toStdString());

        goal.x = ui_->text_goal1->text().toFloat();
        goal.y = ui_->text_goal2->text().toFloat();
        goal.z = ui_->text_goal3->text().toFloat();

        goal_joints.push_back(ui_->text_goal1->text().toFloat());
        goal_joints.push_back(ui_->text_goal2->text().toFloat());
        goal_joints.push_back(ui_->text_goal3->text().toFloat());

        speed = ui_->text_speed->text().toDouble();
        acel = ui_->text_acel->text().toDouble();

        switch (move_type)
        {
        case motionType::kMoveJ:
            command << "movej(p={" << goal.x << "," << goal.y << "," << goal.z <<"},v=" << speed << ",a=" << acel << ",b=0);";
            break;

        case motionType::kMoveJJoints:

            command << "movejj(p={";
            
            for (int i = 0; i < goal_joints.size(); i++)
            {
                command << goal_joints.at(i);

                if(i != goal_joints.size()-1){
                    command << ",";
                }
            }
            
            command <<"},v=" << speed << ",a=" << acel << ",b=0);";
            break;

        case motionType::kMoveL:
            command << "movel(p={" << goal.x << "," << goal.y << "," << goal.z <<"},v=" << speed << ",a=" << acel << ",b=0);";
            break;

        case motionType::kMoveLJoints:
            command << "movelj(p={";
            
            for (int i = 0; i < goal_joints.size(); i++)
            {
                command << goal_joints.at(i);

                if(i != goal_joints.size()-1){
                    command << ",";
                }
            }
            
            command <<"},v=" << speed << ",a=" << acel << ",b=0);";
            break;

        default:
            break;
        }

        igus_arm_driver::IgusCommandGoal goal_as;
        goal_as.target = command.str();
        igus_arm_client_.sendGoal(goal_as);
    }

    motionType IgusRvizPanel::motionTypeToString(std::string move_text){
        if(!move_text.compare("MoveJ")){
            return motionType::kMoveJ;
        }
        else if(!move_text.compare("MoveJJoints")){
            return motionType::kMoveJJoints;
        }
        else if(!move_text.compare("MoveL")){
            return motionType::kMoveL;
        }
        else if(!move_text.compare("MoveLJoints")){
            return motionType::kMoveLJoints;
        }
    }

} // namespace rviz_panel
