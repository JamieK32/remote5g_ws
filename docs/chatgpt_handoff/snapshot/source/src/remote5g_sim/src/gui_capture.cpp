// Optional validation helper: saves the actual gzclient viewport, not a synthetic model render.
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <QTimer>
#include <cstdlib>
#include <string>
#include <iostream>
namespace remote5g {
class GuiCapture final : public gazebo::GUIPlugin {
  QTimer timer_;
  int count_=0;
public:
  GuiCapture() {
    setVisible(false);
    QObject::connect(&timer_,&QTimer::timeout,[this]() {
      const char *dir=std::getenv("XTNETRC_GUI_CAPTURE");
      auto camera=gazebo::gui::get_active_camera();
      if(!dir || !camera || !camera->GetScene()->GetVisual("xtnetrc")) return;
      auto pose=camera->GetScene()->GetVisual("xtnetrc")->WorldPose();
      auto pos=pose.Pos()+pose.Rot().RotateVector(ignition::math::Vector3d(0.819,-0.70,0.75));
      camera->SetWorldPose(ignition::math::Pose3d(pos,ignition::math::Quaterniond(0,0.54,pose.Rot().Yaw()+2.356)));
      if(count_>=4) {
        auto top=pose.Pos()+ignition::math::Vector3d(0.119,0,0.85);
        camera->SetWorldPose(ignition::math::Pose3d(top,ignition::math::Quaterniond(0,1.57079632679,pose.Rot().Yaw())));
      }
      if(count_==0) {
        camera->SetCaptureData(true); ++count_; return;
      }
      if(count_<=8) {
        std::string file=std::string(dir)+"/gui_view_"+std::to_string(count_)+".png";
        std::cout<<"Gazebo viewport capture "<<file<<" success="<<camera->SaveFrame(file)<<std::endl;
        ++count_;
      } else timer_.stop();
    });
    timer_.start(2000);
  }
};
GZ_REGISTER_GUI_PLUGIN(GuiCapture)
}
