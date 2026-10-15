#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/rendering/Scene.hh>
#include <QTimer>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <cstdlib>
#include <iostream>
#include <vector>
namespace competition {
class Capture final: public gazebo::GUIPlugin {
 QTimer timer_;int view_=0;bool placed_=false;
 struct View {std::string name;double x,y,z,pitch,yaw;};std::vector<View> views_;
public:
 Capture() {
  setVisible(false);const char* file=std::getenv("COMPETITION_LAYOUT");if(!file)return;
  QFile input(file);if(!input.open(QIODevice::ReadOnly))return;
  auto root=QJsonDocument::fromJson(input.readAll()).object();auto rows=root["coordinates"].toArray();
  auto find=[&](const QString& name){for(auto row:rows)if(row.toObject()["model_name"].toString()==name)return row.toObject();return QJsonObject();};
  auto addtop=[&](std::string name,const QJsonObject &row,double height){views_.push_back({name,row["x"].toDouble(),row["y"].toDouble(),height,1.57079632679,1.57079632679});};
  views_.push_back({"overview",0,0,260,1.57079632679,1.57079632679});
  addtop("start",find("comp_start_zone"),7);addtop("parking",find("comp_parking_zone"),6);
  auto qr=find("comp_task2_qr");double yaw=qr["yaw"].toDouble();views_.push_back({"qr_stand",qr["x"].toDouble()-.6*cos(yaw),qr["y"].toDouble()-.6*sin(yaw),.38,.388,yaw});
  addtop("crosswalk",find("comp_crosswalk"),5);
  auto first=find("comp_lane_change_sign");QJsonObject last;for(auto row:rows)if(row.toObject()["model_name"].toString().startsWith("comp_guide_yellow_left_"))last=row.toObject();
  auto mid=first;mid["x"]=(first["x"].toDouble()+last["x"].toDouble())/2;mid["y"]=first["y"].toDouble();addtop("guide_cones",mid,13);
  addtop("bend_obstacle",find("comp_obstacle"),2.4);
  auto obstacle=find("comp_obstacle");double obstacleYaw=obstacle["yaw"].toDouble();
  views_.push_back({"bend_obstacle_close",obstacle["x"].toDouble()-.55*cos(obstacleYaw),obstacle["y"].toDouble()-.55*sin(obstacleYaw),.34,.5,obstacleYaw});
  // Leave the visible GUI on the four-row layout after taking the close-up.
  views_.push_back(views_[5]);
  QObject::connect(&timer_,&QTimer::timeout,[this](){auto camera=gazebo::gui::get_active_camera();const char*dir=std::getenv("COMPETITION_CAPTURE_DIR");if(!dir||!camera||!camera->GetScene()->GetVisual("comp_start_zone"))return;
   if(view_>=static_cast<int>(views_.size())){timer_.stop();return;}auto v=views_[view_];
   if(!placed_){camera->SetCaptureData(true);camera->SetViewController("fps");camera->SetWorldPose(ignition::math::Pose3d(v.x,v.y,v.z,0,v.pitch,v.yaw));placed_=true;return;}
   auto path=std::string(dir)+"/"+v.name+".png";std::cout<<"Competition actual GUI capture "<<path<<" success="<<camera->SaveFrame(path)<<std::endl;placed_=false;++view_;
  });timer_.start(2000);
 }
};GZ_REGISTER_GUI_PLUGIN(Capture)
}
