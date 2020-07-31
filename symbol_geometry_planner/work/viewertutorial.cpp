#include <cnoid/corba/OpenHRP/3.1/OnlineViewer.hh>

#include <cnoid/CorbaUtil>
#include <cnoid/ExecutablePath>
#include <cnoid/BodyLoader>
#include <cnoid/Body>
#include <cnoid/Sleep>
#include <cnoid/FileUtil>
#include <iostream>

#include <ros/package.h>

int main(int argc, char* argv[])
{
  std::string filepath;

  filepath = ros::package::getPath("hrp2_models") + "/HRP2JSKNTS_WITH_3HAND_for_OpenHRP3/HRP2JSKNTSmain.wrl";

  cnoid::initializeCorbaUtil();

  cnoid::BodyLoader loader;
  loader.setMessageSink(std::cout);
  loader.setShapeLoadingEnabled(false);
  cnoid::BodyPtr body = loader.load(filepath);
  if(!body){
    std::cout << filepath << " cannot be loaded." << std::endl;
    return 0;
  }

  OpenHRP::OnlineViewer_var viewer =
    cnoid::getDefaultNamingContextHelper()->findObject<OpenHRP::OnlineViewer>("OnlineViewer");

  size_t numrobots = 5;

  viewer->clearLog();
  viewer->clearData();
  for(size_t i=0; i< numrobots; i++){
    viewer->load((body->modelName()+std::to_string(i)).c_str(), filepath.c_str());
  }

  OpenHRP::WorldState world;
  world.characterPositions.length(numrobots);
  world.collisions.length(numrobots);
  for(size_t i=0; i < numrobots; i++){
    OpenHRP::CharacterPosition& position = world.characterPositions[i];
    position.characterName = CORBA::string_dup((body->modelName()+std::to_string(i)).c_str());
    position.linkPositions.length(body->numLinks());
    OpenHRP::Collision& collision = world.collisions[i];
    collision.pair.charName1 = CORBA::string_dup((body->modelName()+std::to_string(i)).c_str());
    collision.pair.linkName1 = CORBA::string_dup("WAIST");
    collision.pair.charName2 = CORBA::string_dup((body->modelName()+std::to_string(i)).c_str());
    collision.pair.linkName2 = CORBA::string_dup("WAIST");
  }
  double q = 0.0;
  double dq = 0.01;

  for(double time = 0.0; time <= 6.4; time += 0.01){
    for(int j=0; j < body->numJoints(); ++j){
      body->joint(j)->q() = q;
    }
    for(size_t i=0 ; i < numrobots; i++){
      body->rootLink()->p()[1] = i;
      body->calcForwardKinematics();

      OpenHRP::CharacterPosition& position = world.characterPositions[i];
      for(int j=0; j < body->numLinks(); ++j){
        cnoid::Link* link = body->link(j);
        Eigen::Map<cnoid::Vector3>(position.linkPositions[j].p) = link->p();
        Eigen::Map<cnoid::Matrix3>(position.linkPositions[j].R) = link->R().transpose();
      }

      OpenHRP::Collision& collision = world.collisions[i];
      if(time < 2.0){
        collision.points.length(1);
        OpenHRP::CollisionPoint& point = collision.points[0];
        point.idepth = 0.02;
        point.normal[0] = 1;
        point.normal[1] = 0;
        point.normal[2] = 0;
        point.position[0] = 0;
        point.position[1] = 0;
        point.position[2] = 0.1;
      }else if( time < 4.0){
        collision.points.length(3);
        OpenHRP::CollisionPoint& point = collision.points[0];
        point.idepth = 0.02;
        point.normal[0] = 0;
        point.normal[1] = 1;
        point.normal[2] = 0;
        point.position[0] = 0.2;
        point.position[1] = 0;
        point.position[2] = 0.12;
        OpenHRP::CollisionPoint& point1 = collision.points[1];
        point1.idepth = 0.02;
        point1.normal[0] = 0;
        point1.normal[1] = 1;
        point1.normal[2] = 0;
        point1.position[0] = 0;
        point1.position[1] = 0;
        point1.position[2] = 0.12;
        OpenHRP::CollisionPoint& point2 = collision.points[2];
        point2.idepth = 0.02;
        point2.normal[0] = 0;
        point2.normal[1] = 1;
        point2.normal[2] = 0;
        point2.position[0] = -0.2;
        point2.position[1] = 0;
        point2.position[2] = 0.12;
      }else{
        collision.points.length(2);
        OpenHRP::CollisionPoint& point = collision.points[0];
        point.idepth = 0.02;
        point.normal[0] = 0;
        point.normal[1] = 0;
        point.normal[2] = 1;
        point.position[0] = 0.2;
        point.position[1] = 0;
        point.position[2] = 0.14;
        OpenHRP::CollisionPoint& point1 = collision.points[1];
        point1.idepth = 0.02;
        point1.normal[0] = 0;
        point1.normal[1] = 0;
        point1.normal[2] = 1;
        point1.position[0] = 0;
        point1.position[1] = 0;
        point1.position[2] = 0.14;
      }
    }

    world.time = time;

    //viewer->drawScene(world);
    viewer->update(world);
    cnoid::msleep(10);
    q += dq;
    if(fabs(q) > 0.4){
      dq = -dq;
    }
  }
}

