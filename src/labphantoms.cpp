#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include <ros/ros.h>
#include <boost/bind/bind.hpp>
#include <tf2_msgs/TFMessage.h>

namespace gazebo
{

struct tokens: std::ctype<char>
{
    tokens(): std::ctype<char>(get_table()) {}

    static std::ctype_base::mask const* get_table()
    {
        typedef std::ctype<char> cctype;
        static const cctype::mask *const_rc= cctype::classic_table();

        static cctype::mask rc[cctype::table_size];
        std::memcpy(rc, const_rc, cctype::table_size * sizeof(cctype::mask));

        rc[','] = std::ctype_base::space;
        rc[' '] = std::ctype_base::space;
        return &rc[0];
    }
};


class LabPhantoms;
void onTFData2(const tf2_msgs::TFMessage::ConstPtr& msg);
void onTFData(const tf2_msgs::TFMessage::ConstPtr& msg);//, LabPhantoms *plugin);

math::Pose Msg2Pose(geometry_msgs::Transform const& msg)
{
    math::Pose pose;
    math::Vector3 v;
    math::Quaternion q;

    v.x = msg.translation.x;
    v.y = msg.translation.y;
    v.z = msg.translation.z;
    q.w = msg.rotation.w;
    q.x = msg.rotation.x;
    q.y = msg.rotation.y;
    q.z = msg.rotation.z;

    pose.Set(v, q);
    return pose;
}

class LabPhantoms : public WorldPlugin
{
  protected:
  bool havePhantom(std::string const& name)
  {
      bool retq = false;
      int maxK = phantoms.size();
      for(int k = 0; (!retq) && (k < maxK); k++)
          retq = (phantoms[k] == name);
      return retq;
  }

  public:
  LabPhantoms()
  {
      n = 0;
      parentWorld.reset();
  }
  ~LabPhantoms()
  {
      if(n)
          delete n;
  }

  void onTFData(const tf2_msgs::TFMessageConstPtr& msg)
  {
      if(!(parentWorld.get()))
          return;
      physics::WorldPtr world = parentWorld;

      int maxK = msg->transforms.size();

      for(int k = 0; k < maxK; k++)
      {
          geometry_msgs::TransformStamped transform = msg->transforms[k];
          std::string parentFrame = transform.header.frame_id;
          std::string childFrame = transform.child_frame_id;

          if(havePhantom(childFrame))
          {
              physics::ModelPtr childModel = world->GetModel(childFrame);
              physics::ModelPtr parentModel = world->GetModel(parentFrame);

              if((childModel.get()) && (parentModel.get()))
              {
                  math::Pose parent2Child = Msg2Pose(transform.transform);
                  math::Pose world2Parent = parentModel->GetWorldPose();
                  childModel->SetWorldPose(parent2Child*world2Parent);
              }
          }
      }
  }

  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    parentWorld = _parent;
    pluginParameters = _sdf;
    char nodeName[] = "labphantoms";
    char* argv[] = {nodeName};
    int argc = 1;
    ros::init(argc, argv, "labphantoms");
    n = new ros::NodeHandle();

    ros::Subscriber sub = n->subscribe("/tf", 1, &LabPhantoms::onTFData, this);

    if(pluginParameters->HasElement("trackedModels"))
    {
        std::string trackedModels = pluginParameters->Get<std::string>("trackedModels");
        ROS_INFO("Tracked Models as given by launch file: %s", trackedModels);

        std::stringstream ss(trackedModels);
        ss.imbue(std::locale(std::locale(), new tokens()));
        std::istream_iterator<std::string> begin(ss);
        std::istream_iterator<std::string> end;
        std::vector<std::string> vstrings(begin, end);
        std::copy(vstrings.begin(), vstrings.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
        phantoms = vstrings;
    }
  }

  protected:
    ros::NodeHandle *n;
    physics::WorldPtr parentWorld;
    sdf::ElementPtr pluginParameters;
    std::vector<std::string> phantoms;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(LabPhantoms)
}
