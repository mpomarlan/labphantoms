#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include <ros/ros.h>
#include <boost/bind/bind.hpp>
#include <tf2_msgs/TFMessage.h>

#include <tf/transform_listener.h>

#include <boost/thread.hpp>

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

typedef struct
{
    physics::WorldPtr parentWorld;
    std::vector<std::string> phantoms;
    tf::TransformListener listener;
}tPhantomContext;

tPhantomContext PhantomContext;

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

math::Pose StampedTransform2Pose(tf::StampedTransform const& transform)
{
    tf::Vector3 vt = transform.getOrigin();
    tf::Quaternion qt = transform.getRotation();
    math::Vector3 v;
    math::Quaternion q;

    v.x = vt.x();
    v.y = vt.y();
    v.z = vt.z();
    /*SUPER DIRTY HAXX*/
    if(0.755 > v.z)
        v.z = 0.755;
    q.w = qt.w();
    q.x = qt.x();
    q.y = qt.y();
    q.z = qt.z();

    math::Pose pose;
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
      running = false;
      if(n)
          delete n;
  }

  void monitorTFThread(void)
  {
      ros::Rate rate(0.5);
      while(running)
      {
          physics::WorldPtr world = parentWorld;
          std::vector<std::string> phantomsL = phantoms;
          tf::TransformListener listenerL;

          int maxK = phantomsL.size();

          listenerL.waitForTransform("bottle_250ml_0_frame", "/map", ros::Time::now(), ros::Duration(5.0));
          for(int k = 0; k < maxK; k++)
          {
              std::string childFrame = "/";
              childFrame += phantomsL[k] + "_frame";

              tf::StampedTransform transform;
              try
              {
                  ros::Time now = ros::Time::now();
                  listenerL.lookupTransform("/map", childFrame, ros::Time(), transform);

                  if(phantomsL[k] == "pipette_accumax")
                  {
                      math::Pose p = StampedTransform2Pose(transform);
                      std::cerr << "PIPETTE TRANSFORM to /map V(" << p.pos.x << " " << p.pos.y << " " << p.pos.z << ") Q(" << p.rot.w << " " << p.rot.x << " " << p.rot.y << " " << p.rot.z << ")" << std::endl;
                  }

                  if(phantomsL[k] == "bottle_250ml")
                  {
                      math::Pose p = StampedTransform2Pose(transform);
                      std::cerr << "BOTTLE_250ML TRANSFORM to /map V(" << p.pos.x << " " << p.pos.y << " " << p.pos.z << ") Q(" << p.rot.w << " " << p.rot.x << " " << p.rot.y << " " << p.rot.z << ")" << std::endl;
                  }

                  if(phantomsL[k] == "mixer_ikamag")
                  {
                      math::Pose p = StampedTransform2Pose(transform);
                      std::cerr << "MIXER_IKAMAG TRANSFORM to /map V(" << p.pos.x << " " << p.pos.y << " " << p.pos.z << ") Q(" << p.rot.w << " " << p.rot.x << " " << p.rot.y << " " << p.rot.z << ")" << std::endl;
                  }

                  if(phantomsL[k] == "flask_400ml_0")
                  {
                      math::Pose p = StampedTransform2Pose(transform);
                      std::cerr << "FLASK_400ml_0 TRANSFORM to /map V(" << p.pos.x << " " << p.pos.y << " " << p.pos.z << ") Q(" << p.rot.w << " " << p.rot.x << " " << p.rot.y << " " << p.rot.z << ")" << std::endl;
                  }

                  physics::ModelPtr childModel = world->GetModel(phantomsL[k]);
                  if(childModel.get())
                  {
                      math::Pose world2Child = StampedTransform2Pose(transform);
                      childModel->SetWorldPose(world2Child);
                      childModel->SetStatic(true);
                      childModel->SetGravityMode(false);
                  }
              }
              catch (tf::TransformException ex)
              {
                  ROS_ERROR("%s", ex.what());
              }
          }
          rate.sleep();
      }
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

          /*DIRTY HAXXXXX TO BE REMOVED.*/
          if("in_r_gripper_tool_frame" == parentFrame)
              parentFrame = "r_gripper_tool_frame";
          if("in_l_gripper_tool_frame" == parentFrame)
              parentFrame = "l_gripper_tool_frame";

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

    //ros::Subscriber sub = n->subscribe("/tf", 1, &LabPhantoms::onTFData, this);

    if(pluginParameters->HasElement("trackedModels"))
    {
        std::string trackedModels = pluginParameters->Get<std::string>("trackedModels");
        std::string dbgStr = "Tracked Models as given by launch file: ";
        dbgStr = dbgStr + trackedModels;
        std::cerr << dbgStr << "\n";

        std::stringstream ss(trackedModels);
        ss.imbue(std::locale(std::locale(), new tokens()));
        std::istream_iterator<std::string> begin(ss);
        std::istream_iterator<std::string> end;
        std::vector<std::string> vstrings(begin, end);
        std::copy(vstrings.begin(), vstrings.end(), std::ostream_iterator<std::string>(std::cerr, "\n"));
        phantoms = vstrings;
    }

    running = true;
    boost::thread(&LabPhantoms::monitorTFThread, this);
  }

  protected:
    bool running;
    ros::NodeHandle *n;
    physics::WorldPtr parentWorld;
    sdf::ElementPtr pluginParameters;
    std::vector<std::string> phantoms;
    tf::TransformListener listener;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(LabPhantoms)
}
