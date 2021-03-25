#ifndef ARTI_GAZEBO_LASER_LIVOX_H
#define ARTI_GAZEBO_LASER_LIVOX_H

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/PointCloud2.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/RayPlugin.hh>

#include <sdf/Param.hh>

#include "livox_simulation/ray_data.h"

namespace gazebo
{

	class ArtiGazeboLaserLivox : public RayPlugin
	{
	public:
		// Public functions
		ArtiGazeboLaserLivox();
		~ArtiGazeboLaserLivox();
		void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

	private:
		// Functions
		void PutLaserData(common::Time &_updateTime);
		int connect_count_;
		void Connect();
		void Disconnect();
		void LoadThread();
		void LaserQueueThread();
		bool AddRayEllipseShape(double rotation_degrees);
		/*void CalculatePoints(livox::RayData rays, double horiz_angles, double vert_angles);*/

		// Variables
		gazebo::transport::NodePtr gazebo_node_;
		physics::WorldPtr world_;
		sensors::SensorPtr parent_sensor_;
		sensors::RaySensorPtr parent_ray_sensor_;

		ros::NodeHandle *nh_;
		ros::Publisher pub_;
		std::string topic_name_;
		std::string frame_name_;
		double update_rate_;
		double update_period_;
		common::Time last_update_time_;
		std::string robot_namespace_;
		ros::CallbackQueue laser_queue_;
		boost::thread callback_queue_thread_;
		sdf::ElementPtr sdf;
		
		// \brief A mutex to lock access
		boost::mutex lock_;

		boost::thread deferred_load_thread_;
		gazebo::physics::PhysicsEnginePtr engine_;

		std::vector<gazebo::physics::RayShapePtr> rays_;

		std::vector<livox::RayData> double_ellipse_rays_;

		physics::CollisionPtr collision_ptr_;
		std::vector<physics::CollisionPtr> collision_ptr_list_;
		ignition::math::Pose3d sensor_pose_;
		std::vector<float> vertical_ray_angles_;
		std::vector<float> horizontal_ray_angles_;
		ignition::math::Vector3d ray_startpoint_;
		sensor_msgs::PointCloud cloud_msg_;
		sensor_msgs::PointCloud2 pc2_msgs_;

		// Subscribe to gazebo laserscan
		gazebo::transport::SubscriberPtr sub_;
		virtual void OnNewLaserScans();
		void OnScan(const ConstLaserScanStampedPtr &_msg);

		int samples_;
		int num_ellipses_;
		size_t interpolation_points_;
		float max_interpolation_distance_;
		float min_range_;
		float max_range_;
		float rotation_increment_;
		double current_rot_angle_ = 0;

		bool init_finished_ = false;

		int debug_counter = 0;

		ros::Publisher pub1_;
		ros::Publisher pub2_;
		ros::Publisher pub3_;
		ros::Publisher pub4_;
		sensor_msgs::PointCloud cloud1_msg_;
		sensor_msgs::PointCloud cloud2_msg_;
		sensor_msgs::PointCloud cloud3_msg_;
		sensor_msgs::PointCloud cloud4_msg_;
		sensor_msgs::PointCloud cloud1all_msg_;
	};
}
#endif // ARTI_GAZEBO_LASER_LIVOX_H
