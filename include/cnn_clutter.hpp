#ifndef CNN_CLUTTER_LAYER_H_
#define CNN_CLUTTER_LAYER_H_

#include <ros/ros.h>

#include <vector>
#include <deque>
#include <sensor_msgs/Image.h>
#include <map>

#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/contrib/contrib.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

#include <clutter_2d/costmap_layer.h>
#include <clutter_2d/layered_costmap.h>

#include <sensor_msgs/CameraInfo.h>

#define TORAD  M_PI / 180.0
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> my_sync_policy;
typedef std::vector<std::pair<int, int>> rgb_object;
typedef std::map<long long unsigned int, unsigned int> id_count_pair;

enum ObjectClass {
	BACKGROUND, BALL, BOOKS, BOXES, CARS, DOLL, STAFFED, BLOCKS
};

struct Euclid3D {
	double x;
	double y;
	double z;
	Euclid3D() :
			x(0.0), y(0.0), z(0.0) {

	}

	Euclid3D(const double x, const double y, const double z) :
			x(x), y(y), z(z) {
	}
};

struct CNNObject {
	long long unsigned int objID;
	ObjectClass objClass;
	std::vector<std::pair<unsigned int, unsigned int>> xy_indices;
};

class CNNClutter: public clutter_2d::CostmapLayer {
public:
	///////////////////////////////////////////////////////////////////////////////////////
//  void publish_locations();
//  void publish_bounds(const unsigned int min_x, const unsigned int min_y, const unsigned int max_x,
//                      const unsigned int max_y);
	ros::Publisher location_publisher_;
	////////////////////////////////////////////////////
	std::string global_frame_;

	std::deque<std::pair<unsigned int, unsigned int>> rg_list;

	Euclid3D robots_pose_in_last_updatebounds;
	std::vector<rgb_object> rgb_object_buffer_;
	std::vector<ObjectClass> rgb_obj_classes_;
	std::vector<std::vector<std::pair<long long unsigned int, ObjectClass>>> clutter_map_;

	std::vector<std::pair<long long unsigned int, ObjectClass>> objIDToObjClass_;
	std::vector<CNNObject> objectBuffer_;

	boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> mask_sub_;
	boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
	boost::shared_ptr<message_filters::Synchronizer<my_sync_policy>> sync_;

	long long unsigned int trackedID_;
	std::vector<std::vector<Euclid3D>> unitImage;
	bool unitIm_isInit;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg;
	std::vector<pcl::PointCloud<pcl::PointXYZ>> pcl_obj_buffer_;
	ros::Publisher pubImageClass, pubImageID, pubPC;
	tf::StampedTransform transform_;

	CNNClutter();
	virtual ~CNNClutter();
	virtual void onInitialize();
	std::vector<size_t> sort_indexes(const std::vector<pcl::PointCloud<pcl::PointXYZ>> &v);
	void markRGBObject(const unsigned int& mx, const unsigned int& my,
			cv_bridge::CvImagePtr img_ptr, boost::shared_ptr<rgb_object> object);
	void regionGrowing(unsigned int& mx, unsigned int& my, cv_bridge::CvImagePtr img_ptr,
			boost::shared_ptr<rgb_object> object, const ObjectClass& obj_class);
	void syncImageCallback(const sensor_msgs::ImageConstPtr& mask,
			const sensor_msgs::ImageConstPtr& depth);

	void getSmallObjectNeighbourCount(const rgb_object object, const int& obj_index,
			const cv_bridge::CvImagePtr img_ptr, std::vector<unsigned int>& counter);

	void getObjectNeighbour(const CNNObject object, id_count_pair& id_count_buffer);

	void incrementIdCounterPair(const long long unsigned int& id, const unsigned int& mx,
			const unsigned int& my, id_count_pair& id_count_buffer);

	void initializeUnitImage(const double imageWidth, const double imageHeight,
			const double openingAngleWidth, const double openingAngleHeight);

	void convertDepthImageToPointCloud(const sensor_msgs::Image &depthImage,
			const std::vector<std::vector<std::pair<int, int> > > &objects,
			std::vector<pcl::PointCloud<pcl::PointXYZ>> &pointClouds);
	void pcObjectsToMapObj(const std::vector<pcl::PointCloud<pcl::PointXYZ>>& objPointClouds,
			const std::vector<ObjectClass> obj_class_vec);
	void sensorPcToMapFrame(std::vector<pcl::PointCloud<pcl::PointXYZ>>& sensorPointClouds,
			std::vector<pcl::PointCloud<pcl::PointXYZ>>& globalPointClouds);
	void cleanObjectBuffer();

	void matchSizeMaps();
	virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
			double* min_y, double* max_x, double* max_y);
	virtual void updateCosts(clutter_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
			int max_j);
	virtual void activate();
	virtual void deactivate();
	virtual void reset();


	double getSignedAngle(std::pair<double, double> vecRef, std::pair<double, double> vecCheck);
	bool isPointInsidePolygon(const std::vector<std::pair<double, double> > &vertices, const std::pair<double, double> &pointCheck);
	bool checkPoints(const std::vector<std::pair<double, double> > &corners, const double robotX, const double robotY, const double robotYaw,
	                 const std::vector<std::pair<unsigned int, unsigned int> > &pointsCheck);

protected:
private:

};

#endif
