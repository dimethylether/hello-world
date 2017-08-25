#ifndef __WAYPOINT_LAYER_HPP__
#define __WAYPOINT_LAYER_HPP__
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <boost/thread.hpp>
#include <waypoint_layer/WaypointLayerConfig.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
  
namespace waypoint_layer
{
  class Line{
    public:
      int x1;
      int x2; 
      int y1; 
      int y2;
      double a;
      double b;
      double c; 
      double denom;
      int getX(int y);
      int getY(int x);
      double distance_m(int x, int y, double resolution);
      void getIntersect(Line tmp, double &dx, double &dy);
      void getFoot(double sx, double sy, double &dx, double &dy);

  };
  class WaypointLayer : public costmap_2d::Layer
  {
    public:
      WaypointLayer() { layered_costmap_ = NULL; }

      virtual void onInitialize();
      virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) ;
      
      bool isDiscretized() { return false; }

    protected:
      void waypointCallback(const geometry_msgs::PoseStamped& waypoint);
      ros::Subscriber waypoint_sub_;
      // people_msgs::People people_list_;
      // std::list<people_msgs::Person> transformed_people_;
      // ros::Duration people_keep_time_;
      boost::recursive_mutex lock_;
      std::vector<geometry_msgs::PoseStamped> waypoint_list_;
      std::vector<geometry_msgs::PoseStamped> transformed_waypoint_list_;
      std::vector<Line> line_list_;
      void configure(WaypointLayerConfig &config, uint32_t level);
      dynamic_reconfigure::Server<WaypointLayerConfig>* server_;
      dynamic_reconfigure::Server<WaypointLayerConfig>::CallbackType f_;

      void setLineList(costmap_2d::Costmap2D* costmap);
      tf::TransformListener tf_;
      bool first_time_;
      double width_, scale_;
      bool use_critical_width_;
      double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
      void setCost(int x, int y, int cost);

  };
};


#endif

