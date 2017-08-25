#include <waypoint_layer/waypoint_layer.hpp>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(waypoint_layer::WaypointLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace waypoint_layer
{
  void saturation(int &value, int min, int max){
    value = std::min(max, std::max(value, min));
  }
  void WaypointLayer::onInitialize()
  {
      ros::NodeHandle nh("~/" + name_), g_nh;
      current_ = true;
      first_time_ = true;
      waypoint_sub_ = nh.subscribe("/move_base/current_goal", 1, &WaypointLayer::waypointCallback, this);
      server_ = new dynamic_reconfigure::Server<WaypointLayerConfig>(nh);
      f_ = boost::bind(&WaypointLayer::configure, this, _1, _2);
      server_->setCallback(f_);
  }

  void WaypointLayer::waypointCallback(const geometry_msgs::PoseStamped& waypoint) {
      boost::recursive_mutex::scoped_lock lock(lock_);
      waypoint_list_.push_back(waypoint); 
      std::cerr << "received_waypoint" << std::endl;
      
  }

  void WaypointLayer::updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y, double* max_x, double* max_y){
      boost::recursive_mutex::scoped_lock lock(lock_);
      
      std::string global_frame = layered_costmap_->getGlobalFrameID();

      transformed_waypoint_list_.clear();
      
      for(unsigned int i=0; i<waypoint_list_.size(); i++){
          geometry_msgs::PoseStamped pt, opt;
          pt= waypoint_list_.at(i);
          
          try{
            tf_.transformPose(global_frame, pt, opt);
            transformed_waypoint_list_.push_back(opt);
            
          }
          catch(tf::LookupException& ex) {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            continue;
          }
          catch(tf::ConnectivityException& ex) {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            continue;
          }
          catch(tf::ExtrapolationException& ex) {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            continue;
          }
      }
      
      for(auto pose: transformed_waypoint_list_){
        *min_x = std::min(*min_x, pose.pose.position.x-width_*1.5 );
        *min_y = std::min(*min_y, pose.pose.position.y-width_*1.5 );
        *max_x = std::max(*max_x, pose.pose.position.x+width_*1.5 );
        *max_y = std::max(*max_y, pose.pose.position.y+width_*1.5 );
      }
      if(first_time_){
          last_min_x_ = *min_x;
          last_min_y_ = *min_y;    
          last_max_x_ = *max_x;
          last_max_y_ = *max_y;    
          first_time_ = false;
      }else{
          double a = *min_x, b = *min_y, c = *max_x, d = *max_y;
          *min_x = std::min(last_min_x_, *min_x);
          *min_y = std::min(last_min_y_, *min_y);
          *max_x = std::max(last_max_x_, *max_x);
          *max_y = std::max(last_max_y_, *max_y);
          last_min_x_ = a;
          last_min_y_ = b;
          last_max_x_ = c;
          last_max_y_ = d;      
      }
      
      tf::StampedTransform transform;
      
      try {
          tf_.waitForTransform(global_frame, "base_link", ros::Time(0), ros::Duration(1.0));
          tf_.lookupTransform(global_frame, "base_link", ros::Time(0), transform);
      }
      catch (tf::TransformException ex) {
          ROS_ERROR("%s", ex.what());
          ros::Duration(0.1).sleep();
          return;
      }
      double robot_x = transform.getOrigin().x(); 
      double robot_y = transform.getOrigin().y();

      if(waypoint_list_.size() > 2){
        double old_distance = 0;
        double old_distance2 = 0;
        for(auto wp = waypoint_list_.begin(); wp != waypoint_list_.end() ; wp++){
          double distance = hypot(wp->pose.position.x - robot_x , wp->pose.position.y - robot_y);
          if(wp - waypoint_list_.begin() < 2){
            old_distance2 = old_distance;
            old_distance = distance;
            continue;
          }
          if(distance < old_distance && old_distance < old_distance2){
            std::cout << "erasing old waypoint" <<std::endl;
            waypoint_list_.erase(wp-2);
          }else if(old_distance > old_distance2 ){
            break;
          }
          old_distance2 = old_distance;
          old_distance = distance;

        }
      }






      
  }

  void WaypointLayer::configure(WaypointLayerConfig &config, uint32_t level) {
      width_ = config.width;
      scale_ = config.scale;
      use_critical_width_ = config.use_critical_width;
  }


  void WaypointLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
    boost::recursive_mutex::scoped_lock lock(lock_);
    // if(!enabled_) return;

    if( waypoint_list_.size() == 0 )
      return;

    // std::list<geometry_msgs::PoseStamped>::iterator p_it;
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    double res = costmap->getResolution();
    // std::cerr << "transformed waypoint list " << transformed_waypoint_list_.size() << std::endl;
    line_list_.clear();
    setLineList(costmap);

    for(auto line: line_list_){
      double theta = atan2(line.a, line.b);
      double width_map;

      if(fabs(theta) < M_PI/4 || fabs(theta) > M_PI*3.0/4 ){
        width_map = fabs(width_ / cos(theta))/res + 1;
        if(line.x1 < line.x2){
          for (int i = line.x1; i < line.x2 ; i++){
            int y_tmp = line.getY(i);
            for (int j = y_tmp - width_map; j <= y_tmp + width_map ; j++){
              if(fabs(theta) != 0 && fabs(theta) != M_PI){
                double foot_x , foot_y; 
                line.getFoot(i,j,foot_x, foot_y);
                if(foot_x < line.x1 || foot_x > line.x2 ) continue;
              }

              double d=line.distance_m(i,j,res);
              int cost = d/width_ * costmap_2d::LETHAL_OBSTACLE;
              saturation(cost, 1, costmap_2d::LETHAL_OBSTACLE); 
              setCost(i,j,cost);
            }
          }
        }else{
          for (int i = line.x1; i > line.x2 ; i--){
            int y_tmp = line.getY(i);
            for (int j = y_tmp - width_map; j <= y_tmp + width_map ; j++){
              if(fabs(theta) != 0 && fabs(theta) != M_PI){
                double foot_x , foot_y; 
                line.getFoot(i,j,foot_x, foot_y);
                if(foot_x > line.x1 || foot_x < line.x2) continue;
              }

              double d=line.distance_m(i,j,res);
              int cost = d/width_ * costmap_2d::LETHAL_OBSTACLE;
              saturation(cost, 1, costmap_2d::LETHAL_OBSTACLE); 
              setCost(i,j,cost);
            }
          }
        }
      }else{
        width_map = fabs(width_ / sin(theta))/res +1;
        if(line.y1 < line.y2){
          for (int j = line.y1; j < line.y2 ; j++){
            int x_tmp = line.getX(j);
            for (int i = x_tmp - width_map; i <= x_tmp + width_map ; i++){
              if(fabs(theta) != M_PI/2){
                double foot_x , foot_y; 
                line.getFoot(i,j,foot_x, foot_y);
                // std::cout << i << " " << j << " " << foot_x << " " << foot_y << std::endl;
                if(foot_y < line.y1 || foot_y > line.y2 ) continue;
              }

              double d=line.distance_m(i,j,res);
              int cost = d/width_ * costmap_2d::LETHAL_OBSTACLE;
              saturation(cost, 1, costmap_2d::LETHAL_OBSTACLE); 
              setCost(i,j,cost);
            }
          }
        }else{
          for (int j = line.y1; j > line.y2 ; j--){
            int x_tmp = line.getX(j);
            for (int i = x_tmp - width_map; i <= x_tmp + width_map ; i++){
              if(fabs(theta) != M_PI/2){
                double foot_x , foot_y; 
                line.getFoot(i,j,foot_x, foot_y);
                if(foot_y > line.y1 || foot_y < line.y2 ) continue;
              }
              double d=line.distance_m(i,j,res);
              int cost = d/width_ * costmap_2d::LETHAL_OBSTACLE;
              saturation(cost, 1, costmap_2d::LETHAL_OBSTACLE); 
              setCost(i,j,cost);
            }
          }
        }
        
      }

    }

    for(auto wp: transformed_waypoint_list_){
      double width_map = width_/res + 1;
      int x,y;        
      costmap->worldToMapNoBounds(wp.pose.position.x, wp.pose.position.y, x, y);
      for (int i = x - width_map ; i <= x + width_map; i++){
        for ( int j = y - width_map ; j <= y + width_map; j++){
          double d = hypot(i-x,j-y);
          if(d > width_map) continue; 
          int cost = d*res/width_ * costmap_2d::LETHAL_OBSTACLE;
          saturation(cost, 1, costmap_2d::LETHAL_OBSTACLE); 
          setCost(i,j,cost);
        }
      }
    }



  }
  void WaypointLayer::setCost(int x, int y, int cost){
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    if( x < 0 || x > costmap->getSizeInCellsX() )return;
    if( y < 0 || y > costmap->getSizeInCellsY() )return;
    int old_cost = costmap->getCost(x,y);
    if(old_cost == costmap_2d::FREE_SPACE || old_cost > cost){
      costmap->setCost(x,y,cost);
    }
  }

  void WaypointLayer::setLineList(costmap_2d::Costmap2D* costmap){
    int old_x, old_y, x, y;
    costmap->worldToMapNoBounds(transformed_waypoint_list_.front().pose.position.x, transformed_waypoint_list_.front().pose.position.y, old_x, old_y);
    for (auto wp: transformed_waypoint_list_){
      costmap->worldToMapNoBounds(wp.pose.position.x, wp.pose.position.y, x, y);
      if(old_x == x && old_y == y) continue;
      Line tmp;
      tmp.x1 = old_x;
      tmp.x2 = x;
      tmp.y1 = old_y;
      tmp.y2 = y;

      //ax + by + c = 0;
      tmp.a = (tmp.y2 - tmp.y1);
      tmp.b = tmp.x1 - tmp.x2;
      tmp.c = tmp.x2 * tmp.y1 - tmp.x1 * tmp.y2;

      tmp.denom = sqrt(tmp.a * tmp.a + tmp.b * tmp.b);

      line_list_.push_back(tmp);
      old_x = x;
      old_y = y;
    }
  }
  double Line::distance_m(int x, int y, double resolution){
    double d =  resolution * fabs(a * x + b * y + c )/denom;
    // std::cout << "d " << d << std::endl;
    return d; 
  } 
  int Line::getX(int y){
    if(a== 0) return 1;
    return (double)(-(b*y + c)/a);
  }
  int Line::getY(int x){
    if(b==0) return 1; 
    return (double)(-(a*x + c)/b);
  }
  void Line::getFoot(double sx, double sy, double &dx, double &dy){
    //step 1. find Line that passes through (sx, sy) and perpendicular to this line
    //step 2. find intersection of two lines
    
    Line tmp;
    tmp.a = -b;
    tmp.b = a;
    tmp.c = -(tmp.a * sx + tmp.b * sy);

    getIntersect(tmp, dx, dy);

  }
  void Line::getIntersect(Line tmp, double &dx, double &dy){
    if( b == 0){
      dx = -c/a;
      dy = -(tmp.a*dx + tmp.c)/tmp.b;
    }else if(tmp.b == 0){
      dx = -tmp.c/tmp.a;
      dy = -(a*dx + c)/b;
    }else{
      if(tmp.a/tmp.b == a/b){
        std::cerr << "failed to calculate intersect between lines due to same slope" << std::endl;
      }
      dx = -(c * tmp.b - tmp.c * b) / (a*tmp.b - tmp.a * b);
      dy = -(a * dx + c ) / b;  
    }
  }

};
