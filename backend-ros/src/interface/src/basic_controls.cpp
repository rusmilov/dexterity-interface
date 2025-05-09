/* Modified version of: https://github.com/ros-visualization/visualization_tutorials/blob/noetic-devel/interactive_marker_tutorials/src/basic_controls.cpp
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


 #include <ros/ros.h>

 #include <interactive_markers/interactive_marker_server.h>
 #include <interactive_markers/menu_handler.h>
 
 #include <tf/transform_broadcaster.h>
 #include <tf/tf.h>
 
 #include <math.h>
 
 using namespace visualization_msgs;
 
 
 boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
 interactive_markers::MenuHandler menu_handler;

 
 Marker makeBox( InteractiveMarker &msg )
 {
   Marker marker;
 
   marker.type = Marker::CUBE;
   marker.scale.x = msg.scale * 0.45;
   marker.scale.y = msg.scale * 0.45;
   marker.scale.z = msg.scale * 0.45;
   marker.color.r = 0.5;
   marker.color.g = 0.5;
   marker.color.b = 0.5;
   marker.color.a = 1.0;
 
   return marker;
 }
 


 void frameCallback(const ros::TimerEvent&)
 {
   static uint32_t counter = 0;
 
   static tf::TransformBroadcaster br;
 
   tf::Transform t;
 
   ros::Time time = ros::Time::now();
 
   t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
   t.setRotation(tf::createQuaternionFromRPY(0.0, 0, 0.0));
   br.sendTransform(tf::StampedTransform(t, time, "world", "base_link"));
 
   counter++;
 }

 void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
 {
   std::ostringstream s;
   s << "Feedback from marker '" << feedback->marker_name << "' "
       << " / control '" << feedback->control_name << "'";
 
   std::ostringstream mouse_point_ss;
   if( feedback->mouse_point_valid )
   {
     mouse_point_ss << " at " << feedback->mouse_point.x
                    << ", " << feedback->mouse_point.y
                    << ", " << feedback->mouse_point.z
                    << " in frame " << feedback->header.frame_id;
   }
 
   switch ( feedback->event_type )
   {

     case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
       ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
       break;
 
     case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
       ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
       break;
 
     case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
       ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
       break;
   }
 
   server->applyChanges();
 }

 
 
 void makeMenuMarker( const tf::Vector3& position )
 {
   InteractiveMarker int_marker;
   int_marker.header.frame_id = "base_link";
   tf::pointTFToMsg(position, int_marker.pose.position);
   int_marker.scale = 1;
 
   int_marker.name = "context_menu";
   int_marker.description = "Context Menu\n(Right Click)";
 
   InteractiveMarkerControl control;
 
   control.interaction_mode = InteractiveMarkerControl::MENU;
   control.name = "menu_only_control";
 
   Marker marker = makeBox( int_marker );
   control.markers.push_back( marker );
   control.always_visible = true;
   int_marker.controls.push_back(control);
 
   server->insert(int_marker);
   server->setCallback(int_marker.name, &processFeedback);
   menu_handler.apply( *server, int_marker.name );
 }


 int main(int argc, char** argv)
 {
   ros::init(argc, argv, "basic_controls");
   ros::NodeHandle n;
 
   // create a timer to update the published transforms
   ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);
 
   server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );
 
   ros::Duration(0.1).sleep();
 
   menu_handler.insert( "First Entry", &processFeedback );
   menu_handler.insert( "Second Entry", &processFeedback );

   tf::Vector3 position;
   position = tf::Vector3(0, 1, 0);

   makeMenuMarker( position );

   server->applyChanges();
 
   ros::spin();
 
   server.reset();
 }
 // %EndTag(main)%