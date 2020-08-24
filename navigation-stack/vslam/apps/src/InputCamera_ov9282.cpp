/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <thread>
#include <functional>

#include "SystemTime.h"
#include "InputCamera_ov9282.h"

#include <fstream>
#include <sstream>

using namespace std;
#define IMAGE_WIDTH  640
#define IMAGE_HEIGHT 400
#define FPS          30
#define CAMERA_ID    1   //ov9282 camera ID

#ifdef OPENCV_TEST
#include <opencv2/opencv.hpp>
using namespace cv;
#endif

#ifdef ROS_BASED
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
using namespace cv;

extern rclcpp::Node::SharedPtr g_node;
extern image_transport::Publisher    color_pub;
extern rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub;
  
void publishColor(unsigned char* buf, const rclcpp::Time & t)
{
    Mat gray_image(Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
    memcpy(gray_image.data, buf, IMAGE_WIDTH * IMAGE_HEIGHT);

    sensor_msgs::msg::Image::SharedPtr img;
    img = cv_bridge::CvImage(
      std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, gray_image).toImageMsg();
	  
    img->width = IMAGE_WIDTH;
    img->height = IMAGE_HEIGHT;
    img->is_bigendian = false;
    img->step = IMAGE_WIDTH;
    img->header.frame_id = "gray_frame";
    img->header.stamp = t;
    color_pub.publish(img);
}

#if 0
void publishRGBCameraInfo(const rs2_intrinsics & intrinsic, const rclcpp::Time & t)
{
    sensor_msgs::msg::CameraInfo info_msg;

    info_msg.width = intrinsic.width;
    info_msg.height = intrinsic.height;
    info_msg.header.frame_id = "camera_info";
    info_msg.header.stamp = t;

    info_msg.k.at(0) = intrinsic.fx;
    info_msg.k.at(2) = intrinsic.ppx;
    info_msg.k.at(4) = intrinsic.fy;
    info_msg.k.at(5) = intrinsic.ppy;
    info_msg.k.at(8) = 1;

    info_msg.p.at(0) = intrinsic.fx;
    info_msg.p.at(1) = 0;
    info_msg.p.at(2) = intrinsic.ppx;
    info_msg.p.at(3) = 0;
    info_msg.p.at(4) = 0;
    info_msg.p.at(5) = intrinsic.fy;
    info_msg.p.at(6) = intrinsic.ppy;
    info_msg.p.at(7) = 0;
    info_msg.p.at(8) = 0;
    info_msg.p.at(9) = 0;
    info_msg.p.at(10) = 1;
    info_msg.p.at(11) = 0;

    for (int i = 0; i < 5; i++) {
       info_msg.d.push_back(intrinsic.coeffs[i]);
    }

    cam_info_pub->publish(info_msg);
}
#endif
#endif


std::string Program_Root = "/data/misc/vwslam/";
void ReadMatrix( std::ifstream & file, float * matrix )
{
   std::string line, valName;
   int rows = 0, cols = 0;

   std::getline( file, line );
   std::istringstream issRow( line );
   issRow >> valName >> rows;

   std::getline( file, line );
   std::istringstream issCol( line );
   issCol >> valName >> cols;

   std::getline( file, line );
   size_t index = line.find_first_of( '[' );
   size_t length = line.length();
   std::string matrixStr = line.substr( index + 1, length - index - 1 );
   while( std::getline( file, line ) )
   {
      size_t index1 = line.find_first_of( ']' );
      if( index1 == std::string::npos )
      {
         matrixStr = matrixStr + line;
      }
      else
      {
         matrixStr = matrixStr + line.substr( 0, index1 );
         break;
      }
   }

   std::istringstream iss( matrixStr );

   std::string numberStr;
   for( int i = 0; i < rows * cols; i++ )
   {
      iss >> matrix[i] >> valName;
   }
}


bool GetCameraParameter( const char *cameraID, rvCameraParams & configuration )
{
   std::string fullName = Program_Root + cameraID;
   std::ifstream cfg( fullName, std::ifstream::in );
   if( !cfg.is_open() )
   {
      printf( "Fail to open configuration file: %s\n", fullName.c_str() );
      return false;
   }

   std::string line;
   std::string itemName;
   while( std::getline( cfg, line ) )
   {
      if( line.length() == 0 )
      {
         continue;
      }
      if( line[0] == '#' )
      {
         continue;
      }
      std::istringstream iss( line );
      iss >> itemName;
      if( itemName.compare( "image_width:" ) == 0 )
      {
         iss >> configuration.inputPixelWidth;
      }
      else if( itemName.compare( "image_height:" ) == 0 )
      {
         iss >> configuration.inputPixelHeight;
      }
      else if( itemName.compare( "camera_matrix:" ) == 0 )
      {
         ReadMatrix( cfg, configuration.inputCameraMatrix );
      }
      else if( itemName.compare( "distortion_coefficients:" ) == 0 )
      {
         memset( configuration.distortionCoefficient, 0, sizeof( configuration.distortionCoefficient ) );
         ReadMatrix( cfg, configuration.distortionCoefficient );   
      }
      else if( itemName.compare( "distortion_model:" ) == 0 )
      {
         std::string distortionModelName;
         iss >> distortionModelName;
         if( distortionModelName.compare( "fisheye" ) == 0 )
         {
            configuration.distortionModel = rvCameraParams::FisheyeModel_4;
         }
         else
         {
            configuration.distortionModel = rvCameraParams::RationalModel_12;
         }
      }

      else if( itemName.compare( "projection_matrix:" ) == 0 )
      {
         float p[12];
         ReadMatrix( cfg, p );
         configuration.outputCameraMatrix[0] = p[0];
         configuration.outputCameraMatrix[1] = p[1];
         configuration.outputCameraMatrix[2] = p[2];
         configuration.outputCameraMatrix[3] = p[4];
         configuration.outputCameraMatrix[4] = p[5];
         configuration.outputCameraMatrix[5] = p[6];
         configuration.outputCameraMatrix[6] = p[8];
         configuration.outputCameraMatrix[7] = p[9];
         configuration.outputCameraMatrix[8] = p[10];
      }
      else if( itemName.compare( "project_image_width:" ) == 0 )
      {
         iss >> configuration.outputPixelWidth;
      }
      else if( itemName.compare( "project_image_height:" ) == 0 )
      {
         iss >> configuration.outputPixelHeight;
      }
   }
   return true;
}


InputCamera_OV9282::InputCamera_OV9282(const char * configureFile)
{
   running = false;
   callback = NULL;
   clockOffset = 0;
   ParsePlaybackParameters( configureFile );
   findClocksOffsetForCamera();
}

InputCamera_OV9282::~InputCamera_OV9282()
{
   printf( "release camera!\n" );
}


void InputCamera_OV9282::findClocksOffsetForCamera()
{
   //int64_t dspClock = (int64_t)getDspClock();
   realClock = (int64_t)getRealTime();
   monotonicClock = getMonotonicTime();
   clockOffset = realClock - monotonicClock;
   //printf( "findClocksOffsetForCamera realClock = %" PRId64 ", monotonicClock=%" PRId64 ", clockOffset=%" PRId64 " \n ", realClock, monotonicClock, clockOffset );
}


void InputCamera_OV9282::proc()
{
   unsigned char gray_buf[IMAGE_WIDTH * IMAGE_HEIGHT];
   int64_t ts;
   
   // start pipeline
   gst_element_set_state(gst_pipeline, GST_STATE_PLAYING);
   /* wait until it's up and running or failed */
   if (gst_element_get_state(gst_pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
       printf("*** pileline failed to move to PLAYING state\n");
   }

   #ifdef OPENCV_TEST
   const auto window_gray = "Gary Image";
   namedWindow(window_gray, WINDOW_AUTOSIZE);
   #endif

   #ifdef ROS_BASED
   rclcpp::Time t;
   #endif

   while (running )
   {
       GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(gst_sink));
       if(!sample) {
          printf("*** Could not get gstreamer sample. *** \n");
          break;
       }
       GstBuffer* buf = gst_sample_get_buffer(sample);
       GstMemory* memory = gst_buffer_get_memory(buf, 0);
       GstMapInfo info;
      
       gst_memory_map(memory, &info, GST_MAP_READ);
       gsize &buf_size = info.size;
       guint8* &buf_data = info.data;
       //printf("*** phil, image buf size %lu\n", buf_size);
	  
       //prepare data
       guint8* desc_tmp = (guint8*)gray_buf;
       guint8* src_tmp = info.data;
       for(int i = 0; i < IMAGE_HEIGHT; i++)
       {
           memcpy(desc_tmp, src_tmp, IMAGE_WIDTH);
           desc_tmp += IMAGE_WIDTH;
           src_tmp += 1024;
       }

#ifdef OPENCV_TEST
       Mat gray_image(Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
       memcpy(gray_image.data, gray_buf, IMAGE_WIDTH * IMAGE_HEIGHT);
       imshow(window_gray, gray_image);
       waitKey(1);
#endif

       GstClockTime bt = gst_element_get_base_time(gst_pipeline);
	  
       #ifdef ROS_BASED
       ts = GST_TIME_AS_USECONDS(buf->pts+bt) * 1000 + time_offset;
       #else
       ts = GST_TIME_AS_USECONDS(buf->pts+bt) * 1000; 
       #endif

       callback( ts, gray_buf, NULL );
      
       #ifdef ROS_BASED
       t = rclcpp::Time(ts, RCL_ROS_TIME);
		  
       publishColor(gray_buf, t);
       //publishRGBCameraInfo(intrinsics, t);
       #endif

       if(!buf)
       {
           printf("stream end\n");
	   break;
       }
       else
       {
           gst_memory_unmap(memory, &info);
           gst_memory_unref(memory);
           gst_buffer_unref(buf);
       }
    }

    gst_element_set_state(gst_pipeline, GST_STATE_PAUSED);
    /* wait until it's up and running or failed */
    if (gst_element_get_state(gst_pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
       printf("pileline failed to move to PAUSED state \n");
    }

    gst_element_set_state(gst_pipeline, GST_STATE_READY);
    if(gst_element_get_state(gst_pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE)
        printf("gst pipeline failed to set state to READY\n");

}

#if 0
void InputCamera_OV9282::onPreviewFrame( ICameraFrame *frame )
{
	
}
#endif

gboolean InputCamera_OV9282::bus_callback (GstBus *bus, GstMessage *msg, gpointer userData)
{
    //GMainLoop *loop = (GMainLoop *)userData;

    switch (GST_MESSAGE_TYPE (msg)) {
        case GST_MESSAGE_ERROR:{
            GError *err = NULL;
            gchar *dbg;

            gst_message_parse_error (msg, &err, &dbg);
            gst_object_default_error (msg->src, err, dbg);
            g_clear_error (&err);
            g_free (dbg);
            //g_main_loop_quit (loop);
            break;
        }
        default:
            break;
    }
    return TRUE;
}

bool InputCamera_OV9282::setup_pipeline()
{
    if(!gst_is_initialized())
    {
        gst_init(0, 0);
    }

    //create elements
    gst_pipeline = gst_pipeline_new("pipeline");
	
    gst_src      = gst_element_factory_make (QMMFSRC, "camerasrc");
	g_object_set(G_OBJECT (gst_src), "name", "qmmf", 
	    "camera", CAMERA_ID, NULL);

    gst_filter   = gst_element_factory_make ("capsfilter", "filter");
    char capString[256];
    snprintf(capString, sizeof(capString),
        "video/x-raw, width=%d, height=%d, framerate=%d/1, format=NV12",
        IMAGE_WIDTH, IMAGE_HEIGHT, FPS);
    //printf("*** phil, capString: %s\n", capString);
    gst_util_set_object_arg (G_OBJECT (gst_filter), "caps", capString);

    gst_sink     = gst_element_factory_make ("appsink", "sink");

    if (gst_pipeline == NULL ||
        gst_src == NULL ||
        gst_filter == NULL ||
        gst_sink == NULL)
    {
        printf("pipeline setup failed\n");
        return false;;
    }

    //link pileline
    gst_bin_add_many (GST_BIN (gst_pipeline), gst_src, gst_filter, gst_sink, NULL);
    gst_element_link_many (gst_src, gst_filter, gst_sink, NULL);

    //gst_debug_set_default_threshold((GstDebugLevel)GST_LEVEL_TRACE);

    //set bus cb
    gst_bus_add_watch(GST_ELEMENT_BUS(gst_pipeline), bus_callback, NULL);
	
    #ifdef ROS_BASED
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    rclcpp::Time ros_time_base = ros_clock.now();
	
    GstClock * clock = gst_system_clock_obtain();
    GstClockTime ct = gst_clock_get_time(clock);
    gst_object_unref(clock);
    time_offset = ros_time_base.nanoseconds() - GST_TIME_AS_USECONDS(ct) * 1000;
    #endif

    return true;
}

bool InputCamera_OV9282::start()
{
   running = true;
   printf("**** start camera.\n");
   //config stream
   setup_pipeline();

   //camera info
   //configuration.inputPixelWidth = IMAGE_WIDTH;
   //configuration.inputPixelHeight = IMAGE_HEIGHT;
   //configuration.outputPixelWidth = IMAGE_WIDTH;
   //configuration.outputPixelHeight = IMAGE_HEIGHT;

   //memset(configuration.inputCameraMatrix, 0, sizeof(configuration.inputCameraMatrix));
#if 0
   configuration.inputCameraMatrix[0] = intrinsics.fx;
   configuration.inputCameraMatrix[2] = intrinsics.ppx;
   configuration.inputCameraMatrix[4] = intrinsics.fy;
   configuration.inputCameraMatrix[5] = intrinsics.ppy;
   configuration.inputCameraMatrix[8] = 1.f;
   memcpy(configuration.outputCameraMatrix, configuration.inputCameraMatrix, sizeof(configuration.inputCameraMatrix));
   configuration.distortionModel = rvCameraParams::NoDistortion;
   memset( configuration.distortionCoefficient, 0, sizeof( configuration.distortionCoefficient ) );
   configuration.distortionCoefficient[0] = intrinsics.coeffs[0];
   configuration.distortionCoefficient[1] = intrinsics.coeffs[1];
   configuration.distortionCoefficient[2] = intrinsics.coeffs[2];
   configuration.distortionCoefficient[3] = intrinsics.coeffs[3];
   configuration.distortionCoefficient[4] = intrinsics.coeffs[4];
#endif

   thread = std::make_shared<std::thread>(std::mem_fn(&InputCamera_OV9282::proc), this);
   return true;
}


bool InputCamera_OV9282::stop()
{
   running = false;
   if(thread)
      thread->join();

   printf("OV9282 thread exits\n");

   //gst_element_send_event(gst_pipeline, gst_event_new_eos());

   //gst_element_set_state(gst_pipeline, GST_STATE_READY);
   //if(gst_element_get_state(gst_pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE)
   //   printf("gst pipeline failed to set state to READY\n");

   gst_element_set_state(gst_pipeline, GST_STATE_NULL);
   if(gst_element_get_state(gst_pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE)
      printf("gst pipeline failed to set state to NULL\n");

   if(gst_pipeline && ((GObject *)gst_pipeline)->ref_count > 0)
      gst_object_unref(gst_pipeline);
   if(gst_src && ((GObject *)gst_src)->ref_count > 0)
      gst_object_unref(gst_src);
   if(gst_filter && ((GObject *)gst_filter)->ref_count > 0)
      gst_object_unref(gst_filter);
   if(gst_sink && ((GObject *)gst_sink)->ref_count > 0)
      gst_object_unref(gst_sink);

   return true;
}

void InputCamera_OV9282::addCallback( CameraCallback _callback )
{
   callback = _callback;
}

const rvCameraParams & InputCamera_OV9282::getCameraConfiguration( ) const
{
   return configuration;
}

bool InputCamera_OV9282::ParsePlaybackParameters( const char *configFile )
{
   bool sequenceGot = false, trajectoryGot = false;

   std::string sequenceName;
   printf( "\n****** Parsing paths......\n" );
   std::string fullName = Program_Root + configFile;
   std::ifstream cfg( fullName, std::ifstream::in );

   if( !cfg.is_open() )
   {
      printf( "Fail to open configuration file: %s\n", fullName.c_str() );
   }
   
   std::string line;
   std::string itemName;
   while( std::getline( cfg, line ) )
   {
      if( line.length() == 0 )
      {
         continue;
      }
      if( line[0] == '#' )
      {
         continue;
      }
      std::istringstream iss( line );
      itemName.clear();
      iss >> itemName;
 
      if (itemName.compare("Camera") == 0)
      {
         std::string cameraID;
         iss >> cameraID;
         GetCameraParameter(cameraID.c_str(), configuration );
         printf("Using camera ID:       %s\n", cameraID.c_str());
      }
   }
   cfg.close();
   
   return true;
}
