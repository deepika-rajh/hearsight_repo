/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include <stdlib.h>
#include <unistd.h>
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
#define FPS          15
#define CAMERA_ID    1   //ov9282 camera ID

int64_t InputCamera_OV9282::time_offset = 0;
CameraCallback InputCamera_OV9282::callback = nullptr;

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
   gst_pipeline = NULL;
   gst_loop = NULL;

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

void InputCamera_OV9282::getRawImgProc()
{
  // Run main loop.
  g_main_loop_run (gst_loop);

  gst_element_set_state (gst_pipeline, GST_STATE_NULL);

}

static unsigned char gray_buf[IMAGE_WIDTH * IMAGE_HEIGHT];
static int64_t raw_img_last_ts = 0; 
GstFlowReturn InputCamera_OV9282::new_sample_cb (GstElement *sink, gpointer userdata)
{
    GstElement *pipeline = GST_ELEMENT (userdata);
    GstSample *sample = NULL;
    GstBuffer *buffer = NULL;
    GstMapInfo info;
    int64_t ts; 

    GstClock * clock = gst_system_clock_obtain();
    GstClockTime ct1 = gst_clock_get_time(clock);
    //gst_object_unref(clock);

   #ifdef ROS_BASED
   rclcpp::Time t;
   #endif

    g_signal_emit_by_name (sink, "pull-sample", &sample);
    if (sample == NULL) {
        printf ("ERROR: Pulled sample is NULL!\n");
        return GST_FLOW_ERROR;
    }

    if ((buffer = gst_sample_get_buffer (sample)) == NULL) {
        printf ("ERROR: Pulled buffer is NULL!");
        gst_sample_unref (sample);
        return GST_FLOW_ERROR;
    }

    if (!gst_buffer_map (buffer, &info, GST_MAP_READ)) {
        printf ("ERROR: Failed to map the pulled buffer!");
        gst_sample_unref (sample);
        return GST_FLOW_ERROR;
    }

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

    GstClockTime bt = gst_element_get_base_time(pipeline);
    ts = GST_TIME_AS_USECONDS(buffer->pts+bt) * 1000 + time_offset;

    printf("*** raw image interval %ldms\n",  (ts - raw_img_last_ts)/1000000);
    raw_img_last_ts = ts;

    gst_buffer_unmap (buffer, &info);
    gst_sample_unref (sample);

    GstClockTime ct2 = gst_clock_get_time(clock);
    gst_object_unref(clock);
    int64_t callback_duration = GST_TIME_AS_USECONDS(ct2) - GST_TIME_AS_USECONDS(ct1);
    printf("new_sample_cb interval %ldus\n", callback_duration);

    #ifdef ROS_BASED
    t = rclcpp::Time(ts, RCL_ROS_TIME);
		  
    publishColor(gray_buf, t);
    #endif

    callback( ts, gray_buf, NULL );
    
    return GST_FLOW_OK;
}

void InputCamera_OV9282::state_change_cb (GstBus *bus, GstMessage *msg, gpointer userData)
{
    GstElement *pipeline = GST_ELEMENT (userData);
    GstState old, new_, pending;

    if (GST_MESSAGE_SRC (msg) != GST_OBJECT_CAST (pipeline))
        return;

    gst_message_parse_state_changed (msg, &old, &new_, &pending);
    printf ("Pipeline state changed from %s to %s, pending: %s\n",
        gst_element_state_get_name (old), gst_element_state_get_name (new_),
        gst_element_state_get_name (pending));

    if ((new_ == GST_STATE_PAUSED) && (old == GST_STATE_READY) && (pending == GST_STATE_VOID_PENDING)) {
        printf ("Setting pipeline to PLAYING state ...\n");

        if (gst_element_set_state (pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            gst_printerr ("Pipeline doesn't want to transition to PLAYING state!\n");
            return;
        }
    }
}

void InputCamera_OV9282::error_callback (GstBus *bus, GstMessage *msg, gpointer userData)
{
    GMainLoop *mloop = (GMainLoop*) userData;
    GError *error = NULL;
    gchar *debug = NULL;

    gst_message_parse_error (msg, &error, &debug);
    gst_object_default_error (GST_MESSAGE_SRC (msg), error, debug);

    g_free (debug);
    g_error_free (error);	

    g_main_loop_quit (mloop);
}

void InputCamera_OV9282::eos_callback (GstBus *bus, GstMessage *msg, gpointer userData)
{
    GMainLoop *mloop = (GMainLoop*) userData;
    printf("received eos event\n");
    g_main_loop_quit (mloop);
}

bool InputCamera_OV9282::setup_pipeline()
{
    gst_init(0, 0);

    {
    GError *error = NULL;

    char pipeline_str[256];
    snprintf(pipeline_str, sizeof(pipeline_str),
         "qtiqmmfsrc name=camera camera=1 ! video/x-raw, width=%d, height=%d, framerate=%d/1, format=NV12 ! appsink name=sink enable-last-sample=false async=false emit-signals=true",
           IMAGE_WIDTH, IMAGE_HEIGHT, FPS);

    gst_pipeline = gst_parse_launch (pipeline_str, &error);
    gst_debug_set_default_threshold((GstDebugLevel)3);
    //gst_debug_set_threshold_for_name("qtiqmmfsrc", (GstDebugLevel)7);
    gst_debug_set_colored(FALSE);

    // Check for errors on pipe creation.
    if ((NULL == gst_pipeline) && (error != NULL)) {
      g_printerr ("Failed to create pipeline, error: %s!\n",
          GST_STR_NULL (error->message));
      g_clear_error (&error);
      return false;
    } else if ((NULL == gst_pipeline) && (NULL == error)) {
      g_printerr ("Failed to create pipeline, unknown error!\n");
      return false;
    } else if ((gst_pipeline != NULL) && (error != NULL)) {
      g_printerr ("Erroneous pipeline, error: %s!\n",
          GST_STR_NULL (error->message));
      g_clear_error (&error);
      gst_object_unref (gst_pipeline);
      return false;
    }
    }

  // Initialize main loop.
  if ((gst_loop = g_main_loop_new (NULL, FALSE)) == NULL) {
    g_printerr ("ERROR: Failed to create Main loop!\n");
    gst_object_unref (gst_pipeline);
    return false;
  }

  {
    GstBus *bus = NULL;

    // Retrieve reference to the pipeline's bus.
    if ((bus = gst_pipeline_get_bus (GST_PIPELINE (gst_pipeline))) == NULL) {
      g_printerr ("ERROR: Failed to retrieve pipeline bus!\n");

      g_main_loop_unref (gst_loop);
      gst_object_unref (gst_pipeline);

      return false;
    }

    // Watch for messages on the pipeline's bus.
    gst_bus_add_signal_watch (bus);

    g_signal_connect (bus, "message::state-changed",
        G_CALLBACK (state_change_cb), gst_pipeline);
    //g_signal_connect (bus, "message::warning", G_CALLBACK (warning_cb), NULL);
    g_signal_connect (bus, "message::error", G_CALLBACK (error_callback), gst_loop);
    g_signal_connect (bus, "message::eos", G_CALLBACK (eos_callback), gst_loop);

    gst_object_unref (bus);
  }

  // Connect a callback to the new-sample signal.
  {
    GstElement *element = gst_bin_get_by_name (GST_BIN (gst_pipeline), "sink");
    g_signal_connect (element, "new-sample", G_CALLBACK (new_sample_cb), gst_pipeline);
  }

  g_print ("Setting pipeline to PAUSED state ...\n");

  switch (gst_element_set_state (gst_pipeline, GST_STATE_PAUSED)) {
    case GST_STATE_CHANGE_FAILURE:
      g_printerr ("ERROR: Failed to transition to PAUSED state!\n");
      break;
    case GST_STATE_CHANGE_NO_PREROLL:
      g_print ("Pipeline is live and does not need PREROLL.\n");
      break;
    case GST_STATE_CHANGE_ASYNC:
      g_print ("Pipeline is PREROLLING ...\n");
      break;
    case GST_STATE_CHANGE_SUCCESS:
      g_print ("Pipeline state change was successful\n");
      break;
  }

    GstClock * clock = gst_system_clock_obtain();
    GstClockTime ct = gst_clock_get_time(clock);
    gst_object_unref(clock);
    #ifdef ROS_BASED
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    rclcpp::Time ros_time_base = ros_clock.now();
	
    time_offset = ros_time_base.nanoseconds() - GST_TIME_AS_USECONDS(ct) * 1000;
    #else
    int64_t current_time = (int64_t)getRealTime();
    time_offset = current_time - GST_TIME_AS_USECONDS(ct) * 1000;
    #endif

    return true;
}

bool InputCamera_OV9282::start()
{
   running = true;
   printf("**** start camera.\n");
   //config stream
   setup_pipeline();

   rawImgThread = std::make_shared<std::thread>(std::mem_fn(&InputCamera_OV9282::getRawImgProc), this);
   return true;
}


bool InputCamera_OV9282::stop()
{
   running = false;
   gst_element_send_event(gst_pipeline, gst_event_new_eos());
   if(rawImgThread)
      rawImgThread->join();

   printf("OV9282 thread exits\n");

   g_main_loop_unref (gst_loop);
   gst_object_unref (gst_pipeline);

   gst_deinit ();

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
