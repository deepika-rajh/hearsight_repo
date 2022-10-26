/*******************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "ParseSensorParam.h"
#include <fstream>
#include <sstream>
#include "memory.h"
#include "math.h"

void EulerToSO3_0( const float32_t* euler, float32_t* rotation );
bool ReadIMUParamters( const char * imuFile, rvIMUConfiguration & imuParameter );

bool ParseSensorParam( const std::string & root, const std::string & configFile, rvWheelConfiguration & wheelConf, rvIMUConfiguration & imuConf, rvTargetImage &targetImage )
{
   bool crossT = false, crossR = false;
   std::ifstream cfg( root+configFile, std::ifstream::in );
   std::string tempRoot = root;

   if( !cfg.is_open() )
   {
      printf( "Fail to open configuration file: %s\n", configFile.c_str() );
      cfg.open( root + "../" + configFile, std::ifstream::in );
      if( !cfg.is_open() )
      {
         printf( "Fail to open configuration file: %s also.\n", ("../" + configFile).c_str() );
         return false;
      }
      tempRoot = root + "../";
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
      if( itemName.compare( "WEF.Tvb" ) == 0 )
      {
         float translation[3];
         iss >> translation[0] >> translation[1] >> translation[2];
         wheelConf.baselinkInCamera.matrix[0][3] = translation[0];
         wheelConf.baselinkInCamera.matrix[1][3] = translation[1];
         wheelConf.baselinkInCamera.matrix[2][3] = translation[2]; 
         crossT = true;
      }
      else if( itemName.compare( "WEF.Rvb" ) == 0 )
      {
         float euler[3];
         iss >> euler[0] >> euler[1] >> euler[2];
         //https://en.wikipedia.org/wiki/Euler_angles#Tait%E2%80%93Bryan_angles
         //Section Conversion to other orientation representations->Rotation matrix
         //This euler angle is defined as Z1Y2X3 according to the conversion table in the section mentioned above
         //Which is different from the defintion of mvPose6DET in mv.h
         float rotation[9];
         EulerToSO3_0( euler, rotation );
         memcpy( wheelConf.baselinkInCamera.matrix[0], rotation + 0, sizeof( float ) * 3 );
         memcpy( wheelConf.baselinkInCamera.matrix[1], rotation + 3, sizeof( float ) * 3 );
         memcpy( wheelConf.baselinkInCamera.matrix[2], rotation + 6, sizeof( float ) * 3 );
         crossR = true;
      }else if( itemName.compare( "IMU" ) == 0 )
      {
         std::string imuFile;
         iss >> imuFile;
         imuFile = root + imuFile;
		 printf("root File path is: %s ,imu file path is: %s\n", root.c_str(), imuFile.c_str());
         ReadIMUParamters( imuFile.c_str(), imuConf );
      }
      else if (itemName.compare("TargetImage") == 0)
      {
          std::string path;
          iss >> path;
          targetImage.path = tempRoot + path;
      }
      else if (itemName.compare("TargetWidth") == 0)
      {
          iss >> targetImage.targetWidth;
      }
      else if (itemName.compare("TargetHeight") == 0)
      {
          iss >> targetImage.targetHeight;
      }
   }
   if( crossR && crossT )
   {
      wheelConf.wheelEnabled = true;
   }
   return true;
}

void EulerToSO3_0( const float32_t* euler, float32_t* rotation )
{
   float32_t cr = (float32_t)cos( euler[0] );
   float32_t sr = (float32_t)sin( euler[0] );
   float32_t cp = (float32_t)cos( euler[1] );
   float32_t sp = (float32_t)sin( euler[1] );
   float32_t cy = (float32_t)cos( euler[2] );
   float32_t sy = (float32_t)sin( euler[2] );
   rotation[0 * 3 + 0] = cy*cp;
   rotation[0 * 3 + 1] = cy*sp*sr - sy*cr;
   rotation[0 * 3 + 2] = cy*sp*cr + sy*sr;
   rotation[1 * 3 + 0] = sy*cp;
   rotation[1 * 3 + 1] = sy*sp*sr + cy*cr;
   rotation[1 * 3 + 2] = sy*sp*cr - cy*sr;
   rotation[2 * 3 + 0] = -sp;
   rotation[2 * 3 + 1] = cp*sr;
   rotation[2 * 3 + 2] = cp*cr;
}

bool ReadIMUParamters( const char * imuFile, rvIMUConfiguration & imuParameter )
{
   std::string fullName = imuFile;
   std::ifstream cfg( fullName, std::ifstream::in );
   if( !cfg.is_open() )
   {
      printf( "Fail to open imu configuration file: %s\n", fullName.c_str() );
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
      if( itemName.compare( "delta:" ) == 0 )
      {
         iss >> imuParameter.deltaInSecond;
      }
      else if( itemName.compare( "Accelerator_bias:" ) == 0 )
      {
         ReadMatrix( cfg, imuParameter.acceBias );
      }
      else if( itemName.compare( "Gyro_bias:" ) == 0 )
      {
         ReadMatrix( cfg, imuParameter.gyroBias );
      }
      else if( itemName.compare( "Camera_in_IMU:" ) == 0 )
      {
         float p[12];
         ReadMatrix( cfg, p );
         for( size_t i = 0, k = 0; i < 3; i++ )
            for( size_t j = 0; j < 4; j++, k++ )
            {
               imuParameter.cameraInIMU.matrix[i][j] = p[k];
            }
      }
   }

   return true;
}


void ReadMatrix( std::ifstream & file, float * matrix )
{
   std::string line, valName;
   int rows=0, cols=0;

   std::getline( file, line );
   std::istringstream issRow ( line );
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
