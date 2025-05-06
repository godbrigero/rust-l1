#include <cstring>  // For strcpy
#include <iostream>

#include "unitree_lidar_sdk.h"

using namespace unitree_lidar_sdk;

struct PointCloudUnitree_Dynamic {
  double stamp;
  uint32_t id;
  uint32_t ringNum;
  PointUnitree* points_ptr;
  size_t points_len;
  size_t points_capacity;
};

extern "C" {

void getCloud(UnitreeLidarReader* reader, PointCloudUnitree_Dynamic* cloud) {
  if (!reader || !cloud) return;

  PointCloudUnitree cloudCpp = reader->getCloud();

  cloud->stamp = cloudCpp.stamp;
  cloud->id = cloudCpp.id;
  cloud->ringNum = cloudCpp.ringNum;

  size_t numPoints = cloudCpp.points.size();

  if (numPoints > 0) {
    cloud->points_ptr = new PointUnitree[numPoints];
    if (cloud->points_ptr) {
      for (size_t i = 0; i < numPoints; i++) {
        cloud->points_ptr[i] = cloudCpp.points[i];
      }
      cloud->points_len = numPoints;
      cloud->points_capacity = numPoints;
    } else {
      cloud->points_len = 0;
      cloud->points_capacity = 0;
    }
  } else {
    cloud->points_ptr = nullptr;
    cloud->points_len = 0;
    cloud->points_capacity = 0;
  }
}

void freePointCloudMemory(PointUnitree* points_ptr) {
  if (points_ptr) {
    delete[] points_ptr;
  }
}

UnitreeLidarReader* createUnitreeLidarReaderCpp() {
  return createUnitreeLidarReader();
}

int initialize(UnitreeLidarReader* reader, uint16_t cloud_scan_num,
               const char* port, uint32_t baudrate, float rotate_yaw_bias,
               float range_scale, float range_bias, float range_max,
               float range_min) {
  if (!reader) return -1;
  try {
    return reader->initialize(cloud_scan_num, std::string(port), baudrate,
                              rotate_yaw_bias, range_scale, range_bias,
                              range_max, range_min);
  } catch (const std::exception& e) {
    std::cerr << "Error initializing LiDAR: " << e.what() << std::endl;
    return -1;
  }
}

MessageType runParse(UnitreeLidarReader* reader) {
  if (!reader) return MessageType::NONE;
  return reader->runParse();
}

void getVersionOfFirmware(UnitreeLidarReader* reader, char* buffer,
                          size_t buffer_size) {
  if (!reader || !buffer) return;
  std::string version = reader->getVersionOfFirmware();
  strncpy(buffer, version.c_str(), buffer_size - 1);
  buffer[buffer_size - 1] = '\0';
}

void getVersionOfSDK(UnitreeLidarReader* reader, char* buffer,
                     size_t buffer_size) {
  if (!reader || !buffer) return;
  std::string version = reader->getVersionOfSDK();
  strncpy(buffer, version.c_str(), buffer_size - 1);
  buffer[buffer_size - 1] = '\0';
}

void reset(UnitreeLidarReader* reader) {
  if (!reader) return;
  reader->reset();
}

void setLidarWorkingMode(UnitreeLidarReader* reader, LidarWorkingMode mode) {
  if (!reader) return;
  reader->setLidarWorkingMode(mode);
}

void delete_reader(UnitreeLidarReader* reader) {
  if (reader) {
    delete reader;
  }
}

}  // extern "C"
