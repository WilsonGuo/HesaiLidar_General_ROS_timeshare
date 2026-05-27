#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
#include <fstream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <pwd.h>
#include <cerrno>
#include <cstring>
#include <cstdint>
// #define PRINT_FLAG 

using namespace std;

struct time_stamp
{
  uint64_t low;   // LiDAR timestamp in nanoseconds
};

class HesaiLidarClient
{
public:
  HesaiLidarClient(ros::NodeHandle node, ros::NodeHandle nh)
    : hsdk(nullptr), pointt(nullptr), timeshare_fd(-1)
  {
    initTimeshare();
    lidarPublisher = node.advertise<sensor_msgs::PointCloud2>("pandar", 10);
    packetPublisher = node.advertise<hesai_lidar::PandarScan>("pandar_packets",10);

    string serverIp;
    int lidarRecvPort;
    int gpsPort;
    double startAngle;
    string lidarCorrectionFile;  // Get local correction when getting from lidar failed
    string lidarType;
    string frameId;
    int pclDataType;
    string pcapFile;
    string dataType;
    string multicastIp;
    bool coordinateCorrectionFlag;
    string targetFrame;
    string fixedFrame;

    nh.getParam("pcap_file", pcapFile);
    nh.getParam("server_ip", serverIp);
    nh.getParam("lidar_recv_port", lidarRecvPort);
    nh.getParam("gps_port", gpsPort);
    nh.getParam("start_angle", startAngle);
    nh.getParam("lidar_correction_file", lidarCorrectionFile);
    nh.getParam("lidar_type", lidarType);
    nh.getParam("frame_id", frameId);
    nh.getParam("pcldata_type", pclDataType);
    nh.getParam("publish_type", m_sPublishType);
    nh.getParam("timestamp_type", m_sTimestampType);
    nh.getParam("data_type", dataType);
    nh.getParam("multicast_ip", multicastIp);
    nh.getParam("coordinate_correction_flag", coordinateCorrectionFlag);
    nh.getParam("target_frame", targetFrame);
    nh.getParam("fixed_frame", fixedFrame);
  
    if(!pcapFile.empty()){
      hsdk = new PandarGeneralSDK(pcapFile, boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType, lidarCorrectionFile, \
      coordinateCorrectionFlag, targetFrame, fixedFrame);
      if (hsdk != NULL) {
        std::ifstream fin(lidarCorrectionFile);
        if (fin.is_open()) {
          std::cout << "Open correction file " << lidarCorrectionFile << " succeed" << std::endl;
          int length = 0;
          std::string strlidarCalibration;
          fin.seekg(0, std::ios::end);
          length = fin.tellg();
          fin.seekg(0, std::ios::beg);
          char *buffer = new char[length];
          fin.read(buffer, length);
          fin.close();
          strlidarCalibration = buffer;
          int ret = hsdk->LoadLidarCorrectionFile(strlidarCalibration);
          if (ret != 0) {
            std::cout << "Load correction file from " << lidarCorrectionFile <<" failed" << std::endl;
          } else {
            std::cout << "Load correction file from " << lidarCorrectionFile << " succeed" << std::endl;
          }
        }
        else{
          std::cout << "Open correction file " << lidarCorrectionFile << " failed" << std::endl;
        }
      }
    }
    else if ("rosbag" == dataType){
      hsdk = new PandarGeneralSDK("", boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType, \
      lidarCorrectionFile, coordinateCorrectionFlag, targetFrame, fixedFrame);
      if (hsdk != NULL) {
        packetSubscriber = node.subscribe("pandar_packets",10,&HesaiLidarClient::scanCallback, (HesaiLidarClient*)this, ros::TransportHints().tcpNoDelay(true));
      }
    }
    else {
      hsdk = new PandarGeneralSDK(serverIp, lidarRecvPort, gpsPort, \
        boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
        boost::bind(&HesaiLidarClient::gpsCallback, this, _1), static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId,\
         m_sTimestampType, lidarCorrectionFile, multicastIp, coordinateCorrectionFlag, targetFrame, fixedFrame);
    }
    
    if (hsdk != NULL) {
        hsdk->Start();
        // hsdk->LoadLidarCorrectionFile("...");  // parameter is stream in lidarCorrectionFile
    } else {
        printf("create sdk fail\n");
    }
  }


  ~HesaiLidarClient()
  {
    if (pointt != nullptr && pointt != MAP_FAILED) {
      munmap(pointt, sizeof(time_stamp));
      pointt = nullptr;
    }
    if (timeshare_fd >= 0) {
      close(timeshare_fd);
      timeshare_fd = -1;
    }
    if (hsdk != nullptr) {
      delete hsdk;
      hsdk = nullptr;
    }
  }

  void initTimeshare()
  {
    const char *home = getenv("HOME");
    std::string path_for_time_stamp;
    if (home && strlen(home) > 0) {
      path_for_time_stamp = std::string(home) + "/timeshare";
    } else {
      struct passwd *pw = getpwuid(getuid());
      if (pw && pw->pw_dir) {
        path_for_time_stamp = std::string(pw->pw_dir) + "/timeshare";
      } else {
        ROS_ERROR("timeshare: cannot get HOME directory");
        return;
      }
    }

    timeshare_fd = open(path_for_time_stamp.c_str(), O_CREAT | O_RDWR, 0666);
    if (timeshare_fd < 0) {
      ROS_ERROR("timeshare: open %s failed: %s", path_for_time_stamp.c_str(), strerror(errno));
      return;
    }

    if (ftruncate(timeshare_fd, sizeof(time_stamp)) != 0) {
      ROS_ERROR("timeshare: ftruncate failed: %s", strerror(errno));
      close(timeshare_fd);
      timeshare_fd = -1;
      return;
    }

    void *addr = mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE, MAP_SHARED, timeshare_fd, 0);
    if (addr == MAP_FAILED) {
      ROS_ERROR("timeshare: mmap failed: %s", strerror(errno));
      close(timeshare_fd);
      timeshare_fd = -1;
      pointt = nullptr;
      return;
    }

    pointt = static_cast<time_stamp *>(addr);
    pointt->low = 0;
    ROS_INFO("timeshare: mapped %s", path_for_time_stamp.c_str());
  }

  void writeTimeshare(double timestamp_sec)
  {
    if (pointt == nullptr || pointt == MAP_FAILED) {
      return;
    }
    if (timestamp_sec <= 0.0) {
      return;
    }
    pointt->low = static_cast<uint64_t>(timestamp_sec * 1000000000.0);
  }

  void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp, hesai_lidar::PandarScanPtr scan) // the timestamp from first point cloud of cld
  {
    writeTimeshare(timestamp);
    if(m_sPublishType == "both" || m_sPublishType == "points"){
      pcl_conversions::toPCL(ros::Time(timestamp), cld->header.stamp);
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*cld, output);
      lidarPublisher.publish(output);
#ifdef PRINT_FLAG
        printf("timestamp: %f, point size: %ld.\n",timestamp, cld->points.size());
#endif        
    }
    if(m_sPublishType == "both" || m_sPublishType == "raw"){
      packetPublisher.publish(scan);
#ifdef PRINT_FLAG
        printf("raw size: %d.\n", scan->packets.size());
#endif
    }
  }

  void gpsCallback(int timestamp) {
#ifdef PRINT_FLAG
      printf("gps: %d\n", timestamp);
#endif      
  }

  void scanCallback(const hesai_lidar::PandarScanPtr scan)
  {
    // printf("pandar_packets topic message received,\n");
    hsdk->PushScanPacket(scan);
  }

private:
  ros::Publisher lidarPublisher;
  ros::Publisher packetPublisher;
  PandarGeneralSDK* hsdk;
  time_stamp *pointt;
  int timeshare_fd;
  string m_sPublishType;
  string m_sTimestampType;
  ros::Subscriber packetSubscriber;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pandar");
  ros::NodeHandle nh("~");
  ros::NodeHandle node;
  HesaiLidarClient pandarClient(node, nh);

  ros::spin();
  return 0;
}
