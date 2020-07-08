#pragma once

#include "ros/ros.h"
#include "ros/time.h"
#include "md5.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Base64.h"
#include <chrono>
#include <serial/serial.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "birdy_config/alert.h"
#include "birdy_config/toweb.h"
#include "std_msgs/String.h"

using namespace std;
#include <rapidjson/document.h>
#include <rapidjson/rapidjson.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#define SOCK_BUFSIZE (20 * 1024)  // buffer size
#define MAX_PLANES 6

static std::string g_socket_address = "/tmp/test_ipc";

enum CNPkgType {
  CNPKG_INVALID = -1,     ///< invalid package type
  CNPKG_DATA = 0,         ///< data package
  CNPKG_RELEASE_MEM = 1,  ///< package with release shared memory info
  CNPKG_EXIT = 2,         ///< package with exit info
  CNPKG_ERROR = 3         ///< package with error info
};

typedef enum {
  PIXEL_FORMAT_YUV420_NV21 = 0,  ///< This frame is in the YUV420SP(NV21) format.
} DataFormat;

typedef struct {
  float x, y, w, h;
} InferBoundingBox;

typedef struct {
  InferBoundingBox bbox;  ///< position info for object
  float score;            ///< score
  std::string label_id;   ///< label id
} InferObjInfo;

typedef struct {
  CNPkgType pkg_type;                     ///< package type
  uint32_t channel_idx = 0;               ///< The index of the channel, stream_index
  std::string stream_id;                  ///< The data stream aliases where this frame is located to.
  size_t flags = 0;                       ///< The mask for this frame, ``CNFrameFlag``.
  uint64_t frame_id;                      ///< The frame index that incremented from 0.
  uint64_t timestamp;                     ///< The time stamp of this frame.
  DataFormat fmt;                         ///< The format of the frame.
  int width;                              ///< The width of the frame.
  int height;                             ///< The height of the frame.
  int stride[MAX_PLANES];                 ///< The strides of the frame.
  std::vector<InferObjInfo> detect_objs;  ///< detection objects.
} InferFramePackage;

class Monitor{
private:
    //ROS related var
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    ros::Publisher _pub;
    ros::Timer _delayTimer;
    
    //Web server related var
    string _usr;
    string _pwd;

    //Serial related var
    serial::Serial _s;
    string _com;
    int32_t _baud;

    //Socket related var
    fd_set _master_fds;  //Master file descriptor for server mode
    fd_set _temp_fds;    //Temporary fd for select() to show which FD has new arrival
    int32_t _fdmax;          //Index keep the number of total socket FD for server mode
    int32_t _listener;   //Listener socket FD
    struct timeval tv;   //Timeout for select()  
    int32_t _port;

    int32_t _PLAY_BACK;
    int32_t _ENABLE_UART;
    int32_t _ENABLE_TCP;
    
    //Data buffer
    #define read_bufSize 64
    #define send_bufSize 400000
    u_int8_t _read_buffer[read_bufSize];
    u_int8_t _send_buffer[send_bufSize];
    uint32_t _length;
    void _alertCallback(const birdy_config::alert::ConstPtr& msg);

    void _delayTimerCallback(const ros::TimerEvent &event);
    bool _delayOver = true;

    bool _parseJsonStrToDataPackage(const std::string& str, InferFramePackage* pkg);
    bool _prepareSendPackage(const CNPkgType& pkg_type, const int& channel_idx, const std::string& stream_id,
                        const uint64_t& frame_id, std::string* str);
    void _readImgFromSharedMem(const InferFramePackage& frame_pkg, char* img_buffer);
    bool _ipc_run(void);

public:
    Monitor();
    ~Monitor();

    bool serial_init();
    bool serial_send();
    bool tcp_server_init(int32_t port, int64_t sec, int64_t micro_sec);
    bool tcp_run();
    void tcp_close_all();
    void run();
};
