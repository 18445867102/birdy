#include "monitor.h"

using namespace std;

Monitor::Monitor()
{
    int32_t sub_queue,pub_queue;
    _nh.getParam("monitor_sub_queue",sub_queue);
    _nh.getParam("monitor_pub_queue",pub_queue);
    _sub = _nh.subscribe("income_alert",sub_queue,&Monitor::_alertCallback, this);
    _pub = _nh.advertise<birdy_config::toweb>("toweb_msg",pub_queue);
    _delayTimer = _nh.createTimer(ros::Duration(1), &Monitor::_delayTimerCallback, this, true, false);
    _nh.getParam("usr",_usr);
    _nh.getParam("pwd",_pwd);
    _nh.getParam("monitor_playback",_PLAY_BACK);
    _nh.getParam("enable_uart",_ENABLE_UART);
    _nh.getParam("com_port",_com);
    _nh.getParam("baud",_baud);
    _nh.getParam("enable_tcp",_ENABLE_TCP);
    _nh.getParam("tcp_port",_port);
    _nh.getParam("ros_ip", _ros_ip);

    _pwd = md5(_usr+_pwd);
    if(_ENABLE_UART)
    {
        serial_init();
    }
    if(_ENABLE_TCP)
    {
        tcp_server_init(_port,0,1000);
    }
    if(_PLAY_BACK)
    {
        cv::namedWindow("monitor_view");
        cv::startWindowThread();
    }
}

Monitor::~Monitor()
{      
    if(_PLAY_BACK)
    {
        cv::destroyWindow("monitor_view");
    }
    if(_ENABLE_TCP)
    {
        tcp_close_all();
    }
    if(_ENABLE_UART)
    {
        _s.flush();
        _s.close();
    }
}

void Monitor::_alertCallback(const birdy_config::alert::ConstPtr& msg)
{
    if(_delayOver == true)
    {
        _delayOver = false;
        _delayTimer.setPeriod(ros::Duration(1));
        _delayTimer.start();
    } else
    {
        return;
    }
    
    //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    
    cv::Mat matOrignal(msg->height,msg->width,msg->encoding,const_cast<uchar*>(&msg->image[0]),msg->step);
    
    cv::Mat matTarget;
    cv::resize(matOrignal, matTarget, cv::Size(msg->width/2, msg->height/2));

    std::vector<uint8_t> buf;
    cv::imencode(".jpg",matTarget,buf);
    auto *enc_msg = reinterpret_cast<unsigned char*>(buf.data());

    birdy_config::toweb m;
    m.account = _usr;
    m.birdImageBase64 = base64_encode(enc_msg,buf.size());
    m.birdNum = msg->birdnum;
    m.birdType = msg->birdtype;
    m.degree = msg->degree;
    m.deviceUuid = msg->uuid;
    m.heightRange = msg->hightrange;
    m.pwd = _pwd;
    m.remark = "";
    m.repeStatus = msg->status;
    m.warningTime = boost::posix_time::to_iso_extended_string(msg->header.stamp.toBoost());
    _pub.publish(m);
    
    if(_ENABLE_TCP || _ENABLE_UART)
    {
        _send_buffer[0] = 0xaa;
        _send_buffer[1] = 0x55;
        _send_buffer[2] = ((uint32_t)msg->birdnum) >> 24;
        _send_buffer[3] = ((uint32_t)msg->birdnum) >> 16; 
        _send_buffer[4] = ((uint32_t)msg->birdnum) >> 8;
        _send_buffer[5] = ((uint32_t)msg->birdnum) ;
        _send_buffer[6] = ((uint32_t)msg->degree) >> 24;
        _send_buffer[7] = ((uint32_t)msg->degree) >> 16; 
        _send_buffer[8] = ((uint32_t)msg->degree) >> 8;
        _send_buffer[9] = ((uint32_t)msg->degree) ;
        _send_buffer[10] = ((uint32_t)msg->status) >> 24;
        _send_buffer[11] = ((uint32_t)msg->status) >> 16; 
        _send_buffer[12] = ((uint32_t)msg->status) >> 8;
        _send_buffer[13] = ((uint32_t)msg->status) ;
        _length = 14;
        size_t len;
        len = _usr.length();
        memcpy(_send_buffer+_length,_usr.c_str(),len);
        _length += len;
        len = _pwd.length();
        memcpy(_send_buffer+_length,_pwd.c_str(),len);
        _length += len;
        len = msg->uuid.length();
        memcpy(_send_buffer+_length,msg->uuid.c_str(),len);
        _length += len;
        len = msg->birdtype.length();
        memcpy(_send_buffer+_length,msg->birdtype.c_str(),len);
        _length += len;
        len = msg->hightrange.length();
        memcpy(_send_buffer+_length,msg->hightrange.c_str(),len);
        _length += len;
        string temp = boost::posix_time::to_iso_extended_string(msg->header.stamp.toBoost());
        len = temp.length();
        memcpy(_send_buffer+_length,temp.c_str(),len);
        _length += len;
        
        temp = base64_encode(enc_msg,buf.size());

        _length = 0;
        len = temp.length();
        memcpy(_send_buffer+_length,temp.c_str(),len);
        _length += len;
        temp = "";
        len = temp.length();
        memcpy(_send_buffer+_length,temp.c_str(),len);
        _length += len;
    }
    ROS_INFO("%d birds detected by device %s. Type: %s Action: %d.",m.birdNum,m.deviceUuid.c_str(), m.birdType.c_str(),m.repeStatus);
    //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    //std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    if(_PLAY_BACK)
    {
        try
        {
            cv::imshow("monitor_view", matTarget);
            cv::waitKey(1);
        }
        catch(cv::Exception& ex)
        {
            ROS_ERROR("Opencv imshow exception.");
        }

    }
}

void Monitor::_delayTimerCallback(const ros::TimerEvent &event)
{
    _delayOver = true;
}


bool Monitor::serial_init()
{
    try
    {
        _s.setPort(_com);
        _s.setBaudrate(_baud);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        _s.setTimeout(to);
        _s.open();
    }
    catch(serial::IOException &e)
    {
        ROS_ERROR("Unable to open port %s",_com.c_str());
        return 0;
    }
    ROS_INFO("Open port %s at baud rate %d",_com.c_str(),_baud);    
}

bool Monitor::serial_send()
{
    _s.write(_send_buffer,_length);
}

bool Monitor::tcp_server_init(int32_t port, int64_t sec, int64_t micro_sec){
    struct sockaddr_in  servaddr;
    FD_ZERO(&_master_fds);
    FD_ZERO(&_temp_fds);

    tv.tv_sec = sec;
    tv.tv_usec = micro_sec;

    /*Socket() get handler*/
    if((_listener = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        ROS_ERROR("Server-socket() error.");
        close(_listener);
        return false;
    }
    ROS_INFO("Server-socket() is OK");

    /*Check if already in use*/
    int32_t yes = 1;
    if((setsockopt(_listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int32_t)) == -1))
    {   
        ROS_ERROR("Server-setsockopt() error.");
        return false;
    }
    ROS_INFO("Server-setsockopt() is OK");

    /*Bind()*/
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET; //协议类型
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);//ip地址
    servaddr.sin_port = htons(port);//端口号
    if((bind(_listener,(sockaddr*)&servaddr,sizeof(servaddr))) == -1)
    { //绑定地址与端口
        ROS_ERROR("Server-bind() error.");
        return false;
    }
    ROS_INFO("Server_bind() is OK");

    /*Listen*/
    if(listen(_listener,10) < 0){//开始监听
        ROS_ERROR("Server-listen() error.");
        return false;
    }
    ROS_INFO("Sever_listen() is OK");

    FD_SET(_listener, &_master_fds);
    _fdmax = _listener;
    return true;
}

bool Monitor::tcp_run(void)
{
    _temp_fds = _master_fds;
    if((select(_fdmax+1, &_temp_fds,NULL,NULL,&tv)) == -1)
    {
        ROS_ERROR("Server-select() error.");
        ROS_INFO("return on select()");
        return false;
    }
    //ROS_INFO("Server-select() is OK");

    /*Go through existing active FD for new data or connection*/ 
    for(int32_t i = 0; i <= _fdmax; i++)
    {
        if(FD_ISSET(i,&_temp_fds))
        {
            /*New connection on listening port*/
            if(i == _listener)
            {
                struct sockaddr_in  clientaddr;
                socklen_t addrlen = sizeof(sockaddr_in);
                int32_t newfd;
                if((newfd = accept(_listener,(struct sockaddr *)&clientaddr, &addrlen)) == -1)
                {
                    ROS_ERROR("Server-accept() error.");
                }
                else
                {
                    //ROS_INFO("Server-accept() is OK");
                    FD_SET(newfd, &_master_fds);
                    if(newfd > _fdmax)
                        _fdmax = newfd;
                }
                ROS_INFO("New connection from %s on socket %d\n", inet_ntoa(clientaddr.sin_addr), newfd);  
            }
            else
            {
                /* Handel new data from connected client */
                int32_t nbytes;
                if ((nbytes = recv(i, _read_buffer, sizeof(_read_buffer), 0)) <= 0)
                {
                    /* got error or connection closed by client */
                    if (nbytes == 0)
                        /* connection closed */
                        ROS_INFO("Socket %d hung up\n", i);
                    else
                        ROS_ERROR("recv() error.");
                    /* close it... */
                    close(i);

                    /* remove from master set */
                    FD_CLR(i, &_master_fds);
                    ROS_INFO("Close fd %d",i);
                }
                else
                {
                    /*Parse*/
                    ROS_INFO("%d bytes received over %d", nbytes,i);
                    //TODO: Process incoming data here
                }
                
            }
        }
    }

    /*Broadcast signal to all connected client*/
    //TODO:/*Update buffer here*/
    for(int32_t i = 0; i <= _fdmax; i++)
    {
        if(FD_ISSET(i,&_master_fds))
        {
            if(i != _listener)
            {
                //TODO:Send out data here
                //rcpm_send_data(i,&robot_status,sizeof(RobotStatus));
                //ROS_INFO("Heart Beat sent on FD %d",i);
                send(i,_send_buffer,_length,0);
            }
        }
    }
    return true;
}

void Monitor::tcp_close_all()
{
    for(int32_t i=0; i <= _fdmax; i++)
    {
        if(FD_ISSET(i,&_master_fds))
        {
            close(i);
        }
    }
    FD_ZERO(&_master_fds);
    FD_ZERO(&_temp_fds);
}

void Monitor::run()
{
    // if(_ENABLE_TCP)
    // {
    //     tcp_run();
    // }
    // if(_ENABLE_UART)
    // {
    //     serial_send();
    // }
    _ipc_run();
}

bool Monitor::_parseJsonStrToDataPackage(const std::string& str, InferFramePackage* pkg) {
  if (!pkg) return false;

  rapidjson::Document doc;
  if (doc.Parse<rapidjson::kParseCommentsFlag>(str.c_str()).HasParseError()) {
    std::cout << "SerializeFromString failed. Error code [" << std::to_string(doc.GetParseError()) << "]"
              << " Offset [" << std::to_string(doc.GetErrorOffset()) << "]. JSON:" << str << std::endl;
    return false;
  }

  // get members
  const auto end = doc.MemberEnd();

  // pkg_type
  if (end == doc.FindMember("pkg_type") || !doc["pkg_type"].IsInt()) {
    return false;
  } else {
    pkg->pkg_type = CNPkgType(doc["pkg_type"].GetInt());
  }

  if (CNPKG_RELEASE_MEM == pkg->pkg_type || CNPKG_DATA == pkg->pkg_type) {
    if (end == doc.FindMember("stream_id") || !doc["stream_id"].IsString()) {
      std::cout << "parse stream_id error.\n";
      return false;
    } else {
      pkg->stream_id = doc["stream_id"].GetString();
    }

    if (end == doc.FindMember("channel_idx") || !doc["channel_idx"].IsUint()) {
      std::cout << "parse channel_idx error.\n";
      return false;
    } else {
      pkg->channel_idx = doc["channel_idx"].GetUint();
    }

    if (end == doc.FindMember("frame_id") || !doc["frame_id"].IsInt64()) {
      std::cout << "parse frame_id error.\n";
      return false;
    } else {
      pkg->frame_id = doc["frame_id"].GetInt64();
    }
  }

  if (CNPKG_DATA == pkg->pkg_type) {
    if (end == doc.FindMember("flags") || !doc["flags"].IsUint()) {
      std::cout << "parse flags error.\n";
      return false;
    } else {
      pkg->flags = doc["flags"].GetUint();
    }

    if (end == doc.FindMember("timestamp") || !doc["timestamp"].IsInt64()) {
      std::cout << "parse timestamp error.\n";
      return false;
    } else {
      pkg->timestamp = doc["timestamp"].GetInt64();
    }

    if (end == doc.FindMember("data_fmt") || !doc["data_fmt"].IsInt()) {
      std::cout << "parse data fmt error.\n";
      return false;
    } else {
      pkg->fmt = DataFormat(doc["data_fmt"].GetInt());
    }

    if (end == doc.FindMember("width") || !doc["width"].IsInt()) {
      std::cout << "parse width error.\n";
      return false;
    } else {
      pkg->width = doc["width"].GetInt();
    }

    if (end == doc.FindMember("height") || !doc["height"].IsInt()) {
      std::cout << "parse height error.\n";
      return false;
    } else {
      pkg->height = doc["height"].GetInt();
    }

    if (end == doc.FindMember("strides") || !doc["strides"].IsArray()) {
      std::cout << "parse strides error.\n";
      return false;
    } else {
      auto values = doc["strides"].GetArray();
      int i = 0;
      for (auto iter = values.begin(); iter != values.end(); ++iter) {
        if (!iter->IsInt()) {
          std::cout << "parse strides type error.\n";
          return false;
        }
        pkg->stride[i] = iter->GetInt();
        i++;
      }
    }

    if (end != doc.FindMember("detect_objs") && doc["detect_objs"].IsArray()) {
      const rapidjson::Value& objs = doc["detect_objs"];
      for (size_t i = 0; i < objs.Size(); ++i) {
        const rapidjson::Value& obj = objs[i];
        InferObjInfo obj_info;
        obj_info.bbox.x = obj["x"].GetDouble();
        obj_info.bbox.y = obj["y"].GetDouble();
        obj_info.bbox.w = obj["w"].GetDouble();
        obj_info.bbox.h = obj["h"].GetDouble();
        obj_info.score = obj["score"].GetDouble();
        obj_info.label_id = obj["label_id"].GetString();
        pkg->detect_objs.push_back(obj_info);
      }
    }
  }
  return true;
}

bool Monitor::_prepareSendPackage(const CNPkgType& pkg_type, const int& channel_idx, const std::string& stream_id,
                        const uint64_t& frame_id, std::string* str) {
  if (!str) return false;
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  writer.StartObject();

  writer.Key("pkg_type");
  writer.Int(static_cast<int>(pkg_type));

  if (CNPKG_RELEASE_MEM == pkg_type) {
    writer.Key("channel_idx");
    writer.Uint(channel_idx);

    writer.Key("stream_id");
    writer.String(stream_id.c_str());

    writer.Key("frame_id");
    writer.Int64(frame_id);
  }

  writer.EndObject();
  *str = strBuf.GetString();
  return true;
}

void Monitor::_readImgFromSharedMem(const InferFramePackage& frame_pkg, char* img_buffer) {
  if (nullptr == img_buffer) {
    std::cout<<"buffer if nullptr"<<std::endl;
    return;
  }
  // open shared memoey
  size_t nbytes = frame_pkg.width * frame_pkg.height * 3 / 2;
  size_t boundary = 1 << 16;
  size_t map_mem_size = (nbytes + boundary - 1) & ~(boundary - 1);
  const std::string key = "stream_id_" + frame_pkg.stream_id + "_frame_id_" + std::to_string(frame_pkg.frame_id);
  int map_mem_fd = shm_open(key.c_str(), O_RDWR, S_IRUSR | S_IWUSR);
  if (map_mem_fd < 0) {
    std::cout << "Shered memory open failed, fd: " << map_mem_fd << ", error code: " << errno << std::endl;
    return;
  }

  void* map_mem_ptr = mmap(NULL, map_mem_size, PROT_READ | PROT_WRITE, MAP_SHARED, map_mem_fd, 0);
  if (map_mem_ptr == MAP_FAILED) {
    std::cout << "Mmap error" << std::endl;
    return;
  }

  if (ftruncate(map_mem_fd, map_mem_size) == -1) {
    std::cout << "truncate shared memory size failed" << std::endl;
    return;
  }

  // copy image  data out and convert to bgr
  char* src_frame_ptr = reinterpret_cast<char*>(map_mem_ptr);
  memcpy(img_buffer, src_frame_ptr, frame_pkg.height * frame_pkg.width);
  memcpy(img_buffer + frame_pkg.height * frame_pkg.width, src_frame_ptr + frame_pkg.height * frame_pkg.stride[0],
         (frame_pkg.height * frame_pkg.stride[1]) / 2);
  //cv::Mat bgr(frame_pkg.height, frame_pkg.stride[0], CV_8UC3);
  matTarget = cv::Mat(frame_pkg.height, frame_pkg.stride[0], CV_8UC3);
  cv::Mat src = cv::Mat(frame_pkg.height * 3 / 2, frame_pkg.stride[0], CV_8UC1, img_buffer);
  cv::cvtColor(src, matTarget, cv::COLOR_YUV2BGR_NV21);
  cv::imwrite("stream_" + std::to_string(frame_pkg.frame_id) + "_.jpg", matTarget);


  // cv::imshow("ddd", matTarget);
  // cv::waitKey(3);

}

bool Monitor::_ipc_run(void) {
  std::string socket_address = g_socket_address;
  int listen_fd_ = -1;
  int socket_fd_ = -1;
  char recv_buf[SOCK_BUFSIZE];
  char send_buf[SOCK_BUFSIZE];
  void* frame_buffer = nullptr;

  /*********  create socket  **********/
  unlink(socket_address.c_str());
  listen_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
  fcntl(listen_fd_, F_SETFL, O_NONBLOCK);
  if (-1 == listen_fd_) {
    std::cout << "create listen_fd for server failed, errno: " << errno << std::endl;
    return false;
  }

  sockaddr_un un;
  memset(&un, 0, sizeof(un));
  un.sun_family = AF_UNIX;
  memcpy(un.sun_path, socket_address.c_str(), socket_address.length());
  unsigned int length = strlen(un.sun_path) + sizeof(un.sun_family);

  /*********  bind socket  **********/
  if (bind(listen_fd_, reinterpret_cast<sockaddr*>(&un), length) < 0) {
    std::cout << "bind server listen_fd failed, errno: " << errno << std::endl;
    return false;
  }

  /*********  listen  **********/
  if (listen(listen_fd_, 1) < 0) {
    std::cout << "start server listen failed, errno: " << errno << std::endl;
    return false;
  } else {
    std::cout << "ipcserver--- server listening connection " << std::endl;
  }

  /*********  listen connection loop **********/
  while (1) {
    unsigned int length = 0;
    // accept connection
    socket_fd_ = accept(listen_fd_, reinterpret_cast<sockaddr*>(&un), &length);
    if (-1 == socket_fd_) {
      //std::cout << "server accept failed, errno: " << errno << std::endl;
      ;
      //return false;
      continue;
    }
    
    // recv data package loop
    while (recv(socket_fd_, recv_buf, sizeof(recv_buf), 0) && ros::ok()) {
      // parse data package
      InferFramePackage recv_pkg;
      std::string recv_str(recv_buf);
      _parseJsonStrToDataPackage(recv_str, &recv_pkg);

      std::cout << "testcns--- recv string: " << recv_str << std::endl;
      // get image data(nv21 format) from shared memory and dump
      if (nullptr == frame_buffer) {
        frame_buffer = malloc(recv_pkg.width * recv_pkg.height * 3 / 2);
      }

      if (!recv_pkg.flags) {  // normal data
        _readImgFromSharedMem(recv_pkg, reinterpret_cast<char*>(frame_buffer));
        std::cout<<"readTmg"<<std::endl;
      }

      // cv::Mat matOrignal(msg->height,msg->width,msg->encoding,const_cast<uchar*>(&msg->image[0]),msg->step);
      // cv::Mat matTarget;
      // cv::resize(matOrignal, matTarget, cv::Size(msg->width/2, msg->height/2));

      int birdNum = (int)recv_pkg.detect_objs.size();
      for (int i=0; i< birdNum; i++) {
        cv::rectangle(matTarget, cv::Rect(recv_pkg.detect_objs[i].bbox.x * recv_pkg.width,
          recv_pkg.detect_objs[i].bbox.y * recv_pkg.height,
          recv_pkg.detect_objs[i].bbox.w * recv_pkg.width,
          recv_pkg.detect_objs[i].bbox.h * recv_pkg.height
          ), cv::Scalar(255,0,0));
      }

      std::vector<uint8_t> buf;
      cv::imencode(".jpg",matTarget,buf);
      auto *enc_msg = reinterpret_cast<unsigned char*>(buf.data());

      birdy_config::toweb m;
      m.warningTime = boost::posix_time::to_iso_extended_string(ros::Time::now().toBoost());
      m.account = _usr;
      m.pwd = _pwd;  
      m.birdNum = birdNum;
      m.birdType = "1";
      m.deviceUuid = "10.8.0.88";
      m.degree = 2;
      m.birdImageBase64 = base64_encode(enc_msg,buf.size());
      m.remark = "";
      m.heightRange = "";
      _pub.publish(m);

      // prepare and send release_mem package to cnstream client
      std::string send_str;
      if (!recv_pkg.flags) {  // normal data
        _prepareSendPackage(CNPKG_RELEASE_MEM, recv_pkg.channel_idx, recv_pkg.stream_id, recv_pkg.frame_id, &send_str);
      } else {  // eos
        _prepareSendPackage(CNPKG_EXIT, recv_pkg.channel_idx, "0", 0, &send_str);
      }

      std::cout << "ipcserver--- send string: " << send_str << std::endl;
      memset(send_buf, 0, sizeof(send_buf));
      memcpy(send_buf, send_str.c_str(), send_str.length());
      if (sizeof(send_buf) != send(socket_fd_, send_buf, sizeof(send_buf), 0)) {
        std::cout << "send data to client failed.\n";
        break;
      }
    }

    break;
  }

  std::cout << "ipcserver end------------\n";
  close(socket_fd_);
  socket_fd_ = -1;
  close(listen_fd_);
  listen_fd_ = -1;
  if (frame_buffer) free(frame_buffer);

  return 1;
}
