#include "socket_link.h"
#include "ros/ros.h"

using namespace std;

int32_t Link::tcp_server_init(int32_t port, int64_t sec, int64_t micro_sec){
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
        return -1;
    }
    ROS_INFO("Server-socket() is OK");

    /*Check if already in use*/
    int32_t yes = 1;
    if((setsockopt(_listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int32_t)) == -1))
    {   
        ROS_ERROR("Server-setsockopt() error.");
        return -1;
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
        return -1;
    }
    ROS_INFO("Server_bind() is OK");

    /*Listen*/
    if(listen(_listener,10) < 0){//开始监听
        ROS_ERROR("Server-listen() error.");
        return -1;
    }
    ROS_INFO("Sever_listen() is OK");

    FD_SET(_listener, &_master_fds);
    _fdmax = _listener;
    return 1;
}

//TCP main loop
int32_t Link::tcp_run(void)
{
    int32_t result = 0;
    _temp_fds = _master_fds;
    if((select(_fdmax+1, &_temp_fds,NULL,NULL,&tv)) == -1)
    {
        ROS_ERROR("Server-select() error.");
        ROS_INFO("return on select()");
        return -1;
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
                if ((nbytes = recv(i, read_buffer, sizeof(read_buffer), 0)) <= 0)
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
            }
        }
    }
    return result;
}

//Close all existing socket FDs
void Link::tcp_close_all()
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
