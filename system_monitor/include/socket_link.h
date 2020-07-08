#ifndef SOCKET_LINK_H
#define SOCKET_LINK_H

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>

class Link
{
private:
    /*Socket related var*/
    
    fd_set _master_fds;  //Master file descriptor for server mode
    fd_set _temp_fds;    //Temporary fd for select() to show which FD has new arrival
    int32_t _fdmax;          //Index keep the number of total socket FD for server mode
    int32_t _listener;   //Listener socket FD
    struct timeval tv;   //Timeout for select()  
    int32_t _port;
public:
    #define read_bufSize 64
    #define send_bufSize 400000
    u_int8_t read_buffer[read_bufSize];
    u_int8_t send_buffer[send_bufSize];

    Link(){}
    Link(int32_t port){_port = port;}
    ~Link(){tcp_close_all();}
    int32_t tcp_server_init(int32_t port, int64_t sec, int64_t micro_sec);
    int32_t tcp_run(void);
    void tcp_close_all(void);
};

#endif