#ifndef __TCP_H_
#define __TCP_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <signal.h>
#include <sys/prctl.h>
#include <pthread.h>


/**
 * 地址类
*/
class Addr
{
public:
    Addr(char *ip, int port);
    const char *getIP();
    int getPort();
    ~Addr();
private:
    int port;
    const char *ip;
};


/**
 * tcp的方法
*/
class yzt_tcp
{
public:
    static int getBindSocket(Addr &addr);                       //获得与本地地址绑定的套接字
    static void connectToServer(Addr &addr, int socktNUM);      //与服务器连接
private:

};


/**
 * 掌管某一个套接字的发送与接收消息
*/
class TCP_Manager
{
public:
    TCP_Manager(int socketNum);
    int getSocket(void);
protected:
    int socketNum;
};


/**
 * 用于通信的TCP管理者
*/
class TCP_CommunicationManager : public TCP_Manager
{
public:
    TCP_CommunicationManager(int socketNum) : TCP_Manager(socketNum){}
    void Send(void *data, size_t size);     //发送消息
    void Recv(void *data, size_t size);     //接收消息
private:
};


/**
 * 服务器用于监听的TCP管理者
*/
class TCP_ListenManager : public TCP_Manager
{
public:
    TCP_ListenManager(int socketNum) : TCP_Manager(socketNum){}
    void Listen(void);
    int Accept(void);
    int GetPort(void);
private:
    Addr *targetAddr;
};

#endif