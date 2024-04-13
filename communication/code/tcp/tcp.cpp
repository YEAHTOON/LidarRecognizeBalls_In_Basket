#include "tcp.hpp"

Addr::Addr(char *ip, int port)
{
    this->port = port;  //设置端口号

    //设置ip地址
    size_t size = strlen(ip) + (size_t)1;
    this->ip = (const char *)malloc(size);
    memcpy((void *)this->ip, ip, size);
}

const char *Addr::getIP()
{
    return this->ip;
}

int Addr::getPort()
{
    return this->port;
}

Addr::~Addr()
{
    free((void *)this->ip);
}

int yzt_tcp::getBindSocket(Addr &addr)
{
    struct sockaddr_in sockaddr;
    sockaddr.sin_addr.s_addr = inet_addr(addr.getIP());
    sockaddr.sin_port = htons(addr.getPort());
    sockaddr.sin_family = AF_INET;

    //获得套接字并绑定
    int socketNUM = socket(AF_INET, SOCK_STREAM, 0);
    bind(socketNUM, (struct sockaddr*)&sockaddr, sizeof(sockaddr));

    //可复用
    int on = 1;
    setsockopt(socketNUM, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    return socketNUM;
}

void yzt_tcp::connectToServer(Addr &addr, int socktNUM)
{
    struct sockaddr_in sockaddr;
    sockaddr.sin_addr.s_addr = inet_addr(addr.getIP());
    sockaddr.sin_port = htons(addr.getPort());
    sockaddr.sin_family = AF_INET;

    int ret = connect(socktNUM, (struct sockaddr*)&sockaddr, sizeof(sockaddr));
}

TCP_Manager::TCP_Manager(int socketNum)
{
    this->socketNum = socketNum;
}

int TCP_Manager::getSocket(void)
{
    return this->socketNum;
}

void TCP_CommunicationManager::Send(void *data, size_t size)
{
    send(this->socketNum, data, size, 0);
}

void TCP_CommunicationManager::Recv(void *data, size_t size)
{
    //重置缓冲区
    recv(this->socketNum, data, size, MSG_WAITFORONE);
}

void TCP_ListenManager::Listen(void)
{
    listen(this->socketNum, 10);
}

int TCP_ListenManager::Accept(void)
{
    struct sockaddr_in temp_addr;
    socklen_t temp_len;
    int socketNum = accept(this->socketNum, (struct sockaddr*)&temp_addr, &temp_len);

    char ip_string[32];
    inet_ntop(AF_INET, &temp_addr, ip_string, strlen(ip_string));
    uint16_t port = ntohs(temp_addr.sin_port);

    targetAddr = new Addr(ip_string, temp_addr.sin_port);
    return socketNum;
}

int TCP_ListenManager::GetPort(void)
{
    return targetAddr->getPort();
}
