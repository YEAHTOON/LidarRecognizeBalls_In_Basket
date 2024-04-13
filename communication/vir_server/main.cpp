#include <iostream>
#include <vector>
#include "tcp.hpp"

//优雅地关闭程序
std::vector<int> socket_list;
void close_gently(int signum)
{
    //迭代关闭
    for(auto it = socket_list.begin(); it != socket_list.end(); it++)
    {
        close(*it);
    }

    exit(1);
}

int main()
{
    signal(SIGTERM, close_gently);
    signal(SIGINT, close_gently);

    //监听管理者
    Addr la((char*)"127.0.0.1", 30000);
    int ls = yzt_tcp::getBindSocket(la);    socket_list.push_back(ls);      //套接字创建
    TCP_ListenManager lm(ls);

    lm.Listen();

    //获得交流套接字
    uint8_t buff[128] = {0};
    int cs = lm.Accept();
    TCP_CommunicationManager cm(cs);

    while(1)
    {
        cm.Recv(buff, 128);
        std::cout << buff << std::endl;
    }

    return 0;
}

