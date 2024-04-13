#include <iostream>
#include <tcp.hpp>

//获得消息
void *GetData(void *args)
{
    TCP_CommunicationManager *tm = (TCP_CommunicationManager *)args;        //tcp通讯管理者

    //循环接收
    uint8_t rd[128] = {0};
    while(1){
        tm->Recv(rd, 128);          //接收消息
        std::cout << (char *)rd << std::endl;
        usleep(1000);
    }
}

//优雅地关闭程序
int CloseSocketNum;
void close_gently(int signum)
{
    close(CloseSocketNum);
    exit(1);
}


/**
 * main函数
*/
int main(int argc, char **argv)
{
    signal(SIGTERM, close_gently);
    signal(SIGINT, close_gently);

    //创建与对方连接的tcp
    int s = socket(AF_INET, SOCK_STREAM, 0);    //得到套接字
    Addr sa((char *)"127.0.0.1", 30000);                //服务器地址
    yzt_tcp::connectToServer(sa, s);            //连接
    TCP_CommunicationManager tm(s);             //管理套接字
    tm.Send((void *)"lidar", strlen("lidar"));  //发送确认消息
    CloseSocketNum = s;

    //创建接收线程
    pthread_t gd_pid;
    pthread_create(&gd_pid, NULL, GetData, &tm);

    //发送的消息
    uint8_t sd[10] = {9, 0x5a, 0xee, 1, 2, 3, 4, 5, 0xff, 0xfe}; 

    //循环发送消息
    while(1){
        tm.Send(sd, 10);
        usleep(300000);     //延迟0.3秒
    }



    return 0;
}
