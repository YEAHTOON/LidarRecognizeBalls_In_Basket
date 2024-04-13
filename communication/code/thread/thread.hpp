#ifndef __WHU_ROBOCON_THREAD_
#define __WHU_ROBOCON_THREAD_

#include "uart.hpp"
#include "tcp.hpp"

/**
 * 各种传入的参数类型
 * 
 * 
*/
typedef struct
{
    uart *u;
    TCP_CommunicationManager *tcpm;
    Decoding_ReceivedUartPack *decoder;
} RecvInfoFromMaster_Args;

typedef struct
{
    uart *u;
    TCP_CommunicationManager *tcpm;
} RecvInfo_From_Modules_Args;


//有关串口的线程
class UartThread
{
private:
public:
    //从主控处接收消息的线程
    static void *RecvInfo_From_Master(void *args_void)
    {
        uint8_t received_command[128];      //接收指令的缓冲区
        RecvInfoFromMaster_Args *args = (RecvInfoFromMaster_Args *)args_void;    

        while(1)
        {
            //读取完毕后进入判断
            if(args->decoder->input(args->u->uartRead()))
            {
                args->decoder->GetData(received_command);                       //获得数据
                args->tcpm->Send(received_command, args->decoder->getSize());   //发送数据，待改
                args->decoder->clean_data();                                    //清除
            }
        }
    }
};

#endif
