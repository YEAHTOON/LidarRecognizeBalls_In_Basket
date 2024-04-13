#ifndef __UART_HPP
#define __UART_HPP

// #include "../common/common.hpp"
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <iostream>
#include <signal.h>


//小摄像头需要的数据
typedef struct smallCamera_data
{
    float distance;
    float angle;
} smallCamera_data;


//把数据转换成一个uint8类型的数组
template<class T>
class data_inUint8
{
public:
    data_inUint8(T input)
    {
        uint8_t *temp = (uint8_t *)malloc(sizeof(input));
        memcpy(temp, &input, sizeof(input));
        this->addr = temp;
        this->size = sizeof(input);
    }

    ~data_inUint8(void)
    {
        if(this->addr != nullptr)
        {
            free(this->addr);
            this->addr = nullptr;
        }
    }

    //获得地址
    uint8_t *getAddr(void)
    {
        return this->addr;
    }

    //获得大小
    size_t getSize(void)
    {
        return this->size;
    }

private:
    uint8_t *addr;
    size_t size;
};




//每个包的包头
class uartPack_head
{
public:
    uartPack_head(uint8_t head_1, uint8_t head_2);
    uint8_t getData(int index);
private:
    uint8_t data[2];
};



//每个包的id
class uartPack_ID
{
public:
    uartPack_ID(uint8_t data);
    uint8_t getData(void);
private:
    uint8_t data;
};



//每个包的校验位
class uartPack_check
{
public:
    uartPack_check(uint8_t data);
    uint8_t getData(void);
private:
    uint8_t data;
};



//每个包的包尾
class uartPack_tail
{
public:
    uartPack_tail(uint8_t tail_1, uint8_t tail_2);
    uint8_t getData(int index);
private:
    uint8_t data[2];
};



/**
 * 串口通信的基本单位
 * 需要释放空间，保护内存
*/
template <class T>
class uartPack
{
public:

    uartPack(uartPack_head head, uartPack_ID id, uartPack_tail tail, uartPack_check check, data_inUint8<T> data_toBeSent)
    {
        //串口包存放的地址
        uint8_t *temp = (uint8_t *)malloc(data_toBeSent.getSize() + 6);

        temp[0] = head.getData(0);
        temp[1] = head.getData(1);
        temp[2] = id.getData();
        memcpy(&temp[3], data_toBeSent.getAddr(), data_toBeSent.getSize());
        temp[data_toBeSent.getSize() + 3] = check.getData();
        temp[data_toBeSent.getSize() + 4] = tail.getData(0);
        temp[data_toBeSent.getSize() + 5] = tail.getData(1);

        this->addr = temp;
        this->size = data_toBeSent.getSize() + 6;
    }

    ~uartPack(void)
    {
        if(this->addr != nullptr)
        {
            free(this->addr);
            this->addr = nullptr;
        }
    }

    //包的地址
    uint8_t *getAddr(void)
    {
        return this->addr;
    }

    //获得大小
    size_t getSize(void)
    {
        return this->size;
    }

private:
    uint8_t *addr;
    size_t size;
};


//指向块数据集的指针
typedef struct
{
    int row;
    int col;
} uart_ReceiveLines_pointer;


//串口接受的块数据集，没用
class uart_ReceiveLines
{
private:
    uint8_t data[10][128];
    int new_row;
    int old_row;
    uart_ReceiveLines_pointer p;

public:
    uart_ReceiveLines(void);
    int calculateRow(int now_row_num, int add_row);
    int GetRowsCountBetween(int UpRow, int DownRow);
    bool IfRowValid(int row);
    bool ChangePointerNextLine(void);
    bool OldRowQuit(void);
    void CreateNewRow(void);
    void recv(int UartFileNum);
    uint8_t ReadByOne(void);
};

//解码器的状态
typedef enum
{
    DECODING_HEAD = 0X00,
    DECODING_TAIL = 0X01,
} decoding_attribute;

//解码包
class Decoding_ReceivedUartPack
{
private:
    uartPack_head head;
    uartPack_tail tail;
    uint8_t attribute;
    uint8_t step;
    uint8_t buff[128];
    size_t size;
public:
    Decoding_ReceivedUartPack(uint8_t head1, uint8_t head2, uint8_t tail1, uint8_t tail2);
    Decoding_ReceivedUartPack(uartPack_head head, uartPack_tail tail);
    void CheckHead(uint8_t input_data);
    bool CheckTail(uint8_t input_data);
    void store_data(uint8_t newU8_data);
    void clean_data(void);
    bool input(uint8_t newU8_data);
    void GetData(uint8_t *target);
    uint8_t getAttribute(void);
    uint8_t getStep(void);
    size_t getSize(void);
};

/**
 * 串口通信
*/
class uart
{
public:
    uart(const char *filename, int baudrate);
    virtual void setparam(void);
    int send(uint8_t *addr, size_t size);
    uint8_t uartRead(void);
    void closeCommu(void);
private:
    int fileDes;            //文件描述符
    int baudrate;           //波特率
};


//tty设备文件
class ttyFile
{
private:
    int numDes;
public:
    ttyFile(const char *filename);
    ~ttyFile();
    int getFileNUm(void);
};


//获得浮点数对应十六位数的后四位
uint8_t getFloat_lastEightBits(float theF);
float FromU8_To_Float(uint8_t *u8_data);

#endif
