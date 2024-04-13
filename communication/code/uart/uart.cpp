#include "uart.hpp"

//获取float类型数据的后八位
uint8_t getFloat_lastEightBits(float theF)
{
    uint8_t temp[4];
    uint8_t result;

    memcpy(temp, &theF, sizeof(theF));
    memcpy(&result, temp+3, sizeof(result));

    return result;
}


uartPack_head::uartPack_head(uint8_t head_1, uint8_t head_2)
{
    this->data[0] = head_1;
    this->data[1] = head_2;
}

uint8_t uartPack_head::getData(int index)
{
    return this->data[index];
}

uartPack_ID::uartPack_ID(uint8_t data)
{
    this->data = data;
}

uint8_t uartPack_ID::getData(void)
{
    return this->data;
}

uartPack_check::uartPack_check(uint8_t data)
{
    this->data = data;
}

uint8_t uartPack_check::getData(void)
{
    return this->data;
}

uartPack_tail::uartPack_tail(uint8_t tail_1, uint8_t tail_2)
{
    this->data[0] = tail_1;
    this->data[1] = tail_2;
}

uint8_t uartPack_tail::getData(int index)
{
    return this->data[index];
}

uart::uart(const char *filename, int baudrate)
{
    if(filename != nullptr)
    {
        this->fileDes = open(filename, O_RDWR | O_NOCTTY);
        if(this->fileDes < 0) 
        {
            //退出进程
            pid_t pid;
            kill(pid, SIGTERM);
            // std::cout << "(WHU_ROBOCON_SENSOR) 错误 ： 打开串口失败，检查串口名、串口权限" << std::endl;
        }
        this->baudrate = baudrate;

        // fcntl(fileDes, F_SETFL, 0);
        int fileflags = fcntl(fileDes, F_GETFL, 0);
        fileflags &= ~O_NONBLOCK;
        fcntl(fileDes, F_SETFL, fileflags);
    }
}

void uart::setparam(void)
{
    //串口参数
    struct termios options;
    memset(&options, 0, sizeof(options));

    //设置串口参数后应用
    tcgetattr(this->fileDes, &options);
    options.c_cflag |= (CLOCAL|CREAD);      //CREAD 开启串行数据接收，CLOCAL并打开本地连接模式
    options.c_cflag &= ~CSIZE;              //先使用CSIZE做位屏蔽
    options.c_cflag |= CS8;                 //设置8位数据位
    options.c_cflag &= ~PARENB;             //无校验位
    options.c_cflag &= ~CSTOPB;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN]  = 1;
    cfsetispeed(&options, this->baudrate);
    cfsetospeed(&options, this->baudrate);
    tcsetattr(this->fileDes, TCSANOW, &options);

    //清空缓冲区数据与请求
    tcflush(this->fileDes ,TCIFLUSH);
}

int uart::send(uint8_t *addr, size_t size)
{   
    if(addr != nullptr)
    {
        //传出
        return write(fileDes, addr, size);
    }
    return 0;
}

void uart::closeCommu(void)
{
    close(fileDes);
}

ttyFile::ttyFile(const char *filename)
{
    this->numDes = open(filename, O_RDWR|O_NOCTTY|O_NDELAY);
    fcntl(this->numDes, F_SETFL, 0);
}

ttyFile::~ttyFile()
{
    close(this->numDes);
}

int ttyFile::getFileNUm(void)
{
    return this->numDes;
}

uint8_t uart::uartRead(void)
{
    uint8_t result;
    int ret = read(fileDes, &result, 1);
    // printf("%x : %ld\n", result, sizeof(result));
    return result;
}

uint8_t uart_ReceiveLines::ReadByOne(void)
{
    uint8_t result;

    if((data[p.row][p.col] == '\0')||(p.col == 127))
    {
        if(!ChangePointerNextLine()) return data[p.row][p.col++];
        else OldRowQuit();
    }
    else
    {
        return data[p.row][p.col++];
    }

    return 0;
}

uart_ReceiveLines::uart_ReceiveLines(void)
{
    this->new_row = 0;
    this->old_row = 0;

    this->p.row = 0;
    this->p.col = 0;
}

int uart_ReceiveLines::calculateRow(int now_row_num, int add_row)
{
    int result = (now_row_num + add_row) % 10;
    return result;
}

int uart_ReceiveLines::GetRowsCountBetween(int UpRow, int DownRow)
{
    if(UpRow < DownRow)
    {
        UpRow += 10;
    }

    int result = UpRow - DownRow;
    return result;
}

bool uart_ReceiveLines::IfRowValid(int row)
{
    if(row < new_row) row = row + 10;
    if(old_row < new_row) old_row = old_row + 10;
    if((row >= new_row) && (row <= old_row)) return true;
    else return false;
}

bool uart_ReceiveLines::ChangePointerNextLine(void)
{
    if(IfRowValid(calculateRow(p.row, 1)))
    {
        p.row = calculateRow(p.row, 1);
        p.col = 0;
        return true;
    }
    
    return false;
}

bool uart_ReceiveLines::OldRowQuit(void)
{
    if(GetRowsCountBetween(new_row, old_row) == 0) return false;
    else
    {
        old_row = calculateRow(old_row, 1);                 //旧行被删除
        if(!IfRowValid(p.row)) ChangePointerNextLine();     //指针已经无效
        return true;
    }
}

void uart_ReceiveLines::CreateNewRow(void)
{
    if(GetRowsCountBetween(new_row, old_row) == 9) OldRowQuit();
    new_row = calculateRow(new_row, 1);
}

void uart_ReceiveLines::recv(int UartFileNum)
{          
    CreateNewRow();              
    read(UartFileNum, this->data[new_row], 128);                //读取至缓冲区lines
}

float FromU8_To_Float(uint8_t *u8_data)
{
    float result;
    memcpy(&result, u8_data, 4);
    return result;
}

Decoding_ReceivedUartPack::Decoding_ReceivedUartPack(uint8_t head1, uint8_t head2, uint8_t tail1, uint8_t tail2)
 : head(head1, head2), tail(tail1, tail2), attribute(0x00), step(0x00)
{
    memset(buff, 0, 128);
    size = 0;
}

Decoding_ReceivedUartPack::Decoding_ReceivedUartPack(uartPack_head head, uartPack_tail tail):
head(head), tail(tail)
{
    memset(buff, 0, 128);
    size = 0;
}

//检测包头
void Decoding_ReceivedUartPack::CheckHead(uint8_t input_data)
{
    if(step == 0x00)
    {
        if(input_data == head.getData(0))
        {
            step++;
            return;
        }
    }
    else if(step == 0x01)
    {
        if(input_data == head.getData(1))
        {
            step = 0;
            attribute = DECODING_TAIL;
            return;
        }
    }

    step = 0x00;
    return;
}

bool Decoding_ReceivedUartPack::CheckTail(uint8_t input_data)
{
    if(step == 0x00)
    {
        if(input_data == tail.getData(0))
        {
            step++;
            return false;
        }
    }
    else if(step == 0x01)
    {
        if(input_data == tail.getData(1))
        {
            attribute = DECODING_HEAD;
            step = 0x00;
            return true;
        }
    }

    if(size > 63)   //说明有问题
    {
        attribute = DECODING_HEAD;
        step = 0x00;
        clean_data();
    }

    return false;
}

void Decoding_ReceivedUartPack::store_data(uint8_t newU8_data)
{
    buff[size] = newU8_data;
    size++;
}

void Decoding_ReceivedUartPack::clean_data(void)
{
    memset(buff, 0, 128);
    size = 0;
}

//输入一个新的u8
bool Decoding_ReceivedUartPack::input(uint8_t newU8_data)
{
    bool result = false;

    switch (attribute)
    {
    case DECODING_HEAD:
        CheckHead(newU8_data);
        break;

    case DECODING_TAIL:
        store_data(newU8_data);
        result = CheckTail(newU8_data);
        break;
    }

    return result;
}

//获得数据
void Decoding_ReceivedUartPack::GetData(uint8_t *target)
{
    memcpy(target, buff, (size-2));
    buff[size - 2] = '\0';
    // clean_data();
}

uint8_t Decoding_ReceivedUartPack::getAttribute(void)
{
    return attribute;
}

uint8_t Decoding_ReceivedUartPack::getStep(void)
{
    return step;
}

size_t Decoding_ReceivedUartPack::getSize(void)
{
    return this->size;
}
