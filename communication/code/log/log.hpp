#ifndef __YZT_LOG_H_
#define __YZT_LOG_H_

#include <fstream>
#include <time.h>
#include <string>
#include <string.h>
#include <pthread.h>

class yzt_log
{
public:
    yzt_log(const char *name, const char *head, pthread_mutex_t *mutex) : valid(0), log_mutex(mutex)
    {
        fstream.open(name, std::ios::app | std::ios::out);      //打开文件
        if(fstream.is_open()) valid = 1;            //打开文件有效
        memset(this->head, 0, 64);
        memcpy(this->head, head, strlen(head));     //获得固定log消息头
    }

    ~yzt_log()
    {
        fstream.close();
    }

    int if_valid(void)
    {
        return valid;
    }

    void operator<<(const char *log_info)
    {
        //获得时间
        char timebuff[64] = {0};
        time_t source_time;
        time(&source_time);
        struct tm *valid_time;
        valid_time = localtime(&source_time);
        sprintf(timebuff, " [ %d分 %d秒 ] ", valid_time->tm_min, valid_time->tm_sec);

        //输出
        pthread_mutex_lock(log_mutex);
        fstream << head  << " : " << log_info << timebuff << std::endl;
        pthread_mutex_unlock(log_mutex);
    }

private:
    std::fstream fstream;
    int valid;
    char head[64];
    pthread_mutex_t *log_mutex;
};


#endif
