#ifndef COMMON__ITC_H__
#define COMMON__ITC_H__

#include <stdint.h>
#include <string.h>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <functional>



#define TRWRINGQUE_READER_MAX       4
#define TRWRINGQUE_RPOS_EMPTY       (-1)
#define TRWRQ_ERROR_READ_TIMEOUT    (-100)


struct TRWRingQueStat {
    uint32_t    total;
    uint32_t    discard;
    uint32_t    discardQueEmpty;
    uint32_t    discardQueCloted;
    uint32_t    discardQueNonupdated;
};


template<typename T>

class TRWRingQueue { // superior uni-writer inferior multi-readers
  public:
    TRWRingQueue(uint count = 0) : count_{count} { memset(rpos_, TRWRINGQUE_RPOS_EMPTY, sizeof(int)*TRWRINGQUE_READER_MAX); }
    ~TRWRingQueue() = default;

    int Init(uint count); // for convenience

    int Read(uint channel, int timeout, const std::function<int(T&)> &UtilizeResource);
    int Write(T &update, const std::function<int(T&)> &ReleaseResource);
    int Clear(const std::function<int(T&)> &ReleaseResource);

  private:
    bool RdFull(uint chn) { return rpos_[chn] >= (int)(count_ - 1); }
    bool RdEmpty(uint chn) { return rpos_[chn] <= TRWRINGQUE_RPOS_EMPTY; }

  private:
    std::deque<T> queue_;
    //std::mutex buck_;
    std::mutex lock_;
    std::condition_variable cond_;
    uint count_; // ring-queue max
    int rpos_[TRWRINGQUE_READER_MAX];
};


template<typename T>
int TRWRingQueue<T>::Init(uint count) 
{ 
    count_ = count; 
    memset(rpos_, TRWRINGQUE_RPOS_EMPTY, sizeof(int)*TRWRINGQUE_READER_MAX); 
    return 0; 
}


template<typename T>
int TRWRingQueue<T>::Read(uint channel, int timeout, const std::function<int(T&)> &UtilizeResource)
{
    uint chn = channel % TRWRINGQUE_READER_MAX;

    // compete between readers, not implemented yet
    // std::lock_guard<std::mutex> dealer(buck_);
    // should use fine-grained schedule, now just for inter-thread validation
    // so only one reader can acess the ring-queue, others get hunger
    std::unique_lock<std::mutex> guard(lock_);

    if (timeout < 0) {
        while (RdEmpty(chn)) {
            cond_.wait(guard); 
        }
    } else {
        std::chrono::milliseconds waitime(timeout);
        while (RdEmpty(chn)) {
            std::cv_status sta = cond_.wait_for(guard, waitime);
            if (sta == std::cv_status::timeout) { return TRWRQ_ERROR_READ_TIMEOUT; }
        }
    }

    T &current = queue_.at(rpos_[chn]);
    --rpos_[chn]; // forward reader position

    return UtilizeResource(current);
}


// bug: if writing is stopped first, then readings may be blocked, maybe use locking constructor
template<typename T>
int TRWRingQueue<T>::Write(T &update, const std::function<int(T&)> &ReleaseResource)
{
    int ret = 0;
    //std::lock_guard<std::mutex> dealer(buck_);
    std::unique_lock<std::mutex> guard(lock_);

    if (queue_.size() == count_) { // full
        T &outdate = queue_.at(count_ - 1);
        ret = ReleaseResource(outdate);        
        queue_.pop_back();
        for (int i = 0; i < TRWRINGQUE_READER_MAX; ++i) {
            if (RdFull(i)) { --rpos_[i]; } // forward (last) popped readers' position
        }
    }

    queue_.emplace_front(update);
    for (int i = 0; i < TRWRINGQUE_READER_MAX; ++i) {
        ++rpos_[i]; // update (all) pushed readers' position;
    }

    cond_.notify_all();
    return ret;
}


template<typename T>
int TRWRingQueue<T>::Clear(const std::function<int(T&)> &ReleaseResource)
{
    int ret = 0;
    std::unique_lock<std::mutex> guard(lock_);

    int index = queue_.size() - 1;
    while (!queue_.empty()) {
        T &current = queue_.at(index);
        queue_.pop_back();
        --index;
        if (ReleaseResource(current) < 0) { ret -= 1; }
    }

    memset(rpos_, TRWRINGQUE_RPOS_EMPTY, sizeof(int)*TRWRINGQUE_READER_MAX); 
    return ret;
}


#endif // COMMON__ITC_H__
