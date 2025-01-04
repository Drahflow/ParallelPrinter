#ifndef H_CBE6A7C9_5A56_4BD3_AB08_7A34E8C510FE
#define H_CBE6A7C9_5A56_4BD3_AB08_7A34E8C510FE

class Epollable {
  protected:
    ~Epollable() { }

  public:
    virtual int getEpollFd() = 0;
    virtual void available() = 0;

    bool addToEpoll(int epollFd);
    void removeFromEpoll(int epollFd);
};

#endif
