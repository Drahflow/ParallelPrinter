#ifndef H_CEA6DE38_E23E_436E_AFA6_5105001FAE5A
#define H_CEA6DE38_E23E_436E_AFA6_5105001FAE5A

#include <vector>

class Tickable {
  protected:
    ~Tickable() { }

  public:
    virtual void tick() = 0;

    void startTicked(std::vector<Tickable *> &);
    void stopTicked(std::vector<Tickable *> &);
};

#endif
