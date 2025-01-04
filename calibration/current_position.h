#ifndef H_6170956E_8FA3_4F88_A04F_6964044E924D
#define H_6170956E_8FA3_4F88_A04F_6964044E924D

#include "globals.h"
#include "geometry.h"

#include <memory>
#include <string>
#include <vector>
#include <optional>

class Connections;

class CurrentPosition {
  public:
    struct TargetPosition {
      int x, y;
    };

    struct StepCounts {
      int counts[mainAxisCount];
    };

  private:
    Connections *connections;
    std::optional<Position> printer;
    std::optional<StepCounts> steps;
    std::optional<TargetPosition> target;
    
  public:
    static std::unique_ptr<CurrentPosition> open(Connections *);

    void parsePrinterCommand(const std::vector<std::string> &args);
    void parsePrinterReply(const std::vector<std::string> &args);
    void parseTabletCommand(const std::vector<std::string> &args);

    auto readPrinter() const -> auto { return printer; }
    auto readSteps() const -> auto { return steps; }
    auto readTarget() const -> auto { return target; }
};

#endif
