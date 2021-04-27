#include <iostream>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_controllers/trajectory_generator.h>
#include <yaml-cpp/yaml.h>

typedef struct {
  std::string item_name;
  int32_t value;
} ItemValue;

class DynamixelHand {
private:
  DynamixelWorkbench *dxl_wb_;

  std::map<std::string, uint32_t> dynamixel_;
  std::map<std::string, const ControlItem*> control_items_;
  std::vector< std::pair<std::string, ItemValue> > dynamixel_info_;
  std::vector<WayPoint> pre_goal_;

  double read_period_;
  double write_period_;

public:
  DynamixelHand();
  ~DynamixelHand();

  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool getDynamixelsInfo(const std::string yaml_file);
  bool loadDynamixels(void);
  bool initDynamixels(void);
  bool initControlItems(void);
  bool initSDKHandlers(void);
  bool getPresentPositions(std::vector<std::string> dxl_name);

  double getReadPeriod() { return read_period; }
  double getWritePeriod() { return write_period_; }
  
};
