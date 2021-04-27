#include "dynamixel_hand_controller.hpp"

DynamixelHand::DynamixelHand()
{
}

bool DynamixelHand::initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
  bool result = false;
  const char* log;
  result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
  if (result == false) {
    std::cerr << log << std::endl;
  }
  return result;
}

bool DynamixelHand::getDynamixelsInfo(const std::string yaml_file)
{
  YAML::Node dynamixel;
  dynamixel = YAML::LoadFile(yaml_file.c_str());

  if (dynamixel == NULL) return false;

  for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++) {
    std::string name = it_file->first.as<std::string>();
    if (name.size() == 0) continue;

    YAML::Node item = dynamixel[name];
    for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++) {
      std::string item_name = it_item->first.as<std::string>();
      int32_t value = it_item->second.as<int32_t>();

      if (item_name == "ID") dynamixel_[name] = value;
      ItemValue item_value = {item_name, value};
      std::pair<std::string, ItemValue> info(name, item_value);

      dynamixel_info_.push_back(info);
    }
  }
  return true;
}

bool DynamixelHand::loadDynamixels(void)
{
  bool result = false;
  const char* log;

  for (auto const& dxl:dynamixel_) {
    uint16_t model_number = 0;
    result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
    if (result == false) {
      std::cerr << log << std::endl;
      std::cerr << "Can't find Dynamixel ID: " << dxl.second << std::endl;
      return result;
    } else {
      std::cerr << "Name: " << dxl.first.c_str() << ", ID: " << dxl.second << ", Model Number: " << model_number << std::endl;
    }
  }
  return result;
}

bool DynamixelHand::initDynamixels(void)
{
  const char* log;
  for (auto const& dxl:dynamixel_) {
    dxl_wb_->torqueOff((uint8_t)dxl.second);
    for (auto const& info:dynamixel_info) {
      if (dxl.first == info.first) {
        if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate") {
          bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
          if (result == false) {
            std::cerr << log << std::endl;
            std::cerr << "Failed to write value[" << info.second.value << "] on items[" << info.second.item_name.c_str() << "] to Dynamixel[Name: " << dxl.first.c_str() << ", ID: " << dxl.second << "]" << std::endl;
            return false;
          }
        }
      }
    }
    dxl_wb_->torqueOn((uint8_t)dxl.second);
  }
  return true;
}

bool DynamixelHand::initControllerItems(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  const ControlItem *goal_position = dxl_wb_->getItemInfo(it->second, "Goal_Position");
  if (goal_position == NULL) return false;

  const ControlItem *goal_velocity = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
  if (goal_velocity == NULL) goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
  if (goal_velocity == NULL) return false;

  const ControlItem *present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
  if (present_position == NULL) return false;

  const ControlItem *present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
  if (present_velocity == NULL) present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
  if (present_velocity == NULL) return false;

  const ControlItem *present_current = dxl_wb_->getItemInfo(it->second, "Present_Current");
  if (present_current == NULL) present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
  if (present_current == NULL) return false;

  control_items_["Goal_Position"] = goal_position;
  control_items_["Goal_Velocity"] = goal_velocity;

  control_items_["Present_Position"] = present_position;
  control_items_["Present_Velocity"] = present_velocity;
  control_items_["Present_Current"] = present_current;

  return true;
}

bool DynamixelHand::initSDKHandlers(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);
  if (result == false) {
    std::cerr << log << std::endl;
    return result;
  } else {
    std::cerr << log << std::endl;
  }

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);
  if (result == false) {
    std::cerr << log << std::endl;
    return result;
  } else {
    std::cerr << log << std::endl;
  }

  if (dxl_wb_->getProtocolVersion() == 2.0f) {
    uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);
    uint16_t read_length = control_items["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length + 2;

    result = dxl_wb_->addSyncReadHandler(start_address, read_length, &log);
    if (result == false) {
      std::cerr << log << std::endl;
      return result;
    }
  }
  return result;
}

bool DynamixelHand::getPresentPosition(std::vector<std::string> dxl_name)
{
  bool result = false;
  const char* log = NULL;

  int32_t get_position[dxl_name.size()];

  uint8_t id_array[dxl_name.size()];
  uint8_t id_cnt = 0;

  for (auto const& name:dxl_name) {
    id_array[id_cnt++] = dynamixel_[name];
  }

  if (dxl_wb_->getPresentPosition() == 2.0f) {
    result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
        id_array,
        dxl_name.size(),
        &log);
    if (result == false) std::cerr << log << std::endl;
    WayPoint wp;
    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
        id_array,
        id_cnt,
        control_items_["Present_Position"]->address,
        control_items_["Present_Position"]->data_length,
        get_position,
        &log);
    if (result == false) {
      std::cerr << log << std::endl;
    } else {
      for (uint8_t index = 0; index < id_cnt; index++) {
        wp.position = dxl_wb_->convertValue2Radian(id_array[index], get_position[index]);
        pre_goal_.push_back(wp);
      }
    }
  } else if (dxl_wb_->getProtocolVersion() == 1.0f) {
    WayPoint wp;
    uint32_t read_position;
    for (auto const& dxl:dynamixel_) {
      result = dxl_wb_->readRegister((uint8_t)dxl.second,
          control_items_["Present_Position"]->address,
          control_items_["Present_Position"]->data_length,
          &read_position,
          &log);
      if (result == false) std::cerr << log << std::endl;
      wp.position = dxl_wb_->convertValue2Radian((uint8_t)dxl.second, read_position);
      pre_goal_.push_back(wp);
    }
  }
  return result;
}

