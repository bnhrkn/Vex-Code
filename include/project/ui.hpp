#pragma once
#include <fstream>
#include <memory>
#include "display/lv_core/lv_obj.h"
#include "main.h"

class ui {
 public:
  ui(std::unique_ptr<lv_obj_t> ihomeScreen);
  ~ui();
  void setPosition(const okapi::OdomState& state);
  int getAuton();
  bool isBlueTeam();
  void graph(double value, size_t which);

 private:
  lv_obj_t* homeScreen;

  lv_obj_t* tabView;

  lv_obj_t* graphTab;
  lv_obj_t* chart;
  std::array<lv_chart_series_t*, 2> series;

  lv_obj_t* autonTab;
  lv_obj_t* autonRoller;
  lv_obj_t* colorRoller;

  lv_obj_t* settingTab;

  lv_obj_t* logTab;
  lv_obj_t* log;

  lv_obj_t* positionTab;
  lv_obj_t* positionLabel;

};  // screen is 480 x 240 pixel