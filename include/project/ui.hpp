#pragma once
#include <fstream>
#include <memory>
#include "liblvgl/lvgl.h"
#include "main.h"
#include "project/Camera.hpp"
#include "project/auton.hpp"

class UI {
 public:
  explicit UI(std::unique_ptr<lv_obj_t> ihomeScreen);
  ~UI();
  void setPosition(const okapi::OdomState& state);
  [[nodiscard]] auton::AutonMode getAuton();
  void setAutonList(const std::vector<std::string>& autonList);
  [[nodiscard]] bool isBlueTeam() const;
  void graph(double value, size_t which);
  okapi::IterativePosPIDController::Gains getPIDGains();

 private:
  // std::fstream cache = std::fstream("/usd/auton.txt");
  //  void storeAuton(auton::AutonMode auton);
  //  void storeColor(bool blue);
  //  auton::AutonMode loadAuton();
  //  bool loadColor();
  bool blue = true;
  lv_obj_t* homeScreen;

  lv_obj_t* tabView;

  lv_obj_t* homeTab;
  lv_obj_t* autonRoller;
  lv_obj_t* colorRoller;

  lv_obj_t* graphTab;
  lv_obj_t* chart;
  std::array<lv_chart_series_t*, 2> series;

  lv_obj_t* positionTab;
  lv_obj_t* positionLabel;
};  // screen is 480 x 240 pixel