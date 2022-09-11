#include "project/ui.hpp"
#include "display/lv_core/lv_obj.h"
#include "display/lv_objx/lv_label.h"
#include "display/lv_objx/lv_tabview.h"
#include "main.h"
#include "okapi/api/odometry/odomState.hpp"
#include "project/ui.hpp"
#include <memory>
#include <sstream>
#include <string>
constexpr int hres = 240, vres = 240;

ui::ui(){};

std::shared_ptr<ui> ui::getInstance() {
  static std::shared_ptr<ui> instance(new ui);
  return instance;
}

void ui::init() {
  lv_theme_set_current(lv_theme_night_init(266, &lv_font_dejavu_20));

  homeScreen = lv_scr_act();
  tabView = lv_tabview_create(homeScreen, NULL);

  autonTab = lv_tabview_add_tab(tabView, "Auto");



  graphTab = lv_tabview_add_tab(tabView, "Graph");
  chart = lv_chart_create(graphTab, NULL);
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_obj_set_size(chart, 460, 161);
  


  settingTab = lv_tabview_add_tab(tabView, "Settings");

  logTab = lv_tabview_add_tab(tabView, "Log");
  log = lv_label_create(logTab, NULL);
  lv_label_set_long_mode(log, LV_LABEL_LONG_BREAK);
  lv_obj_set_width(log, hres);
  //lv_label_set_text(lv_obj_t *label, const char *text) // log output

  positionTab = lv_tabview_add_tab(tabView, "Position");
  positionLabel = lv_label_create(positionTab, NULL);
  lv_label_set_long_mode(positionLabel, LV_LABEL_LONG_BREAK);
  lv_obj_set_width(positionLabel, hres);
  //lv_obj_align(positionLabel, positionTab, LV_ALIGN_IN_TOP_LEFT, 0, 0);

}

void ui::setPosition(const okapi::OdomState &state) {
  using namespace okapi::literals;
  std::ostringstream out;
  out << "X: " << state.x.convert(1_in) << 
          "\"\nY: " << state.y.convert(1_in) <<
          "\"\nYaw: " << state.theta.convert(1_deg) << "°";
  lv_label_set_text(positionLabel, out.str().c_str());
}