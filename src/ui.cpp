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

ui::ui(std::unique_ptr<lv_obj_t> ihomeScreen)
    : homeScreen(
          ihomeScreen.release()) { // Try to enforce an ownership transfer
  lv_theme_set_current(lv_theme_night_init(266, &lv_font_dejavu_20));

  tabView = lv_tabview_create(homeScreen, NULL);

  autonTab = lv_tabview_add_tab(tabView, "Auto");
  autonRoller = lv_roller_create(autonTab, NULL);
  
  lv_roller_set_options(autonRoller, "Disabled\nCross Field\nRight Full\nRight Roller\nLeft Roller\nLeft Full\nTest\n");
  lv_roller_set_action(autonRoller, NULL);

  colorRoller = lv_roller_create(autonTab, NULL);
  lv_roller_set_options(colorRoller, "Blue\nRed\n");
  
  graphTab = lv_tabview_add_tab(tabView, "Graph");
  chart = lv_chart_create(graphTab, NULL);
  series[0] = lv_chart_add_series(chart, LV_COLOR_BLUE);
  series[1] = lv_chart_add_series(chart, LV_COLOR_RED);
  lv_chart_set_point_count(chart, 500);
  lv_chart_set_range(chart, 0, 600);
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_obj_set_size(chart, 460, 161);

  settingTab = lv_tabview_add_tab(tabView, "Settings");

  logTab = lv_tabview_add_tab(tabView, "Log");
  log = lv_label_create(logTab, NULL);
  lv_label_set_long_mode(log, LV_LABEL_LONG_BREAK);
  lv_obj_set_width(log, hres);
  // lv_label_set_text(lv_obj_t *label, const char *text) // log output

  positionTab = lv_tabview_add_tab(tabView, "Position");
  positionLabel = lv_label_create(positionTab, NULL);
  lv_label_set_long_mode(positionLabel, LV_LABEL_LONG_BREAK);
  lv_obj_set_width(positionLabel, hres);
  // lv_obj_align(positionLabel, positionTab, LV_ALIGN_IN_TOP_LEFT, 0, 0);
};
ui::~ui() {
  lv_obj_del(tabView);

  lv_obj_del(graphTab);
  lv_obj_del(chart);

  lv_obj_del(autonTab);
  lv_obj_del(autonRoller);
  lv_obj_del(colorRoller);

  lv_obj_del(settingTab);
  lv_obj_del(settingTab);

  lv_obj_del(logTab);
  lv_obj_del(log);

  lv_obj_del(positionTab);
  lv_obj_del(positionLabel);
}

void ui::setPosition(const okapi::OdomState &state) {
  using namespace okapi::literals;
  auto [x, y, yaw] = state;
  std::ostringstream out;
  out << "X: " << x.convert(1_in) << "\"\nY: " << y.convert(1_in)
      << "\"\nYaw: " << yaw.convert(1_deg) << "°";
  lv_label_set_text(positionLabel, out.str().c_str());
}
int ui::getAuton() {
  return lv_roller_get_selected(autonRoller);
}
bool ui::isBlueTeam(){
  return !static_cast<bool>(lv_roller_get_selected(colorRoller));
}
void ui::graph(double value, size_t which){
  lv_chart_set_next(chart, series[which], value);
}