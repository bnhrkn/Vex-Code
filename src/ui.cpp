#include "project/ui.hpp"
#include <memory>
#include <sstream>
#include <string>
#include "liblvgl/lvgl.h"
#include "main.h"
#include "okapi/api/odometry/odomState.hpp"
constexpr int hres = 240, vres = 240;

ui::ui(std::unique_ptr<lv_obj_t> ihomeScreen)
    : homeScreen(ihomeScreen.release()),
      tabView(lv_tabview_create(homeScreen, LV_DIR_TOP, 20)),
      graphTab(lv_tabview_add_tab(tabView, "Graph")),
      chart(lv_chart_create(graphTab)),
      series({lv_chart_add_series(chart,
                                  lv_palette_main(LV_PALETTE_BLUE),
                                  LV_CHART_AXIS_PRIMARY_X),
              lv_chart_add_series(chart,
                                  lv_palette_main(LV_PALETTE_RED),
                                  LV_CHART_AXIS_PRIMARY_X)}),
      autonTab(lv_tabview_add_tab(tabView, "Auto")),
      autonRoller(lv_roller_create(autonTab)),
      colorRoller(lv_roller_create(autonTab)),
      settingTab(lv_tabview_add_tab(tabView, "Settings")),
      positionTab(lv_tabview_add_tab(tabView, "Position")),
      positionLabel(lv_label_create(positionTab)) {
  // Try to enforce an ownership transfer
  // lv_theme_set_current(lv_theme_night_init(266, LV_FONT_MONTSERRAT_10));
  lv_obj_set_size(autonRoller,
                  lv_obj_get_width(autonTab) / static_cast<short>(2),
                  lv_obj_get_height(autonTab));
  lv_obj_align(autonRoller, LV_ALIGN_RIGHT_MID,
               -lv_obj_get_width(autonRoller) / 2 + 5, 0);
  lv_roller_set_options(autonRoller,
                        "Disabled\nCross Field\nRight Full\nRight Roller\nLeft "
                        "Roller\nLeft Full\nProg Skills\nTest\n",
                        LV_ROLLER_MODE_INFINITE);

  lv_roller_set_options(colorRoller, "Blue\nRed\n", LV_ROLLER_MODE_INFINITE);

  lv_chart_set_point_count(chart, 500);
  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 600);
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_obj_set_size(chart, 460, 161);

  lv_label_set_long_mode(positionLabel, LV_LABEL_LONG_CLIP);
  lv_obj_set_width(positionLabel, hres);
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

  lv_obj_del(positionTab);
  lv_obj_del(positionLabel);
}

void ui::setPosition(const okapi::OdomState& state) {
  using namespace okapi::literals;
  auto [x, y, yaw] = state;
  auto out = std::format("X: {}\"\nY: {}\"\nYaw: {}°", x.convert(1_in),
                         y.convert(1_in), yaw.convert(1_deg));
  lv_label_set_text(positionLabel, out.c_str());
}
int ui::getAuton() {
  return lv_roller_get_selected(autonRoller);
}
bool ui::isBlueTeam() {
  return !static_cast<bool>(lv_roller_get_selected(colorRoller));
}
void ui::graph(double value, size_t which) {
  try {
    lv_chart_set_next_value(chart, series.at(which), value);
  } catch (std::out_of_range except) {
    std::cout << std::format("No ui chart series exists at index {}. Error: {}",
                             which, except.what());
  }
}