#include "project/ui.hpp"
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include "liblvgl/lvgl.h"
#include "main.h"
#include "okapi/api/odometry/odomState.hpp"
#include "project/auton.hpp"
constexpr int hres = 480, vres = 240;

UI::UI(std::unique_ptr<lv_obj_t> ihomeScreen)
    : homeScreen(ihomeScreen.release()),
      tabView(lv_tabview_create(homeScreen, LV_DIR_TOP, 30)),
      homeTab(lv_tabview_add_tab(tabView, "Home")),
      autonRoller(lv_roller_create(homeTab)),
      colorRoller(lv_roller_create(homeTab)),
      colorBtn(lv_btn_create(homeTab)),
      graphTab(lv_tabview_add_tab(tabView, "Graph")),
      chart(lv_chart_create(graphTab)),
      series({lv_chart_add_series(chart,
                                  lv_palette_main(LV_PALETTE_BLUE),
                                  LV_CHART_AXIS_PRIMARY_X),
              lv_chart_add_series(chart,
                                  lv_palette_main(LV_PALETTE_RED),
                                  LV_CHART_AXIS_PRIMARY_X)}),
      positionTab(lv_tabview_add_tab(tabView, "Position")),
      positionLabel(lv_label_create(positionTab)) {
  constexpr lv_coord_t pad = 5;
  constexpr lv_coord_t tabview_height = vres - 30;
  lv_roller_set_options(autonRoller, auton::autonNames.c_str(),
                        LV_ROLLER_MODE_INFINITE);

  lv_obj_set_size(autonRoller, 160 - 2 * pad, tabview_height - 2 * pad);
  lv_obj_align_to(autonRoller, homeTab, LV_ALIGN_TOP_LEFT, pad, pad);
  // lv_roller_set_options(colorRoller, "Blue\nRed\n", LV_ROLLER_MODE_NORMAL);

  lv_chart_set_point_count(chart, 500);
  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 600);
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_obj_set_size(chart, 460, 161);

  lv_obj_add_event_cb(
      colorBtn,
      [](lv_event_t* event) {
        auto* obj = lv_event_get_target(event);
        UI* gui = static_cast<UI*>(obj->user_data);
        gui->blue = lv_obj_get_state(obj) == LV_STATE_CHECKED;
      },
      LV_EVENT_CLICKED, this);

  lv_label_set_long_mode(positionLabel, LV_LABEL_LONG_CLIP);
  lv_obj_set_width(positionLabel, hres);
};
UI::~UI() {
  lv_obj_del(tabView);

  lv_obj_del(graphTab);
  lv_obj_del(chart);

  lv_obj_del(homeTab);
  lv_obj_del(autonRoller);
  lv_obj_del(colorRoller);
  lv_obj_del(colorBtn);

  lv_obj_del(positionTab);
  lv_obj_del(positionLabel);
}

void UI::setPosition(const okapi::OdomState& state) {
  using namespace okapi::literals;
  auto [x, y, yaw] = state;
  auto out = std::format("X: {}\"\nY: {}\"\nYaw: {}°", x.convert(1_in),
                         y.convert(1_in), yaw.convert(1_deg));
  lv_label_set_text(positionLabel, out.c_str());
}
auton::AutonMode UI::getAuton() {
  return static_cast<auton::AutonMode>(lv_roller_get_selected(autonRoller));
}
[[nodiscard, deprecated("FIX ME!")]] bool UI::isBlueTeam() const {
  return false;
}
void UI::graph(double value, size_t which) {
  try {
    lv_chart_set_next_value(chart, series.at(which), value);
  } catch (std::out_of_range& except) {
    std::cout << std::format("No ui chart series exists at index {}. Error: {}",
                             which, except.what());
  }
}

// void UI::storeAuton(auton::AutonMode auton) {

//   if (cache.is_open()) {
//     std::string autonLine;
//     std::getline(cache, autonLine);

//   }
//   else{

//   }
// };
// void UI::storeColor(bool blue){

// };
// auton::AutonMode UI::loadAuton(){

// };
// bool UI::loadColor(){

// };
