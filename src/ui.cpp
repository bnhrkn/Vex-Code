#include "project/ui.hpp"
#include "display/lv_core/lv_obj.h"
#include "display/lv_objx/lv_tabview.h"
#include "main.h"
#include "project/ui.hpp"
#include <memory>

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



}