#pragma once
#include "display/lv_core/lv_obj.h"
#include "main.h"
#include <memory>

namespace std {
template <> class default_delete<lv_obj_t> {
public:
  void operator()(lv_obj_t *ptr);
};
} // namespace std

class ui {
public:
  static std::shared_ptr<ui> getInstance();

  void init();

  void setPosition(const okapi::OdomState &state);

 


protected:
  ui();

  lv_obj_t *homeScreen;

  lv_obj_t *tabView;

  lv_obj_t *graphTab;
  lv_obj_t *chart;

  lv_obj_t *autonTab;

  lv_obj_t *settingTab;

  lv_obj_t *logTab;
  lv_obj_t *log;

  lv_obj_t *positionTab;
  lv_obj_t *positionLabel;

}; // screen is 480 x 240 pixel