// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 8.3.11
// Project name: UI

#include "ui.h"

void ui_hienthi_screen_init(void)
{
    ui_hienthi = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_hienthi, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_doAmHT = lv_bar_create(ui_hienthi);
    lv_bar_set_value(ui_doAmHT, 25, LV_ANIM_OFF);
    lv_bar_set_start_value(ui_doAmHT, 0, LV_ANIM_OFF);
    lv_obj_set_width(ui_doAmHT, 150);
    lv_obj_set_height(ui_doAmHT, 10);
    lv_obj_set_x(ui_doAmHT, -1);
    lv_obj_set_y(ui_doAmHT, 18);
    lv_obj_set_align(ui_doAmHT, LV_ALIGN_CENTER);

    ui_anhSangHT = lv_bar_create(ui_hienthi);
    lv_bar_set_value(ui_anhSangHT, 25, LV_ANIM_OFF);
    lv_bar_set_start_value(ui_anhSangHT, 0, LV_ANIM_OFF);
    lv_obj_set_width(ui_anhSangHT, 150);
    lv_obj_set_height(ui_anhSangHT, 10);
    lv_obj_set_x(ui_anhSangHT, -1);
    lv_obj_set_y(ui_anhSangHT, 49);
    lv_obj_set_align(ui_anhSangHT, LV_ALIGN_CENTER);

    ui_nhietDoHT = lv_bar_create(ui_hienthi);
    lv_bar_set_value(ui_nhietDoHT, 25, LV_ANIM_OFF);
    lv_bar_set_start_value(ui_nhietDoHT, 0, LV_ANIM_OFF);
    lv_obj_set_width(ui_nhietDoHT, 150);
    lv_obj_set_height(ui_nhietDoHT, 10);
    lv_obj_set_x(ui_nhietDoHT, -1);
    lv_obj_set_y(ui_nhietDoHT, -11);
    lv_obj_set_align(ui_nhietDoHT, LV_ALIGN_CENTER);

    ui_nhietDo_txt = lv_label_create(ui_hienthi);
    lv_obj_set_width(ui_nhietDo_txt, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_nhietDo_txt, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_nhietDo_txt, -39);
    lv_obj_set_y(ui_nhietDo_txt, -29);
    lv_obj_set_align(ui_nhietDo_txt, LV_ALIGN_CENTER);
    lv_label_set_text(ui_nhietDo_txt, "nhiet do : --°C");
    lv_obj_set_style_text_font(ui_nhietDo_txt, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_doAm_txt = lv_label_create(ui_hienthi);
    lv_obj_set_width(ui_doAm_txt, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_doAm_txt, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_doAm_txt, -45);
    lv_obj_set_y(ui_doAm_txt, 4);
    lv_obj_set_align(ui_doAm_txt, LV_ALIGN_CENTER);
    lv_label_set_text(ui_doAm_txt, "do am : --%");
    lv_obj_set_style_text_font(ui_doAm_txt, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_anhSang_txt = lv_label_create(ui_hienthi);
    lv_obj_set_width(ui_anhSang_txt, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_anhSang_txt, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_anhSang_txt, -39);
    lv_obj_set_y(ui_anhSang_txt, 33);
    lv_obj_set_align(ui_anhSang_txt, LV_ALIGN_CENTER);
    lv_label_set_text(ui_anhSang_txt, "anh sang : --%");
    lv_label_set_recolor(ui_anhSang_txt, "true");
    lv_obj_set_style_text_font(ui_anhSang_txt, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_time = lv_label_create(ui_hienthi);
    lv_obj_set_width(ui_time, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_time, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_time, 52);
    lv_obj_set_y(ui_time, -45);
    lv_obj_set_align(ui_time, LV_ALIGN_CENTER);
    lv_label_set_text(ui_time, "18:20");

    lv_obj_add_event_cb(ui_hienthi, ui_event_hienthi, LV_EVENT_ALL, NULL);
    uic_hienthi = ui_hienthi;
    uic_doAmHT = ui_doAmHT;
    uic_anhSangHT = ui_anhSangHT;
    uic_nhietDoHT = ui_nhietDoHT;
    uic_nhietDo_txt = ui_nhietDo_txt;
    uic_doAm_txt = ui_doAm_txt;
    uic_anhSang_txt = ui_anhSang_txt;
    uic_time1 = ui_time;

}
