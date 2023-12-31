// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.2
// LVGL version: 8.3.6
// Project name: Annealer

#include "ui.h"


void ui_event_comp_BtnAutoLbl_BtnAutoLbl( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
lv_obj_t **comp_BtnAutoLbl = lv_event_get_user_data(e);
if ( event_code == LV_EVENT_CLICKED) {
      toggleAutoFn( e );
}
}

// COMPONENT BtnAutoLbl

lv_obj_t *ui_BtnAutoLbl_create(lv_obj_t *comp_parent) {

lv_obj_t *cui_BtnAutoLbl;
cui_BtnAutoLbl = lv_btn_create(comp_parent);
lv_obj_set_width( cui_BtnAutoLbl, 100);
lv_obj_set_height( cui_BtnAutoLbl, 30);
lv_obj_set_x( cui_BtnAutoLbl, 35 );
lv_obj_set_y( cui_BtnAutoLbl, -14 );
lv_obj_set_align( cui_BtnAutoLbl, LV_ALIGN_LEFT_MID );
lv_obj_add_flag( cui_BtnAutoLbl, LV_OBJ_FLAG_CHECKABLE | LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( cui_BtnAutoLbl, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(cui_BtnAutoLbl, lv_color_hex(0x0042C3), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(cui_BtnAutoLbl, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_color(cui_BtnAutoLbl, lv_color_hex(0x3C3C3C), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_shadow_opa(cui_BtnAutoLbl, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_ofs_x(cui_BtnAutoLbl, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_ofs_y(cui_BtnAutoLbl, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_color(cui_BtnAutoLbl, lv_color_hex(0xFFED00), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(cui_BtnAutoLbl, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(cui_BtnAutoLbl, &lv_font_montserrat_18, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_t ** children = lv_mem_alloc(sizeof(lv_obj_t *) * _UI_COMP_BTNAUTOLBL_NUM);
children[UI_COMP_BTNAUTOLBL_BTNAUTOLBL] = cui_BtnAutoLbl;
lv_obj_add_event_cb(cui_BtnAutoLbl, get_component_child_event_cb, LV_EVENT_GET_COMP_CHILD, children);
lv_obj_add_event_cb(cui_BtnAutoLbl, del_component_child_event_cb, LV_EVENT_DELETE, children);
lv_obj_add_event_cb(cui_BtnAutoLbl, ui_event_comp_BtnAutoLbl_BtnAutoLbl, LV_EVENT_ALL, children);
ui_comp_BtnAutoLbl_create_hook(cui_BtnAutoLbl);
return cui_BtnAutoLbl; 
}

