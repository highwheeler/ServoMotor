#include <lvgl.h>
#include <Arduino.h>
#define HOR_RES 320
#define VER_RES 240
lv_style_t style_bg;
extern lv_obj_t *numpadScr;
const char *btnm_map[] = {"1", "2", "3", "\n",
                          "4", "5", "6", "\n",
                          "7", "8", "9", "\n",
                          "-", LV_SYMBOL_BACKSPACE, "0", ".",
                          LV_SYMBOL_OK, LV_SYMBOL_CLOSE,
                          ""};
lv_obj_t *txtarea;
lv_obj_t *targVal, *lastScr;

void openNumpad(lv_obj_t *valPtr)
{
  lastScr = lv_scr_act(); // save the last screen to return to it
  targVal = valPtr;
  lv_obj_t *lab = (lv_obj_t *)targVal->user_data;
  lv_textarea_set_text(txtarea, lv_label_get_text(lab));
  lv_scr_load(numpadScr);
  lv_obj_add_state(txtarea, LV_STATE_FOCUSED);
}

void btnm_event_cb(lv_event_t *e)
{
  float fv;
  lv_obj_t *obj = lv_event_get_target(e);
  const char *txt = lv_btnmatrix_get_btn_text(obj, lv_btnmatrix_get_selected_btn(obj));

  if (strcmp(txt, LV_SYMBOL_BACKSPACE) == 0)
  {
    lv_textarea_del_char(txtarea);
  }
  else if (strcmp(txt, LV_SYMBOL_CLOSE) == 0)
  {
    lv_scr_load(lastScr);
  }
  else if (strcmp(txt, LV_SYMBOL_OK) == 0)
  {
    //    lv_event_send(ta, LV_EVENT_READY, NULL);
    const char *res = lv_textarea_get_text(txtarea);
    sscanf(res, "%f", &fv);

    lv_obj_t *lab = (lv_obj_t *)targVal->user_data;
    lv_label_set_text(lab, res);
    double *dval = (double *)lab->user_data;
    *dval = fv;
    lv_scr_load(lastScr);
  }
  else
  {
    lv_textarea_add_text(txtarea, txt);
  }
}

lv_obj_t *buildNumpadScreen()
{
  lv_obj_t *scr = lv_obj_create(NULL);

  lv_obj_t *numpadLabel = lv_label_create(scr);
  lv_label_set_text(numpadLabel, "Enter Value: ");
  lv_obj_set_pos(numpadLabel, 10, 10);
  lv_obj_set_width(numpadLabel, HOR_RES / 2);
  lv_obj_set_height(numpadLabel, 30);

  txtarea = lv_textarea_create(scr);
  lv_textarea_set_one_line(txtarea, true);
  lv_obj_set_pos(txtarea, HOR_RES / 2 + 10, 5);
  lv_obj_set_width(txtarea, HOR_RES / 2 - 30);
  lv_obj_set_height(txtarea, 35);

  lv_obj_t *btnmat = lv_btnmatrix_create(scr);
  lv_obj_set_pos(btnmat, 10, 40);
  lv_obj_set_width(btnmat, HOR_RES - 20);
  lv_obj_set_height(btnmat, VER_RES - 10 - 40);

  lv_style_init(&style_bg);
  lv_style_set_pad_all(&style_bg, 2);
  lv_style_set_pad_gap(&style_bg, 2);
  lv_obj_add_style(btnmat, &style_bg, 0);

  lv_obj_clear_flag(btnmat, LV_OBJ_FLAG_CLICK_FOCUSABLE);
  lv_btnmatrix_set_map(btnmat, btnm_map);
  lv_obj_add_event_cb(btnmat, btnm_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  // lv_obj_add_flag(btnm, LV_OBJ_FLAG_HIDDEN);

  return scr;
}