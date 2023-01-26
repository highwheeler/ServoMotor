#include <Arduino.h>
#include <lvgl.h>
#include "FS.h"
#include <TFT_eSPI.h>
#include <Ticker.h>
#include <UltraServo.h>

// THIS IS THE H BRIDGE PID VERSION!

#define CALIBRATION_FILE "/TouchCalData"
#define SETTINGS_FILE "/SettingsData"
#define REPEAT_CAL false
#define LVGL_TICK_PERIOD 20
#define HOR_RES 320
#define VER_RES 240
#define M1ENCP1 21
#define M1ENCP2 22
#define M1PIND1 16
#define M1PIND2 17
#define PWM1PIN 2
#define M2ENCP1 39
#define M2ENCP2 34
#define M2PIND1 14
#define M2PIND2 12
#define PWM2PIN 26
#define SAMPLERATE 200
#define PWMBIAS 125
#define MOTCOUNT 2
#define COUNTSPERREVOLITION 445

UltraServo servo1(M1ENCP1, M1ENCP2, PWM1PIN, M1PIND1, M1PIND2, SAMPLERATE, PWMBIAS);
UltraServo servo2(M2ENCP1, M2ENCP2, PWM2PIN, M2PIND1, M2PIND2, SAMPLERATE, PWMBIAS);
UltraServo *m[MOTCOUNT] = {&servo1, &servo2};
Ticker tick;
TFT_eSPI tft = TFT_eSPI(); /* TFT instance */
lv_disp_draw_buf_t disp_buf;
lv_disp_drv_t disp_drv;
lv_indev_drv_t indev_drv;

lv_color_t buf[HOR_RES * 10];
lv_obj_t *pidchart[MOTCOUNT], *numpadScr;
lv_chart_series_t *ser1[MOTCOUNT];
lv_obj_t *slider_label[MOTCOUNT];
lv_obj_t *slider[MOTCOUNT];
lv_obj_t *buildNumpadScreen();
lv_obj_t *mainScr,*tabView;
int lstTab = -1;
extern void openNumpad(lv_obj_t *valPtr);
void buildConfigScreen();
lv_style_t styleBloom;
int lstFlt[MOTCOUNT], lstEp[MOTCOUNT];
#if LV_USE_LOG != 0
/* Serial debugging */
// void my_print(lv_log_level_t level, const char *file, uint32_t line, const char *fn_name, const char *dsc)
void my_print(const char *buf)
{

  // Serial.printf("%s@%d->%s\r\n", file, line, dsc);
  // Serial.flush();
  // Serial.println(buf);
  tft.println(buf);
  while (true)
  {
  }; // hang so that error is visible
}
#endif

void touch_calibrate()
{
  uint16_t calData[5];
  uint8_t calDataOK = 0;
  if (SPIFFS.exists(CALIBRATION_FILE))
  {
    if (REPEAT_CAL)
    {
      SPIFFS.remove(CALIBRATION_FILE);
    }
    else
    {
      File f = SPIFFS.open(CALIBRATION_FILE, "r");
      if (f)
      {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CAL)
  {
    tft.setTouch(calData);
  }
  else
  {
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.println("Touch corners as indicated");
    tft.setTextFont(1);
    tft.println();
    if (REPEAT_CAL)
    {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");
    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f)
    {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}

#if LV_USE_LOG != 0
void my_print(lv_log_level_t level, const char *file, uint32_t line, const char *fn_name, const char *dsc)
{
  Serial.printf("%s@%d->%s\r\n", file, line, dsc);
  Serial.flush();
}
#endif

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors(&color_p->full, w * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  uint16_t touchX, touchY;
  bool touched = tft.getTouch(&touchX, &touchY, 600);
  if (!touched)
  {
    data->state = LV_INDEV_STATE_REL;
  }
  else
  {
    data->state = LV_INDEV_STATE_PR;
  }
  if (touchX > HOR_RES || touchY > VER_RES)
  {
  }
  else
  {
    if (3 == tft.getRotation())
    {
      data->point.x = HOR_RES - touchX;
      data->point.y = VER_RES - touchY;
    }
    else if (1 == tft.getRotation())
    {
      data->point.x = touchX;
      data->point.y = touchY;
    }
  }
}

static void lv_tick_handler(void)
{
  lv_tick_inc(LVGL_TICK_PERIOD);
}

void setup()
{

  Serial.begin(115200);
  lv_init();
  tick.attach_ms(LVGL_TICK_PERIOD, lv_tick_handler);

#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

  tft.begin();        /* TFT init */
  tft.setRotation(1); /* Portrait orientation */
  // check file system exists
  if (!SPIFFS.begin())
  {
    Serial.println("Formating file system");
    SPIFFS.format();
    SPIFFS.begin();
  }
  touch_calibrate();
  lv_disp_draw_buf_init(&disp_buf, buf, NULL, HOR_RES * 10);
  lv_disp_drv_init(&disp_drv);
  disp_drv.draw_buf = &disp_buf;
  disp_drv.hor_res = HOR_RES;
  disp_drv.ver_res = VER_RES;
  disp_drv.flush_cb = my_disp_flush;
  lv_disp_drv_register(&disp_drv);
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
  numpadScr = buildNumpadScreen();
  buildConfigScreen();
}

void loop()
{
  lv_task_handler();
  for(int i = 0; i < MOTCOUNT; i++) {
    if (m[i]->getRampRun())
    {
      lv_slider_set_value(slider[i], m[i]->getRampRpm(), LV_ANIM_ON);
    }
    lv_chart_set_next_value(pidchart[i], ser1[i], m[i]->getError() + 32);
    if(lv_scr_act() == mainScr ) {
      int ep1 = m[i]->getEncPos();
      int yofs = 12;
      int xp = 40;
      int yp = 80;
      int r = 38;
      float deg1;
      int ix1, iy1;
      bool doDraw = false;
      int n = lv_tabview_get_tab_act(tabView);
      if (lstTab!=n) {
        doDraw = true;
         lstTab = n;
     }
      if((ep1 != lstEp[i] && n == i) || doDraw) {
        lstEp[i] = ep1;
//      Serial.println(ep1);
        tft.fillCircle(xp, yp + yofs, r+2, TFT_CYAN);
        deg1 = ep1 * 6.28 / COUNTSPERREVOLITION;
        ix1 = cos(deg1) * r + xp;
        iy1 = sin(deg1) * r + yp;
        tft.drawLine(ix1, iy1 + yofs, xp, yp + yofs, TFT_BLACK);
      }
    }
  } 
/*    if (servo1.cntFlt != lstFlt)
    {
      Serial.print("flt ");
      Serial.println(servo1.cntFlt);
      lstFlt = servo1.cntFlt;
    
    }
*/

}

void ramp_event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_CLICKED)
  {
    UltraServo *s = m[lv_tabview_get_tab_act(tabView)];
    if(s->getRampRun()) {
        s->stop();
      } else {
        s->startRamp(50);
      }    
  }
}

void stop_event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_CLICKED)
  {
    char buf[8];
    int i = lv_tabview_get_tab_act(tabView);
    m[i]->enable(false);

    snprintf(buf, 4, "%u", m[i]->getTargetPos());
    lv_label_set_text(slider_label[i], buf);
    lv_slider_set_value(slider[i], m[i]->getTargetPos(), LV_ANIM_ON);
  }
}

void slider_event_cb(lv_event_t *e)
{
  lv_obj_t *slider = lv_event_get_target(e);
  int i = lv_tabview_get_tab_act(tabView);
  m[i]->setTargetPos( lv_slider_get_value(slider));

  char buf[8];
  snprintf(buf, 4, "%u", m[i]->getTargetPos());
  lv_label_set_text(slider_label[i], buf);
}

void sw_event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
      int i = lv_tabview_get_tab_act(tabView);
      m[i]->enable(lv_obj_has_state(obj, LV_STATE_CHECKED));
  }
}

void pid_event_cb(lv_event_t *e)
{
  lv_obj_t *obj = lv_event_get_target(e);
  if (lv_event_get_code(e) == LV_EVENT_CLICKED)
  {
    openNumpad(obj);
  }
}

/*
 void slider_x_event_cb(lv_event_t * e)
{
    lv_obj_t * obj = lv_event_get_target(e);
    int32_t v = lv_slider_get_value(obj);
    lv_chart_set_zoom_x(chart, v);
}

 void slider_y_event_cb(lv_event_t * e)
{
    lv_obj_t * obj = lv_event_get_target(e);
    int32_t v = lv_slider_get_value(obj);
    lv_chart_set_zoom_y(chart, v);
}

*/
lv_obj_t *bloomButton(lv_obj_t *scr,
                      lv_coord_t x, lv_coord_t y,
                      lv_coord_t w, lv_coord_t h,
                      const char *lab, lv_event_cb_t cb)
{

  lv_obj_t *btn1 = lv_btn_create(scr);
  lv_obj_add_style(btn1, &styleBloom, LV_STATE_PRESSED);
  lv_obj_set_pos(btn1, x, y);
  lv_obj_set_size(btn1, w, h);
  lv_obj_add_event_cb(btn1, cb, LV_EVENT_ALL, NULL);
  lv_obj_t *label = lv_label_create(btn1);
  lv_label_set_text(label, lab);
  lv_obj_center(label);
  return btn1;
}

lv_obj_t *floatButton(lv_obj_t *scr, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h, double *dval)
{
  char buf[8];
  lv_obj_t *btn = lv_btn_create(scr);
  lv_obj_set_pos(btn, x, y);
  lv_obj_set_size(btn, w, h);
  lv_obj_add_event_cb(btn, pid_event_cb, LV_EVENT_CLICKED, NULL);
  snprintf(buf, 8, "%01.2f", *dval);
  lv_obj_t *label = lv_label_create(btn);
  lv_label_set_text(label, buf);
  btn->user_data = label;
  label->user_data = dval;
  return btn;
}

void buildConfigScreen()
{
  mainScr = lv_scr_act();
  tabView = lv_tabview_create(mainScr, LV_DIR_TOP, 30);
  for (int i = 0; i < MOTCOUNT; i++) {
    char buf[10];
    sprintf(buf,"Motor %d", i + 1 );
    lv_obj_t * tab = lv_tabview_add_tab(tabView, buf); 
    
    pidchart[i] = lv_chart_create(tab);
    lv_obj_set_size(pidchart[i], 220, 100);
    lv_obj_set_pos(pidchart[i], 75, 0);
    lv_chart_set_type(pidchart[i], LV_CHART_TYPE_LINE); /*Show lines and points too*/
    lv_chart_set_range(pidchart[i], LV_CHART_AXIS_PRIMARY_Y, 0, 64);
    lv_obj_set_style_size(pidchart[i], 0, LV_PART_INDICATOR);

    ser1[i] = lv_chart_add_series(pidchart[i], lv_color_make(255, 0, 0), LV_CHART_AXIS_PRIMARY_Y);

    //bloomButton(scr, 0, 110, 45, 30, "Stop", stop_event_cb);
    floatButton(tab, 0, 110, 45, 30, &m[i]->kp);
    floatButton(tab, 50, 110, 45, 30, &m[i]->ki);
    floatButton(tab, 100, 110, 45, 30, &m[i]->kd);
    bloomButton(tab, 150, 110, 50, 30, "R", ramp_event_cb);

    lv_obj_t *sw = lv_switch_create(tab);
    lv_obj_set_pos(sw, 220, 110);
    lv_obj_set_size(sw, 50, 30);
    lv_obj_add_event_cb(sw, sw_event_cb, LV_EVENT_ALL,NULL);
    //sw->user_data=(void *)"1";
    /*
    lv_obj_t *sw2 = lv_switch_create(tabM1);
    lv_obj_set_pos(sw2, 260, 110);
    lv_obj_set_size(sw2, 50, 30);
    lv_obj_add_event_cb(sw2, sw_event_cb, LV_EVENT_ALL, NULL);
    sw2->user_data=(void *)"2";
  */
    slider[i] = lv_slider_create(tab);
    lv_obj_set_pos(slider[i], 20, 150);
    lv_obj_set_size(slider[i], 270, 30);
    lv_obj_add_event_cb(slider[i], slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_slider_set_range(slider[i], 0, COUNTSPERREVOLITION + 1); // one full revolution

    slider_label[i] = lv_label_create(tab);
    lv_label_set_text(slider_label[i], "0%");
    lv_obj_align_to(slider_label[i], slider[i], LV_ALIGN_CENTER, 0, 0);
  }
}
