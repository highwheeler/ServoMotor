#include <Arduino.h>
#include <lvgl.h>
#include "FS.h"
#include <TFT_eSPI.h>
#include <Ticker.h>
#include <UltraServo.h>

#define CALIBRATION_FILE "/TouchCalData"
#define SETTINGS_FILE "/SettingsData"
#define LEARN_FILE "/LearnData"
#define PID_FILE "/PidData"
#define REPEAT_CAL false
#define LVGL_TICK_PERIOD 20
#define HOR_RES 320
#define VER_RES 240
#define M1ENCP1 21
#define M1ENCP2 22
#define M1PIND1 17
#define M1PIND2 16
#define PWM1PIN 2
#define M2ENCP1 39
#define M2ENCP2 34
#define M2PIND1 12
#define M2PIND2 14
#define PWM2PIN 26
#define SAMPLERATE 1500
#define PWMBIAS 125
#define MOTCOUNT 2
#define SEQUENCEDELAY 25
#define ISRFREQ 500
#define MAXLEARN 10000
UltraServo servo1(M1ENCP1, M1ENCP2, PWM1PIN, M1PIND1, M1PIND2, SAMPLERATE, PWMBIAS);
UltraServo servo2(M2ENCP1, M2ENCP2, PWM2PIN, M2PIND1, M2PIND2, SAMPLERATE, PWMBIAS);
UltraServo *m[MOTCOUNT] = {&servo1, &servo2};
int seq[] = {-200, 0, 150, -200, 0, -150, 200, -150, 200, -250, 300, 0, -270, 0, 230, -270, 0, 230, -270, 230, -180, 230, -150, 0};
int seqTime[] = {200, 100, 100, 200, 100, 100, 100, 100, 100, 100, 200, 200, 200, 100, 100, 200, 100, 100, 100, 100, 100, 100, 200, 500};
// int seq[] =     {-200, 0  , 150, -200,  0, -150, 200, -150, 200, -250, 300, 0  , -270, 0  , 230, -270, 0  , 230 , -270, 230 , -180, 230, -150,0,
// 200,  0  ,-200,   0, 250,   0, 225,   0, 200,   0, 200, 150,   0, 150, 200,   0, 250, 150,   0, 225,  0, 200,   0, 200,   0};
// int seqTime[] = { 200, 100, 100, 200, 100, 100 , 100, 100 , 100,  100, 200, 200,  200, 100, 100, 200 , 100, 100 , 100 , 100 , 100 , 100, 200,500,
// 100 , 100,200 , 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
int qtrCtr = 0;
int seqPtr = 0;
int seqDly = 0;
bool seqRun = false;
int learnPtr, learnCnt = 0;
int learnAry[MAXLEARN];
bool writeLearn = false;
bool writePid = false;
double m0psav, m0dsav, m1psav, m1dsav;
Ticker tick;
TFT_eSPI tft = TFT_eSPI();
lv_disp_draw_buf_t disp_buf;
lv_disp_drv_t disp_drv;
lv_indev_drv_t indev_drv;
lv_color_t buf[HOR_RES * 10];
lv_obj_t *pidchart[MOTCOUNT], *numpadScr;
lv_chart_series_t *ser1[MOTCOUNT], *ser2[MOTCOUNT], *ser3[MOTCOUNT];
lv_obj_t *slider_label[MOTCOUNT];
lv_obj_t *slider[MOTCOUNT];
lv_obj_t *pwrsw[MOTCOUNT];
lv_obj_t *meter[MOTCOUNT];
bool showErr[MOTCOUNT];
bool showVel[MOTCOUNT];
bool showPWM[MOTCOUNT];
lv_meter_indicator_t *indic[MOTCOUNT];
lv_obj_t *buildNumpadScreen();
lv_obj_t *mainScr, *tabView, *dropdown, *runcb;
int lstTab = -1;
bool repeat = false;
bool clrRunflg = false;
extern void openNumpad(lv_obj_t *valPtr);
void buildConfigScreen();
lv_style_t styleBloom;

int randDly = 0;
int randPos = 0;
int secDiv = 0;
int mode = -1;

// int lstFlt[MOTCOUNT], lstEp[MOTCOUNT];
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
        {
          calDataOK = 1;
        }
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
    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");
    delay(2000);
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
void IRAM_ATTR timerISR()
{
  if (mode == 1)
  { // random
    randDly--;
    if (randDly <= 0)
    {
      randDly = RAMPPAUSE * 1;
      int targetPos = random(200);
      for (int i = 0; i < MOTCOUNT; i++)
      {
        m[i]->setTargetPos(targetPos);
      }
    }
  }
  else if (mode == 2)
  { // m2 follow m1
    m[1]->setTargetPos(m[0]->getEncPos());
  }
  else if (mode == 3)
  { // m1 follow m2
    m[0]->setTargetPos(m[1]->getEncPos());
  }
  else if (mode == 4)
  { // m2 follow m1 rev
    m[1]->setTargetPos(m[0]->getEncPos() * -1);
  }
  else if (mode == 5)
  { // m1 follow m2 rev
    m[0]->setTargetPos(m[1]->getEncPos() * -1);
  }
  else if (mode == 6)
  { // sequence
    if (seqDly > 0)
    {
      seqDly--;
    }
    else
    {
      seqDly = seqTime[seqPtr];
      m[0]->setRpm(seq[seqPtr]);
      //   m[1]->setRpm(seq[seqPtr] * -1);
      seqPtr++;
      if (seqPtr >= sizeof(seq) / sizeof(int))
      {
        seqPtr = 0;
      }
    }
  }
  else if (mode == 7)
  { // clock
    secDiv++;
    if (secDiv % ISRFREQ == 0)
    {                                                                  // modulo math - causes next line to execute once a sec
      m[1]->setTargetPos(COUNTSPERREVOLUTION / 60 * secDiv / ISRFREQ); // snap to seconds
    }
    m[0]->setTargetPos(COUNTSPERREVOLUTION / 60 * secDiv / ISRFREQ); // continuous
  }
  else if (mode == 8)
  {
    learnAry[learnPtr] = m[1]->getEncPos();
    learnCnt = learnPtr++;
    if (learnPtr == MAXLEARN)
    {
      mode = -1;
      clrRunflg = true;
      writeLearn = true;
    }
  }
  else if (mode == 9)
  {
    if (learnPtr >= learnCnt)
    {
      mode = -1;
      clrRunflg = true;
      m[0]->enable(false);
      m[1]->enable(false);
    }
    else
    {
      m[0]->setTargetPos(learnAry[learnPtr]);
      m[1]->setTargetPos(learnAry[learnPtr++]);
    }
  }
  else if (mode == 10)
  { // haptic
    m[0]->setTargetPos(m[1]->getEncPos());
    m[1]->setTargetPos(m[0]->getEncPos());
  }
  else if (mode == 11)
  { // Speed Ctrl
    m[0]->setRpm(m[1]->getEncPos());
  }
  else if (mode == 12)
  { // Quarters
    secDiv++;
    if (secDiv % qtrCtr == 0)
    {
      m[0]->setTargetPos(COUNTSPERREVOLUTION / 4 * secDiv / qtrCtr);
      m[1]->setTargetPos(COUNTSPERREVOLUTION / 4 * secDiv / qtrCtr);
      if (secDiv % (qtrCtr * 8) == 0)
      {
        qtrCtr -= 10;
        secDiv = 0;

        if (qtrCtr <= 0)
        {
          qtrCtr = 100;
        }
      }
    }
  }
  else if (mode == 13)
  { // Friction

    secDiv++;
    if (secDiv > 1)
    {
      secDiv = 0;
      m[0]->setTargetPos(m[0]->getEncPos());
      m[1]->setTargetPos(m[1]->getEncPos());
      
      /*
      int r0 = m[0]->getRpm();
      if (r0 > 0)
        r0--;
      else if (r0 < 0)
        r0++;
      m[0]->setRpm(r0);
      int r1 = m[1]->getRpm();
      if (r1 > 0)
        r1--;
      else if (r1 < 0)
        r1++;
      m[1]->setRpm(r1);
      */
    }
  }
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
  tft.setRotation(3); /* Portrait orientation */
  // check file system exists
  if (!SPIFFS.begin())
  {
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(40, 80);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.println("Formatting file system - please wait...");
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

  File f = SPIFFS.open(LEARN_FILE, "r");
  if (f)
  {
    f.readBytes((char *)&learnCnt, sizeof(int));
    f.readBytes((char *)&learnAry, sizeof(int) * learnCnt);
    f.close();
  }
  File fp = SPIFFS.open(PID_FILE, "r");
  if (fp)
  {
    fp.readBytes((char *)&m[0]->kp, sizeof(double));
    fp.readBytes((char *)&m[0]->ki, sizeof(double));
    fp.readBytes((char *)&m[0]->kd, sizeof(double));
    fp.readBytes((char *)&m[1]->kp, sizeof(double));
    fp.readBytes((char *)&m[1]->ki, sizeof(double));
    fp.readBytes((char *)&m[1]->kd, sizeof(double));
    fp.close();
  }

  numpadScr = buildNumpadScreen();
  buildConfigScreen();
  hw_timer_t *timer = timerBegin(MOTCOUNT + 1, 80, true); // Begin timer with 1 MHz frequency (80MHz/80)
  timerAttachInterrupt(timer, timerISR, true);
  unsigned int timerFactor = 1000000 / ISRFREQ;
  timerAlarmWrite(timer, timerFactor, true);
  timerAlarmEnable(timer);
}

static void mb_event_cb(lv_event_t *e)
{
  if (lv_event_get_code(e) == LV_EVENT_PRESSED)
  {
    lv_msgbox_close(lv_obj_get_parent(lv_event_get_target(e)));
  }
}
void loop()
{
  lv_task_handler();
  Serial.println(m[0]->getRpm());
  for (int i = 0; i < MOTCOUNT; i++)
  {
    if (m[i]->getStallFlg())
    {
      m[i]->setStallFlg(false);
      lv_obj_clear_state(pwrsw[i], LV_STATE_CHECKED);
      static const char *btns[] = {"Close", ""};
      static char msg[25];
      sprintf(msg, "Motor %d stalled", i + 1);
      lv_obj_t *mbox = lv_msgbox_create(NULL, "Fault Detected", msg, btns, false);
      lv_obj_add_event_cb(mbox, mb_event_cb, LV_EVENT_ALL, NULL);
      lv_obj_center(mbox);
    }
    if (m[i]->getRampRun())
    {
      lv_slider_set_value(slider[i], m[i]->getRampRpm(), LV_ANIM_ON);
    }
    if (showErr[i])
    {
      lv_chart_set_next_value(pidchart[i], ser1[i], m[i]->getError() + 32);
    }
    if (showVel[i])
    {
      lv_chart_set_next_value(pidchart[i], ser2[i], m[i]->getVelocity() * 5 + 32);
    }
    if (showPWM[i])
    {
      lv_chart_set_next_value(pidchart[i], ser3[i], m[i]->pwm / 8 + 32);
    }
    int v = m[i]->getEncPos() % COUNTSPERREVOLUTION;
    if (v < 0)
    {
      v += COUNTSPERREVOLUTION;
    }
    lv_meter_set_indicator_end_value(meter[i], indic[i], v);
    // this needs to happen outside ISR
    if (clrRunflg)
    {
      clrRunflg = false;
      lv_obj_clear_state(runcb, LV_STATE_CHECKED);
    }
    if (writeLearn)
    {
      writeLearn = false;
      File f = SPIFFS.open(LEARN_FILE, "w");
      if (f)
      {
        f.write((const unsigned char *)&learnCnt, sizeof(int));
        f.write((const unsigned char *)&learnAry, sizeof(int) * learnCnt);
        f.close();
      }
    }
    if (writePid)
    {
      writePid = false;
      File fp = SPIFFS.open(PID_FILE, "w");
      if (fp)
      {
        fp.write((const unsigned char *)&(m[0]->kp), sizeof(double));
        fp.write((const unsigned char *)&m[0]->ki, sizeof(double));
        fp.write((const unsigned char *)&m[0]->kd, sizeof(double));
        fp.write((const unsigned char *)&m[1]->kp, sizeof(double));
        fp.write((const unsigned char *)&m[1]->ki, sizeof(double));
        fp.write((const unsigned char *)&m[1]->kd, sizeof(double));
        fp.close();
      }

      if (m[i]->cntFlt != m[i]->lstFlt)
      {
        Serial.print("M ");
        Serial.print(i);
        Serial.print(" flt ");
        Serial.println(m[i]->cntFlt);
        m[i]->lstFlt = m[i]->cntFlt;
      }
    }
    // Serial.println(m[0]->pwm);
    // Serial.println(m[0]->targetPos);
    /*
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
     */
  }
}

void rnd_event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_CLICKED)
  {
    UltraServo *s = m[lv_tabview_get_tab_act(tabView)];
    lv_obj_t *obj = lv_event_get_target(e);
    if (!(lv_obj_get_state(obj) & LV_STATE_CHECKED))
    {
      s->stop();
    }
    else
    {
      s->setRpm(lv_slider_get_value(slider[lv_tabview_get_tab_act(tabView)]));
    }
    /*
    if (s->getRandomRun())
    {
      s->stop();
    }
    else
    {
      s->startRandom(lv_slider_get_value(slider[lv_tabview_get_tab_act(tabView)]));
    }
    */
  }
}

void ramp_event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_CLICKED)
  {
    UltraServo *s = m[lv_tabview_get_tab_act(tabView)];
    lv_obj_t *obj = lv_event_get_target(e);
    if (!(lv_obj_get_state(obj) & LV_STATE_CHECKED))
    {
      s->stop();
    }
    else
    {
      s->startRamp(lv_slider_get_value(slider[lv_tabview_get_tab_act(tabView)]));
    }
  }
}

void slider_event_cb(lv_event_t *e)
{
  int i = lv_tabview_get_tab_act(tabView);
  lv_obj_t *sl = slider[i];
  int s = lv_slider_get_value(sl);
  if (m[i]->getRpmRun())
  {
    m[i]->setRpm(s);
  }
  else
  {
    m[i]->setTargetPos(s);
  }
  char buf[8];
  snprintf(buf, 4, "%u", lv_slider_get_value(sl));
  lv_label_set_text(slider_label[i], buf);
}

void up_event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    lv_obj_t *sl = slider[lv_tabview_get_tab_act(tabView)];
    int s = lv_slider_get_value(sl);
    lv_slider_set_value(sl, s + 1, LV_ANIM_ON);

    slider_event_cb(NULL);
  }
}
void dn_event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    lv_obj_t *sl = slider[lv_tabview_get_tab_act(tabView)];
    int s = lv_slider_get_value(sl);
    lv_slider_set_value(sl, s - 1, LV_ANIM_ON);
    slider_event_cb(NULL);
  }
}

void err_event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    int i = lv_tabview_get_tab_act(tabView);
    showErr[i] = lv_obj_get_state(obj) & LV_STATE_CHECKED;
    if (!showErr[i])
    {
      lv_chart_set_all_value(pidchart[i], ser1[i], LV_CHART_POINT_NONE);
    }
  }
}

void vel_event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    int i = lv_tabview_get_tab_act(tabView);
    showVel[i] = lv_obj_get_state(obj) & LV_STATE_CHECKED;
    if (!showVel[i])
    {
      lv_chart_set_all_value(pidchart[i], ser2[i], LV_CHART_POINT_NONE);
    }
  }
}

void pwm_event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    int i = lv_tabview_get_tab_act(tabView);
    showPWM[i] = lv_obj_get_state(obj) & LV_STATE_CHECKED;
    if (!showPWM[i])
    {
      lv_chart_set_all_value(pidchart[i], ser3[i], LV_CHART_POINT_NONE);
    }
  }
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

void dd_event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_VALUE_CHANGED)
  {
    if (mode == 10 )
    { //  restore pid from haptic
      m[0]->kp = m0psav;
      m[0]->kd = m0dsav;
      m[1]->kp = m1psav;
      m[1]->kd = m1dsav;
    }
    lv_obj_clear_state(runcb, LV_STATE_CHECKED);
    mode = -1;
    m[0]->enable(false);
    m[1]->enable(false);
  }
}
void run_event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    bool b = lv_obj_has_state(obj, LV_STATE_CHECKED);
    char buf[32];
    lv_dropdown_get_selected_str(dropdown, buf, sizeof(buf));
    if (!b)
    {
      if (mode == 8)
      {
        writeLearn = true;
      }
      if (mode == 10 )
      { //  restore pid from haptic
        m[0]->kp = m0psav;
        m[0]->kd = m0dsav;
        m[1]->kp = m1psav;
        m[1]->kd = m1dsav;
      }
      mode = -1;
      m[0]->enable(false);
      m[1]->enable(false);
    }
    else
    {
      if (!strcmp(buf, "Random"))
      {
        m[0]->enable(true);
        m[1]->enable(true);
        mode = 1;
      }
      else if (!strcmp(buf, "M2 follow M1"))
      {
        m[0]->enable(false);
        m[1]->enable(true);
        mode = 2;
      }
      else if (!strcmp(buf, "M1 follow M2"))
      {
        m[0]->enable(true);
        m[1]->enable(false);
        mode = 3;
      }
      else if (!strcmp(buf, "M2 follow M1 REV"))
      {
        m[0]->enable(false);
        m[1]->enable(true);
        mode = 4;
      }
      else if (!strcmp(buf, "M1 follow M2 REV"))
      {
        m[0]->enable(true);
        m[1]->enable(false);
        mode = 5;
      }
      else if (!strcmp(buf, "Mot-zart"))
      {
        m[0]->enable(true);
        m[1]->enable(true);
        seqDly = SEQUENCEDELAY;
        seqPtr = 0;
        mode = 6;
      }
      else if (!strcmp(buf, "Clock"))
      {
        m[0]->enable(true);
        m[1]->enable(true);
        secDiv = 0;
        mode = 7;
      }
      else if (!strcmp(buf, "Learn M2"))
      {
        m[1]->enable(false);
        learnPtr = 0;
        mode = 8;
      }
      else if (!strcmp(buf, "Play Both"))
      {
        learnPtr = 0;
        m[0]->enable(true);
        m[1]->enable(true);
        mode = 9;
      }
      else if (!strcmp(buf, "Haptic"))
      {
        m0psav = m[0]->kp;
        m[0]->kp = 5;
        m0dsav = m[0]->kd;
        m[0]->kd = 0;
        m1psav = m[1]->kp;
        m[1]->kp = 5;
        m1dsav = m[1]->kd;
        m[1]->kd = 0;

        m[0]->enable(true);
        m[1]->enable(true);
        mode = 10;
      }
      else if (!strcmp(buf, "Speed Ctrl"))
      {
        m[0]->enable(true);
        m[1]->enable(true); // toggle to reset encoder
        m[1]->enable(false);
        mode = 11;
      }
      else if (!strcmp(buf, "Quarters"))
      {
        qtrCtr = 100;
        secDiv = 0;
        m[0]->enable(true);
        m[1]->enable(true);
        mode = 12;
      }
      else if (!strcmp(buf, "Friction"))
      {
        /*
        m0psav = m[0]->kp;
        m[0]->kp = 5;
        m0dsav = m[0]->kd;
        m[0]->kd = 0;
        m1psav = m[1]->kp;
        m[1]->kp = 5;
        m1dsav = m[1]->kd;
        m[1]->kd = 0;
*/
        secDiv = 0;
        m[0]->enable(true);
        m[1]->enable(true);
        mode = 13;
      }
    }
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

  for (int i = 0; i < MOTCOUNT; i++)
  {
    char buf[10];
    sprintf(buf, "Motor %d", i + 1);
    lv_obj_t *tab = lv_tabview_add_tab(tabView, buf);

    pidchart[i] = lv_chart_create(tab);
    lv_obj_set_size(pidchart[i], 220, 100);
    lv_obj_set_pos(pidchart[i], 75, 0);
    lv_chart_set_type(pidchart[i], LV_CHART_TYPE_LINE); /*Show lines and points too*/
    lv_chart_set_range(pidchart[i], LV_CHART_AXIS_PRIMARY_Y, 0, 64);
    lv_obj_set_style_size(pidchart[i], 0, LV_PART_INDICATOR);

    ser1[i] = lv_chart_add_series(pidchart[i], lv_color_make(255, 0, 0), LV_CHART_AXIS_PRIMARY_Y);
    ser2[i] = lv_chart_add_series(pidchart[i], lv_color_make(0, 255, 0), LV_CHART_AXIS_PRIMARY_Y);
    ser3[i] = lv_chart_add_series(pidchart[i], lv_color_make(0, 0, 255), LV_CHART_AXIS_PRIMARY_Y);

    floatButton(tab, 0, 110, 35, 30, &m[i]->kp);
    floatButton(tab, 40, 110, 35, 30, &m[i]->ki);
    floatButton(tab, 80, 110, 35, 30, &m[i]->kd);
    lv_obj_add_flag(bloomButton(tab, 120, 110, 35, 30, "Rmp", ramp_event_cb), LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_flag(bloomButton(tab, 160, 110, 35, 30, "Rpm", rnd_event_cb), LV_OBJ_FLAG_CHECKABLE);

    pwrsw[i] = lv_switch_create(tab);
    lv_obj_set_pos(pwrsw[i], 200, 110);
    lv_obj_set_size(pwrsw[i], 50, 30);
    lv_obj_add_event_cb(pwrsw[i], sw_event_cb, LV_EVENT_ALL, NULL);

    slider[i] = lv_slider_create(tab);
    lv_obj_set_pos(slider[i], 40, 160);
    lv_obj_set_size(slider[i], 250, 30);
    lv_obj_add_event_cb(slider[i], slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_slider_set_range(slider[i], 0, COUNTSPERREVOLUTION + 1); // one full revolution

    slider_label[i] = lv_label_create(tab);
    lv_label_set_text(slider_label[i], "0%");
    lv_obj_align_to(slider_label[i], slider[i], LV_ALIGN_CENTER, 0, 0);

    meter[i] = lv_meter_create(tab);
    lv_obj_set_size(meter[i], 80, 80);
    lv_obj_set_pos(meter[i], -10, 10);
    lv_meter_scale_t *scale = lv_meter_add_scale(meter[i]);
    lv_meter_set_scale_ticks(meter[i], scale, 0, 0, 0, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_range(meter[i], scale, 0, COUNTSPERREVOLUTION, 360, 0);
    indic[i] = lv_meter_add_needle_line(meter[i], scale, 3, lv_palette_main(LV_PALETTE_GREY), 15);
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *cb = lv_checkbox_create(tab);
    lv_checkbox_set_text(cb, "E");
    lv_obj_set_style_text_color(cb, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_pos(cb, 255, 100);
    lv_obj_set_size(cb, 40, 30);
    lv_obj_add_event_cb(cb, err_event_cb, LV_EVENT_ALL, (void *)i);

    cb = lv_checkbox_create(tab);
    lv_checkbox_set_text(cb, "V");
    lv_obj_set_style_text_color(cb, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_pos(cb, 255, 120);
    lv_obj_set_size(cb, 40, 30);
    lv_obj_add_event_cb(cb, vel_event_cb, LV_EVENT_ALL, (void *)i);

    cb = lv_checkbox_create(tab);
    lv_checkbox_set_text(cb, "P");
    lv_obj_set_style_text_color(cb, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_set_pos(cb, 255, 135);
    lv_obj_set_size(cb, 40, 30);
    lv_obj_add_event_cb(cb, pwm_event_cb, LV_EVENT_ALL, (void *)i);

    bloomButton(tab, 0, 150, 20, 20, "^", up_event_cb);
    bloomButton(tab, 0, 170, 20, 20, "V", dn_event_cb);
  }
  lv_obj_t *tabTest = lv_tabview_add_tab(tabView, "Both");
  dropdown = lv_dropdown_create(tabTest);
  lv_dropdown_set_options(dropdown, "Random\n"
                                    "M1 follow M2\n"
                                    "M2 follow M1\n"
                                    "M1 follow M2 REV\n"
                                    "M2 follow M1 REV\n"
                                    "Mot-zart\n"
                                    "Clock\n"
                                    "Learn M2\n"
                                    "Play Both\n"
                                    "Haptic\n"
                                    "Speed Ctrl\n"
                                    "Quarters\n"
                                    "Friction");

  lv_obj_set_pos(dropdown, 10, 0);
  lv_obj_set_size(dropdown, 200, 40);
  lv_obj_add_event_cb(dropdown, dd_event_cb, LV_EVENT_ALL, NULL);

  runcb = lv_checkbox_create(tabTest);
  lv_checkbox_set_text(runcb, "Run");

  lv_obj_set_pos(runcb, 40, 60);
  lv_obj_set_size(runcb, 200, 30);
  lv_obj_add_event_cb(runcb, run_event_cb, LV_EVENT_ALL, NULL);
}
