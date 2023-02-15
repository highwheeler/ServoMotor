#include "UltraServo.h"

// UltraServo: Servo driver module for up to 4 brushed motor servos with H bridge driver
UltraServo::UltraServo(int _enc1pin, int _enc2pin, int _pwmPin, int _dir1pin,
                       int _dir2pin, int _sampleRate, int _bias)
{
  instance[numInst] = this;
  enc1pin = _enc1pin;
  enc2pin = _enc2pin;
  pwmPin = _pwmPin;
  dir1pin = _dir1pin;
  dir2pin = _dir2pin;
  sampleRate = _sampleRate;
  bias = _bias;
  pinMode(_enc1pin, INPUT);
  pinMode(_enc2pin, INPUT);
  pinMode(_dir1pin, OUTPUT);
  pinMode(_dir2pin, OUTPUT);
  digitalWrite(_dir1pin, false);
  digitalWrite(_dir2pin, true);
  timer = timerBegin(numInst, 80, true); // Begin timer with 1 MHz frequency (80MHz/80)
  timerAttachInterrupt(timer, (void (*)())timerIsr[numInst], true);
  unsigned int timerFactor = 1000000 / _sampleRate;
  timerAlarmWrite(timer, timerFactor, true);
  timerAlarmEnable(timer);
  if (_enc1pin < 32)
  {
    enc1mask = 1 << _enc1pin;
    enc1port = GPIO_IN_REG;
  }
  else
  {
    enc1mask = 1 << (_enc1pin - 32);
    enc1port = GPIO_IN1_REG;
  }
  if (_enc2pin < 32)
  {
    enc2mask = 1 << _enc2pin;
    enc2port = GPIO_IN_REG;
  }
  else
  {
    enc2mask = 1 << (_enc2pin - 32);
    enc2port = GPIO_IN1_REG;
  }
  attachInterrupt(_enc1pin, (void (*)())encIsr[numInst], CHANGE);
  attachInterrupt(_enc2pin, (void (*)())encIsr[numInst], CHANGE);
  ledcSetup(numInst, freq, resolution);
  ledcAttachPin(pwmPin, numInst);
  instNum = numInst++;
}

// define all the class statics here
UltraServo *UltraServo::instance[4];
int UltraServo::numInst = 0;
// arrays of pointers to functions
// this allows for the ISRs to be called with the proper instance
// maybe there's a better way to do this but this is simple and works.
void *UltraServo::timerIsr[4] = {(void *)&timerISR1, (void *)&timerISR2, (void *)&timerISR3, (void *)&timerISR4};
void *UltraServo::encIsr[4] = {(void *)&encoderISR1, (void *)&encoderISR2, (void *)&encoderISR3, (void *)&encoderISR4};

// direct port manipulation, ISR safe
void IRAM_ATTR UltraServo::myledcWrite(uint8_t chan, uint32_t duty)
{
  uint8_t group = (chan / 8), channel = (chan % 8);
  LEDC_CHAN(group, channel).duty.duty = duty << 4; // 25 bit (21.4)
  if (duty)
  {
    LEDC_CHAN(group, channel).conf0.sig_out_en = 1; // This is the output enable control bit for channel
    LEDC_CHAN(group, channel).conf1.duty_start = 1; // When duty_num duty_cycle and duty_scale has been configured. these register won't take effect until set duty_start. this bit is automatically cleared by  hardware.
    if (group)
    {
      LEDC_CHAN(group, channel).conf0.low_speed_update = 1;
    }
    else
    {
      LEDC_CHAN(group, channel).conf0.clk_en = 1;
    }
  }
  else
  {
    LEDC_CHAN(group, channel).conf0.sig_out_en = 0; // This is the output enable control bit for channel
    LEDC_CHAN(group, channel).conf1.duty_start = 0; // When duty_num duty_cycle and duty_scale has been configured. these register won't take effect until set duty_start. this bit is automatically cleared by hardware.
    if (group)
    {
      LEDC_CHAN(group, channel).conf0.low_speed_update = 1;
    }
    else
    {
      LEDC_CHAN(group, channel).conf0.clk_en = 0;
    }
  }
}

// direct port manipulation, ISR safe
void IRAM_ATTR UltraServo::__digitalWrite(uint8_t pin, uint8_t val)
{
  if (val)
  {
    if (pin < 32)
    {
      GPIO.out_w1ts = ((uint32_t)1 << pin);
    }
    else if (pin < 34)
    {
      GPIO.out1_w1ts.val = ((uint32_t)1 << (pin - 32));
    }
  }
  else
  {
    if (pin < 32)
    {
      GPIO.out_w1tc = ((uint32_t)1 << pin);
    }
    else if (pin < 34)
    {
      GPIO.out1_w1tc.val = ((uint32_t)1 << (pin - 32));
    }
  }
}

void IRAM_ATTR UltraServo::timerISR(UltraServo *inst)
{
  //  portENTER_CRITICAL(&myMutex);
  // digitalWrite(OUTPULSE, LOW);

  inst->tmpRpm = inst->m1Ctr;
  inst->error = inst->targetPos - inst->m1Ctr;
  inst->velocity = inst->m1Ctr - inst->m1Prev;
  inst->m1Prev = inst->m1Ctr;

  if (inst->runFlg)
  {
    if(inst->rpmRun) {
      inst->rpmCnt++;
      inst->targetPos = (COUNTSPERREVOLITION / 60 * inst->rpmVal* inst->rpmCnt / inst->sampleRate) + inst->rpmOffset;
    }
    if(inst->randRun) {
      inst->randDly --;
      if (inst->randDly <= 0) {
        inst->randDly = RAMPPAUSE * 1;
        inst->targetPos = random(inst->randLen);
      }
    }
    if (inst->rampRun) // ramp
    {
      if (inst->rampSkip > 0)
      {
        inst->rampSkip--;
      }
      else
      {
        inst->rampSkip = RAMPSKIP;
        if (inst->rampPause > 0)
        {
          inst->rampPause--;
        }
        else
        {
          inst->rampCtr++;
          if (inst->rampCtr < inst->rampHalfLen)
          {
            if (inst->rampPos <= RAMPMAX)
            {
              inst->rampPos += RAMPSTEP;
            }
            else
            {
              inst->rampPeakCnt++;
            }
          }
          else
          {
            if (inst->rampPeakCnt > 0)
            {
              inst->rampPeakCnt--;
            }
            else
            {
              inst->rampPos -= RAMPSTEP;
            }
          }
          if (inst->rampCtr == inst->rampLen)
          {
            // start over but reverse direction
            inst->rampCtr = 0;
            inst->rampPos = 0;
            inst->rampPeakCnt = 0;
            inst->rampHalfLen = inst->rampLen / 2;
            inst->rampDir = -inst->rampDir;
            inst->rampPause = RAMPPAUSE;
          }
        }
        inst->targetPos += inst->rampPos * inst->rampDir;
      }
    }
    //    int sumErrorHist = error;
    //    for (int i = NUMHIST - 1; i > 0; i--)
    //    {
    //      sumErrorHist += errHist[i];
    //      errHist[i] = errHist[i - 1];
    //    }
    //   errHist[0] = error;

    //      if (error > MAXERR)
    //        error = MAXERR;
    //      else if (error < -MAXERR)
    //        error = -MAXERR;

    inst->sumError += inst->error;
    if (inst->sumError > SUMMAX)
    {
      inst->sumError = SUMMAX;
    }
    if (inst->sumError < -SUMMAX)
    {
      inst->sumError = -SUMMAX;
    }
    // PID (or really PV) calculaion
    inst->pwm = inst->error * inst->kp - inst->sumError * inst->ki - inst->velocity * inst->kd;

    if (inst->pwm != 0)
    {
      if (inst->pwm > 0)
      {
        inst->pwm += inst->bias;
      }
      else
      {
        inst->pwm -= inst->bias;
      }
    }
    if (inst->pwm > MAXERR)
    {
      inst->pwm = MAXERR;
    }
    else if (inst->pwm < -MAXERR)
    {
      inst->pwm = -MAXERR;
    }
    if (abs(inst->pwm) > STALLVAL && abs(inst->velocity) < VELMIN) {
      inst->stallCnt++;
      if (inst->stallCnt> STALLMAX) {
    //    inst->stallFlg = true;
     //   inst->enable(false);
      }
    }
    else
    {
      inst->stallCnt=0;
    }
  }
  else
  {

    inst->sumError = 0;
    inst->pwm = 0;
    //   for (int i = 0; i < NUMHIST; i++)
    //   {
    //     errHist[i] = 0;
    //   }
  }

  if (inst->pwm < 0)
  {
    __digitalWrite(inst->dir2pin, false);
    __digitalWrite(inst->dir1pin, true);
    myledcWrite(inst->instNum, -inst->pwm);
  }
  else
  {
    __digitalWrite(inst->dir1pin, false);
    __digitalWrite(inst->dir2pin, true);
    myledcWrite(inst->instNum, inst->pwm);
  }
  //  digitalWrite(OUTPULSE, HIGH);
  //  portEXIT_CRITICAL_ISR(&myMutex);
}

void IRAM_ATTR UltraServo::timerISR1()
{
  timerISR(instance[0]);
}
void IRAM_ATTR UltraServo::timerISR2()
{
  timerISR(instance[1]);
}
void IRAM_ATTR UltraServo::timerISR3()
{
  timerISR(instance[2]);
}
void IRAM_ATTR UltraServo::timerISR4()
{
  timerISR(instance[3]);
}

// this handles the quadrature encoders.
void IRAM_ATTR UltraServo::encoderISR(UltraServo *inst)
{
  // portENTER_CRITICAL(&myMutex);
  // digitalWrite(OUTPULSE, LOW);
  // read the direct gpio ports masked with the calculated bitmask

  int m1Now =
      (((*(volatile uint32_t *)inst->enc1port) & inst->enc1mask) ? 1 : 0) |
      (((*(volatile uint32_t *)inst->enc2port) & inst->enc2mask) ? 2 : 0);

  // if / then implementation of a "state machine"
  if (inst->m1Lst == 0)
  {
    if (m1Now == 2)
      inst->m1Ctr++;
    else if (m1Now == 1)
      inst->m1Ctr--;
    else
      inst->cntFlt++;
  }
  else if (inst->m1Lst == 2)
  {
    if (m1Now == 3)
      inst->m1Ctr++;
    else if (m1Now == 0)
      inst->m1Ctr--;
    else
      inst->cntFlt++;
  }
  else if (inst->m1Lst == 3)
  {
    if (m1Now == 1)
      inst->m1Ctr++;
    else if (m1Now == 2)
      inst->m1Ctr--;
    else
      inst->cntFlt++;
  }
  else if (inst->m1Lst == 1)
  {
    if (m1Now == 0)
      inst->m1Ctr++;
    else if (m1Now == 3)
      inst->m1Ctr--;
    else
      inst->cntFlt++;
  }
  inst->m1Lst = m1Now;
}

void IRAM_ATTR UltraServo::encoderISR1()
{
  encoderISR(instance[0]);
}
void IRAM_ATTR UltraServo::encoderISR2()
{
  encoderISR(instance[1]);
}
void IRAM_ATTR UltraServo::encoderISR3()
{
  encoderISR(instance[2]);
}
void IRAM_ATTR UltraServo::encoderISR4()
{
  encoderISR(instance[3]);
}
void UltraServo::startRandom(int l) 
{
  stop();
  randLen = l;
  randDly = 0;
  randRun = true;
}
void UltraServo::setRpm(int l) 
{


 // rpmCnt = (COUNTSPERREVOLITION / 60 * rpmVal / sampleRate)
//  - (COUNTSPERREVOLITION / 60 * l  / sampleRate);
if(l > 0 && rpmVal > 0) {
rpmCnt = (targetPos * sampleRate) / (COUNTSPERREVOLITION * 60 * rpmVal)
- (targetPos * sampleRate) / (COUNTSPERREVOLITION * 60 * l);
}
 // m1Ctr = 0;
 // m1Lst = 0;
 // m1Prev = 0;
 // rpmCnt = 0;
  rpmOffset = targetPos;
  rpmVal = l;
  rpmRun = true;
}
void UltraServo::startRamp(int len)
{
  stop();
  rampCtr = 0;
  rampPos = 0;
  rampPeakCnt = 0;
  rampLen = len;
  rampHalfLen = rampLen / 2;
  rampRun = true;
  rampDir = 1;
}
bool UltraServo::getRampRun()
{
  return rampRun;
}
bool UltraServo::getRpmRun()
{
  return rpmRun;
}
bool UltraServo::getRandomRun()
{
  return randRun;
}
int UltraServo::getRampRpm()
{
  return rampPos;
}
int UltraServo::getError()
{
  return error;
}
int UltraServo::getVelocity()
{
  return velocity;
}
int UltraServo::getTargetPos()
{
  return targetPos;
}
void UltraServo::setTargetPos(int pos)
{
  targetPos = pos;
}
void UltraServo::stop() {
    rampRun = false;
    randRun = false;
    rpmRun = false;
}
void UltraServo::enable(bool flg)
{
  runFlg = flg; 
  m1Ctr = 0;
  m1Prev = 0;
  if (!flg)
  {
    targetPos = 0;
    rampRun = false;
    randRun = false;
    m1Lst = 0;
    m1Now = 0;
    rpmRun = false;
    rpmCnt = 0;
  }
}
int UltraServo::getEncPos()
{
  return m1Ctr;
}

bool UltraServo::getStallFlg() {
  return stallFlg;
}

void UltraServo::setStallFlg(bool val) {
 stallFlg = val;
}