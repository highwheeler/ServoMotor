#ifndef UltraServo_h
#define UltraServo_h
#include <Arduino.h>
#include "soc/ledc_reg.h"
#include "soc/ledc_struct.h"

#define LEDC_CHAN(g, c) LEDC.channel_group[(g)].channel[(c)]
#define LEDC_TIMER(g, t) LEDC.timer_group[(g)].timer[(t)]
#define MAXERR 255
#define SUMMAX 255
#define RAMPMAX 20
#define RAMPSTEP 1
#define RAMPSKIP 2
#define RAMPPAUSE 100
#define STALLMAX 1000
#define STALLVAL 220
#define COUNTSPERREVOLUTION 445
#define VELMIN 2
#define OUTPULSE 32

/* this supports up to 4 servos*/

class UltraServo
{
public:
	/**
	 * @param enc1pin : quadrature encoder pin 1
	 * @param enc2pin : quadrature encoder pin 2
	 * @param pwmPin : PWM output pin
	 * @param dir1pin : H bridge direction pin 1
	 * @param dir2pin : H bridge direction pin 2
	 * @param sampleRate : timer sample rate in HZ
	 * @param bias : minimum PWM value to get motor moving
	 */
	UltraServo(int enc1pin, int enc2pin, int pwmPin, int dir1pin, int dir2pin, int sampleRate, int bias);
	void enable(bool flg);
	void stop();
	void startRandom(int);
	void setRpm(int);
	bool getRandomRun();
	bool getRpmRun();
	void startRamp(int);
	bool getRampRun();
	int getRampRpm();
	int getError();
	int getVelocity();
	int getTargetPos();
	void setTargetPos(int);
	int getEncPos();
	bool getStallFlg();
	void setStallFlg(bool);
	double kp = 15;
	double ki = 0;
	double kd = 350;

private:
	static void myledcWrite(uint8_t chan, uint32_t duty);
	static void __digitalWrite(uint8_t pin, uint8_t val);
	static void *timerIsr[4];
	static void *encIsr[4];
	static UltraServo *instance[4];
	static int numInst;
	hw_timer_t *timer;
	int enc1pin, enc2pin, pwmPin, dir1pin, dir2pin, sampleRate, bias;
	static void encoderISR(UltraServo *inst);
	static void encoderISR1();
	static void encoderISR2();
	static void encoderISR3();
	static void encoderISR4();
	static void timerISR(UltraServo *inst);
	static void timerISR1();
	static void timerISR2();
	static void timerISR3();
	static void timerISR4();
	int randLen;
	const int freq = 5000;
	short int rpmCnt = 0;
	short int encLst = 0, encCtr = 0, encPrev = 0, cntFlt = 0, targetPos = 0, tmpRpm = 0, error = 0, sumError = 0;
	const int resolution = 8;
	int instNum;
	bool runFlg = false;
	bool randRun = false;
	bool rpmRun = false;
	int rpmVal = 0;
	int randDly = 0;
	int randPos = 0;
	int rpmOffset = 0;
	int stallCnt = 0; // for detecting motor stall
	bool stallFlg = false;
	int rampCtr = 0;
	int rampPos = 0;
	int rampPause = 0;
	int rampPeakCnt = -1;
	int rampHalfLen = -1;
	int rampLen = 0;
	int rampDir = 1;
	bool rampRun = false;
	volatile int rampSkip = 0;
	volatile int velocity;
	int pwm = 0;
	unsigned int enc1mask, enc2mask;
	int enc1port, enc2port;
};

#endif /* UltraServo_h */
