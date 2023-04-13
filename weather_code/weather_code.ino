

#define servo_Vcc 12

#define servo_invert 1
#define battery_min 3000
#define battery_max 4200

#define servo_Vcc 12
 
#if servo_invert == 1
#define servo_180 0
#define servo_0 180
#else
#define servo_0 0
#define servo_180 180
#endif

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <LowPower.h>

boolean wake_flag, move_arrow;
int sleep_count, angle, delta, last_angle = 90;
float k = 0.8;
float my_vcc_const = 1.080;
unsigned long pressure, aver_pressure, pressure_array[6], time_array[6];
unsigned long sumX, sumY, sumX2, sumXY;
float a, b;
 
Servo servo;
Adafruit_BMP085 bmp; 
 
void setup() {
  Serial.begin(9600);
  pinMode(servo_Vcc, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A2, OUTPUT);
  digitalWrite(servo_Vcc, 1);
  digitalWrite(A3, 1);
  digitalWrite(A2, 0);
  delay(500);
  bmp.begin(BMP085_ULTRAHIGHRES);
  servo.attach(2);
  servo.write(servo_0);
  delay(1000);
  int voltage = readVcc();
 
  voltage = map(voltage, battery_min, battery_max, servo_0, servo_180);
  voltage = constrain(voltage, 0, 180);
  servo.write(voltage);
  delay(3000);
  servo.write(90);
  delay(2000);
  digitalWrite(servo_Vcc, 0);
  pressure = aver_sens();
  for (byte i = 0; i < 6; i++) {
    pressure_array[i] = pressure;
    time_array[i] = i;
  }
}
 
void loop() {
  if (wake_flag) {
    delay(500);
    pressure = aver_sens();
    for (byte i = 0; i < 5; i++) {
      pressure_array[i] = pressure_array[i + 1];
    }
    pressure_array[5] = pressure;
 
    sumX = 0;
    sumY = 0;
    sumX2 = 0;
    sumXY = 0;
    for (int i = 0; i < 6; i++) {
      sumX += time_array[i];
      sumY += (long)pressure_array[i];
      sumX2 += time_array[i] * time_array[i];
      sumXY += (long)time_array[i] * pressure_array[i];
    }
    a = 0;
    a = (long)6 * sumXY;
    a = a - (long)sumX * sumY;
    a = (float)a / (6 * sumX2 - sumX * sumX);
    delta = a * 6;
 
    angle = map(delta, -250, 250, servo_0, servo_180);
    angle = constrain(angle, 0, 180);
 
    
    if (abs(angle - last_angle) > 7) move_arrow = 1;
 
    if (move_arrow) {
      last_angle = angle;
      digitalWrite(servo_Vcc, 1);
      delay(300);
      servo.write(angle);
      delay(1000);
      digitalWrite(servo_Vcc, 0);
      move_arrow = 0;
    }
 
    if (readVcc() < battery_min) LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
    wake_flag = 0;
    delay(10);
  }
 
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  sleep_count++;
  if (sleep_count >= 70) {
    wake_flag = 1;
    sleep_count = 0;
    delay(2);
  }
}
 
long aver_sens() {
  pressure = 0;
  for (byte i = 0; i < 10; i++) {
    pressure += bmp.readPressure();
  }
  aver_pressure = pressure / 10;
  return aver_pressure;
}
 
long readVcc() { 
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  uint8_t low  = ADCL;
  uint8_t high = ADCH;
  long result = (high << 8) | low;
 
  result = my_vcc_const * 1023 * 1000 / result; 
  return result; 
}
