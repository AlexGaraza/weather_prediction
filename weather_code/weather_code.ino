/*
  Електронний прогнозувальник погоди за зміною тиску 
  на основі барометричного датчику BMP 180
*/


//-----------------------НАЛАШТУВАННЯ---------------------
#define servo_invert 1       // якщо серва обертається в інший бік, змініть значення (1 на 0, 0 на 1)
#define battery_min 3000     // мінімальний рівень заряду батареї для відображення
#define battery_max 5000     // максимальний рівень заряду батареї для відображення
//-----------------------НАЛАШТУВАННЯ---------------------

#define servo_Vcc 12           // пін живлення, куди підключений мосфет

//------ІНВЕРСІЯ------
#if servo_invert == 1
#define servo_180 0
#define servo_0 180
#else
#define servo_0 0
#define servo_180 180
#endif
//------ІНВЕРСІЯ------

//------БІБЛІОТЕКИ------
#include <Servo.h>             // бібліотека серво
#include <Wire.h>              // допоміжна бібліотека датчика
#include <Adafruit_BMP085.h>   // бібліотека датчика
#include <LowPower.h>          // бібліотека сну
//------ІНВЕРСІЯ------

boolean wake_flag, move_arrow;
int sleep_count, angle, delta, last_angle = 90;
float k = 0.8;
float my_vcc_const = 1.080;    // константа вольтметра
unsigned long pressure, aver_pressure, pressure_array[6], time_array[6];
unsigned long sumX, sumY, sumX2, sumXY;
float a, b;

Servo servo;
Adafruit_BMP085 bmp; //оголосити датчик з ім'ям bmp

void setup() {
  Serial.begin(9600);
  pinMode(servo_Vcc, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A2, OUTPUT);
  digitalWrite(A3, 1);             // подати живлення на датчик
  digitalWrite(A2, 0);
  delay(500);
  bmp.begin(BMP085_ULTRAHIGHRES);  // увімкнути датчик
  servo.attach(2);                 // підключити серво
  servo.write(servo_0);            // увести серво в крайнє ліве положення
  delay(1000);
  int voltage = readVcc();         // зчитати напругу живлення

  // перетворити її в діапазон повороту вала сервомашинки
  voltage = map(voltage, battery_min, battery_max, servo_0, servo_180);
  voltage = constrain(voltage, 0, 180);
  servo.write(voltage);            // повернути серво під кут заряду
  delay(3000);
  servo.write(90);                 // поставити серво в центр
  delay(2000);
  digitalWrite(servo_Vcc, 0);      // відключити серво
  pressure = aver_sens();          // знайти поточний тиск за середнім арифметичним
  for (byte i = 0; i < 6; i++) {   // лічильник від 0 до 5
    pressure_array[i] = pressure;  // заповнити весь масив поточним тиском
    time_array[i] = i;             // заповнити масив часу числами 0 - 5
  }
}

void loop() {
  if (wake_flag) {
    delay(500);
    pressure = aver_sens();                          // знайти поточний тиск за середнім арифметичним
    for (byte i = 0; i < 5; i++) {                   // лічильник від 0 до 5 (так, до 5. Так як 4 менше 5)
      pressure_array[i] = pressure_array[i + 1];     // зсунути масив тисків КРІМ ОСТАННЬОЇ КОМІРКИ на крок назад
    }
    pressure_array[5] = pressure;                    // останній елемент масиву тепер - новий тиск
    sumX = 0;
    sumY = 0;
    sumX2 = 0;
    sumXY = 0;
    for (int i = 0; i < 6; i++) {                    // для всіх елементів масиву
      sumX += time_array[i];
      sumY += (long)pressure_array[i];
      sumX2 += time_array[i] * time_array[i];
      sumXY += (long)time_array[i] * pressure_array[i];
    }
    a = 0;
    a = (long)6 * sumXY;             // розрахунок коефіцієнта нахилу прямої
    a = a - (long)sumX * sumY;
    a = (float)a / (6 * sumX2 - sumX * sumX);
    delta = a * 6;                   // розрахунок зміни тиску
    angle = map(delta, -250, 250, servo_0, servo_180);  // перерахувати в кут повороту серви
    angle = constrain(angle, 0, 180);                   // обмежити діапазон

    // далі така штука: якщо кут не сильно змінився з попереднього разу, то немає сенсу зайвий раз включати серво
    // і витрачати енергію/жужжати. Тому знаходимо різницю, і якщо зміна суттєва - то повертаємо стрілку
    if (abs(angle - last_angle) > 7) move_arrow = 1;
    if (move_arrow) {
      last_angle = angle;
      digitalWrite(servo_Vcc, 1);      // подати живлення на серво
      delay(300);                      // затримка для стабільності
      servo.write(angle);              // повернути серво
      delay(1000);                     // дати час на поворот
      digitalWrite(servo_Vcc, 0);      // відключити серво
      move_arrow = 0;
    }

    if (readVcc() < battery_min) LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); // вічний сон, якщо акумулятор розряджений
    wake_flag = 0;
    delay(10);                       // затримка для стабільності
  }

  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);      // спати 8 секунд. режим POWER_OFF, АЦП вимкнено
  sleep_count++;            // +1 до лічильника прокидань
  
  if (sleep_count >= 70) {  // якщо час сну перевищив 10 хвилин (75 разів по 8 секунд - підгонка = 70)
    wake_flag = 1;          // дозволити виконання розрахунку
    sleep_count = 0;        // обнулити лічильник
    delay(2);               // затримка для стабільності
  }
}

// середнє арифметичне від тиску
long aver_sens() {
  pressure = 0;
  for (byte i = 0; i < 10; i++) {
    pressure += bmp.readPressure();
  }
  aver_pressure = pressure / 10;
  return aver_pressure;
}

long readVcc() { //функція зчитування внутрішнього опорного напруги, універсальна (для всіх ардуін)
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  delay(2); // Зачекати, поки Vref стабілізується
  ADCSRA |= _BV(ADSC); // Почати перетворення
  while (bit_is_set(ADCSRA, ADSC)); // вимірювання
  uint8_t low  = ADCL; // спочатку читаємо ADCL - воно блокує ADCH
  uint8_t high = ADCH; // розблокувати обидва
  long result = (high << 8) | low;

  result = my_vcc_const * 1023 * 1000 / result; // розрахунок реального VCC
  return result; // повертає VCC
}
