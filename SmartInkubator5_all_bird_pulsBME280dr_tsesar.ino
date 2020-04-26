#include <PID_v1.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM2.h>
#include "RTClib.h"
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Sensirion.h>
#include <avr/wdt.h>
#include <Encoder.h>
#include "SparkFunBME280.h"
#include "SPI.h"

int del = 80;                     // переменная ожидания между выборами меню
unsigned int interval = 300;      // интервал сколько будет длиться цикл while, после чего перейдёт к следующему меню.(кол-во итераций)
//#define EXT_HEATING                  // ИСПОЛЬЗУЕМ ДОП.НАГРЕВАТЕЛЬ ВМЕСТО УВЛАЖНИТЕЛЯ. Если нужен увлажнитель, просто закомментируйте эту строку.
#define heater_pin 13                  // нагреватель
#define humidifer_pin 12               // увлажнитель
#define fan_pin 11                     // вентилятор
#define alarm_pin 14                   // пин аварии
#define beeper_pin 9                   //пищалка по аварии
#define dataPin 5                      //SHT10
#define clockPin 6                     //SHT10 
#define turn_pin 10                    // управление поворотом
#define extend_heater_pin 8          // дополнительный нагреватель
//#define button_minus_pin 2            //пин кнопки "минус"
//#define button_plus_pin 3             //пин кнопки "плюс"
#define button_enter_pin 4            //пин кнопки "enter"
#define DS18B20_Pin 7                 //пин термометра
#define setSampleTime 300            //время цикла ПИД
#define voltmeter_pin 1               //вход А1 через делитель (22к/10к) подключен к питанию модуля. Измеряет до 16В.
#define T_correction -0.1             // коррекция температуры SHT10
#define T2_correction -0.8             // коррекция температуры bme280
#define h_histeresis 1.0              // гистерезис влажности
#define door_pin 16                 // пин открытой двери (А2)
//#define turnLenght 10                 // время поворота в импульсном режиме (сек.)

//boolean button_minus;                // статус нажатия кнопок
//boolean button_plus;
//boolean button_enter;
boolean turnFlag = 0;              // флаг поворота для случайного периода
boolean heater_off;               // флаг запрета нагреателя
boolean needFan;					   // флаг аварийной вентиляции
float humidity;                    // Влажность
float temp1Ink;                    // Температура DS18B20
float temp2Ink;                    // Температура SHT10
float needTemp = 37.6;             // нужная для текущего дня температура инкубации (по умолчанию)
float needHum = 60.5;              // ---- влажность
//float dewpoint;                    // Точка росы

unsigned char button_minus;               // статус нажатия кнопок
unsigned char button_plus;
unsigned char button_enter;
long oldPosition  = 500;  			// позиция энкодера
unsigned int rawData;
unsigned long currentTime;            // задаем переменные для тайминга поворота
unsigned long loopTime;
unsigned long serialTime; //this will help us know when to talk with processing
unsigned long now;
unsigned long trhMillis = 0;             // период опроса датчиков
byte measActive = false;
byte measType = TEMP;
const unsigned long TRHSTEP   = 300UL;  // Период опроса датчиков
unsigned int currentDay;                // текущий день в юникс-формате
String birdPrint = "CHICK";

BME280 mySensor;
LiquidCrystal_I2C lcd(0x27, 20, 4);  // инициализация библиотеки дисплея
//SHT1x sht1x(dataPin, clockPin);
OneWire oneWire(DS18B20_Pin);
DallasTemperature sensors(&oneWire);
Sensirion sht = Sensirion(dataPin, clockPin);

double Setpoint, Input, Output;            //объявляем переменные для ПИД
PID myPID(&Input, &Output, &Setpoint, 50, 0.5, 0.1, DIRECT); //Инициализируем ПИД-библиотеку и коэффициенты
int WindowSize = 1000;                  // ширина окна терморегулятора 1 секунда.
unsigned long windowStartTime;
unsigned long alarmDelay;
RTC_DS1307 RTC;
Encoder myEnc(3, 2);

//SimpleTimer timer;

/*  EEPROM1 -     tempInk (float)
    EEPROM(13) -  set_humidity (float)
    EEPROM5 -     +-alarmTemp  (float)
    EEPROM9 -     alarm_fan    (bool)
    EEPROM11 -    turnPeriod   (int)
    EEPROM17 -     bird         (int)
   */


////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);

  RTC.begin();         // Старт часов
  lcd.init();          // Старт дисплея
  Wire.begin();        // старт I2C
  lcd.backlight();     // Включаем подсветку дисплея
  windowStartTime = millis();
  //  byte stat;
  //  byte error = 0;
  //  float tempInk;
  //  float set_humidity;
  //  float alarmTemp;

  mySensor.settings.commInterface = I2C_MODE;
  mySensor.settings.I2CAddress = 0x76;
  mySensor.settings.runMode = 3; //Normal mode
  mySensor.settings.tStandby = 0;
  mySensor.settings.filter = 0;
  mySensor.settings.tempOverSample = 1;
  mySensor.settings.pressOverSample = 1;
  mySensor.settings.humidOverSample = 1;
  mySensor.begin();
  delay(15);
  wdt_enable (WDTO_8S); //взводим сторожевой таймер на 8 секунд.
  myPID.SetOutputLimits(0, WindowSize); //задаем лимиты ширины ПИД-импульса от 0 до 1 секунды.
  myPID.SetMode(AUTOMATIC);             //включаем ПИД-регулирование
  myPID.SetSampleTime(setSampleTime);

  pinMode(extend_heater_pin, OUTPUT);		// пин дополнительного нагревателя
  digitalWrite(extend_heater_pin, LOW);		// переводим в 0 чтобы не включать омрон
  pinMode(heater_pin, OUTPUT);				// нагрватель
  pinMode(turn_pin, OUTPUT);				// устанавливаем выводы поворота
  digitalWrite(turn_pin, LOW);				// 0 выкл, 1 вкл (омрон)
  pinMode(humidifer_pin, OUTPUT);			// увлажнитель на вывод
  pinMode(fan_pin, OUTPUT);					// вентилятор на вывод
  digitalWrite(fan_pin, HIGH);				// 1 выкл, 0 вкл (реле)
  pinMode(alarm_pin, OUTPUT);				// авария на выход
  digitalWrite(alarm_pin, HIGH);			// 1 выкл, 0 вкл (реле)
  //pinMode(button_minus_pin, INPUT_PULLUP); //подтягиваем входы кнопок к плюсу встроенными резисторами
  //pinMode(button_plus_pin, INPUT_PULLUP);
  pinMode(button_enter_pin, INPUT_PULLUP);
  pinMode(door_pin, INPUT_PULLUP);
  alarmDelay = millis();
  sensors.begin();
  sensors.setResolution(12);    // установить разрешение (точность)
  sensors.setWaitForConversion(false);  // отключить ожидание
  birdSelect();	// читаем птицу
  //windowStartTime = millis();
  //RTC.adjust(DateTime(__DATE__, __TIME__));   //раскоментируйте для установки системмных даты и времени
  //EEPROM_write(5, 2.5); //    alarm temp
  //EEPROM_write(21,10);  //  turn lenght setup
}

//////////////////////////////////////////////////////////////////////////
void loop() {
  // Input = getTemp();
  unsigned int startDayUnixtime;          // хранящийся в памяти день старта программы в юникс-формате
  //float needTemp = 37.6;                  // нужная для текущего дня температура инкубации (по умолчанию)
  //float needHum = 60.5;                   // ---- влажность
  int ventTime = 2;                       // длительность проветривания
  boolean needTurn = false;               // нужен ли поворот яиц?
  unsigned int currentTime_day;           //текущий день в юникс-формате (сколько дней минуло с 1 января 1970)
  int bird;                         // выбор птицы
  int r_array[7][4][5] = {
    {{2, 382, 650, 0, 1}, {12, 377, 540, 5, 1}, {18, 374, 480, 20, 1}, {20, 371, 855, 10, 0}},  // chick
    {{2, 382, 600, 0, 1}, {12, 376, 600, 3, 1}, {15, 374, 480, 10, 1}, {18, 370, 855, 10, 0}},  // quail
    {{7, 381, 700, 0, 1}, {13, 376, 600, 3, 1}, {25, 373, 560, 20, 1}, {30, 370, 855, 10, 0}},  // duck
    {{15, 380, 610, 5, 1}, {26, 375, 520, 20, 1}, {27, 374, 690, 10, 0}, {35, 370, 800, 0, 0}}, // muskus
    {{3, 379, 630, 0, 0}, {13, 378, 540, 3, 1}, {26, 375, 560, 20, 1}, {30, 372, 590, 10, 0}},  // goose
    {{6, 379, 560, 0, 1}, {12, 376, 530, 3, 1}, {26, 373, 520, 20, 1}, {30, 370, 855, 10, 0}},  // turkey
    {{3, 376, 650, 5, 1}, {15, 375, 550, 9, 1}, {24, 375, 500, 10, 1}, {26, 374, 700, 5, 0}},  // tsesar
  };

  DateTime now = RTC.now();
  currentTime_day = (now.unixtime() / 86400L);
  //timer.run();
  button_read();
  if (button_enter) {
    delay(del);
    lcd.clear();
    menu();
    lcd.clear();
  }
  if (button_minus) {
    delay(del);
    lcd.clear();
    alarmDelay = millis();    // задержка аварии по нажатии кнопки Минус
  }
  if (button_plus) {
    delay(del);
    lcd.clear();
    digitalWrite(turn_pin, !digitalRead(turn_pin));       // включаем/выключаем реле поворота по кнопке Плюс
  }
  if (button_plus && button_plus) {
    delay(del);
    lcd.clear();
    // bottomView = !bottomView;       // переключаем режим показа нижней строки.
  }
  //send-receive with processing if it's time
  if (millis() > serialTime * 5)
  {
    SerialReceive();
    SerialSend();
    serialTime += 500;
  }
  EEPROM_read(1, startDayUnixtime);
  currentDay = (currentTime_day - startDayUnixtime);

  EEPROM_read(17, bird);
  for (int d = 3; d >= 0; d--) {
    if (currentDay < r_array[bird][d][0]) {
      needTemp = float(r_array[bird][d][1] / 10.0);
      needHum = float(r_array[bird][d][2] / 10.0);
      ventTime = r_array[bird][d][3];
      needTurn = r_array[bird][d][4];
    }
  }
  getSensors();
  thermostat(needTemp, needHum);    // влажность передаем только для вывода на дисплей )))
  humidifer(needHum);
  turn(needTurn);
  //turnPulse();
  fan(ventTime);
  alarm(needTemp);
  ext_heater(needTemp);
  //outpuPower();
  wdt_reset();
}


/// чтиаем энкодер ////////////////////////////////////////////////////////////////////////
void button_read() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    if (newPosition >= oldPosition + 2) {
      button_plus = 1;
    }
    if (newPosition <= oldPosition - 2) {
      button_minus = 1;
    }
    oldPosition = newPosition;
    //Serial.println(newPosition);
  }
  else {
    button_plus = 0;
    button_minus = 0;
  }

  if (digitalRead(button_enter_pin) == LOW )  {
    delay(del / 2);
    button_enter = 1;
  }
  if (digitalRead(button_enter_pin) == HIGH )  {
    delay(del / 2);
    button_enter = 0;
  }
  if (button_minus || button_plus || button_enter) beeper(50);
  wdt_reset();
}


////меню////////////////////////////////////////////////////////////////
void menu() {
  bird_setup();
  turn_setup();
  turn_pulse_setup();
  alarm_setup();
  vent_setup();
  startInk();
  //data_time_setup();
}


// записываем в память день начала инкубации в юникс-формате////////////
void startInk() {
  unsigned int currentTime_day;
  unsigned int memoryDay;
  DateTime now = RTC.now();
  currentTime_day = (now.unixtime() / 86400L);

  delay(del);
  button_read();
  lcd.setCursor(4, 0);
  lcd.print("START INK");
  delay(1000);
  lcd.clear();
  int x = 0;
  while (1) {
    x++;
    if (x > interval) break;
    button_read();
    EEPROM_read(1, memoryDay);
    //Serial.print(currentTime_day);
    //Serial.print("-");
    //Serial.print(memoryDay);
    //Serial.print("=");
    //Serial.println(currentTime_day-memoryDay);
    if (button_enter) {
      delay(del);
      lcd.clear(); //очищаем экран
      break;
    }
    if (button_minus) {
      EEPROM_write(1, currentTime_day);
      lcd.clear();
    }
    if (button_plus) {
      EEPROM_write(1, memoryDay - 1);
      lcd.clear();
    }
    EEPROM_read(1, memoryDay);
    lcd.setCursor(5, 0);
    lcd.print("DAY = ");
    lcd.print(currentTime_day - memoryDay);
    lcd.setCursor(2, 1);
    lcd.print("ZERO START +1");
    delay(del);
  }
}


//выбираем птицу /////////////////////////////////////////////////////////
void bird_setup() {
  int birdType;
  lcd.clear();
  delay(del);
  button_read();
  lcd.setCursor(4, 0);
  lcd.print("BIRD SETUP");
  delay(1000);
  lcd.clear();
  int x = 0;
  while (1) {
    x++;
    if (x > interval) {
      break;
    }
    button_read();
    EEPROM_read(17, birdType);
    if (button_enter) {
      delay(del);
      lcd.clear(); //очищаем экран
      break;
    }
    if (button_plus) {
      x = 0;
      if (birdType >= 6)      //проверяем, если выше или равно 6,
        EEPROM_write(17, 0);  //пишем в память 0
      else EEPROM_write(17, birdType + 1);
      lcd.clear();
    }
    if (button_minus) {
      x = 0;
      if (birdType <= 0)      //проверяем, если ниже или равно 0,
        EEPROM_write(17, 6);  //пишем в память 6
      else EEPROM_write(17, birdType - 1);
      lcd.clear();
    }
    birdSelect();
    lcd.setCursor(7, 0);
    lcd.print(birdPrint);
    lcd.setCursor(1, 1);
    lcd.print("minus NEXT plus");
    delay(del);
  }
}

//выбираем из памяти птицу///////////////////////////////////////
void birdSelect() {
  int birdType;
  EEPROM_read(17, birdType);
  lcd.setCursor(2, 0);
  lcd.print("BIRD - ");
  switch (birdType) {
    case 0:
      //lcd.print("CHICK");  // курица
      birdPrint = "CHICK";
      break;
    case 1:
      //lcd.print("QUAIL");  // перепелка
      birdPrint = "QUAIL";
      break;
    case 2:
      //lcd.print("DUCK");   // утка
      birdPrint = "DUCK";
      break;
    case 3:
      //lcd.print("MUSKUS");  // индоутка
      birdPrint = "MUSKUS";
      break;
    case 4:
      //lcd.print("GOOSE");  // гусь
      birdPrint = "GOOSE";
      break;
    case 5:
      //lcd.print("TURKEY"); // индюк
      birdPrint = "TURKEY";
      break;
    case 6:
      //lcd.print("TSESAR"); // цесарка
      birdPrint = "TSESAR";
      break;
  }
}

//устанавливаем поворот/////////////////////////////////////////////////
void turn_setup() {
  int turnPeriod;

  delay(del);
  button_read();
  lcd.setCursor(4, 0);
  lcd.print("TURN SETUP");
  delay(1000);
  lcd.clear();
  int x = 0;
  while (1) {
    x++;
    if (x > interval) {
      break;
    }
    button_read();
    EEPROM_read(11, turnPeriod);
    if (button_enter) {
      delay(del);
      lcd.clear(); //очищаем экран
      break;
    }
    if (button_plus) {
      x = 0;
      EEPROM_write(11, turnPeriod + 1);
      if (turnPeriod >= 13) {     //проверяем, если выше 13,
        EEPROM_write(11, 13);  //пишем в память 13
      }
      lcd.clear();
    }
    if (button_minus) {
      x = 0;
      EEPROM_write(11, turnPeriod - 1);
      if (turnPeriod <= 0) {     //проверяем, если ниже 0,
        EEPROM_write(11, 0);  //пишем в память 0
      }
      lcd.clear();
    }
    EEPROM_read(11, turnPeriod);
    lcd.setCursor(0, 0);
    lcd.print("PERIOD = ");
    if (turnPeriod < 13)lcd.print(turnPeriod);
    if (turnPeriod > 12) lcd.print("RND");
    lcd.print(" Hour");
    lcd.setCursor(1, 1);
    lcd.print("minus NEXT plus");
    delay(del);
  }
}

//устанавливаем длительность сигнала поворота/////////////////////////////////////////////////
void turn_pulse_setup() {
  int turnLenght;

  delay(del);
  button_read();
  lcd.setCursor(1, 0);
  lcd.print("TURN LENGHT SETUP");
  delay(1000);
  lcd.clear();
  int x = 0;
  while (1) {
    x++;
    if (x > interval) {
      break;
    }
    button_read();
    EEPROM_read(21, turnLenght);
    if (button_enter) {
      delay(del);
      lcd.clear(); //очищаем экран
      break;
    }
    if (button_plus) {
      x = 0;
      EEPROM_write(21, turnLenght + 1);
      if (turnLenght >= 60) {     //проверяем, если выше 60,
        EEPROM_write(21, 60);  //пишем в память 60
      }
      lcd.clear();
    }
    if (button_minus) {
      x = 0;
      EEPROM_write(21, turnLenght - 1);
      if (turnLenght <= 1) {     //проверяем, если ниже 1,
        EEPROM_write(21, 1);  //пишем в память 0
      }
      lcd.clear();
    }
    EEPROM_read(21, turnLenght);
    lcd.setCursor(0, 0);
    lcd.print("TURN LEN = ");
    lcd.print(turnLenght);
    lcd.print(" Sec");
    lcd.setCursor(1, 1);
    lcd.print("minus NEXT plus");
    delay(del);
  }
}


//устанавливаем сигнализацию///////////////////////////////////////////////////////////////
void alarm_setup() {
  float alarmTemp;
  delay(del);
  button_read();
  lcd.setCursor(4, 0);
  lcd.print("ALARM SETUP");
  delay(1000);
  lcd.clear();
  int x = 0;
  while (1) {
    x++;
    if (x > interval) break;
    button_read();
    EEPROM_read(5, alarmTemp);
    if (button_enter) {
      delay(del);
      lcd.clear(); //очищаем экран
      break;
    }
    if (button_plus) {
      x = 0;
      EEPROM_write(5, alarmTemp + 0.1);
      if (alarmTemp >= 10.0)      //проверяем, если больше или равно 10,
        EEPROM_write(5, 10.0);  //пишем в память 10
    }
    if (button_minus) {
      x = 0;
      EEPROM_write(5, alarmTemp - 0.1);
      if (alarmTemp <= 1.0)     //проверяем, если ниже 1,
        EEPROM_write(5, 1.0);  //пишем в память 1
    }
    lcd.setCursor(1, 0);
    lcd.print("T.Alarm +-");
    lcd.print(alarmTemp, 1);
    lcd.print((char)223);
    lcd.print("C  ");
    lcd.setCursor(1, 1);
    lcd.print("minus NEXT plus");
    delay(del);
  }
}


//устанавливаем вентиляцию///////////////////////////////////////////////////////////
void vent_setup() {
  boolean fanEnable;
  delay(del);
  button_read();
  lcd.setCursor(3, 0);
  lcd.print("A.FAN SETUP");
  delay(1000);
  lcd.clear();
  int x = 0;
  while (1) {
    x++;
    if (x > interval) break;
    button_read();
    EEPROM_read(9, fanEnable);
    if (fanEnable > 1) fanEnable = 1;
    if (button_enter) {
      delay(del);
      lcd.clear(); //очищаем экран
      break;
    }
    if (button_minus) {
      x = 0;
      EEPROM_write(9, 0);  //пишем в память 0, не включаем принудительную вентиляцию при превышении температуры
    }
    if (button_plus) {
      x = 0;
      EEPROM_write(9, 1);  //пишем в память 1, включаем принудительную вентиляцию при превышении температуры
    }
    lcd.setCursor(2, 0);
    lcd.print("Ext.Fan ");
    if (fanEnable == 0) lcd.print("disable");
    else lcd.print("enable ");
    lcd.setCursor(2, 1);
    lcd.print("OFF  NEXT  ON");
    delay(del);
  }
}


/// читаем датчики //////////////////////////////////////////////////////
void getSensors() {
  unsigned long curMillis = millis();          // Получаем текущее время работы
  if (curMillis - trhMillis >= TRHSTEP) {    // время для нового измерения?
    sensors.requestTemperatures();
    temp1Ink = sensors.getTempCByIndex(0);
    if (temp1Ink == -127.0) temp1Ink = 85.0;
  }
  if (curMillis - trhMillis >= TRHSTEP * 4) {    // время для нового измерения?
    // measActive = true;
    //measType = TEMP;
    // sht.meas(TEMP, &rawData, NONBLOCK);        // измеряем температуру.
    temp2Ink = mySensor.readTempC() + T2_correction;        // измеряем температуру.
    humidity = mySensor.readFloatHumidity();
    trhMillis = curMillis;

  }
  //if (measActive && sht.measRdy()) {           // проверяем статус измерения
  //  if (measType == TEMP) {                    // обрабатываем температуру или влажность?
  //    measType = HUMI;
  //    temp2Ink = sht.calcTemp(rawData);     // Конвертируем сырые данные с сенсора
  //    temp2Ink = (temp2Ink + (T_correction)); // Корректируем показания текрмометра
  //    sht.meas(HUMI, &rawData, NONBLOCK);      // измеряем влажность
  //  }
  //  else {
  //    measActive = false;
  //    humidity = sht.calcHumi(rawData, temp2Ink); // конвертируем данные с сенсора
  //   dewpoint = sht.calcDewpoint(humidity, temp2Ink);
  //}
  //}
}


//используем терморегулятор ////////////////////////////////////////////
void thermostat(float tempPoint, float set_humidity) {
  DateTime now = RTC.now();
  unsigned long now1 = millis();
  float alarmTemp;
  EEPROM_read(5, alarmTemp);
  if (digitalRead(door_pin)) { //дверь закрыта *для концевика закрыто - разомкнуто
    myPID.SetMode(AUTOMATIC);
    heater_off = false;
  }
  else { //дверь открыта
    heater_off = true;
    alarmDelay = millis();
    myPID.SetMode(MANUAL);
    Output = 300;
  }
  Setpoint = tempPoint;
  myPID.Compute();
  if (now1 - windowStartTime > WindowSize) { //время для перещелкивания периода окна
    windowStartTime = windowStartTime + WindowSize;
    //voltmeter();                        //запускаем функцию измерения напряжения
    Input = temp1Ink;
    lcd.setCursor(0, 2);                 // устанавливаем курсор в 0-ом столбце, 0 строка (начинается с 0)
    lcd.print("T1=");
    lcd.print(temp1Ink, 1);              // печать температуры на дисплей
    lcd.print((char)223);
    lcd.setCursor(8, 3);
    lcd.print("H=");
    lcd.print(humidity, 1);           // печать влажности на дисплей
    //lcd.print("%");
    lcd.print(" ");
    lcd.print("day");
    if (currentDay > 100) lcd.print("99");
    else lcd.print(currentDay);      // текущий день инкубации
    lcd.setCursor(0, 1);
    lcd.print("t");
    lcd.print(Setpoint, 1);
    lcd.setCursor(6, 1);
    lcd.print("h");
    lcd.print(set_humidity, 1);
    lcd.setCursor(7, 0);
    lcd.print("[");
    if (RTC.isrunning()) {
      if (now.hour() < 10) lcd.print(" ");
      lcd.print(now.hour(), DEC);
      lcd.print(":");
      if (now.minute() < 10)lcd.print(0);
      lcd.print(now.minute(), DEC);
    }
    lcd.setCursor(13, 0);
    lcd.print("]");
    lcd.setCursor(8, 2);
    lcd.print("T2=");
    lcd.print(temp2Ink, 1);            // печать температуры на дисплей
    lcd.print((char)223);
    lcd.setCursor(0, 3);
    lcd.print("*t");
    lcd.print(temp1Ink - temp2Ink, 1);
    lcd.setCursor(16, 2);
    lcd.print("W");
    if (Output > 990) lcd.print("99");
    else lcd.print(Output / 10, 0);
    lcd.print("%");
    lcd.setCursor(0, 0);
    lcd.print(birdPrint);
    lcd.setCursor(12, 1);
    lcd.print("Al+-");
    lcd.print(alarmTemp, 1);

  }
  if (Output > (now1 - windowStartTime) && temp1Ink < 39.9 && heater_off == false) digitalWrite(heater_pin, HIGH);
  else digitalWrite(heater_pin, LOW);
}


//управляем влажностью///////////////////////////////////////////////////////////////
void humidifer(float set_humidity) {
  //float humidity;
  unsigned long humMillis = 0;
  unsigned long curMillis = millis();
  //  if (curMillis - humMillis >= humStep) {
  //    humMillis = curMillis;
  //    //humidity = sht1x.readHumidity();
  //  }
  if (set_humidity > humidity) digitalWrite(humidifer_pin, LOW); //сравниваем измеренную влажность с заданной
  if (set_humidity < humidity + h_histeresis) digitalWrite(humidifer_pin, HIGH);
}


//управляем поворотом///////////////////////////////////////////////////////////////////
void turn(boolean needTurn) {
  int turnPeriod;                //период поворота лотков в часах
  int turnCommand;
  int turnLenght;
  int printTurn;
  EEPROM_read(21, turnLenght);
  EEPROM_read(11, turnPeriod);
  //lcd.setCursor(10, 3);
  //lcd.print("P");
  //lcd.print(turnPeriod);
  lcd.setCursor(15, 0);
  lcd.print("R");
  if (turnPeriod == 0)
  { lcd.print(" OFF");
    return;           //если нулевой период поворота, то не поворачиваем яйца.
  }
  if (turnPeriod < 13) turnCommand = turnPeriod;
  else if (turnPeriod > 12 && turnFlag == 0) { //если произошел поворот (сброшен флаг) и значение в памяти 13, то
    turnCommand = random(1, 6);        //берем случайное значение часов 1-6
    turnFlag = 1;                     //защелкиваем флаг вычисления случайных значений до следующего поворота
  }
  currentTime = millis() / 1000;
  printTurn = 0;


  if (needTurn == true) {
    if (currentTime > (loopTime + turnCommand * 3600UL)) {  // 3600000 сравниваем текущий таймер с переменной loopTime + период поворота в часах.
      digitalWrite(turn_pin, HIGH);       // включаем/выключаем реле поворота
      printTurn = 1;
      if (currentTime > ((loopTime + turnLenght) + turnCommand * 3600UL)) {
        digitalWrite(turn_pin, LOW);       // включаем/выключаем реле поворота
        printTurn = 0;
        loopTime = currentTime;    // в loopTime записываем новое значение
        turnFlag = 0;    //сбрасываем флаг поворота
      }
    }
    //lcd.print((loopTime - currentTime + turnCommand * 3600UL) / 60UL);
    //lcd.print("m");
  }
  switch (printTurn) {
    case 0:
      lcd.print((loopTime - currentTime + turnCommand * 3600UL) / 60UL);
      lcd.print("m");
      break;
    case 1:
      lcd.print((loopTime + turnLenght) - currentTime + turnCommand * 3600UL);
      lcd.print("s");
      break;
  }
}
/*
void turnPulse() {
  int turnPeriod;
  static unsigned long loopTimePulse;
  EEPROM_read(11, turnPeriod);
  if (turnPeriod == 0) return;
  currentTime = millis() / 1000;
  if (currentTime > (loopTimePulse + turnPeriod * 3600UL)) {  // 3600000 сравниваем текущий таймер с переменной loopTime + период поворота в часах.
    digitalWrite(turn_pin, HIGH);       // включаем/выключаем реле поворота
  }
  if (currentTime > ((loopTimePulse + turnLenght) + turnPeriod * 3600UL)) {
    digitalWrite(turn_pin, LOW);       // включаем/выключаем реле поворота
    loopTimePulse = currentTime;    // в loopTime записываем новое значение
  }
} */


//управляем авариями///////////////////////////////////////////////////////////////
void alarm(float needTemp) {
  float tempInk = sensors.getTempCByIndex(0);
  float alarmTemp;
  boolean fanState;
  EEPROM_read(5, alarmTemp);
  EEPROM_read(9, fanState);
  //lcd.setCursor(0, 3);
  //lcd.print("A");
  //lcd.print(needTemp + alarmTemp, 1);
  if ((millis() - alarmDelay) > 1800000) {
    if (tempInk > (needTemp + alarmTemp) || tempInk < (needTemp - alarmTemp)) {
      beeper(10);
      digitalWrite(alarm_pin, LOW); //если измеренная температура выше заданной на величину аварии
    }
    else digitalWrite(alarm_pin, HIGH); //то включаем аварийный сигнал.
  }
  if (tempInk > (needTemp + alarmTemp) && fanState == 1) needFan = 1;
  if (tempInk < (needTemp + alarmTemp - 2)) needFan = 0;
}


//пищалка////////////////////////////////////////////////////////////////////////
void beeper(int duration) {
  tone(beeper_pin, 2000, duration);
}


//управляем вентиляторами///////////////////////////////////////////////////////
void fan(int fanTime) {
  //float tempInk = sht1x.readTemperatureC();
  DateTime now = RTC.now();
  if ((now.hour() == 7 && now.minute() < fanTime) || (now.hour() == 19 && now.minute() < fanTime) || needFan == 1) {
    digitalWrite(fan_pin, LOW);
    //если наступило время проветривания или измеренная температура выше заданной на величину аварии, то включаем продувку.
    digitalWrite(extend_heater_pin, LOW);  // при этом отключаем обогрев
    digitalWrite(heater_pin, LOW);
    heater_off = true;
  }
  else {
    digitalWrite(fan_pin, HIGH); //иначе выключаем.
    heater_off = false;
  }
}

// вольтметр//////////////////////////////////////////////////////////////////////////////
//void voltmeter() {
//  float outputValue = 0;
//  outputValue = float(analogRead(voltmeter_pin)) / 63, 9;
//  //if(outputValue < 4.5) beeper(50);
//  //Serial.print("Voltage = " );
//  //Serial.println(outputValue);
//  lcd.setCursor(14, 3);
//  lcd.print("V");
//  lcd.print(outputValue, 1);
//}
//
//// Печать мощности нагрвателя
//void outpuPower() {
//  lcd.setCursor(14, 3);
//  lcd.print("W");
//  lcd.print(Output, 0);
//  lcd.print(" ");
//}


// дополнительный нагреватель /////////////////////////////////////////////////////////////
void ext_heater(float needTemp) { // управление дополнительным нагревателем на 8 ножке через блок реле.
  float tempInk = sensors.getTempCByIndex(0);
  if (tempInk < (needTemp - 3) && heater_off == false) digitalWrite(extend_heater_pin, HIGH);
  else digitalWrite(extend_heater_pin, LOW);
}


/********************************************
   ПИД и отсылка данных в порт
 ********************************************/

union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array
void SerialReceive()
{
  // read the bytes sent from Processing
  int index = 0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  while (Serial.available() && index < 26)  {
    if (index == 0) Auto_Man = Serial.read();
    else if (index == 1) Direct_Reverse = Serial.read();
    else foo.asBytes[index - 2] = Serial.read();
    index++;
  }
  // if the information we got was in the correct format,
  // read it into the system
  if (index == 26  && (Auto_Man == 0 || Auto_Man == 1) && (Direct_Reverse == 0 || Direct_Reverse == 1))
  {
    Setpoint = double(foo.asFloat[0]);
    //Input=double(foo.asFloat[1]);       // * the user has the ability to send the
    //   value of "Input"  in most cases (as
    //   in this one) this is not needed.
    if (Auto_Man == 0)                    // * only change the output if we are in
    { //   manual mode.  otherwise we'll get an
      Output = double(foo.asFloat[2]);    //   output blip, then the controller will
    }                                     //   overwrite.
    double p, i, d;                       // * read in and set the controller tunings
    p = double(foo.asFloat[3]);           //
    i = double(foo.asFloat[4]);           //
    d = double(foo.asFloat[5]);           //
    myPID.SetTunings(p, i, d);            //
    if (Auto_Man == 0) myPID.SetMode(MANUAL); // * set the controller mode
    else myPID.SetMode(AUTOMATIC);             //
    if (Direct_Reverse == 0) myPID.SetControllerDirection(DIRECT); // * set the controller Direction
    else myPID.SetControllerDirection(REVERSE);          //
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}


/// шлем данные в порт //////////////////////////////////////////////
void SerialSend() {
  //Serial.print(millis() / 1000);
  //  Serial.print("PID ");
  //  Serial.print(Setpoint);
  //  Serial.print(" ");
  //  Serial.print(Input);
  //  Serial.print(" ");
  //  Serial.print(Output);
  //  Serial.print(" ");
  //  Serial.print(myPID.GetKp());
  //  Serial.print(" ");
  //  Serial.print(myPID.GetKi());
  //  Serial.print(" ");
  //  Serial.print(myPID.GetKd());
  //  Serial.print(" ");
  //  if (myPID.GetMode() == AUTOMATIC) Serial.print("Automatic");
  //  else Serial.print("Manual");
  //  Serial.print(" ");
  //  if (myPID.GetDirection() == DIRECT) Serial.println("Direct");
  //  else Serial.println("Reverse");

  delay(5);
  Serial.print("~Mink/set ");
  Serial.print(Setpoint);
  Serial.print("^~Mink/outpwr ");
  Serial.print(Output / 10);
  Serial.print("^~Mink/t1 ");
  Serial.print(temp1Ink);
  Serial.print("^~Mink/t2 ");
  Serial.print(temp2Ink);
  Serial.print("^~Mink/hum ");

  Serial.println(humidity);

  //Serial.print("^~Mink/alarm ");
  //Serial.print(digitalRead(alarm_pin));
  //Serial.print("^~Mink/extheat ");

  //Serial.print(digitalRead(extend_heater_pin));
  //Serial.print("^~Mink/fan ");
  //Serial.print(digitalRead(fan_pin));

  //Serial.print("^~Mink/door ");
  //Serial.println(!digitalRead(door_pin));
}
