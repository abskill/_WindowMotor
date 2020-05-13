
// by Melnikov Anton

bool debug = 0; // Serial.print если = 1

// Pins // -----------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------

// NodeMCU
#define motorPin1  D5 // IN1 на 1-м драйвере ULN2003
#define motorPin2  D6 // D6 - IN2 на 1-м драйвере ULN2003
#define motorPin3  D7 // D7 - IN3 на 1-м драйвере ULN2003
#define motorPin4  D8 // D8 - IN4 на 1-м драйвере ULN2003

#define magnetPinUp   D2 // Data-pin верхнего датчика
#define magnetPinDown 10// Data-pin нижнего датчика

#define rfPin D1 // Data-pin RF приемника
#define rfPowerPin D3 // D3 - Power-pin RF приемника

/* Управление питанием RF приемника введено, т.к. по неизвестным причинам
   WiFi не коннектится, пока включен RF приемник.
   Питание восстанавливается после включения успешного подключения к WiFi
*/

//#define LED_BUILIN 16 // D0 - Пин встроенного светодиода


// Variables // ------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------

bool OTA_on = true; // Включение прощивки "по-воздуху"
uint32_t OTA_timeout = 300000; // После истечения этого времени (мс) с момента подачи питания режим будет выключен

bool MQTT_on = true;
byte mqtt_err_counter = 0;       // Счетчик ошибок подлючения
byte mqtt_number_of_trying = 10; // Максимальное количество попыток подключения
uint32_t mqtt_refresh_time = 500; // интервал обмена с MQTT сервером (мс)
uint32_t mqtt_timer = 0; // вспомогательный таймер


bool WIFI_on = true;
byte wifi_err_counter = 0;
uint32_t delay_of_trying_to_connect_wf = 60000 * 10; // пауза между попытками соединиться по WiFi
uint32_t timer_of_trying_to_connect_wf = millis(); // вспомогательный таймер

bool night_alarm = true; // Включение авто-режима (по времени)
bool night_alarm_from_outside = night_alarm; //

bool RF_on = true; // Включение RF433 приемника



bool motor_rotate = false; // вращение мотора
//bool motor_man_control = 1; //
bool motor_go_up = false;
bool motor_go_down = false;

float motor_man_speed = 800;  // скорость мотора (0...1000)
float speed_from_outside = motor_man_speed;  // скорость мотора из брокера


//uint32_t max_steps = 0; // количество шагов между крайними положениями
//bool max_steps_calibrated = false; // признак успешной калибровки max_steps
bool calibrate_on = false; // включение режима калибровки максимального количества шагов
bool start_pos_is_calibrated = false; // признак перехода к 3-му шагу калибровки
float calibrated_speed = 0; // значение скорости, на которой производилась калибровка
uint32_t calibration_timer = 0; //
uint32_t start_rotating_time = 0; //
uint32_t rotating_time_tolerance = 1000; // допуск максимального времени вращения (мс)
//uint32_t rotating_time = 0;     // таймер вращения (мс)
uint32_t max_rotating_time = 0; // максимальное время вращения (мс)


byte current_position = 0; // Текущая позиция ($UP, $DOWN, $UNKNOWN)
byte current_direction = 0; // Текущее направление
byte $UP = 1;
byte $DOWN = 2;
byte $UNKNOWN = 0;



uint32_t alarm_block_delay = 61000; // Время (мс) блокировки будильника после срабатывания
uint32_t alarm_block_timer = millis();  // вспомогательный таймер
bool alarm_block = false; // признак блокировки

uint32_t calculate_time_delay = 500; // Интервал (мс) запроса врмени с NTP сервера
uint32_t calculate_time_timer = millis(); // вспомогательный таймер


byte hh_down = 14; // Время закрытия шторы
byte mm_down = 48;

byte hh_up = 14; // Время открытия шторы
byte mm_up = 50;

byte hh_time_request = 02; // Время, в которое будет происходить уточнение текущего времени с сервера (лучше ночью)
byte mm_time_request = 00; // (корректировка будет происходить многократно в течение минуты)

int currentHour = -1;  // переменные, в которых будет храниться текущее время
int currentMinute = -1;
int currentSecond = -1;

byte hh_up_from_outside = hh_up; // Для перенастройки времени открытия шторы
byte mm_up_from_outside = mm_up;

byte hh_down_from_outside = hh_down;  // Для перенастройки времени закрытия шторы
byte mm_down_from_outside = mm_down;


// Serial Port // ----------------------------------------------------------------------------------

#define serialRate 74880 // Baudrate



// Step motor // ------------------------------------------------------------------------------

#include<AccelStepper.h>

#define HALFSTEP 8 // полушаговый режим

// Инициализируемся с последовательностью выводов IN1-IN3-IN2-IN4
// для использования AccelStepper с 28BYJ-48
AccelStepper stepper1(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4);



// WiFi // ---------------------------------------------------------------------------------------

#include <ESP8266WiFi.h> //Библиотека для работы с WIFI 
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
const char* ssid = "EnergaiZer"; //Имя точки доступа WIFI
const char* password = "ferromed"; //пароль точки доступа WIFI


#include <ArduinoOTA.h> // Библиотека для OTA-прошивки
bool OTA_started = false;


#include <PubSubClient.h> // Библиотека для обмена через MQTT-брокер
/*
  const char *mqtt_server = "m21.cloudmqtt.com"; // Имя сервера MQTT
  const int mqtt_port = 16195; // Порт для подключения к серверу MQTT
  const char *mqtt_user = "jrthidnj"; // Логи для подключения к серверу MQTT
  const char *mqtt_pass = "5ZYKoip4ef_S"; // Пароль для подключения к серверу MQTT
  const char *mqtt_unique_client_id = "arduinoClient_WindowMotor";
*/

const char *mqtt_server = "broker.hivemq.com"; // Имя сервера MQTT
const int mqtt_port = 1883; // Порт для подключения к серверу MQTT
const char *mqtt_user = ""; // Логи для подключения к серверу MQTT
const char *mqtt_pass = ""; // Пароль для подключения к серверу MQTT
const char *mqtt_unique_client_id = "arduinoClient_WindowMotor"; // Уникальный идентификатор

#define BUFFER_SIZE 100
WiFiClient wclient;
PubSubClient client(wclient, mqtt_server, mqtt_port);
int tm = 300;
float temp = 0;


//ADC_MODE(ADC_VCC); // A0 будет считытвать значение напряжения VCC


bool wf_is_connected = false;
bool mqtt_is_connected = false;
//void mqtt_call();


// TIME // ---------------------------------------------------------

#include <NTPClient.h>
#include <WiFiUdp.h>

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

//Week Days
String weekDays[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//Month names
String months[12] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};

// Set offset time in seconds to adjust for your timezone, for example:
// GMT +1 = 3600
// GMT +8 = 28800
// GMT -1 = -3600
// GMT 0 = 0
uint32_t GMT_plus4 = 14400; // Ulyanovsk


// RF 433 // -------------------------------------------------------

#include <RCSwitch.h>
RCSwitch mySwitch = RCSwitch();

#define btn_code_A 600258 //10434 // код кнопки радиоканала
#define btn_code_B 300130 //10434 // код кнопки радиоканала

bool btn_pressed_A = false;  // признак нажатия кнопки
bool btn_pressed_B = false;  // признак нажатия кнопки

bool mode_auto = true;  // режим автоматического управления с датчика движения
bool btn_block = false;  // блокировка обработкки нажатия кнопки
uint32_t btn_timer = 0;  // вспомогательный таймер

uint32_t btn_delay = 300;  // после нажатия на кнопку ее обработка блокируется на это количество миллисекунд
int btn_value = 0;  // значение кода кнопки


// EEPROM // -------------------------------------------------------

#include <EEPROM.h>

// Задаем адреса в EEPROM
/*
  byte EEMEM init_key_address;
  //bool EEMEM OTA_on_address;
  //bool EEMEM MQTT_on_address;
  //bool EEMEM WIFI_on_address;
  //bool EEMEM RF_on_address;
  bool EEMEM night_alarm_address;
  byte EEMEM hh_down_address;
  byte EEMEM mm_down_address;
  byte EEMEM hh_up_address;
  byte EEMEM mm_up_address;
  float    EEMEM motor_man_speed_address;
  uint32_t EEMEM max_steps_address;
*/




byte init_key_req = 11; // Уникальный (для каждого проекта) ключ инициализации EEPROM [1..254]
byte init_key_fact = 0; // Ключ, хранящийся в EEPROM на момент включения питания
bool request_eeprom_update = false; // Признак необходимости обновления данных в EEPROM


byte init_key_address = 0;
byte night_alarm_address = 0;
byte hh_down_address = 0;
byte mm_down_address = 0;
byte hh_up_address = 0;
byte mm_up_address = 0;
byte motor_man_speed_address = 0;
byte max_rotating_time_address = 0;
byte calibrated_speed_address = 0;


// --------------------------------------------------------------------------------------------------------------
// SETUP // -----------------------------------------------------------------------------------------------------

void setup() {



  // initialize serial communication -------------------------------
  if (debug == 1)  Serial.begin(serialRate);

  // устанавливаем режим пинов // ----------------------------------
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  //pinMode(LED_BUILTIN, OUTPUT);

  pinMode(magnetPinUp, INPUT);
  pinMode(magnetPinDown, INPUT);

  pinMode(rfPowerPin, OUTPUT);


  // Инициализация EEPROM -------------------------------------------
  eeprom_init(); // чтение или запись, в зависимости от значения уникального ключа


  // настариваем скорость и направление движения мотора // ----------
  stepper1.setMaxSpeed(1000.0);
  stepper1.setSpeed(motor_man_speed);

  // Коннектимся к WiFi // -----------------------------------------

  // try_to_connect_wf(wf_is_connected);
  if (WIFI_on) {
    if (debug == 1) Serial.print("Connecting to wifi...");

    digitalWrite(rfPowerPin, LOW); // временно выкл RF-приемник
    WiFi.mode(WIFI_STA);  WiFi.begin(ssid, password);  delay(1000);  if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
      if (debug == 1) Serial.println("fail");

      wf_is_connected = false;
      timer_of_trying_to_connect_wf = millis();
      wifi_err_counter++;
      //      digitalWrite(rfPowerPin, HIGH); // временно выкл RF-приемник
    }
    else
    {
      if (debug == 1) Serial.println("ok");
      wf_is_connected = true;

      timeClient.update(); // обновление вркмени с сервера
      if (RF_on) digitalWrite(rfPowerPin, HIGH); // вкл RF-приемник после подключения к WiFi
    }
  }

  else
  {
    if (RF_on) digitalWrite(rfPowerPin, HIGH); // вкл RF-приемник
  }


  // Первоначальное определение положения // ---------------------------------

  if (digitalRead(magnetPinUp) == HIGH) current_position = $UP;
  else if (digitalRead(magnetPinDown) == HIGH) current_position = $DOWN;
  else current_position = $UNKNOWN;

  // сбой
  if (digitalRead(magnetPinUp) == HIGH && digitalRead(magnetPinDown) == HIGH) current_position = $UNKNOWN;

  if (debug == 1) Serial.println("Started position = " + String(current_position) + "    /1-up, 2-down, else unknown/");


  // TIME // ---------------------------------------------------------

  // Initialize a NTPClient to get time
  timeClient.begin();
  timeClient.setTimeOffset(GMT_plus4);



  // инициализируем приемник rf433
  if (RF_on) mySwitch.enableReceive(rfPin);  // Receiver on interrupt rfPin



  // Сброс таймера
  //  timer = millis();

}



// -----------------------------------------------------------------
// LOOP // ---------------------------------------------------------

void loop() {

  // Активности WiFi // ----------------------------------------------
  if (WIFI_on) {
    if (wf_is_connected == false) {
      if (wifi_err_counter < 5 && (millis() - timer_of_trying_to_connect_wf) > delay_of_trying_to_connect_wf) {
        // try_to_connect_wf(wf_is_connected);
        if (debug == 1) Serial.print("Connecting to wifi...");
        //digitalWrite(LED_BUILTIN, HIGH);

        if (RF_on) digitalWrite(rfPowerPin, LOW); // временно выкл RF-приемник

        WiFi.mode(WIFI_STA);  WiFi.begin(ssid, password);  delay(1000);  if (WiFi.waitForConnectResult() != WL_CONNECTED)
        {
          if (debug == 1) Serial.println("fail");
          wf_is_connected = false;
          timer_of_trying_to_connect_wf = millis();
          wifi_err_counter++;
        }
        else
        {
          if (debug == 1) Serial.println("ok");
          wf_is_connected = true;
          wifi_err_counter = 0; // сброс счетчика ошибок в случае успешного подключения

          timeClient.update(); // обновление вркмени с сервера
          if (RF_on) digitalWrite(rfPowerPin, HIGH); // вкл RF-приемник после подключения к WiFi
        }
        //digitalWrite(LED_BUILTIN, LOW);
      }

    }

    if (wf_is_connected == true) {
      if (MQTT_on) mqtt_call();


      if (OTA_on == false && MQTT_on == false) {
        WIFI_on = false;
        if (debug == 1) Serial.println("wifi_stop");

        if (RF_on) digitalWrite(rfPowerPin, HIGH); // вкл RF-приемник
      }
      else {
        if (OTA_on) {

          if (millis() > OTA_timeout) {
            OTA_on = false;
            if (debug == 1) Serial.println("!!!_OTA is stoped by timeout_!!!");
          }

          if (OTA_started == false) {
            if (debug == 1) Serial.println("OTA is started");
            // подключаем возможность прошивки по воздуху
            ArduinoOTA.setHostname("OTA_NodeMCU_WindowMotor"); // Задаем имя сетевого порта
            //ArduinoOTA.setPassword((const char *)"0000"); // Задаем пароль доступа для удаленной прошивки
            ArduinoOTA.begin(); // Инициализируем OTA
            OTA_started = true;
          }
          else {
            ArduinoOTA.handle(); // ожидание старта прошивки
          }
        }
      }
    }

    if (wf_is_connected == true) {         // TIME
      if ((millis() - calculate_time_timer) > calculate_time_delay)  {

        unsigned long epochTime = timeClient.getEpochTime();
        //if (debug == 1) Serial.println("Epoch Time: " + String(epochTime));

        String formattedTime = timeClient.getFormattedTime();
        if (debug == 1) Serial.println("Formatted Time: " + String(formattedTime));

        currentHour = timeClient.getHours();
        currentMinute = timeClient.getMinutes();
        currentSecond = timeClient.getSeconds();

        if (currentHour == hh_time_request && currentMinute == mm_time_request)  timeClient.update(); // обновление с сервера

        /*
          String weekDay = weekDays[timeClient.getDay()];
          //Get a time structure
          struct tm *ptm = gmtime ((time_t *)&epochTime);
          int monthDay = ptm->tm_mday;
          int currentMonth = ptm->tm_mon+1;
          String currentMonthName = months[currentMonth-1];
          int currentYear = ptm->tm_year+1900;
        */

        calculate_time_timer = millis();
      }
    }

  } // if (WIFI_on)
  else
  {
    //    if (RF_on) digitalWrite(rfPowerPin, HIGH); // вкл RF-приемник
  }




  if (RF_on) rf_control(); //



  // Применение внешних настроек // ---------------------------------------------

  if (hh_up != hh_up_from_outside or
      mm_up != mm_up_from_outside or
      hh_down != hh_down_from_outside or
      mm_down != mm_down_from_outside or
      motor_man_speed != speed_from_outside)
  {

    // Обновим переменные
    hh_up = hh_up_from_outside;
    mm_up = mm_up_from_outside;

    hh_down = hh_down_from_outside;
    mm_down = mm_down_from_outside;

    motor_man_speed = speed_from_outside;

    // Установим признак необходимости обновления данных в EEPROM
    request_eeprom_update = true;
  }

  // Обновим данные в EEPROM, когда двигатель не вращается (т.е. отсутсвуют просадки по питанию)
  if (request_eeprom_update && motor_rotate == false) eeprom_update;



  // Определение текущего положения // ---------------------------------

  if (motor_rotate)  // проверяем только в движении, чтобы исключить паразитные срабатывания
  {
    // штатная работа
    if (digitalRead(magnetPinUp) == HIGH) current_position = $UP;
    else if (digitalRead(magnetPinDown) == HIGH) current_position = $DOWN;
    else current_position = $UNKNOWN;

    // сбой
    if (digitalRead(magnetPinUp) == HIGH && digitalRead(magnetPinDown) == HIGH) current_position = $UNKNOWN;

    //if (debug == 1) Serial.println("Current position = "+String(current_position)+"    /1-up, 2-down, else unknown/");
  }



  // Калибровка количества шагов ---------------------------------------

  if (calibrate_on)
  {
    // шаг 1: Запускаем движение в нижнюю позицию
    if (current_position != $DOWN && motor_rotate == false && start_pos_is_calibrated == false) 
    {
    motor_go_down = true;
    if (debug == 1) Serial.println("Calibration_STEP_1");
    }
    
    // шаг 2: Стартуем таймер и запускаем движение в верхниюю позицию
    if (current_position == $DOWN && motor_rotate == false && start_pos_is_calibrated == false) {
      //stepper1.setCurrentPosition( stepper1.currentPosition() )
      calibration_timer = millis(); // стартуем таймер
      motor_go_up = true; // запускаем движение
      start_pos_is_calibrated = true; // разблокируем шаг 3
    if (debug == 1) Serial.println("Calibration_STEP_2");
    }

    // шаг 3: Определяем пройденное время, запоминаем и выключаем режим калибровки
    if (start_pos_is_calibrated && current_position == $UP && motor_rotate == false) {
      //max_steps=currentPosition()+(int)currentPosition()*0.01; // подсчитанное колличество шагов + 1%
      max_rotating_time = millis() - calibration_timer;
      calibrate_on = false; // выключаем режим калибровки
      calibrated_speed = motor_man_speed; // запоминаем скорость
      eeprom_update(); // обновляем данные в EEPROM
      if (debug == 1) Serial.println("Calibration_END:  max_rotating_time = " + String(max_rotating_time) + " ms");
      start_pos_is_calibrated = false;
    }

  }



  // Снятие блокировки будильника --------------------------------------

  if (alarm_block)
  {
    if ((millis() - alarm_block_timer) > alarm_block_delay) alarm_block = false;
  }


  // Управление двигателем  // -----------------------------------------

  if (motor_rotate)
  {
    stepper1.runSpeed(); // Крутим двигатель (нужно вызывать как можно чаще)
    //int32_t CP = stepper1.currentPosition();
    //if (debug == 1) Serial.println("currentPosition = " + String(stepper1.currentPosition()));
    //!Подсчет количества сделанных шагов
  }
  else
  {
    // Выключаем напряжение на обмотках мотора (защита от перегрева)
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, LOW);

    //digitalWrite(LED_BUILTIN, HIGH); // встроенный светодиод: HIGH - выкл, LOW - вкл
  }


  // Обработка команд управления двигателем // -----------------------------------------------

  // Команда ВВЕРХ - вручную
  if (motor_go_up)
  {
    if (motor_rotate && current_direction == $UP)
    {
      motor_rotate = false; // Если уже вращается вверх, то останавливаем
    }
    else
    {
      motor_rotate = true;
      current_direction = $UP;
      stepper1.setSpeed(motor_man_speed); // Настраеваем скорость и направление движения
      start_rotating_time = millis(); // Стартуем таймер
    }
    motor_go_up = 0; // команда обработана => сброс команды
  }

  // Команда ВНИЗ - вручную
  if (motor_go_down)
  {
    if (motor_rotate && current_direction == $DOWN)
    {
      motor_rotate = false; // Если уже вращается вверх, то останавливаем
    }
    else
    {
      motor_rotate = true;
      current_direction = $DOWN;
      stepper1.setSpeed(-motor_man_speed); // Настраеваем скорость и направление движения
      start_rotating_time = millis(); // Стартуем таймер
    }
    motor_go_down = 0; // команда обработана => сброс команды
  }


  // Установка команд управления двигателем // -----------------------------------------------

  // Команда ВВЕРХ - RF433
  if (btn_pressed_A) motor_go_up = true;

  // Команда ВНИЗ - RF433
  if (btn_pressed_B) motor_go_down = true;



  // Команда ВВЕРХ - по времени
  if (night_alarm == true && currentHour == hh_up && currentMinute == mm_up && alarm_block == false) {
    if (motor_rotate) {
      if (debug == 1) Serial.println("UP alarm is canceled");
    }
    else {
      motor_go_up = 1;
      alarm_block_timer = millis();
      if (debug == 1) Serial.println("UP alarm");
    }
    alarm_block = true;
  }

  // Команда ВНИЗ - по времени
  if (night_alarm == true && currentHour == hh_down && currentMinute == mm_down  && alarm_block == false) {
    if (motor_rotate) {
      if (debug == 1) Serial.println("DOWN alarm is canceled");
    }
    else {
      motor_go_down = 1;
      alarm_block_timer = millis();
      if (debug == 1) Serial.println("DOWNP alarm");
    }
    alarm_block = true;
  }


  // Команды ВВЕРХ, ВНИЗ, СТОП - wifi
  // поступают через MQTT-брокер в виде готовых комманд motor_go_up, motor_go_down, motor_rotate


  // Команда СТОП - от датчиков
  if (motor_rotate)
  {
    if (current_direction == $UP && current_position == $UP) motor_rotate = false;
    if (current_direction == $DOWN && current_position == $DOWN) motor_rotate = false;
  }


  // Команда СТОП - по истечению максимального времени вращения
  if (motor_man_speed == calibrated_speed && motor_rotate)
  {
    //rotating_time = millis() - start_rotating_time;
    if ((millis() - start_rotating_time) > (max_rotating_time + rotating_time_tolerance))
    {
      motor_rotate = false;
      if (debug == 1) Serial.println("STOP by max_rotating_time");
    }

  }

}


// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
