
bool debug = 0; // Serial.print если = 1

#include<AccelStepper.h>


// ----------------------------------------------------------
// Определение пинов для управления двигателем

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


// Variables // ---------------------------------------------

bool OTA_on = true;
uint32_t OTA_timeout = 300000; // После истечения этого времени (мс) с момента подачи питания режим будет выключен

bool MQTT_on = true;
byte mqtt_err_counter = 0;       // Счетчик ошибок подлючения
byte mqtt_number_of_trying = 10; // Максимальное количество попыток подключения
uint32_t mqtt_delay = 500; // интервал обмена с MQTT сервером
uint32_t mqtt_timer = 0; // вспомогательный таймер


bool WIFI_on = true;
byte wifi_err_counter = 0;
uint32_t delay_of_trying_to_connect_wf = 60000 * 10; // пауза между попытками соединиться по WiFi
uint32_t timer_of_trying_to_connect_wf = millis(); // вспомогательный таймер

bool night_alarm = true; // Включение авто-режима (по времени)
bool night_alarm_from_outside = night_alarm; //

bool RF_on = true; // Включение RF433 приемника

/*
  uint32_t delay_ms = 8000; // таймер
  uint32_t timer = millis(); // вспомогательный таймер
*/

float spd = 800;

bool motor_rotate = false; // вращение мотора
bool motor_man_control = 1; //
bool motor_go_up = false;
bool motor_go_down = false;

float motor_man_speed = 800;  // скорость мотора (0...1000)
float speed_from_outside = motor_man_speed;  // скорость мотора из брокера

//float speed_correction = 0.5; // коэффициент мультипликатор значения скорости

uint32_t max_steps = 1000; // количество шагов между крайними положениями
//bool max_steps_calibrated = false; // признак успешной калибровки max_steps
uint32_t steps_start = 0; // количество шагов, опредленное в момент начала движения
uint32_t current_steps = 0; // количество шагов, сделанных мотором с момента начала движения

byte current_position = 0; // Текущая позиция ($UP, $DOWN, $UNKNOWN)
byte current_direction = 0; // Текущее направление
byte $UP = 1;
byte $DOWN = 2;
byte $UNKNOWN = 0;



uint32_t alarm_block_delay = 61000; // Время (мс) блокировки будильника после срабатывания
uint32_t alarm_block_timer = millis();  //
bool alarm_block = false; //

uint32_t calculate_time_delay = 500; // Интервал (мс) запроса врмени с NTP сервера
uint32_t calculate_time_timer = millis(); //


byte hh_down = 14; // Время закрытия шторы
byte mm_down = 48;

byte hh_up = 14; // Время открытия шторы
byte mm_up = 50;

byte hh_time_request = 02; // Время, в которое будет происходить уточнение текущего времени с сервера (лучше ночью)
byte mm_time_request = 00; // Корректировка будет происходить в течение минуты

int currentHour = -1;  // переменные, в которых будет храниться текущее время
int currentMinute = -1;
int currentSecond = -1;

byte hh_up_from_outside = hh_up; // Для перенастройки будильника
byte mm_up_from_outside = mm_up;

byte hh_down_from_outside = hh_down;  // Для перенастройки будильника
byte mm_down_from_outside = mm_down;


// Meta data // ---------------------------------------------

#define HALFSTEP 8 // полушаговый режим

#define serialRate 74880 // Baudrate



// Инициализируемся с последовательностью выводов IN1-IN3-IN2-IN4
// для использования AccelStepper с 28BYJ-48
AccelStepper stepper1(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4);

// WiFi // --------------------------------------------------

#include <ESP8266WiFi.h> //Библиотека для работы с WIFI 
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

#include <ArduinoOTA.h> // Библиотека для OTA-прошивки

const char* ssid = "EnergaiZer"; //Имя точки доступа WIFI
const char* password = "ferromed"; //пароль точки доступа WIFI

#include <PubSubClient.h>
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
const char *mqtt_unique_client_id = "arduinoClient_WindowMotor";

#define BUFFER_SIZE 100
WiFiClient wclient;
PubSubClient client(wclient, mqtt_server, mqtt_port);
int tm = 300;
float temp = 0;


//ADC_MODE(ADC_VCC); // A0 будет считытвать значение напряжения VCC


bool wf_is_connected = false;
bool mqtt_is_connected = false;
//void mqtt_call();
bool OTA_started = false;


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
uint32_t GMT_p4 = 14400; // Ulyanovsk


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



// -----------------------------------------------------------------
// SETUP // --------------------------------------------------------

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

  // настариваем скорость и направление движения мотора // ----------
  stepper1.setMaxSpeed(1000.0);
  stepper1.setSpeed(spd);


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
  timeClient.setTimeOffset(GMT_p4);



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

  hh_up = hh_up_from_outside;
  mm_up = mm_up_from_outside;

  hh_down = hh_down_from_outside;
  mm_down = mm_down_from_outside;

  motor_man_speed = speed_from_outside;


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

  /*
    if (millis() - timer > 2000)
    {
      if (debug == 1) Serial.println("up=  " + String(digitalRead(magnetPinUp)));
      if (debug == 1) Serial.println("down=" + String(digitalRead(magnetPinDown)));
      timer = millis();
    }
  */

  // Калибровка количества шагов ---------------------------------------
  /*
    if (max_steps_calibrated == false)
    {
    if (motor_rotate)
    }
  */

  // Коэффициент скорости ----------------------------------------------
  //  if (max_steps_calibrated == true) speed_correction = 1;


  // Снятие блокировки будильника --------------------------------------

  if (alarm_block)
  {
    if ((millis() - alarm_block_timer) > alarm_block_delay) alarm_block = false;
  }

  // Управление двигателем  // -----------------------------------------

  // DEMO-режим
  if (motor_man_control == false)
  {
    /*
       stepper1.runSpeed();
       //stepper.runSpeedToPosition();

       if (millis() - timer > delay_ms)
       {
         //stepper1.stop();

         // Выключаем напряжение на обмотках мотора
         digitalWrite(motorPin1, LOW);
         digitalWrite(motorPin2, LOW);
         digitalWrite(motorPin3, LOW);
         digitalWrite(motorPin4, LOW);

         //motor_rotate = false;

         delay(20000);
         spd = -spd;
         //  stepper1.setMaxSpeed(spd); //1000.0);
         stepper1.setSpeed(spd);
         timer = millis();

         //motor_rotate = true;
       }
    */
  }

  // Штатный режим
  else
  {
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

    // Команда ВВЕРХ - вручную
    if (motor_go_up)
    {
      //digitalWrite(LED_BUILTIN, LOW); // встроенный светодиод: HIGH - выкл, LOW - вкл
      motor_go_up = 0;
      motor_go_down = 0;
      motor_rotate = 1;
      current_direction = $UP;
      stepper1.setSpeed(motor_man_speed); // Настраеваем скорость и направление движения
      //if (debug == 1) Serial.println("up=  " + String(digitalRead(magnetPinUp)));
      //if (debug == 1) Serial.println("down=" + String(digitalRead(magnetPinDown)));
    }

    // Команда ВНИЗ - вручную
    if (motor_go_down)
    {
      //digitalWrite(LED_BUILTIN, LOW); // встроенный светодиод: HIGH - выкл, LOW - вкл
      motor_go_up = 0;
      motor_go_down = 0;
      motor_rotate = 1;
      current_direction = $DOWN;
      stepper1.setSpeed(-motor_man_speed); // Настраеваем скорость и направление движения
      //if (debug == 1) Serial.println("up=  " + String(digitalRead(magnetPinUp)));
      //if (debug == 1) Serial.println("down=" + String(digitalRead(magnetPinDown)));
    }

    //if (debug == 1) Serial.println(String(currentHour)+":"+String(currentMinute));



    // Команда ВВЕРХ - вручную RF
    if (btn_pressed_A) motor_go_up = 1;

    // Команда ВНИЗ - вручную RF
    if (btn_pressed_B) motor_go_down = 1;




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


    // Команда СТОП - вручную
    // поступает через MQTT-брокер


    // Команда СТОП - от датчиков
    if (motor_rotate)
    {
      if (current_direction == $UP && current_position == $UP) motor_rotate = false;
      if (current_direction == $DOWN && current_position == $DOWN) motor_rotate = false;
    }

  }



}

// -----------------------------------------------------------------
// -----------------------------------------------------------------


// ФУНКЦИИ

// MQTT // ---------------------------------------------------------

void mqtt_call()
{
  // подключаемся к MQTT серверу
  if (WiFi.status() == WL_CONNECTED && millis() - mqtt_timer > mqtt_delay)
  {
    mqtt_timer = millis(); // сброс таймера
    if (!client.connected() && (mqtt_err_counter < mqtt_number_of_trying))
    {
      if (debug == 1) Serial.print("Connecting to MQTT server ");
      if (debug == 1) Serial.print(mqtt_server);
      if (debug == 1) Serial.println("...");
      //if (client.connect(MQTT::Connect(mqtt_unique_client_id).set_auth(mqtt_user, mqtt_pass)))
      if (client.connect(mqtt_unique_client_id))
      {
        if (debug == 1) Serial.println("Connected to MQTT server ");
        mqtt_err_counter = 0; // сброс счетчика ошибок в случае успешного подключения

        client.set_callback(callback);
        // подписываемся под топики
        /*
          client.subscribe("led_state");
          client.subscribe("mode_auto");
          client.subscribe("sensor_command");
          client.subscribe("low_light");
          client.subscribe("btn_value");
          //client.subscribe("vcc");
          client.subscribe("d_time");
        */
        client.subscribe("OTA_on");
        client.subscribe("MQTT_on");
        //client.subscribe("btn_pressed");
        //client.subscribe("br_target");

        client.subscribe("motor_rotate");
        client.subscribe("motor_go_up");
        client.subscribe("motor_go_down");
        client.subscribe("motor_man_control");

        client.subscribe("hh_up_from_outside");
        client.subscribe("mm_up_from_outside");
        client.subscribe("hh_down_from_outside");
        client.subscribe("mm_down_from_outside");

        client.subscribe("speed_from_outside");
      }
      else
      {
        if (debug == 1) Serial.println("Could not connect to MQTT server");
        mqtt_err_counter++;
        if (mqtt_err_counter == mqtt_number_of_trying) Serial.println("!!!_Stop trying to connect MQTT server_!!!");
      }
    }

    if (client.connected()) {
      client.loop();
      refreshData();
    }
  }
}


// MQTT: Функция отправки показаний // ----------------------------------

void refreshData() {
  //client.publish("motor_rotate", String(motor_rotate));
  //client.publish("motor_man_control", String(motor_man_control));
  //client.publish("motor_go_up", String(motor_go_up));
  //client.publish("motor_go_down", String(motor_go_down));

  client.publish("hh_up", String(hh_up));
  client.publish("mm_up", String(mm_up));
  client.publish("hh_down", String(hh_down));
  client.publish("mm_down", String(mm_down));

  //client.publish("vcc", String(ESP.getVcc())); // считаем напряжение на VCC (через пин A0)
  //client.publish("OTA_on", String(OTA_on));

  delay(1);
}



// MQTT: Функция получения данных от сервера // -----------------------------------------

void callback(const MQTT::Publish & pub)
{
  String payload = pub.payload_string();
  String topic = pub.topic();
  if (debug == 1) {
    Serial.print(pub.topic()); // выводим в сериал порт название топика
    Serial.print(" => ");
    Serial.println(payload); // выводим в сериал порт значение полученных данных
  }

  // проверяем из нужного ли нам топика пришли данные

  if (topic == "OTA_on") OTA_on = payload.toInt();
  if (topic == "MQTT_on") MQTT_on = payload.toInt();
  if (topic == "motor_rotate") motor_rotate = payload.toInt();
  if (topic == "motor_go_up") motor_go_up = payload.toInt();
  if (topic == "motor_go_down") motor_go_down = payload.toInt();
  if (topic == "motor_man_control") motor_man_control = payload.toInt();

  if (topic == "hh_up_from_outside") hh_up_from_outside = payload.toInt();
  if (topic == "mm_up_from_outside") mm_up_from_outside = payload.toInt();
  if (topic == "hh_down_from_outside") hh_down_from_outside = payload.toInt();
  if (topic == "mm_down_from_outside") mm_down_from_outside = payload.toInt();

  if (topic == "speed_from_outside") speed_from_outside = payload.toInt();

  // if (debug == 1) Serial.println("OTA_on = " + payload); // выводим в сериал порт значение полученных данных
}
