
bool debug = 1; // Serial.print если = 1

#include<AccelStepper.h>


// ----------------------------------------------------------
// Определение пинов для управления двигателем

// NodeMCU
#define motorPin1  14 // D5 - IN1 на 1-м драйвере ULN2003
#define motorPin2  12 // D6 - IN2 на 1-м драйвере ULN2003
#define motorPin3  13 // D7 - IN3 на 1-м драйвере ULN2003
#define motorPin4  15 // D8 - IN4 на 1-м драйвере ULN2003

#define magnetPinUp   5 // D1 - Data-pin верхнего датчика
#define magnetPinDown 4 // D2 - Data-pin нижнего датчика

//#define LED_BUILIN 16 // D0 - Пин встроенного светодиода


// Variables // ---------------------------------------------

bool OTA_on = false;
bool MQTT_on = true;
bool WIFI_on = true;
byte wifi_err_couter = 0;

uint32_t delay_ms = 8000; // таймер
uint32_t timer = millis(); // вспомогательный таймер
float spd = 800;

bool motor_rotate = false; // вращение мотора
bool motor_man_control = 1; //
bool motor_go_up = false;
bool motor_go_down = false;

float motor_man_speed = 800;  // скорость мотора в штатном режиме (0...1000)
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



// Meta data // ---------------------------------------------

#define HALFSTEP 8 // полушаговый режим

#define serialRate 74880 // Baudrate

uint32_t mqtt_delay = 500; // интервал обмена с MQTT сервером
uint32_t mqtt_timer = 0; // вспомогательный таймер


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
//const char *mqtt_server = "test.mosquitto.org"; // Имя сервера MQTT
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


uint32_t delay_of_trying_to_connect_wf = 60000 * 10;
uint32_t timer_of_trying_to_connect_wf = millis();
bool wf_is_connected = false;
bool mqtt_is_connected = false;
//void mqtt_call();
bool OTA_started = false;


// -----------------------------------------------------------------
// -----------------------------------------------------------------

void setup() {

  // initialize serial communication -------------------------------
  if (debug == 1)  Serial.begin(serialRate);

  // устанавливаем режим пинов // ----------------------------------
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(magnetPinUp, INPUT);
  pinMode(magnetPinDown, INPUT);


  // настариваем скорость и направление движения мотора // ----------
  stepper1.setMaxSpeed(1000.0);
  stepper1.setSpeed(spd);


  // Коннектимся к WiFi // -----------------------------------------

  // try_to_connect_wf(wf_is_connected);
  if (WIFI_on) {
    if (debug == 1) Serial.print("Connecting to wifi...");
    WiFi.mode(WIFI_STA);  WiFi.begin(ssid, password);  delay(1000);  if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
      if (debug == 1) Serial.println("fail");

      wf_is_connected = false;
      timer_of_trying_to_connect_wf = millis();
      wifi_err_couter++;
    }
    else
    {
      if (debug == 1) Serial.println("ok");
      wf_is_connected = true;
    }
  }


  // Первоначальное определение положения // ---------------------------------

  if (motor_rotate)
  {
    // штатная работа
    if (digitalRead(magnetPinUp) == LOW) current_position = $UP;
    else if (digitalRead(magnetPinDown) == LOW) current_position = $DOWN;
    else current_position = $UNKNOWN;

    // сбой
    if (digitalRead(magnetPinUp) == LOW && digitalRead(magnetPinDown) == LOW) current_position = $UNKNOWN;
  }




  // Сброс таймера
  timer = millis();
}



// -----------------------------------------------------------------
// -----------------------------------------------------------------

void loop() {

  // Активности WiFi // ----------------------------------------------
  if (WIFI_on) {
    if (wf_is_connected == false) {
      if (wifi_err_couter < 5 && (millis() - timer_of_trying_to_connect_wf) > delay_of_trying_to_connect_wf) {
        // try_to_connect_wf(wf_is_connected);
        if (debug == 1) Serial.print("Connecting to wifi...");
        //digitalWrite(LED_BUILTIN, HIGH);
        WiFi.mode(WIFI_STA);  WiFi.begin(ssid, password);  delay(1000);  if (WiFi.waitForConnectResult() != WL_CONNECTED)
        {
          if (debug == 1) Serial.println("fail");
          wf_is_connected = false;
          timer_of_trying_to_connect_wf = millis();
          wifi_err_couter++;
        }
        else
        {
          if (debug == 1) Serial.println("ok");
          wf_is_connected = true;
        }
        //digitalWrite(LED_BUILTIN, LOW);
      }

    }

    if (wf_is_connected == true) {
      if (MQTT_on) mqtt_call();


      if (OTA_on == false && MQTT_on == false) {
        WIFI_on = false;
        if (debug == 1) Serial.println("wifi_stop");
        //STOP_WF()
      }
      else {
        if (OTA_on) {
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
  } // if (WIFI_on)
  else delay(1);
  //!Убери задержку


  // Определение текущего положения // ---------------------------------

  if (motor_rotate)  // проверяем только в движении, чтобы исключить паразитные срабатывания
  {
    // штатная работа
    if (digitalRead(magnetPinUp) == LOW) current_position = $UP;
    else if (digitalRead(magnetPinDown) == LOW) current_position = $DOWN;
    else current_position = $UNKNOWN;

    // сбой
    if (digitalRead(magnetPinUp) == LOW && digitalRead(magnetPinDown) == LOW) current_position = $UNKNOWN;
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




  // Управление двигателем  // -----------------------------------------

  // DEMO-режим
  if (motor_man_control == false)
  {
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

  }

  // Штатный режим
  else
  {
    if (motor_rotate)
    {
      stepper1.runSpeed(); // Крутим двигатель (нужно вызывать как можно чаще)
      //!Подсчет количества сделанных шагов
    }
    else
    {
      // Выключаем напряжение на обмотках мотора (защита от перегрева)
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      digitalWrite(motorPin3, LOW);
      digitalWrite(motorPin4, LOW);

      digitalWrite(LED_BUILTIN, HIGH); // встроенный светодиод: HIGH - выкл, LOW - вкл
    }

    // Команда ВВЕРХ - вручную
    if (motor_go_up)
    {
      digitalWrite(LED_BUILTIN, LOW); // встроенный светодиод: HIGH - выкл, LOW - вкл
      motor_go_up = 0;
      motor_go_down = 0;
      motor_rotate = 1;
      current_direction = $UP;
      stepper1.setSpeed(motor_man_speed); // Настраеваем скорость и направление движения
      if (debug == 1) Serial.println("up=  " + String(digitalRead(magnetPinUp)));
      if (debug == 1) Serial.println("down=" + String(digitalRead(magnetPinDown)));
    }

    // Команда ВНИЗ - вручную
    if (motor_go_down)
    {
      digitalWrite(LED_BUILTIN, LOW); // встроенный светодиод: HIGH - выкл, LOW - вкл
      motor_go_up = 0;
      motor_go_down = 0;
      motor_rotate = 1;
      current_direction = $DOWN;
      stepper1.setSpeed(-motor_man_speed); // Настраеваем скорость и направление движения
      if (debug == 1) Serial.println("up=  " + String(digitalRead(magnetPinUp)));
      if (debug == 1) Serial.println("down=" + String(digitalRead(magnetPinDown)));
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
    mqtt_timer = millis();
    if (!client.connected())
    {
      if (debug == 1) Serial.print("Connecting to MQTT server ");
      if (debug == 1) Serial.print(mqtt_server);
      if (debug == 1) Serial.println("...");
      //if (client.connect(MQTT::Connect(mqtt_unique_client_id).set_auth(mqtt_user, mqtt_pass)))
      if (client.connect(mqtt_unique_client_id))
      {
        if (debug == 1) Serial.println("Connected to MQTT server ");

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

      }
      else
      {
        if (debug == 1) Serial.println("Could not connect to MQTT server");
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
  //client.publish("timer", String(constrain(delay_ms - (millis() - timer), 0, delay_ms)));

  //client.publish("br", String(br));
  //client.publish("sensor_command", String(sensor_command));
  //client.publish("low_light", String(low_light));
  //client.publish("btn_value", String(btn_value));
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

  // if (debug == 1) Serial.println("OTA_on = " + payload); // выводим в сериал порт значение полученных данных
}
