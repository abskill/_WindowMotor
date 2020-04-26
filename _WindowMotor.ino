
bool debug = false; // Serial.print если = 1

#include<AccelStepper.h>

// ----------------------------------------------------------
// Определение пинов для управления двигателем

#define motorPin1  14 // IN1 на 1-м драйвере ULN2003
#define motorPin2  12 // IN2 на 1-м драйвере ULN2003
#define motorPin3  13 // IN3 на 1-м драйвере ULN2003
#define motorPin4  15 // IN4 на 1-м драйвере ULN2003

#define magnetPin1  2

// Variables // ---------------------------------------------

bool OTA_on = true;
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
float motor_man_speed = 800;

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
const char *mqtt_server = "m21.cloudmqtt.com"; // Имя сервера MQTT
const int mqtt_port = 16195; // Порт для подключения к серверу MQTT
const char *mqtt_user = "jrthidnj"; // Логи для подключения к серверу MQTT
const char *mqtt_pass = "5ZYKoip4ef_S"; // Пароль для подключения к серверу MQTT
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




  // Управление двигателем  // -----------------------------------------

  if (motor_man_control == false)
  {
    stepper1.runSpeed();
    //stepper.runSpeedToPosition();

    if (millis() - timer > delay_ms)
    {
      //stepper1.stop();

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
  
  
  else
  {
    if (motor_rotate)   stepper1.runSpeed();
    else
    {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      digitalWrite(motorPin3, LOW);
      digitalWrite(motorPin4, LOW);

//      motor_go_up = 0;
//      motor_go_down = 0;
    }

    if (motor_go_up)
    {
      motor_go_up = 0;
      motor_go_down = 0;
      motor_rotate = 1;
      spd = motor_man_speed;
      stepper1.setSpeed(spd);
    }

    if (motor_go_down)
    {
      motor_go_up = 0;
      motor_go_down = 0;
      motor_rotate = 1;
      spd = -motor_man_speed;
      stepper1.setSpeed(spd);
    }

    /*
        if (motor_rotate == false)
        {
          motor_go_up = 0;
          motor_go_down = 0;
       }
    */

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
      if (client.connect(MQTT::Connect(mqtt_unique_client_id).set_auth(mqtt_user, mqtt_pass)))
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
