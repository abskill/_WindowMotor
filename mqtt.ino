

// MQTT // ---------------------------------------------------------

void mqtt_call()
{
  // подключаемся к MQTT серверу
  if (WiFi.status() == WL_CONNECTED && millis() - mqtt_timer > mqtt_refresh_time)
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
        client.subscribe("OTA_on");
        client.subscribe("MQTT_on");
        client.subscribe("motor_rotate");
        client.subscribe("motor_go_up");
        client.subscribe("motor_go_down");
        client.subscribe("hh_up_from_outside");
        client.subscribe("mm_up_from_outside");
        client.subscribe("hh_down_from_outside");
        client.subscribe("mm_down_from_outside");
        client.subscribe("speed_from_outside");
        client.subscribe("calibrate_on");
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
  client.publish("hh_up", String(hh_up));
  client.publish("mm_up", String(mm_up));
  client.publish("hh_down", String(hh_down));
  client.publish("mm_down", String(mm_down));
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
  if (topic == "hh_up_from_outside") hh_up_from_outside = payload.toInt();
  if (topic == "mm_up_from_outside") mm_up_from_outside = payload.toInt();
  if (topic == "hh_down_from_outside") hh_down_from_outside = payload.toInt();
  if (topic == "mm_down_from_outside") mm_down_from_outside = payload.toInt();
  if (topic == "speed_from_outside") speed_from_outside = payload.toInt();
  if (topic == "calibrate_on") calibrate_on = payload.toInt();
  // if (debug == 1) Serial.println("OTA_on = " + payload); // выводим в сериал порт значение полученных данных
}

