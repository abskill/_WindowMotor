

// MQTT // -----------------------------------------------------------------------------

void mqtt_call() {

   delay(3); // Крайне важная задержка в обеспечение wifi
    
  // подключаемся к MQTT серверу
  if (WiFi.status() == WL_CONNECTED && millis() - mqtt_timer > mqtt_refresh_time)
  {
    mqtt_timer = millis(); // сброс таймера
    if (!client.connected() && (mqtt_err_counter < mqtt_number_of_trying))
    {
      if (debug == 1) Serial.print("Connecting to MQTT server " + String(mqtt_server) + "...");

      //mqtt_unique_client_id.set_will("this_project_status", "NO CONNECTION"); // топик статуса данного клиента и сообщение на случай потери связи с брокером

      client.setServer(mqtt_server, mqtt_port  );
      client.setCallback(callback);

      //if (client.connect(MQTT::Connect(mqtt_unique_client_id).set_auth(mqtt_user, mqtt_pass)))
      //if (client.connect(mqtt_unique_client_id))  // connect without willTopic
        if (client.connect(mqtt_unique_client_id, "this_project_status", 1, true, String("OFFLINE").c_str())) // connect with willTopic
      {
        if (debug == 1) Serial.println("ok");
        mqtt_err_counter = 0; // сброс счетчика ошибок в случае успешного подключения

        //client.set_callback(callback);


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

        client.subscribe("up_time_from_outside", 1);
        client.subscribe("down_time_from_outside", 1);
        client.subscribe("night_alarm_from_outside", 1);

      }
      else
      {
        if (debug == 1) Serial.println("fail");
        mqtt_err_counter++;
        if (mqtt_err_counter == mqtt_number_of_trying) Serial.println("!!!_Stop trying to connect MQTT server_!!!");
      }
    }

    if (client.connected()) {
     //if (debug) Serial.println("------client.connected()"); 

      client.loop();
      refreshData();
    }
  }
}



// MQTT: Функция отправки показаний // -------------------------------------------------

void refreshData() {
  //if (debug) Serial.println("------------refreshData");

  if (mqtt_sending_request)
  {
    client.publish("this_project_status", String("ONLINE").c_str(), true); // Сообщаем в топик willTopic, что связь есть
  
    //bool retain = true;

    client.publish("hh_up", String(hh_up).c_str());
    client.publish("mm_up", String(mm_up).c_str());
    client.publish("hh_down", String(hh_down).c_str());
    client.publish("mm_down", String(mm_down).c_str());

    String up_time = time2string(hh_up, mm_up);
    client.publish("up_time", up_time.c_str(), true);

    String down_time = time2string(hh_down, mm_down);
    client.publish("down_time", down_time.c_str(), true);

    client.publish("night_alarm", String(night_alarm).c_str(), true);

    client.publish("motor_rotate_out", motor_rotate_out.c_str(), true);
    client.publish("current_position_out", current_position_out.c_str(), true);

    mqtt_sending_request = false;
  }
  delay(1);
}


String time2string(byte HH, byte MM) {
  String time_string = "";
  if (HH < 10) time_string += "0";
  time_string += String(HH) + ":";
  if (MM < 10) time_string += "0";
  time_string += String (MM);

  return time_string;
}





// MQTT: Функция получения данных от сервера // -----------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  if (debug) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
  }


  String payload_string = "";

  for (int i = 0; i < length; i++) {
    //if (debug) Serial.print((char)payload[i]);
    payload_string.concat((char)payload[i]);
  }
  if (debug) Serial.println(String(payload_string));

  payload_string = (String)payload_string;

  // проверяем из нужного ли нам топика пришли данные

  if (String(topic) == "OTA_on") OTA_on = payload_string.toInt();
  if (String(topic) == "MQTT_on") MQTT_on = payload_string.toInt();
  if (String(topic) == "motor_rotate") motor_rotate = payload_string.toInt();
  if (String(topic) == "motor_go_up") motor_go_up = payload_string.toInt();
  if (String(topic) == "motor_go_down") motor_go_down = payload_string.toInt();
  if (String(topic) == "hh_up_from_outside") hh_up_from_outside = payload_string.toInt();
  if (String(topic) == "mm_up_from_outside") mm_up_from_outside = payload_string.toInt();
  if (String(topic) == "hh_down_from_outside") hh_down_from_outside = payload_string.toInt();
  if (String(topic) == "mm_down_from_outside") mm_down_from_outside = payload_string.toInt();
  if (String(topic) == "speed_from_outside") {
    speed_from_outside = payload_string.toInt();
    change_time = true;
  }
  if (String(topic) == "calibrate_on") {
    calibrate_on = payload_string.toInt();
    change_time = true;
  }

  if (String(topic) == "up_time_from_outside") up_time_from_outside = payload_string;  // эти данные принимаются типом стринг
  if (String(topic) == "down_time_from_outside") down_time_from_outside = payload_string;  // эти данные принимаются типом стринг
  if (String(topic) == "night_alarm_from_outside") night_alarm_from_outside = payload_string.toInt();


  if (String(topic) == "up_time_from_outside" or String(topic) == "down_time_from_outside" or String(topic) == "night_alarm_from_outside") change_time = true;


}
