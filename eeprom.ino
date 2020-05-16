/*
  if (EEPROM.commit()) {
      Serial.println("EEPROM successfully committed");
    } else {
      Serial.println("ERROR! EEPROM commit failed");
    }
*/




// functions

void eeprom_init() // инициализация EEPROM (чтение или запись, в зависимости от значения уникального ключа)
{
  byte addr = 0;
  init_key_address = addr; addr += sizeof(init_key_req);
  night_alarm_address = addr; addr += sizeof(night_alarm_address);
  hh_down_address = addr; addr += sizeof(hh_down);
  mm_down_address = addr; addr += sizeof(mm_down);
  hh_up_address = addr; addr += sizeof(hh_up);
  mm_up_address = addr; addr += sizeof(mm_up);
  motor_man_speed_address = addr; addr += sizeof(motor_man_speed);
  max_rotating_time_address = addr; addr += (int)sizeof(max_rotating_time);
  calibrated_speed_address = addr; addr += sizeof(calibrated_speed);

  EEPROM.begin(512);

  EEPROM.get(init_key_address, init_key_fact); // Прочитаем ключ, хранящийся в EEPROM на момент включения питания

  if (init_key_req == init_key_fact)
  {
    eeprom_read(); // читаем данные из EEPROM

    hh_up_from_outside = hh_up;
    mm_up_from_outside = mm_up;
    hh_down_from_outside = hh_down;
    mm_down_from_outside = mm_down;
    speed_from_outside = motor_man_speed;
    night_alarm_from_outside = night_alarm;
  }
  else {
    if (debug == 1) Serial.println("Init EEPROM... (readed key is " + String(init_key_fact) + ")");
    EEPROM.put(init_key_address, init_key_req); // запишем в EEPROM ключ
    EEPROM.commit();
    eeprom_update(); // запишем в EEPROM остальные даннные
    if (debug == 1) eeprom_read();
  }
}


void eeprom_read() // чтение данных из EEPROM
{
  //EEPROM.get(OTA_on_address,  OTA_on);
  //EEPROM.get(MQTT_on_address, MQTT_on);
  //EEPROM.get(WIFI_on_address, WIFI_on);
  //EEPROM.get(RF_on_address,   RF_on);

  EEPROM.get(night_alarm_address, night_alarm);
  EEPROM.get(hh_down_address, hh_down);
  EEPROM.get(mm_down_address, mm_down);
  EEPROM.get(hh_up_address,   hh_up);
  EEPROM.get(mm_up_address,   mm_up);

  EEPROM.get(motor_man_speed_address, motor_man_speed);

  EEPROM.get(max_rotating_time_address, max_rotating_time);
  EEPROM.get(calibrated_speed_address, calibrated_speed);

  if (debug == 1) Serial.println("Reading EEPROM...");
  if (debug == 1) Serial.println("  night_alarm = " + String(night_alarm));
  if (debug == 1) Serial.println("  hh_down = " + String(hh_down));
  if (debug == 1) Serial.println("  mm_down = " + String(mm_down));
  if (debug == 1) Serial.println("  hh_up = " + String(hh_up));
  if (debug == 1) Serial.println("  mm_up = " + String(mm_up));
  if (debug == 1) Serial.println("  motor_man_speed = " + String(motor_man_speed));
  if (debug == 1) Serial.println("  max_rotating_time = " + String(max_rotating_time));
  if (debug == 1) Serial.println("  calibrated_speed = " + String(calibrated_speed));
  if (debug == 1) Serial.println();

}



void eeprom_update() // запись данных в EEPROM
{
  if (debug == 1) Serial.print("Updating EEPROM...");
  //EEPROM.put(OTA_on_address,  OTA_on);
  //EEPROM.put(MQTT_on_address, MQTT_on);
  //EEPROM.put(WIFI_on_address, WIFI_on);
  //EEPROM.put(RF_on_address,   RF_on);

  EEPROM.put(night_alarm_address, night_alarm);
  EEPROM.put(hh_down_address, hh_down);
  EEPROM.put(mm_down_address, mm_down);
  EEPROM.put(hh_up_address,   hh_up);
  EEPROM.put(mm_up_address,   mm_up);

  EEPROM.put(motor_man_speed_address, motor_man_speed);

  EEPROM.put(max_rotating_time_address, max_rotating_time);
  EEPROM.put(calibrated_speed_address, calibrated_speed);

  EEPROM.commit();
  if (debug == 1) Serial.println("ok");

  request_eeprom_update = false; // сброс признака необходимости записи в eeprom
}
