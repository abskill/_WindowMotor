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
  // Задаем адреса
  byte addr = 0;
  init_key_address = addr; addr += sizeof(init_key_req);
  time_command_address = addr; addr += sizeof(time_command_address);
  hh_down_address = addr; addr += sizeof(hh_down);
  mm_down_address = addr; addr += sizeof(mm_down);
  hh_up_address = addr; addr += sizeof(hh_up);
  mm_up_address = addr; addr += sizeof(mm_up);
  motor_speed_address = addr; addr += sizeof(motor_speed);
  max_rotating_time_address = addr; addr += (int)sizeof(max_rotating_time);
  calibrated_speed_address = addr; addr += sizeof(calibrated_speed);

  hh_down_weekend_address = addr; addr += sizeof(hh_down_weekend);
  mm_down_weekend_address = addr; addr += sizeof(mm_down_weekend);
  hh_up_weekend_address = addr; addr += sizeof(hh_up_weekend);
  mm_up_weekend_address = addr; addr += sizeof(mm_up_weekend);

  EEPROM.begin(512);

  EEPROM.get(init_key_address, init_key_fact); // Прочитаем ключ, хранящийся в EEPROM на момент включения питания

  if (init_key_req == init_key_fact) // если ключ валидный, то читаем данные
  {
    eeprom_read();

    hh_up_from_outside = hh_up;
    mm_up_from_outside = mm_up;
    hh_down_from_outside = hh_down;
    mm_down_from_outside = mm_down;
    motor_speed_in = motor_speed;
    time_command_in = time_command;

    hh_up_weekend_from_outside = hh_up_weekend;
    mm_up_weekend_from_outside = mm_up_weekend;
    hh_down_weekend_from_outside = hh_down_weekend;
    mm_down_weekend_from_outside = mm_down_weekend;
  }
  else // А если ключ невалидный, то инициализируем данные в EEPROM начальными значениями
  {
    if (debug) Serial.println("Init EEPROM... (readed key is " + String(init_key_fact) + ")");
    EEPROM.put(init_key_address, init_key_req); // запишем в EEPROM ключ
    EEPROM.commit();
    eeprom_update(); // запишем в EEPROM остальные даннные
    if (debug) eeprom_read();
  }
}


void eeprom_read() // чтение данных из EEPROM
{
  //EEPROM.get(OTA_on_address,  OTA_on);
  //EEPROM.get(MQTT_on_address, MQTT_on);
  //EEPROM.get(WIFI_on_address, WIFI_on);
  //EEPROM.get(RF_on_address,   RF_on);

  EEPROM.get(time_command_address, time_command);
  EEPROM.get(hh_down_address, hh_down);
  EEPROM.get(mm_down_address, mm_down);
  EEPROM.get(hh_up_address,   hh_up);
  EEPROM.get(mm_up_address,   mm_up);

  EEPROM.get(motor_speed_address, motor_speed);

  EEPROM.get(max_rotating_time_address, max_rotating_time);
  EEPROM.get(calibrated_speed_address, calibrated_speed);

  EEPROM.get(hh_down_weekend_address, hh_down_weekend);
  EEPROM.get(mm_down_weekend_address, mm_down_weekend);
  EEPROM.get(hh_up_weekend_address,   hh_up_weekend);
  EEPROM.get(mm_up_weekend_address,   mm_up_weekend);


  if (debug) Serial.println("Reading EEPROM -------------");
  if (debug) Serial.println("  time_command = " + String(time_command));
  if (debug) Serial.println("");
  if (debug) Serial.println("  hh_down = " + String(hh_down));
  if (debug) Serial.println("  mm_down = " + String(mm_down));
  if (debug) Serial.println("  hh_up = " + String(hh_up));
  if (debug) Serial.println("  mm_up = " + String(mm_up));
  if (debug) Serial.println("");
  if (debug) Serial.println("  hh_down_weekend = " + String(hh_down_weekend));
  if (debug) Serial.println("  mm_down_weekend = " + String(mm_down_weekend));
  if (debug) Serial.println("  hh_up_weekend = " + String(hh_up_weekend));
  if (debug) Serial.println("  mm_up_weekend = " + String(mm_up_weekend));
  if (debug) Serial.println("");
  if (debug) Serial.println("  motor_speed = " + String(motor_speed));
  if (debug) Serial.println("  max_rotating_time = " + String(max_rotating_time));
  if (debug) Serial.println("  calibrated_speed = " + String(calibrated_speed));
  if (debug) Serial.println("----------------------------");
}



void eeprom_update() // запись данных в EEPROM
{
  if (debug) Serial.print("Updating EEPROM...");

  EEPROM.put(time_command_address, time_command);

  EEPROM.put(hh_down_address, hh_down);
  EEPROM.put(mm_down_address, mm_down);
  EEPROM.put(hh_up_address,   hh_up);
  EEPROM.put(mm_up_address,   mm_up);

  EEPROM.put(hh_down_weekend_address, hh_down_weekend);
  EEPROM.put(mm_down_weekend_address, mm_down_weekend);
  EEPROM.put(hh_up_weekend_address,   hh_up_weekend);
  EEPROM.put(mm_up_weekend_address,   mm_up_weekend);

  EEPROM.put(motor_speed_address, motor_speed);

  EEPROM.put(max_rotating_time_address, max_rotating_time);
  EEPROM.put(calibrated_speed_address, calibrated_speed);

  EEPROM.commit(); // необходимо для ESP8266

  if (debug) Serial.println("ok");

  //request_eeprom_update = false; // сброс признака необходимости записи в eeprom
}
