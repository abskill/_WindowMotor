
void rf_control()
{

  btn_pressed_A = false;  // сброс признака нажатия кнопки
  btn_pressed_B = false;  // сброс признака нажатия кнопки

  // считываем код нажатой кнопки или 0
  if (mySwitch.available())
  {
    //yield();
    btn_value = mySwitch.getReceivedValue();
    //yield();
    mySwitch.resetAvailable();
    //yield();
    if (debug == 1) Serial.println("btn_value = " + String(btn_value));
  }
  else
  {
    btn_value = 0;
  }



  if (btn_value != 0) // если пришла команла о нажатии кнопки на пульте
  {
    if (btn_block) // если включена блокировка управления по кнопке
    {
      if (millis() - btn_timer > btn_delay) //если время блокировки истекло
      {
        btn_block = false; // снимаем блокировку
      }
    }

    if (btn_block == false) // блокировки кнопки нет
    {
      if (btn_value == btn_code_A) // если код кнопки верный
      {
        btn_block = true; // включаем блокировку кнопки
        btn_timer = millis(); // сбрасываем таймер блокировки
        if (btn_pressed_A) btn_pressed_A = false; // инвертируем статус признака нажатия кнопки
        else btn_pressed_A = true;

        if (debug == 1) Serial.println("btn_pressed_A");// + String(btn_pressed));

      }
      else
      {
        if (btn_value == btn_code_B) // если код кнопки верный
        {
          btn_block = true; // включаем блокировку кнопки
          btn_timer = millis(); // сбрасываем таймер блокировки
          if (btn_pressed_B) btn_pressed_B = false; // инвертируем статус признака нажатия кнопки
          else btn_pressed_B = true;

          if (debug == 1) Serial.println("btn_pressed_B");// + String(btn_pressed));

        }

        else
        {
          if (debug == 1) Serial.println("btn_value = unknown");
        }
      }
    }
    else
    {
      if (debug == 1) Serial.println("btn_block");
    }
  }


}
