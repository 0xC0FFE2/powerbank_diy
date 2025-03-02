#include <DFRobotDFPlayerMini.h>
#include <DHT.h>
#include <EEPROM.h>

#define NEXTION Serial2
#define MP3_SERIAL Serial1
#define BUILD_ID "NANUSWDG Prod v2.9P"

#define BAT_TEMP_PIN A0
#define LOAD_TEMP_PIN A2
#define HEATER_PIN 10
#define COOLER_PIN 11
#define BATTERY_POWER_PIN 12
#define DHTTYPE DHT11

DFRobotDFPlayerMini mp3_player;
DHT bat_temp_sensor(BAT_TEMP_PIN, DHTTYPE);
DHT load_temp_sensor(LOAD_TEMP_PIN, DHTTYPE);

bool bat_status = false;
int battery_usage = 100;
int battery_max_usage = 500;
int battery_level = 0;
int time_left = 9999;
float battery_capacity = 80.0;
float batt_voltage = 0.0;
int battery_status = 0;
bool bat_preconditioning = false;

float prev_voltage = 0.0;
unsigned long voltage_timer = 0;
const int VOLTAGE_CHECK_INTERVAL = 5000;
float voltage_samples[10] = { 0 };
int sample_index = 0;
float avg_voltage = 0.0;
const float LOAD_VOLTAGE_THRESHOLD = 0.02;
// 충전 감지 전압 임계값을 13.2V로 변경
const float CHARGING_VOLTAGE_THRESHOLD = 13.2;
const float CALCULATE_OFFSET = 0.4;

unsigned long discharge_timer = 0;
bool is_discharging = false;
int prev_battery_level = 0;

int battery_cycle = 0;
int battery_on_cycle = 0;

int sys_volume = 30;
int bat_max_usg = 50;
int bat_max_chg = 100;

int gas_sensor_pin = A1;
int voltage_pin = A3;

float bat_temp = 0.0, bat_humid = 0.0;
float load_temp = 0.0, load_humid = 0.0;
int gas_value = 0;
float voltage = 0.0;

const float MAX_VOLTAGE = 13.0;
const float MIN_VOLTAGE = 10.5;
const float VOLTAGE_RANGE = MAX_VOLTAGE - MIN_VOLTAGE;

int hot_temp_limit = 30;
int cold_temp_limit = 18;
int run_temp = 3;

String current_command = "";
bool waiting_for_value = false;

bool low_voltage_detected = false;
const float LOW_VOLTAGE_THRESHOLD = 12.50;
const float RECOVERY_VOLTAGE_THRESHOLD = 12.8;

void setup() {
  NEXTION.begin(9600);
  MP3_SERIAL.begin(9600);
  Serial.begin(9600);

  pinMode(HEATER_PIN, OUTPUT);
  pinMode(COOLER_PIN, OUTPUT);
  pinMode(BATTERY_POWER_PIN, OUTPUT);

  digitalWrite(HEATER_PIN, HIGH);
  digitalWrite(COOLER_PIN, HIGH);
  digitalWrite(BATTERY_POWER_PIN, LOW);

  load_system_settings();
  load_battery_cycle();

  bat_temp_sensor.begin();
  load_temp_sensor.begin();

  if (!mp3_player.begin(MP3_SERIAL)) {
  }
  mp3_player.volume(sys_volume);

  update_sensor_values();

  NEXTION.print("t0.txt=\"Battery Status Check..\"");
  send_command("");
  
  if (gas_value > 500 || bat_temp > 50 || load_temp > 50) {
    while (1) {
      NEXTION.print("t0.txt=\"GAS IN BOX! \n Action Required\"");
      send_command("");
      NEXTION.print("p0.pic=2");
      send_command("");
      NEXTION.print("init.bco=63618");
      send_command("");
      mp3_player.volume(35);
      mp3_player.play(7);
      delay(10000);
    }
  }
  delay(500);

  NEXTION.print("t0.txt=\"Temp Check..\"");
  send_command("");
  if ((bat_temp < cold_temp_limit || bat_temp > hot_temp_limit || bat_humid > 70) || (load_temp < cold_temp_limit || load_temp > hot_temp_limit || load_humid > 70)) {
    mp3_player.volume(35);
    mp3_player.play(8);
    delay(2000);
  }
  delay(500);

  NEXTION.print("t0.txt=\"Welcome! owo\"");
  send_command("");
  delay(200);

  for (int i = 0; i < 10; i++) {
    voltage_samples[i] = batt_voltage;
  }
  avg_voltage = batt_voltage;
  prev_voltage = batt_voltage;

  calculate_battery_level();
  prev_battery_level = battery_level;
  calculate_battery_usage();

  calculate_time_left();
  delay(500);
  NEXTION.print("page loader");
  send_command("");
}

void loop() {
  update_sensor_values();
  
  if (bat_temp > 50 || load_temp > 50 || gas_value > 500) {
    NEXTION.print("t0.txt=\"GAS IN BOX! \n Action Required\"");
    send_command("");
    NEXTION.print("p0.pic=2");
    send_command("");
    NEXTION.print("init.bco=63618");
    send_command("");
    mp3_player.volume(35);
    mp3_player.play(7);
    delay(10000);
    return;
  }

  if (NEXTION.available()) {
    String message = "";
    while (NEXTION.available()) {
      char c = NEXTION.read();
      message += c;
      delay(2);
    }

    if (waiting_for_value) {
      if (message.length() > 0) {
        int value = (int)(unsigned char)message[1];

        if (current_command == "volset") {
          mp3_player.play(9);
          sys_volume = value;
          save_system_settings();
          load_system_info();
        } else if (current_command == "batmaxset") {
          mp3_player.play(9);
          int proposed_bat_max_usg = 50 + (value / 2);
          if (proposed_bat_max_usg + 10 <= bat_max_chg) {
            bat_max_usg = proposed_bat_max_usg;
            save_system_settings();
            load_system_info();
          } else {
            bat_max_usg = bat_max_chg - 10;
            save_system_settings();
            load_system_info();
          }
        } else if (current_command == "batchgset") {
          mp3_player.play(9);
          int proposed_bat_max_chg = 50 + (value / 2);
          if (proposed_bat_max_chg >= bat_max_usg + 10) {
            bat_max_chg = proposed_bat_max_chg;
            save_system_settings();
            load_system_info();
          } else {
            bat_max_chg = bat_max_usg + 10;
            save_system_settings();
            load_system_info();
          }
        }
      }

      waiting_for_value = false;
      current_command = "";

      while (NEXTION.available()) {
        NEXTION.read();
      }
    } else {
      if (message.indexOf("volset_") != -1) {
        current_command = "volset";
        waiting_for_value = true;
        delay(200);
        while (NEXTION.available()) {
          NEXTION.read();
        }
      } else if (message.indexOf("batmaxset_") != -1) {
        current_command = "batmaxset";
        waiting_for_value = true;
        delay(200);
        while (NEXTION.available()) {
          NEXTION.read();
        }
      } else if (message.indexOf("batchgset_") != -1) {
        current_command = "batchgset";
        waiting_for_value = true;
        delay(200);
        while (NEXTION.available()) {
          NEXTION.read();
        }
      } else if (message.indexOf("sound") != -1) {
        mp3_player.play(3);
        delay(200);
      } else if (message.indexOf("warn") != -1) {
        mp3_player.play(5);
        delay(200);
      } else if (message.indexOf("pre_on") != -1) {
        bat_preconditioning = true;
        mp3_player.play(3);
        delay(200);
      } else if (message.indexOf("pre_off") != -1) {
        bat_preconditioning = false;
        mp3_player.play(3);
        delay(200);
      } else if (message.indexOf("load_bms_settting") != -1) {
        load_battery_info();
      } else if (message.indexOf("load_settting") != -1) {
        load_system_info();
      } else if (message.indexOf("main") != -1) {
        mp3_player.volume(sys_volume);
        if (bat_status) {
          NEXTION.print("page main");
        } else {
          NEXTION.print("page p_success");
          send_command("");
          delay(100);
          NEXTION.print("t3.txt=\"" + String(BUILD_ID) + "\"");
        }
        send_command("");
      } else if (message.indexOf("stop_bat") != -1) {
        run_stop_bat();
      } else if (message.indexOf("start_bat") != -1) {
        bat_status = true;
        calculate_battery_level();
        mp3_player.play(1);
        delay(500);
        NEXTION.print("usagespeed.val=0");
        send_command("");
        delay(50);
        show_alert(5, 2);
        NEXTION.print("alert.pic=7");
        send_command("");
        battery_on_cycle++;
        save_battery_cycle();
        if (bat_max_usg > battery_level) {
          run_stop_bat();
          delay(3000);
          mp3_player.play(4);
        } else {
          digitalWrite(BATTERY_POWER_PIN, HIGH);
        }
      } else if (message.indexOf("reset_bat") != -1) {
        mp3_player.play(3);
        delay(500);
        battery_on_cycle = 0;
        battery_cycle = 0;
        save_battery_cycle();
        load_battery_info();
      }
    }
  }

  if (bat_status) {
    update_main_screen();
    update_sensor_values();
    update_status_screen();
    update_battery_metrics();
    if (bat_preconditioning) {
      if ((load_temp < cold_temp_limit) && (bat_temp < cold_temp_limit)) {
        digitalWrite(HEATER_PIN, LOW);
      } else {
        digitalWrite(HEATER_PIN, HIGH);
      }
    } else {
      digitalWrite(HEATER_PIN, HIGH);
    }
    if ((load_temp > hot_temp_limit) && (bat_temp > cold_temp_limit)) {
      digitalWrite(COOLER_PIN, LOW);
    } else {
      digitalWrite(COOLER_PIN, HIGH);
    }
    if (bat_max_usg > battery_level) {
      mp3_player.play(4);
      delay(5000);
      run_stop_bat();
    }
    delay(10);
  }
}

void run_stop_bat() {
  bat_status = false;
  mp3_player.play(2);
  delay(500);
  NEXTION.print("usagespeed.val=0");
  send_command("");
  delay(50);
  digitalWrite(BATTERY_POWER_PIN, LOW);
  show_alert(6, 2);
  NEXTION.print("page p_success");
  send_command("");
}

float get_reference_voltage_for_level(int level) {
  float voltage_percent;

  if (level >= 90) {
    voltage_percent = 0.9 + (level - 90) * 0.01;
  } else if (level >= 30) {
    voltage_percent = 0.2 + (level - 30) * 0.01;
  } else {
    voltage_percent = level * 0.006667;
  }

  float reference_voltage = MIN_VOLTAGE + (voltage_percent * VOLTAGE_RANGE);
  reference_voltage = constrain(reference_voltage, MIN_VOLTAGE, MAX_VOLTAGE);

  return reference_voltage;
}

unsigned long charging_detected_time = 0;
bool was_charging = false;
bool charging_notification_played = false;

void update_battery_metrics() {
  // 전압 샘플 업데이트 및 평균 계산
  voltage_samples[sample_index] = batt_voltage;
  sample_index = (sample_index + 1) % 10;

  float sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += voltage_samples[i];
  }
  avg_voltage = sum / 10;

  // 충전 상태 단순 확인 (13.2V 이상이면 충전 중)
  bool is_charging = (avg_voltage >= CHARGING_VOLTAGE_THRESHOLD);
  
  if (is_charging && !was_charging) {
    // 충전 시작 감지
    battery_status = 1;
    was_charging = true;
    mp3_player.play(10); // 충전 알림음
  } else if (!is_charging && was_charging) {
    // 충전 종료 감지
    battery_status = 0;
    was_charging = false;
  }
  
  // 항상 배터리 레벨 계산 (전압에 따라 바로 업데이트)
  calculate_battery_level();
  
  // 배터리 사용량 및 남은 시간 계산
  calculate_battery_usage();
  calculate_time_left();
  
  // 저전압 감지 및 배터리 사이클 관리
  if (avg_voltage <= LOW_VOLTAGE_THRESHOLD && !low_voltage_detected) {
    battery_cycle++;
    low_voltage_detected = true;
    save_battery_cycle();
  } else if (avg_voltage >= RECOVERY_VOLTAGE_THRESHOLD && low_voltage_detected) {
    low_voltage_detected = false;
  }
}

void calculate_battery_level() {
  // AGM 배터리 전압-잔량 관계 (예시적인 값)
  if (avg_voltage >= 12.8) {
    battery_level = 100;
  } else if (avg_voltage >= 12.6) {
    battery_level = 90 + ((avg_voltage - 12.6) / 0.2) * 10;
  } else if (avg_voltage >= 12.3) {
    battery_level = 60 + ((avg_voltage - 12.3) / 0.3) * 30;
  } else if (avg_voltage >= 12.0) {
    battery_level = 40 + ((avg_voltage - 12.0) / 0.3) * 20;
  } else if (avg_voltage >= 11.8) {
    battery_level = 20 + ((avg_voltage - 11.8) / 0.2) * 20;
  } else if (avg_voltage >= 11.0) {
    battery_level = ((avg_voltage - 11.0) / 0.8) * 20;
  } else {
    battery_level = 0;
  }
  
  battery_level = constrain(battery_level, 0, 100);
}

float prev_battery_usage = 0;

void calculate_battery_usage() {
  // 참조 전압을 고정된 값(100% 충전 시 전압)으로 설정
  float reference_voltage = 12.7;  // 완충 상태의 전압으로 고정
  
  // 참조 전압과 현재 전압의 차이 계산
  float voltage_diff = reference_voltage - batt_voltage;
  
  // 음수 값은 0으로 처리
  float voltage_drop = (voltage_diff > 0) ? voltage_diff : 0;
  
  // 적절한 스케일링을 적용 (전압 차이가 작기 때문에 스케일링 필요)
  battery_usage = voltage_drop * 150;  // 150을 곱해 더 큰 값으로 표시
  
  // 범위 제한 
  if (battery_usage < 5) battery_usage = 5;  // 최소값 설정
  battery_usage = constrain(battery_usage, 5, battery_max_usage);
}


void calculate_time_left() {
  const float FIXED_POWER_CONSUMPTION = 20.0;  // 고정 20W 소비로 설정
  
  if (FIXED_POWER_CONSUMPTION <= 0) {
    time_left = 999;
  } else {
    float battery_wh = battery_capacity * batt_voltage;
    float usable_capacity_factor = 1 - (bat_max_usg * 0.01);
    
    // 배터리 레벨을 고려한 사용 가능한 총 와트시
    float usable_wh = battery_wh * (battery_level / 100.0) * usable_capacity_factor;
    
    // 고정 소비량으로 나누어 시간 계산
    float hours_left = usable_wh / FIXED_POWER_CONSUMPTION;
    
    // 정수로 반올림
    time_left = round(hours_left);
    time_left = constrain(time_left, 0, 999);
  }
}

void update_main_screen() {
  NEXTION.print("usage.txt=\"" + String(battery_usage) + "W\"");
  send_command("");
  delay(50);

  NEXTION.print("bat.txt=\"" + String(battery_level) + "%\"");
  send_command("");
  delay(50);

  NEXTION.print("timeleft.txt=\"" + String(time_left) + "Hr\"");
  send_command("");
  delay(50);

  NEXTION.print("volatge.txt=\"" + String(batt_voltage, 2) + "v\"");
  send_command("");
  delay(50);

  double usage_percent = (((double)battery_usage / battery_max_usage) * 20) - 30;
  usage_percent = round(usage_percent / 2) * 2;

  NEXTION.print("usagespeed.val=" + String((int)usage_percent));
  send_command("");
}

unsigned long lastUpdate = 0;
int toggle = 0;

void update_status_screen() {
  if (millis() - lastUpdate >= 3000) {
    lastUpdate = millis();

    if (toggle == 0) {
      NEXTION.print("status.txt=\"Temp: " + String(bat_temp, 1) + "C ," + String(load_temp, 1) + "C\"");
    } else if (toggle == 1) {
      NEXTION.print("status.txt=\"PreCond: " + String(bat_preconditioning ? "ON" : "OFF") + "\"");
    } else if (toggle == 2) {
      NEXTION.print("status.txt=\" " + String(battery_status ? "CHARGING" : "NOT CHARGING") + "\"");
    } else {
      NEXTION.print("status.txt=\"LIMIT1:" + String(bat_max_usg) +"%\"");
    }

    send_command("");

    toggle = (toggle + 1) % 4;
  }
}
void load_battery_info() {
  NEXTION.print("t1.txt=\"" + String(batt_voltage, 2) + "v\"");
  send_command("");
  delay(30);
  NEXTION.print("t3.txt=\"" + String(battery_cycle) + "t\"");
  send_command("");
  delay(30);
  NEXTION.print("t6.txt=\"" + String(bat_temp, 1) + "c\"");
  send_command("");
  delay(30);
  NEXTION.print("t7.txt=\"" + String(gas_value) + "\"");
  send_command("");
  delay(30);
  NEXTION.print("t10.txt=\"" + String(load_temp, 1) + "c\"");
  send_command("");
  delay(30);
  NEXTION.print("t11.txt=\"" + String(battery_on_cycle) + "t\"");
  send_command("");
  delay(30);
  NEXTION.print("t15.txt=\"" + String(battery_capacity, 1) + "\"");
  send_command("");
  delay(30);
  NEXTION.print(String("t18.txt=\"") + "H:" + String(hot_temp_limit) + "C:" + String(cold_temp_limit) + "\"");
  send_command("");
  delay(30);
  NEXTION.print(String("t19.txt=VAL\"")  + String(bat_preconditioning ? "ON" : "OFF") +  "\"");
  send_command("");
  delay(30);
}

void load_system_info() {
  NEXTION.print("vol.txt=\"" + String(sys_volume) + "%\"");
  send_command("");
  NEXTION.print("h2.val=" + String(sys_volume));
  send_command("");
  delay(30);

  NEXTION.print("batmax.txt=\"" + String(bat_max_usg) + "%\"");
  send_command("");
  int restored_bat_max_usg = (bat_max_usg - 50) * 2;
  NEXTION.print("h0.val=" + String(restored_bat_max_usg));
  send_command("");
  delay(30);

  NEXTION.print("batchg.txt=\"" + String(bat_max_chg) + "%\"");
  send_command("");
  int restored_bat_max_chg = (bat_max_chg - 50) * 2;
  NEXTION.print("h1.val=" + String(restored_bat_max_chg));
  send_command("");
  delay(30);
}

void show_alert(int pic1, int pic2) {
  for (int i = 0; i < 4; i++) {
    NEXTION.print("alert.pic=" + String(pic1));
    send_command("");
    delay(500);
    NEXTION.print("alert.pic=" + String(pic2));
    send_command("");
    delay(300);
  }
}

void load_battery_cycle() {
  int cycle = EEPROM.read(0);
  battery_cycle = (cycle == 255) ? 0 : cycle;

  int on_cycle = EEPROM.read(1);
  battery_on_cycle = (on_cycle == 255) ? 0 : on_cycle;
}

void save_battery_cycle() {
  EEPROM.write(0, battery_cycle);
  EEPROM.write(1, battery_on_cycle);
}

void load_system_settings() {
  const int DEFAULT_VOLUME = 30;
  const int DEFAULT_BAT_MAX_USG = 50;
  const int DEFAULT_BAT_MAX_CHG = 100;

  int volume = EEPROM.read(2);
  sys_volume = (volume == 255) ? DEFAULT_VOLUME : volume;

  int max_usg = EEPROM.read(3);
  bat_max_usg = (max_usg == 255) ? DEFAULT_BAT_MAX_USG : max_usg;

  int max_chg = EEPROM.read(4);
  bat_max_chg = (max_chg == 255) ? DEFAULT_BAT_MAX_CHG : max_chg;

  sys_volume = constrain(sys_volume, 0, 100);
  bat_max_usg = constrain(bat_max_usg, 0, 100);
  bat_max_chg = constrain(bat_max_chg, 0, 100);
}

void save_system_settings() {
  EEPROM.write(2, sys_volume);
  EEPROM.write(3, bat_max_usg);
  EEPROM.write(4, bat_max_chg);
}

void send_command(String cmd) {
  NEXTION.write(0xFF);
  NEXTION.write(0xFF);
  NEXTION.write(0xFF);
  delay(5);
}

void update_sensor_values() {
  bat_temp = bat_temp_sensor.readTemperature();
  bat_humid = bat_temp_sensor.readHumidity();

  load_temp = load_temp_sensor.readTemperature();
  load_humid = load_temp_sensor.readHumidity();

  if (isnan(bat_temp) || isnan(bat_humid)) {
    Serial.println("Failed to read from bat_temp_sensor!");
  }
  if (isnan(load_temp) || isnan(load_humid)) {
    Serial.println("Failed to read from load_temp_sensor!");
  }

  gas_value = analogRead(gas_sensor_pin);

  voltage = analogRead(voltage_pin) * (5.0 / 1023.0) * 4.3;
  batt_voltage = voltage;
}
