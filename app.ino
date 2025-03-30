#include <DFRobotDFPlayerMini.h>
#include <DHT.h>
#include <EEPROM.h>

#define MP3_SERIAL Serial1
#define BUILD_ID "NANUSWDG Prod v3.0P"

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

float voltage_samples[10] = { 0 };
int sample_index = 0;
float avg_voltage = 0.0;
const float CHARGING_VOLTAGE_THRESHOLD = 13.2;

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

bool low_voltage_detected = false;
const float LOW_VOLTAGE_THRESHOLD = 12.50;
const float RECOVERY_VOLTAGE_THRESHOLD = 12.8;

unsigned long last_battery_notification = 0;
const unsigned long BATTERY_NOTIFICATION_INTERVAL = 300000;

void setup() {
  MP3_SERIAL.begin(9600);
  Serial.begin(9600);

  pinMode(HEATER_PIN, OUTPUT);
  pinMode(COOLER_PIN, OUTPUT);
  pinMode(BATTERY_POWER_PIN, OUTPUT);

  digitalWrite(HEATER_PIN, HIGH);
  digitalWrite(COOLER_PIN, HIGH);
  digitalWrite(BATTERY_POWER_PIN, LOW);

  load_battery_cycle();

  bat_temp_sensor.begin();
  load_temp_sensor.begin();

  if (!mp3_player.begin(MP3_SERIAL)) {
    Serial.println("MP3 모듈 초기화 실패");
  }
  mp3_player.volume(sys_volume);

  update_sensor_values();

  if (gas_value > 500 || bat_temp > 50 || load_temp > 50) {
    while (1) {
      Serial.println("경고: 가스 또는 고온 감지!");
      mp3_player.volume(35);
      mp3_player.play(7);
      delay(10000);
    }
  }

  if ((bat_temp < cold_temp_limit || bat_temp > hot_temp_limit || bat_humid > 70) || (load_temp < cold_temp_limit || load_temp > hot_temp_limit || load_humid > 70)) {
    mp3_player.volume(35);
    mp3_player.play(8);
    delay(2000);
  }

  mp3_player.play(3);
  delay(500);

  for (int i = 0; i < 10; i++) {
    voltage_samples[i] = batt_voltage;
  }
  avg_voltage = batt_voltage;

  calculate_battery_level();
  calculate_battery_usage();
  calculate_time_left();

  bat_status = true;
  digitalWrite(BATTERY_POWER_PIN, HIGH);
  battery_on_cycle++;
  save_battery_cycle();
  mp3_player.play(1);
  delay(4000);

  notify_battery_level();
}

void loop() {
  update_sensor_values();

  if (bat_temp > 50 || load_temp > 50 || gas_value > 500) {
    Serial.println("경고: 가스 또는 고온 감지!");
    mp3_player.volume(35);
    mp3_player.play(7);
    delay(2000);
    return;
  }

  if (bat_status) {
    update_battery_metrics();

    if (millis() - last_battery_notification >= BATTERY_NOTIFICATION_INTERVAL) {
      notify_battery_level();
      last_battery_notification = millis();
    }

    if (bat_preconditioning) {
      if ((load_temp < cold_temp_limit) && (bat_temp < cold_temp_limit)) {
        digitalWrite(HEATER_PIN, LOW);
      } else {
        digitalWrite(HEATER_PIN, HIGH);
      }
    } else {
      digitalWrite(HEATER_PIN, HIGH);
    }

    if ((load_temp > hot_temp_limit) || (bat_temp > hot_temp_limit)) {
      digitalWrite(COOLER_PIN, LOW);
    } else {
      digitalWrite(COOLER_PIN, HIGH);
    }

    if (bat_max_usg > battery_level) {
      mp3_player.play(4);
      delay(2000);
      run_stop_bat();
    }

    delay(10);
  }
}

void notify_battery_level() {
  Serial.print("배터리 잔량: ");
  Serial.print(battery_level);
  Serial.println("%");

  int beep_count = 0;

  if (battery_level >= 100) {
    beep_count = 10;
  } else {
    beep_count = battery_level / 10;
  }

  if (beep_count < 1) beep_count = 1;

  Serial.print("알림음 횟수: ");
  Serial.println(beep_count);

  for (int i = 0; i < beep_count; i++) {
    mp3_player.play(3);
    delay(800);
  }
}

void run_stop_bat() {
  bat_status = false;
  mp3_player.play(9);
  delay(2000);

  mp3_player.play(2);
  delay(2000);
  digitalWrite(BATTERY_POWER_PIN, LOW);
}

bool was_charging = false;

void update_battery_metrics() {
  voltage_samples[sample_index] = batt_voltage;
  sample_index = (sample_index + 1) % 10;

  float sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += voltage_samples[i];
  }
  avg_voltage = sum / 10;

  bool is_charging = (avg_voltage >= CHARGING_VOLTAGE_THRESHOLD);

  if (is_charging && !was_charging) {
    battery_status = 1;
    was_charging = true;
    mp3_player.play(10);
    delay(2000);
  } else if (!is_charging && was_charging) {
    battery_status = 0;
    was_charging = false;
  }

  calculate_battery_level();
  calculate_battery_usage();
  calculate_time_left();

  if (avg_voltage <= LOW_VOLTAGE_THRESHOLD && !low_voltage_detected) {
    battery_cycle++;
    low_voltage_detected = true;
    save_battery_cycle();
  } else if (avg_voltage >= RECOVERY_VOLTAGE_THRESHOLD && low_voltage_detected) {
    low_voltage_detected = false;
  }

  Serial.print("전압: ");
  Serial.print(avg_voltage);
  Serial.print("V, 배터리: ");
  Serial.print(battery_level);
  Serial.print("%, 사용량: ");
  Serial.print(battery_usage);
  Serial.print("W, 남은시간: ");
  Serial.print(time_left);
  Serial.println("Hr");
}

void calculate_battery_level() {
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

void calculate_battery_usage() {
  float reference_voltage = 12.7;
  float voltage_diff = reference_voltage - batt_voltage;
  float voltage_drop = (voltage_diff > 0) ? voltage_diff : 0;
  battery_usage = voltage_drop * 150;

  if (battery_usage < 5) battery_usage = 5;
  battery_usage = constrain(battery_usage, 5, battery_max_usage);
}

void calculate_time_left() {
  const float FIXED_POWER_CONSUMPTION = 20.0;

  if (FIXED_POWER_CONSUMPTION <= 0) {
    time_left = 999;
  } else {
    float battery_wh = battery_capacity * batt_voltage;
    float usable_capacity_factor = 1 - (bat_max_usg * 0.01);
    float usable_wh = battery_wh * (battery_level / 100.0) * usable_capacity_factor;
    float hours_left = usable_wh / FIXED_POWER_CONSUMPTION;
    time_left = round(hours_left);
    time_left = constrain(time_left, 0, 999);
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

void update_sensor_values() {
  bat_temp = bat_temp_sensor.readTemperature();
  bat_humid = bat_temp_sensor.readHumidity();

  load_temp = load_temp_sensor.readTemperature();
  load_humid = load_temp_sensor.readHumidity();

  if (isnan(bat_temp) || isnan(bat_humid)) {
    Serial.println("배터리 온도 센서 읽기 실패!");
  }
  if (isnan(load_temp) || isnan(load_humid)) {
    Serial.println("부하 온도 센서 읽기 실패!");
  }

  gas_value = analogRead(gas_sensor_pin);

  voltage = analogRead(voltage_pin) * (5.0 / 1023.0) * 4.85;
  batt_voltage = voltage;
}
