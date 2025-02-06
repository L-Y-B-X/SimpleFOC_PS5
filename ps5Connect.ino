#include <ps5Controller.h>
#include <SimpleFOC.h>

// Define Motors
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

// Target speed variable
float target_velocity = 0;
uint32_t prev_millis;

// Voltage threshold
#define UNDERVOLTAGE_THRES 11.1

// Serial Command
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}

// Function Prototypes
void board_check();
float get_vin_Volt();
void board_init();
bool flag_under_voltage = false;

void setup() {
  Serial.begin(115200);
  ps5.begin("bc:c7:46:2f:f3:c1");  // Replace with your controller MAC address
  Serial.println("PS5 Controller Ready.");

  board_init();

  // Initialize drivers
  driver.voltage_power_supply = get_vin_Volt();
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_limit = 2;   
  motor.velocity_limit = 30;  

  driver1.voltage_power_supply = get_vin_Volt();
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.voltage_limit = 2;    
  motor1.velocity_limit = 30;  

  // Open-loop velocity control
  motor.controller = MotionControlType::velocity_openloop;
  motor1.controller = MotionControlType::velocity_openloop;

  motor.init();
  motor1.init();

  // Add serial command
  command.add('T', doTarget, "target velocity");

  Serial.println("Motors Ready!");
}

unsigned long lastSerialCheck = 0;  // 记录上一次串口检测时间

void loop() {
  // PS5 手柄控制逻辑
  if (ps5.isConnected()) {
    int leftStickY = ps5.LStickY();  // -127 到 127

    // 设置一个小的死区，防止摇杆小幅度抖动时影响速度
    if (abs(leftStickY) < 10) {
      target_velocity = 0;
    } else {
      target_velocity = map(leftStickY, -127, 127, -20, 20);
    }

    // 立即应用速度
    motor.move(target_velocity);
    motor1.move(target_velocity);

    Serial.printf("Motor Speed: %.2f rad/s\n", target_velocity);
  }

  // 处理低电压保护
  board_check();

  // 串口命令检测，每 100ms 处理一次，不阻塞主循环
  if (millis() - lastSerialCheck > 100) {
    lastSerialCheck = millis();
    if (!flag_under_voltage)
      command.run();
  }
}


void board_init() {
  pinMode(32, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);
  pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);

  analogReadResolution(12);  

  float VIN_Volt = get_vin_Volt();
  while (VIN_Volt <= UNDERVOLTAGE_THRES) {
    VIN_Volt = get_vin_Volt();
    delay(100);
    Serial.printf("Waiting for power, current voltage: %.2fV\n", VIN_Volt);
  }
  Serial.printf("Calibrating motors... Current voltage: %.2fV\n", VIN_Volt);
}

float get_vin_Volt() {
  return analogReadMilliVolts(13) * 8.5 / 1000;
}

void board_check() {
  uint32_t curr_millis = millis();
  static uint8_t enableState = 0;

  if (curr_millis - prev_millis >= 1000) {
    float vin_Volt = get_vin_Volt();

    if (vin_Volt < UNDERVOLTAGE_THRES) {
      flag_under_voltage = true;
      enableState = 0;
      uint8_t count = 5;
      while (count--) {
        vin_Volt = get_vin_Volt();
        if (vin_Volt > UNDERVOLTAGE_THRES) {
          flag_under_voltage = false;
          break;
        }
      }
    } else {
      flag_under_voltage = false;
    }
    if (flag_under_voltage) {
      motor.disable();
      motor1.disable();
    } else if (0 == enableState && flag_under_voltage == false) {
      enableState = 1;
      motor.enable();
      motor1.enable();
    }
    prev_millis = curr_millis;
  }
}
