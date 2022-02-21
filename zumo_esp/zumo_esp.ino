#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>

#define LSM303D_ADDR 0x1D
#define LSM303D_TEMP_OUT_L 0x05
#define LSM303D_TEMP_OUT_H 0x06
#define LSM303D_CTRL1 0x20
#define LSM303D_CTRL2 0x21
#define LSM303D_CTRL5 0x24
#define LSM303D_CTRL6 0x25
#define LSM303D_CTRL7 0x26
#define LSM303D_OUT_X_L_A 0x28

WiFiServer local_server(8888);
WiFiClient local_client;
const char *ssid = "IMURover";
const char *password = "84TnykNT";
IPAddress local_ip(192, 168, 4, 2);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
unsigned long t_last;

void setup() {
  Wire.begin();
  // disable internal pullups
  pinMode(D1, INPUT);
  pinMode(D2, INPUT);
  init_lsm303d();
  Serial.begin(115200);
  Serial.println();
  Serial.print("Setting soft-AP configuration ... ");
  Serial.println(WiFi.softAPConfig(local_ip, gateway, subnet) ? "Ready" : "Failed");
  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAP(ssid, password, 1, false, 4) ? "Ready" : "Failed");
  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());
  // start server
  local_server.begin();
  local_server.setNoDelay(true);
  Serial.println("Local server started.");
  Serial.print("Port D1 is pin number ");
  Serial.println(D1);
  t_last = millis();
  pinMode(D4, OUTPUT);
  digitalWrite(D4, LOW);
}

void init_lsm303d()
{
  // init accelerometer
  write_address(LSM303D_ADDR, LSM303D_CTRL2, 0x00);
  // config accel, all axes enabled, 50 HZ
  write_address(LSM303D_ADDR, LSM303D_CTRL1, 0x57);
  // enable temperature, disable magnetometer
  write_address(LSM303D_ADDR, LSM303D_CTRL5, 0x64);//0x98);
  // +- 4 gauss
  write_address(LSM303D_ADDR, LSM303D_CTRL6, 0x20);
  write_address(LSM303D_ADDR, LSM303D_CTRL7, 0x00);
}

void write_address(const uint8_t device, const uint8_t address, uint8_t value)
{
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t read_address(const uint8_t device, const uint8_t address)
{
  uint8_t data = 0;
  
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 8);
  
  return data;
}

int16_t read_address_16(const uint8_t dev_addr, const uint8_t reg_addr)
{
  int16_t data = 0;
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.endTransmission(false);
  Wire.requestFrom(dev_addr, 2, true);
  data = Wire.read() << 8 | Wire.read();
  return data;
}

void send_imu_data()
{
  Wire.beginTransmission(LSM303D_ADDR);
  Wire.write(LSM303D_OUT_X_L_A | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(LSM303D_ADDR, (byte)6);
  int16_t accel_x = Wire.read() | (Wire.read() << 8);
  int16_t accel_y = Wire.read() | (Wire.read() << 8);
  int16_t accel_z = Wire.read() | (Wire.read() << 8);
  
  //local_client.write(temp, 15);
  Serial.print("Accel_x: ");
  Serial.println(accel_x);
  Serial.print("Accel_y: ");
  Serial.println(accel_y);
  Serial.print("Accel_z: ");
  Serial.println(accel_z);
}

void loop() {
  // A new client available?
  if (local_server.hasClient())
  {
    
    if (!local_client.connected())
    {
      if (local_client) {
        local_client.stop();
        Serial.println("Stopping local client");
      }
      local_client = local_server.available();
      digitalWrite(D4, HIGH);
      Serial.println("Attaching local client");
    }
  }
  
  if (millis() - t_last >= 20) {
      // Send data to connected client
      if (local_client && local_client.connected())
      {
        send_imu_data();
        t_last = millis();
      }
      else {
        digitalWrite(D4, LOW);
      }
  }
}
