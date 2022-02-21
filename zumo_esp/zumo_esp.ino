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

#define L3GD20H_ADDR 0x6B
#define L3GD20H_OUT_TEMP 0x26 
#define L3GD20H_LOW_ODR 0x39
#define L3GD20H_CTRL1 0x20
#define L3GD20H_CTRL2 0x21
#define L3GD20H_CTRL3 0x22
#define L3GD20H_CTRL4 0x23
#define L3GD20H_CTRL5 0x24
#define L3GD20H_OUT_X_L 0x28

#define RIGHT_XOR D6
#define LEFT_XOR D7
#define RIGHT_B D3
#define LEFT_B D5

WiFiServer local_server(8888);
WiFiClient local_client;

const char *ssid = "calcrtd";
const char *password = "inexnxpu";

unsigned long t_last;
unsigned long packet_count = 0;

static volatile bool lastLeftA;
static volatile bool lastLeftB;
static volatile bool lastRightA;
static volatile bool lastRightB;

static volatile uint32_t countLeft;
static volatile uint32_t countRight;
static volatile uint32_t lastLeftTime;
static volatile uint32_t lastRightTime;
static volatile uint32_t leftSpeed;
static volatile uint32_t rightSpeed;

uint32_t agm_count = 0;

void ICACHE_RAM_ATTR ISRLeftEncoder()
{
  bool B = digitalRead(LEFT_B);
  bool A = digitalRead(LEFT_XOR) ^ B;
  countLeft += (A ^ lastLeftB) - (lastLeftA ^ B);
  lastLeftA = A;
  lastLeftB = B;
  uint32_t trig_time = micros();
  leftSpeed = trig_time - lastLeftTime;
  lastLeftTime = trig_time;
}

void ICACHE_RAM_ATTR ISRRightEncoder()
{
  bool B = digitalRead(RIGHT_B);
  bool A = digitalRead(RIGHT_XOR) ^ B;
  countRight += (A ^ lastRightB) - (lastRightA ^ B);
  lastRightA = A;
  lastRightB = B;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2000);
  Serial.begin(115200);
  Serial.println();
  Serial.print("Configuring access point...");
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  
  Wire.begin();
  // disable internal pullups
  pinMode(D1, INPUT);
  pinMode(D2, INPUT);
  // encoder inputs
  pinMode(RIGHT_XOR, INPUT_PULLUP);
  pinMode(LEFT_XOR, INPUT_PULLUP);
  pinMode(RIGHT_B, INPUT);
  pinMode(LEFT_B, INPUT_PULLUP);

  init_lsm303d();
  init_l3gd20h(); 
 
  // start server
  local_server.begin();
  local_server.setNoDelay(true);
  Serial.println("Local server started.");
  Serial.print("Port D1 is pin number ");
  Serial.println(D1);
  t_last = millis();
  pinMode(D4, OUTPUT);
  digitalWrite(D4, LOW);

  // set counts nonzero to prevent overflow
  countLeft = 100;
  countRight = 100;
  // enable interrupts
  attachInterrupt(RIGHT_XOR, ISRRightEncoder, CHANGE);
  attachInterrupt(LEFT_XOR, ISRLeftEncoder, CHANGE);
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

void init_l3gd20h()
{
  // low speed ODR enabled
  write_address(L3GD20H_ADDR, L3GD20H_LOW_ODR, 0x01);
  // 500 dps
  write_address(L3GD20H_ADDR, L3GD20H_CTRL4, 0x10);
  // 50 HZ mode, no cut-off
  //write_address(L3GD20H_ADDR, L3GD20H_CTRL1, 0x4F);
  // 50 Hz mode 16.6 Hz cutt-off
  write_address(L3GD20H_ADDR, L3GD20H_CTRL1, 0x8F);
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
  /* Obtain accel data */
  Wire.beginTransmission(LSM303D_ADDR);
  Wire.write(LSM303D_OUT_X_L_A | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(LSM303D_ADDR, (byte)6);
  uint32_t timestamp = micros();
  int16_t accel_x = Wire.read() | (Wire.read() << 8);
  int16_t accel_y = Wire.read() | (Wire.read() << 8);
  int16_t accel_z = Wire.read() | (Wire.read() << 8);
  
  /* Obtain gyro data */
  Wire.beginTransmission(L3GD20H_ADDR);
  Wire.write(L3GD20H_OUT_X_L | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(L3GD20H_ADDR, (byte)6);
  int16_t gyro_x = Wire.read() | (Wire.read() << 8);
  int16_t gyro_y = Wire.read() | (Wire.read() << 8);
  int16_t gyro_z = Wire.read() | (Wire.read() << 8);
  
  char temp[80];
  packet_count++;
  sprintf(temp,
          "%12lu %8lu %8lu %6d %6d %6d %6d %6d %6d %6lu\n",
          timestamp,
          countLeft,
          countRight,
          accel_x,
          accel_y,
          accel_z,
          gyro_x,
          gyro_y,
          gyro_z,
          packet_count);

  local_client.write(temp);
}

void send_agm_packet()
{
  char message[40];
  uint32_t timestamp = micros();
  sprintf(message, "agm %12lu %8lu\n", timestamp, agm_count);
  agm_count++;
  local_client.write(message);
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
      Serial.println("Attaching local client");
    }
  }
  
  if (millis() - t_last >= 17) {
      // Send data to connected client
      if (local_client && local_client.connected())
      {
        send_imu_data();
        t_last = millis();
      }
  }
  if (Serial.available())
  {
    char c = Serial.read();
    if (c == '@') {
      send_agm_packet();
    }
  }
}
