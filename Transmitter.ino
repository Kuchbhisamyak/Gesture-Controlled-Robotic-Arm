#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Sensor Pins
const int pinkie_Data = 34;
const int finger_Data = 39;
const int thumb_Data = 36;

int response_time = 30;

// MPU6050 object
Adafruit_MPU6050 mpu;

// ESP-NOW data structure
typedef struct struct_message {
  char command;
} struct_message;

struct_message myData;

// Threshold Variables
int thumb_high, thumb_low, finger_high, finger_low, pinkie_high, pinkie_low;
bool bool_calibrate = false;

// Receiver's MAC address (replace with actual address)
uint8_t receiverMAC[] = {0x94, 0xb9, 0x7e, 0xe6, 0x21, 0x00};

// Callback when data is sent
void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Could not find MPU6050!");
    while (1);
  }
  Serial.println("MPU6050 Initialized!");

  // Initialize Wi-Fi in STA mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register send callback
  esp_now_register_send_cb(onSent);

  // Add peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // Read analog values
  int pinkie = analogRead(pinkie_Data);
  int finger = analogRead(finger_Data);
  int thumb = analogRead(thumb_Data);

  // Calibrate thresholds once
  if (!bool_calibrate) {
    calibrateThresholds(thumb, finger, pinkie);
  }

  // Determine the command from thresholds
  char commandToSend = determineCommand(thumb, thumb_high, thumb_low, finger, finger_high, finger_low, pinkie, pinkie_high,pinkie_low );
  sendCommand(commandToSend);

  // Process MPU6050 acceleration data
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  processAcceleration(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);

  delay(500); // Control loop frequency
}

void calibrateThresholds(int thumb, int finger, int pinkie) {
  thumb_high = thumb * 1;
  thumb_low = thumb * 0.7;
  finger_high = finger * 1;
  finger_low = finger * 0.8;
  pinkie_high = pinkie * 1;
  pinkie_low = pinkie * 0.9;
  bool_calibrate = true;
}

char determineCommand(int thumb, int thumb_high, int thumb_low, int finger, int finger_high, int finger_low, int pinkie, int pinkie_high,int pinkie_low ) {
  if (thumb >= thumb_high && finger >= finger_high) 
  {
    return 'X';
    delay(response_time); // Both high
  } 
  else if (thumb <= thumb_low && finger <= finger_low) 
  {
    return 'Y';
    delay(response_time); // Both low
  } 

  if (thumb <= thumb_low && pinkie <= pinkie_low ) 
  {
    // message = 'C';  // Clockwise
    // esp_now_send(receiverAddress, (uint8_t *)&message, sizeof(message));
    // Serial.println("Sent: C");
    return 'C';
    delay(response_time);
  } 
  else if (pinkie <= pinkie_low && finger <=  finger_low ) 
  {
  //   message = 'A';  // Anticlockwise
  //   esp_now_send(receiverAddress, (uint8_t *)&message, sizeof(message));
  //   Serial.println("Sent: A");
    return 'A';
    delay(response_time);
  }
  else 
  {
    return 'N';
    delay(response_time); // Default no-action condition
  }
}


void processAcceleration(float accelX, float accelY, float accelZ) {
  if (accelZ < 10.8 && accelZ > 8.5)
  {
    myData.command = 'N';
    delay(response_time);
  }
  else if (accelY > -10.8 && accelY < -8.5)
  {
     myData.command = 'L';
     delay(response_time);
  }
  else if (accelY < 10.8 && accelY > 8.5)
  {
     myData.command = 'R';
     delay(response_time);
  }
  else if (accelX < -8.5 && accelZ > 1 && accelZ < 2 ) 
  {
     myData.command = 'Q';
     delay(response_time); //W
  }
  else if (accelX > 5 && accelZ > 7.7)
  {
     myData.command = 'P';
     delay(response_time); //WU
  }
  else if (accelX < -8.5 && accelZ < 1 && accelZ > -1)
  {
     myData.command = 'D';
     delay(response_time);
  }
  else if (accelX > 9 && accelZ > -1.5 && accelZ < 3)
  {
     myData.command = 'U';
     delay(response_time);
  }

  sendCommand(myData.command);
}

void sendCommand(char command) {
  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&command, sizeof(command));

  Serial.print("Sent Command: ");
  Serial.println(command);

  if (result == ESP_OK) {
    Serial.println("Sent successfully");
  } else {
    Serial.println("Error sending data");
  }
}
