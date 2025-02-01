#include <SCServo.h>
#include <ArduinoJson.h>  // Include the ArduinoJson library


SMS_STS st;

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19
float base;
float r_shoulder;
float l_shoulder;
float elbow;
float hand;
float base_temp;
float r_shoulder_temp;
float l_shoulder_temp;
float elbow_temp;
float hand_temp;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(1000);
}

void loop()
{
  st.EnableTorque(11, 0);
  st.EnableTorque(12, 0);
  st.EnableTorque(13, 0);
  st.EnableTorque(14, 0);
  st.EnableTorque(15, 0);

  base_temp = st.ReadPos(11);
  r_shoulder_temp = st.ReadPos(12);
  l_shoulder_temp = st.ReadPos(13);
  elbow_temp = st.ReadPos(14);
  hand_temp = st.ReadPos(15);
  
  base = float(base_temp*0.00153435536 - M_PI);
  l_shoulder = float((l_shoulder_temp-980)*(M_PI)/(3115-980) -M_PI/2);
  elbow = float((elbow_temp-325)*(M_PI+1)/2730 - 1);
  hand = float((hand_temp-351)*1.5/1774);
  String outputLine = String(base,5) + "," + String(l_shoulder,5) + "," + String(elbow,5) + "," + String(hand,5);
  
  Serial.println(outputLine);
  
  delay(200);// Add a delay to avoid spamming the serial output
}
