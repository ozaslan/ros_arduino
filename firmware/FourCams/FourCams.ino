#define USE_USBCON

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>

#define LED0 12
#define CAM0 11
#define LED1 10
#define CAM1 9
#define LED2 7
#define CAM2 8
#define LED3 6
#define CAM3 5

void messageCb( const std_msgs::Int16MultiArray& msg);

ros::NodeHandle  nh;
ros::Subscriber<std_msgs::Int16MultiArray> subs("arduino_trigger/data", messageCb);

std_msgs::Int64 int64_msg;
ros::Publisher heart_beat("/arduino_trigger/heart_beat", &int64_msg);

uint8_t CAMS[] = {CAM0, CAM1, CAM2, CAM3};
uint8_t LEDS[] = {LED0, LED1, LED2, LED3};

int16_t  led_exposures[4];
int16_t  cam_exposures[4];
int16_t  led_delays[4];
int16_t  cam_delays[4];
uint64_t cam_exposure_ends[4];
uint64_t led_exposure_ends[4];
int16_t  periods[4];
uint64_t nh_spin_time;


uint64_t time;

bool led_state = false;

void messageCb( const std_msgs::Int16MultiArray& msg){
  int16_t id = msg.data[0];
  periods[id] = msg.data[1];
  cam_delays[id]    = msg.data[2];
  cam_exposures[id] = msg.data[3];
  led_delays[id]    = msg.data[4];
  led_exposures[id] = msg.data[5];

  digitalWrite(13, led_state ? LOW : HIGH);
  led_state = !led_state;

  //str_msg.data = "got the message";
  //chatter.publish( &str_msg );
}

// the setup function runs once when you press reset or power the board
void setup() {
  nh.initNode();
  nh.subscribe(subs);
  nh.advertise(heart_beat);

  time = millis();
  
  for(uint8_t i = 0 ; i < 4 ; i++){
    pinMode(CAMS[i], OUTPUT);
    pinMode(LEDS[i], OUTPUT);
    digitalWrite(CAMS[i], HIGH);
    digitalWrite(LEDS[i], LOW);
    led_exposures[i] = 0;
    cam_exposures[i] = 0;
    led_delays[i] = 999;
    cam_delays[i] = 999;
    periods[i] = 1;
  }

  nh_spin_time = time;
  
  pinMode(13, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
    uint64_t dt, time, t0, t1;
    time = millis();
    for(uint8_t i = 0 ; i < 4 ; i++){
      dt = time % periods[i];
      
      t0 = time - dt + led_delays[i];
      t1 = t0 + led_exposures[i];

      if(time >= t0 && time < t1)
        digitalWrite(LEDS[i], HIGH);
      else
        digitalWrite(LEDS[i], LOW);

      t0 = time - dt + cam_delays[i];
      t1 = t0 + cam_exposures[i];

      if(time >= t0 && time < t1)
        digitalWrite(CAMS[i], HIGH);
      else
        digitalWrite(CAMS[i], LOW);
    }
    
    if(time - nh_spin_time > 100){
      int64_msg.data++;
      heart_beat.publish( &int64_msg );
      nh.spinOnce();
      nh_spin_time = time;
    }
}
