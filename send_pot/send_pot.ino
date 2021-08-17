#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <stdlib.h>

ros::NodeHandle  nh;

std_msgs::Int16MultiArray pot_array, joysw_state;
ros::Publisher send_pot("send_pot", &pot_array);
// Analog
const int pot_num = 7;
const int JOYX = 8;
const int JOYY = 9;
// Digital
int iotopin[8] = {22, 23, 24, 25, 26, 27, 28, 29};
const int JOYSW = 30;
const int PUSHSW = 31;
// active low         A     B     C     D     E     F     G     H
int mode[6][8] = {{HIGH,  LOW,  LOW,  LOW,  LOW,  LOW,  LOW,  LOW}, //mode0 : ALL
                  {HIGH,  LOW,  LOW,  LOW,  LOW, HIGH, HIGH, HIGH}, //mode1 : fingers flex
                  {HIGH,  LOW,  LOW, HIGH, HIGH, LOW, HIGH, HIGH}, //mode2 : palm flex
                  {HIGH,  LOW,  LOW, HIGH, HIGH, HIGH,  LOW, HIGH}, //mode3 : thumb rot.
                  {HIGH,  LOW,  LOW, HIGH,  LOW,  LOW, HIGH, HIGH}, //mode4 : pick by thumb and index finger
                  { LOW, HIGH,  LOW,  LOW, HIGH,  LOW, HIGH, HIGH}  //mode5 : for keep grasp function
                  };

int pot_offset[3][pot_num] = {{725,515,950,820,150,855,150}, //min angle
                              {-1 ,1  ,-1 ,-1 ,1  ,-1 ,1  }, //sign
                              {550,780,560,540,485,555,480}}; //max angle

void modeChangeCb(const std_msgs::Int16& mode_msg){
  nh.loginfo("\n");
  char str[8];
  itoa(mode_msg.data, str, 10);
  nh.loginfo(str);
  for(int i = 0; i < 8; i++){
    itoa(mode[mode_msg.data][i], str, 10);
    nh.loginfo(str); // print high or low
    digitalWrite(iotopin[i], mode[mode_msg.data][i]); 
    delay(10);
  }
}

ros::Subscriber<std_msgs::Int16> mode_changer("mode_setter", modeChangeCb);

void setup()
{
//  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(send_pot);
  nh.subscribe(mode_changer);
  pot_array.data = (int16_t*)malloc(sizeof(int16_t) * (pot_num + 4));
  pot_array.data_length = pot_num + 4;

  for(int i = 0; i < 8; i++){
    pinMode(iotopin[i], OUTPUT);
    digitalWrite(iotopin[i], HIGH);
  }
  pinMode(JOYSW, INPUT_PULLUP);
  pinMode(PUSHSW, INPUT_PULLUP);
}

void loop()
{
  for(int i = 0; i < pot_num; i++){
    pot_array.data[i] = 100*(float)(analogRead(i)-pot_offset[0][i])/(float)(pot_offset[2][i]-pot_offset[0][i]);
  }
  pot_array.data[pot_num] = analogRead(JOYX);
  pot_array.data[pot_num+1] = analogRead(JOYY);
  pot_array.data[pot_num+2] = digitalRead(JOYSW);
  pot_array.data[pot_num+3] = digitalRead(PUSHSW);
  send_pot.publish(&pot_array);
  nh.spinOnce();
  delay(50);
}
