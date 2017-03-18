#include<ros.h>
#include<geometry_msgs/Vector3.h>

ros::NodeHandle nh;
geometry_msgs::Vector3 data;
ros::Publisher pos("pos",&data);

long left_count=0, right_count=0;

void setup()
{
 Serial.begin(57600);
 nh.initNode();
 nh.advertise(pos);
 attachInterrupt(0,isr_right,FALLING);
 attachInterrupt(1,isr_left,FALLING);
 pinMode(11,OUTPUT);
}


void loop()
{
 digitalWrite(11,HIGH);
 data.x=left_count;
 data.y=right_count;
 Serial.print("right:   "); Serial.print(right_count);  Serial.print("left:   "); Serial.println(left_count);
 pos.publish(&data);
 nh.spinOnce();
 delay(2);
}

void isr_right()
{
  right_count++;
}

void isr_left()
{
  left_count++;
}


