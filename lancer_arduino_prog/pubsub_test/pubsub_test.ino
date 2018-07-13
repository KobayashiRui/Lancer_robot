/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */
#define USE_USBCON
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>

float data_list[2] = {0,0};
ros::NodeHandle_<ArduinoHardware,1,1,16,256> nh;//pub, sub, inputbuff, outputbuff

void messageCb( const std_msgs::Int16MultiArray& toggle_msg){
  data_list[0] = toggle_msg.data[0] * 0.001;
  data_list[1] = toggle_msg.data[1] * 0.001;
}

//std_msgs::Int8MultiArray array_data;
ros::Subscriber<std_msgs::Int16MultiArray> sub("Testmessage", messageCb );
//ros::Publisher chatter("chatter", &array_data);
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void setup()
{
  //array_data.layout.dim_length=2;
  //array_data.data_length=8;
  nh.getHardware() -> setBaud(9600);
  nh.initNode();
  nh.subscribe(sub);
  //Serial.begin(115200);
  nh.advertise(chatter);
}

char data_array[100];
String data_string;
String data_list0,data_list1;
int len;
void loop()
{
   data_list0 = String(data_list[0]);
   data_list1 = String(data_list[1]);
   data_string = String(data_list0 + "," + data_list1);
   len = data_string.length()+1;
   data_string.toCharArray(data_array,len);
   str_msg.data = data_array;
  //Serial.println(data_list[0]);
  //Serial.println(data_list[1]);
  //array_data.data[0] = data_list[0];
  //array_data.data[1] = data_list[1];
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(3000);
}
