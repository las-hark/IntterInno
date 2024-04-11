#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>//WiFi
#include <PubSubClient.h> //MQTT
#include <ArduinoJson.h>
#include <Ticker.h>
#include <Adafruit_NeoPixel.h>
#include <MPU9250_asukiaaa.h>
#include <HTTPClient.h>
#include <driver/i2s.h>
#include <SPIFFS.h>
#include "Audio.h" // I2S的音乐库

#define ENV_SENSOR         7
#define ENV_STRIP          6
#define WEA_MEA_STRIP      8

#define MEA_LED           16
#define MOTOR             38

#define SPK_I2S_DOUT      14
#define SPK_I2S_BCLK      42
#define SPK_I2S_LRC       41

#define I2S_WS            15
#define I2S_SD            13
#define I2S_SCK            2
#define MICRO_ENABLE      20
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE   (16000)
#define I2S_SAMPLE_BITS   (16)
#define I2S_READ_LEN      (16 * 1024)
#define RECORD_TIME       (12) //Seconds
#define I2S_CHANNEL_NUM   (1)
#define FLASH_RECORD_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * I2S_SAMPLE_BITS / 8 * RECORD_TIME)
int flag = 1;
File file;
const char filename[] = "/recording.wav";
const int headerSize = 44; // wave header size

#define MOVEMENT_SENSOR_SDA 10
#define MOVEMENT_SENSOR_SCL  9

#define FAN                 38

//#define JDQ 16

#define PLAY_SPRING         18

//#define RELAY               38

#define WS2812_NUM 13                  //灯的数量
#define Bright  254                     //灯珠初始亮度


//--------------*关键参数初始化*-----------------//
//wifi
const char *ssid = "Galaxy S23 74B6";//"TP-Link_029B";//"PassengerONtheEve";//"TP-Link_029B";"NOEN";//wifi名
const char *password = "aaaaaaaa";//"86180627";//"wjnnkddabm";//"86180627";"Koffienoen“;//wifi密码
// MQTT相关配置信息
const char *mqtt_broker_addr = "192.168.48.78"; // 服务器地址
const uint16_t mqtt_broker_port = 1883; // 服务端口号            
const char *mqtt_username = "usertestB"; // 账号（非必须）
const char *mqtt_password = "123456"; // 密码（非必须）
const uint16_t mqtt_client_buff_size = 4096; // 客户端缓存大小（非必须）
String mqtt_client_id = "esp32_client"; // 客户端ID
const char *mqtt_topic_pub = "esp32s3userB/normal"; // 需要发布到的主题
const char *mqtt_topic_sub = "esp32s3userA/normal"; // 需要订阅的主题
WiFiClient espClient;
PubSubClient  mqttClient;

char msg_buf[300];                                //发送信息缓冲区
char dataTemplate[] = "{\"ENV\":%.5f,\"moveflag\":%d,\"mesflag\":%d}"; //信息模板
char msgJson[100];                                 //要发送的json格式的数据
StaticJsonDocument<100> jsonBuffer;
unsigned short json_len = 0;
//http
const char* httpserverName = "http://192.168.48.78:5000/upload"; // 上传API的URL
//时钟
Ticker tim1;
Ticker tim2;
//传感器
MPU9250_asukiaaa accSensor;
int myShakeFlag = 0;
//灯带，使用RGB模式控制ws2812类型灯带
float preEnvData = 1;
Adafruit_NeoPixel strip1(WS2812_NUM, ENV_STRIP, NEO_GRB + NEO_KHZ800);
//天气
Adafruit_NeoPixel strip2(3, WEA_MEA_STRIP, NEO_GRB + NEO_KHZ800);
//发送消息
int myMesFlag = 0;
//对方参数初始化
double anoEnvData = 0.72135;
int anoShakeFlag  = 0;
int anoMesFlag    = 0;
//函数申明
void SPIFFSInit();
void i2sInit();
void i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len);
void i2s_adc(void *arg);
void wavHeader(byte* header, int wavSize);
void listSPIFFS(void);
void send_voicefile();
void uploadFile();
Audio audio;
//-----------****自我状态更新****--------------//
//--------------*灯带函数*-----------------//
//2灯带初始化
void strip_init(void)
{
  strip1.begin();              //初始化灯带
  strip1.setBrightness(Bright);//设置亮度数值S(max=255)
  strip1.fill(strip1.Color(255, 255, 255), 0, WS2812_NUM - 1);
  strip1.show();               //灯带显示
  strip2.begin();              //初始化灯带
  strip2.setBrightness(Bright/3);//设置亮度数值S(max=255)
  strip2.setPixelColor(0, strip2.Color(0, 0, 255));
  strip2.show();               //灯带显示
}

void strip_update(double envrate, double preenvrate)
{
  //过渡性进行光线变化，耗时3s
   double ratenow = max(0.008, envrate);
   double ratebef = max(0.008, preenvrate);
   double pre_env = Bright * ratebef;
   double now_env = Bright * ratenow;
   double interval = (now_env - pre_env)/30;
   for (int i = 0; i < 30; i++)
   {
    strip1.setBrightness(pre_env + (i+1) * interval);
    strip1.show();
    delay(100);    
   }
 
}


//--------------*风扇转动函数*----------------//
void fancallback()
{
  digitalWrite(FAN, LOW);
}
void fan_blow(int shakestate)
{
  Serial.println("fanblow");
  if (shakestate == 1)
  {
    digitalWrite(FAN, HIGH);
    tim1.once(10,fancallback);
  }
  else
  {
    digitalWrite(FAN, LOW);
  }
}
//void fan_blow(int shakestate)
//{
//  Serial.println("fanblow");
//  if (shakestate == 1)
//  {
//    digitalWrite(FAN, HIGH);
//    delay(3000);
//    digitalWrite(FAN, LOW);
//  }
//  else
//  {
//    digitalWrite(FAN, LOW);
//  }
//}

//---------------*录音函数*------------------//


//---------------*播音函数*-----------------//
void play_sound()
{
  digitalWrite(MOTOR,HIGH);
  Audio audio;
  audio.setPinout(SPK_I2S_BCLK, SPK_I2S_LRC, SPK_I2S_DOUT); // 初始化，配置所用引脚
  audio.setVolume(20); // 设置音量大小 0...21 
  audio.connecttohost("http://192.168.48.78:5000/uploads/ESP32s3UserA.wav");
  int i=0;
  while(i<1000){
    audio.loop();
    delay(10);
    i++;
  }
  strip2.setPixelColor(2,strip2.Color(0, 0, 0));
  strip2.show();
  digitalWrite(MOTOR, LOW);
}

//-------------******通信函数******----------------//
//--------------*wifi连接测试函数*-----------------//

void wifi_reconnect()
{
  //连接wifi
  WiFi.begin(ssid, password);
  Serial.print("connecting WiFi");
  //检测是否成功
  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
    Serial.print(".");
    //WiFi.begin(ssid, password);
  }
  Serial.print("wifi connected!");
  Serial.println(WiFi.localIP());
}

//---------------*http函数*-------------------//


//--------------*mqtt函数*---------------------//

//连接与连接状态测试
//void mqtt_reconnect() {
//  while (!mqttClient.connected()) {
//    Serial.print("Attempting MQTT connection...");
//    // Attempt to connect
//    mqttClient.connect(CLIENT_ID, MQTT_USRNAME, MQTT_PASSWD);
//    delay(500);
//    if (mqttClient.connected()) {
//      Serial.println("connected");
//      // 连接成功时订阅主题
//      mqttClient.subscribe(SUB_TOPIC_NOR,1);
//    } else {
//      Serial.print("failed, rc=");
//      Serial.print(mqttClient.state());
//      Serial.println(" try again in 2 seconds");
//      // Wait 5 seconds before retrying
//      delay(2000);
//    }
//  }
//}


//向主题发送信息
void send_data(double env_data, int sha_flag, int mea_flag)
{
  if (mqttClient.connected())
  {
    //Serial.println(env_data);
    if (env_data < 4095) {
      env_data *= 0.8;
    }
    env_data /= 8190;
    double env_data_scaled = - env_data / log(env_data);
    //Serial.println(env_data_scaled);
    //Serial.println("shake" + String(sha_flag));
    snprintf(msgJson, 100, dataTemplate, env_data_scaled, sha_flag, mea_flag); //将数据套入dataTemplate模板中, 生成的字符串传给msgJson
    json_len = strlen(msgJson);                   //msgJson的长度
    msg_buf[0] = char(0x03);                       //要发送的数据必须按照ONENET的要求发送, 根据要求,数据第一位是3
    msg_buf[1] = char(json_len >> 8);              //数据第二位是要发送的数据长度的高八位
    msg_buf[2] = char(json_len & 0xff);            //数据第三位是要发送数据的长度的低八位
    memcpy(msg_buf + 3, msgJson, strlen(msgJson)); //从msg_buf的第四位开始,放入要传的数据msgJson
    msg_buf[3 + strlen(msgJson)] = 0;              //添加一个0作为最后一位, 这样要发送的msg_buf准备好了
    Serial.print("public message:");
    Serial.println(msgJson);
    mqttClient.publish(mqtt_topic_pub, (uint8_t *)msg_buf, 3 + strlen(msgJson), true); //发送数据到主题$dp
  }
}

//回传
//bug:回传未调用 update:de了,是必须retain才可以
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  
//  strip2.setPixelColor(2,strip2.Color(167, 52, 216));
//  strip2.show();
  Serial.print("Message arrived [");
  Serial.print(topic);   // 打印主题信息
  Serial.println("] ");
  if (*topic == *mqtt_topic_sub)
  { 
    char normalJson[length-3]; 
    for (int i = 3; i < length; i++) 
    {
      char chara = (char)payload[i];
      Serial.print(chara); // 打印主题内容
      normalJson[i-3] = chara;
    }
    // 反序列化JSON
    DeserializationError error = deserializeJson(jsonBuffer, normalJson);
    if (error) 
    {
     Serial.print(F("deserializeJson() failed: "));
     Serial.println(error.f_str());
     return;
    }
    // 解析JSON
    anoEnvData = jsonBuffer["ENV"];          // 读取字符串
    anoShakeFlag = jsonBuffer["moveflag"];   // 读取整形数据
    anoMesFlag = jsonBuffer["mesflag"];      // 读取嵌套对象
  }
  Serial.println((String)anoEnvData);
  Serial.println((String)anoShakeFlag);
  Serial.println((String)anoMesFlag);
//
//  strip2.setPixelColor(2,strip2.Color(0, 0, 0));
//  strip2.show();
  //参数更新
  //如果有消息进来亮灯
  if (anoMesFlag == 4095){
   strip2.setPixelColor(2,strip2.Color(255, 0, 0));
   strip2.show();
  }
  //环境
  strip_update(anoEnvData, preEnvData);
  //风扇
  fan_blow(anoShakeFlag);
  preEnvData = anoEnvData;
  Serial.println();
}


//------------****主函数****---------------//
//---------------*初始化*------------------//
void setup()
{
  Serial.begin(115200);
  //连接wifi
  wifi_reconnect();
  //*引脚初始化*//
  pinMode(MOTOR, OUTPUT);//旋转电机
  pinMode(MEA_LED, OUTPUT);//录音提示
  pinMode(ENV_SENSOR, INPUT);//环境光
  pinMode(PLAY_SPRING, INPUT);
  pinMode(MICRO_ENABLE, INPUT);
  audio.setPinout(SPK_I2S_BCLK, SPK_I2S_LRC, SPK_I2S_DOUT); // 初始化，配置所用引脚

  strip_init();
  //*连接云端*//
  mqttClient.setClient(espClient);
  mqttClient.setServer(mqtt_broker_addr, mqtt_broker_port);
  mqttClient.setBufferSize(mqtt_client_buff_size);
  mqttClient.setCallback(mqtt_callback);
  //*摇晃检测*//
  Wire.begin(MOVEMENT_SENSOR_SDA, MOVEMENT_SENSOR_SCL); //sensor sda, scl
  accSensor.setWire(&Wire);
  accSensor.beginGyro();
}

unsigned long previousConnectMillis = 0; // 毫秒时间记录
const long intervalConnectMillis = 2000; // 时间间隔
unsigned long previousPublishMillis = 0; // 毫秒时间记录
const long intervalPublishMillis = 2000; // 时间间隔
int now_play_flag = analogRead(PLAY_SPRING);
int preMesFlag = 0; //tbc

//---------------*循环*------------------//
void loop()
{
   unsigned long currentMillis = millis(); // 读取当前时间
  //*检测*//
  //移动
  //tim2.detach();
  accSensor.gyroUpdate();
  int myShakeFlag = 0;
  if (accSensor.gyroX() > 150 || accSensor.gyroY() > 150 || accSensor.gyroZ() > 150)
  {
    myShakeFlag = 1;
  }
  //环境
  double myEnvData = 0;
  myEnvData = analogRead(ENV_SENSOR);
////  //消息
//  myMesFlag = analogRead(MICRO_ENABLE);
//  if (myMesFlag == 4095){
//    if (preMesFlag == 4095){
//      send_voicefile();
//      }
//  }
//  preMesFlag = myMesFlag;
//  //检测播放
  now_play_flag = analogRead(PLAY_SPRING);
  Serial.print("now");
  Serial.println((String)now_play_flag);
  if (now_play_flag > 4000){
    mqttClient.disconnect();
    play_sound();
  }
   // 连接MQTT服务器
    if (!mqttClient.connected()) // 如果未连接
    {
      Serial.println("mqtt not connected");
        if (currentMillis - previousConnectMillis > intervalConnectMillis)
        {
            previousConnectMillis = currentMillis;
            mqtt_client_id += String(WiFi.macAddress()); // 每个客户端需要有唯一的ID，不然上线时会把其他相同ID的客户端踢下线
            if (mqttClient.connect(mqtt_client_id.c_str(), mqtt_username, mqtt_password))
            {
              Serial.println("mqtt connected!");
              mqttClient.subscribe(mqtt_topic_sub); // 连接成功后可以订阅主题
            }
            else{
                    Serial.println("mqtt still not connected");
            }
        }
    }

    // 定期发送消息
    if (mqttClient.connected())
    {
      Serial.println("mqtt connected");
        if (currentMillis - previousPublishMillis >= intervalPublishMillis) // 如果和前次时间大于等于时间间隔
        {
            Serial.println("published!");
            previousPublishMillis = currentMillis;
            send_data(myEnvData, myShakeFlag, myMesFlag);
        }
    }

  // 处理MQTT事务
  mqttClient.loop();

  myMesFlag     = 0;
  myShakeFlag   = 0;
  //*连接检测*//
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WIFI disconnected!");
    wifi_reconnect();
  }
}
