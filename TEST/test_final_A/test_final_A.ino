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
//
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
const char *mqtt_username = "usertestA"; // 账号（非必须）
const char *mqtt_password = "123456"; // 密码（非必须）
const uint16_t mqtt_client_buff_size = 4096; // 客户端缓存大小（非必须）
String mqtt_client_id = "esp32_client"; // 客户端ID
const char *mqtt_topic_pub = "esp32s3userA/normal"; // 需要发布到的主题
const char *mqtt_topic_sub = "esp32s3userB/normal"; // 需要订阅的主题
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
double preEnvData = 1;
Adafruit_NeoPixel strip1(WS2812_NUM, ENV_STRIP, NEO_GRB + NEO_KHZ800);
//天气
Adafruit_NeoPixel strip2(3, WEA_MEA_STRIP, NEO_GRB + NEO_KHZ800);
//发送消息
int myMesFlag = 0;
//对方参数初始化
double anoEnvData = 0.72135;
int anoShakeFlag  = 0;
int anoMesFlag    = 0;
int onceflag = 1;
//函数申明
void SPIFFSInit();
void i2sInit();
void i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len);
void i2s_adc(void *arg);
void wavHeader(byte* header, int wavSize);
void listSPIFFS(void);
void send_voicefile();
void uploadFile();
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
  strip2.setPixelColor(1, strip2.Color(255, 171, 0));
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


// 初始化
void SPIFFSInit() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialisation failed!");
    while (1) yield();
  }

  SPIFFS.remove(filename); // 移除掉已存在的file
  file = SPIFFS.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("File is not available!");
  }

  byte header[headerSize];
  wavHeader(header, FLASH_RECORD_SIZE); // 生成wave header

  file.write(header, headerSize); // 写入header
  //listSPIFFS();
}

void i2sInit() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S | I2S_COMM_FORMAT_STAND_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 64,
    .dma_buf_len = 1024,  // 1024 samples per buffer
    .use_apll = 1         // use APLL-CLK,frequency 16MHZ-128MHZ,it's for audio
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}

// digital to analog：麦克风输入为digital code(binary)， 需要将编码转换成相应的电压值才能播放
void i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len)
{
  uint32_t j = 0;
  uint32_t dac_value = 0;
  // 一个采样点是2byte，每2个byte
  for (int i = 0; i < len; i += 2) {
    dac_value = ((((uint16_t) (s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
    d_buff[j++] = 0;
    d_buff[j++] = dac_value * 256 / 4096;
  }
}

// 录音任务
void i2s_adc(void *arg)
{
   //亮灯提示可以说了
  digitalWrite(MEA_LED, HIGH);
  Serial.println(" *** Recording Start *** ");
  int i2s_read_len = I2S_READ_LEN;
  // 已写入flash的大小
  int flash_wr_size = 0;
  // in i2s_read(),Number of bytes read, if timeout, bytes read will be less than the size passed in
  size_t bytes_read;
  // 每一次读取一个i2s_read_buff，大小：16 * 1024 bytes,DMA Buffer大小应该是其倍数，不然会报错：读取内存错误
  char* i2s_read_buff = (char*) calloc(i2s_read_len, sizeof(char));
  // 经过scale之后每次读取写入flash的buff
  uint8_t* flash_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
  // 已经开始写入了，试试删掉，因为INMP441需要准备时间
  i2s_read(I2S_PORT, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
  i2s_read(I2S_PORT, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
  while (analogRead(MICRO_ENABLE)==4095) {
    //read data from I2S bus, in this case, from ADC.
    i2s_read(I2S_PORT, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
    //save original data from I2S(ADC) into flash.
    // 先转换成电压值
    i2s_adc_data_scale(flash_write_buff, (uint8_t*)i2s_read_buff, i2s_read_len);
    file.write((const byte*) flash_write_buff, i2s_read_len);
    flash_wr_size += i2s_read_len;
    ets_printf("Sound recording %u%%\n", flash_wr_size * 100 / FLASH_RECORD_SIZE);
    ets_printf("Never Used Stack Size: %u\n", uxTaskGetStackHighWaterMark(NULL));
  }
  file.close();
  digitalWrite(MEA_LED, LOW);

  // 清空buff
  free(i2s_read_buff);
  i2s_read_buff = NULL;
  free(flash_write_buff);
  flash_write_buff = NULL;

  listSPIFFS();
  //http上传文件
  uploadFile();
  vTaskDelete(NULL);
}

// 生成wav header，16bit 位深
void wavHeader(byte* header, int wavSize) { // 数字小端格式，字符大端格式
  header[0] = 'R';
  header[1] = 'I';
  header[2] = 'F';
  header[3] = 'F';
  //unsigned int fileSize = 163384;
  unsigned int fileSize = wavSize + headerSize - 8;
  Serial.println((String)fileSize);
  header[4] = (byte)(fileSize & 0xFF); // file size, 4byte integer
  header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF);
  header[7] = (byte)((fileSize >> 24) & 0xFF);
  header[8] = 'W';
  header[9] = 'A';
  header[10] = 'V';
  header[11] = 'E';
  header[12] = 'f';
  header[13] = 'm';
  header[14] = 't';
  header[15] = ' ';
  header[16] = 0x10; // length of format data = 16, 4byte integer
  header[17] = 0x00;
  header[18] = 0x00;
  header[19] = 0x00;
  header[20] = 0x01; // format type:1(PCM), 2byte integer
  header[21] = 0x00;
  header[22] = 0x01; // channel number:1, 2byte integer
  header[23] = 0x00;
  header[24] = 0x80; // sample rate:16000=0x00003E80, 4byte integer
  header[25] = 0x3E;
  header[26] = 0x00;
  header[27] = 0x00;
  header[28] = 0x00; // SampleRate*BitPerSample*ChannelNum/8=16000*16*1/8=0x00007D00, 4byte integer
  header[29] = 0x7D;
  header[30] = 0x00;
  header[31] = 0x00;
  header[32] = 0x02; // BitPerSample*ChannelNum/8 = 2, 2byte integer
  header[33] = 0x00;
  header[34] = 0x10; // BitPerSample:16 = 0x0010, 2byte integer
  header[35] = 0x00;
  header[36] = 'd';
  header[37] = 'a';
  header[38] = 't';
  header[39] = 'a';
  header[40] = (byte)(wavSize & 0xFF);
  header[41] = (byte)((wavSize >> 8) & 0xFF);
  header[42] = (byte)((wavSize >> 16) & 0xFF);
  header[43] = (byte)((wavSize >> 24) & 0xFF);

}


void listSPIFFS(void) {
  Serial.println(F("\r\nListing SPIFFS files:"));
  static const char line[] PROGMEM =  "=================================================";

  Serial.println(FPSTR(line));
  Serial.println(F("  File name                              Size"));
  Serial.println(FPSTR(line));

  fs::File root = SPIFFS.open("/");
  if (!root) {
    Serial.println(F("Failed to open directory"));
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(F("Not a directory"));
    return;
  }

  fs::File file = root.openNextFile();
  while (file) {

    if (file.isDirectory()) {
      Serial.print("DIR : ");
      String fileName = file.name();
      Serial.print(fileName);
    } else {
      String fileName = file.name();
      Serial.print("  " + fileName);
      // File path can be 31 characters maximum in SPIFFS
      int spaces = 33 - fileName.length(); // Tabulate nicely
      if (spaces < 1) spaces = 1;
      while (spaces--) Serial.print(" ");
      String fileSize = (String) file.size();
      spaces = 10 - fileSize.length(); // Tabulate nicely
      if (spaces < 1) spaces = 1;
      while (spaces--) Serial.print(" ");
      Serial.println(fileSize + " bytes");
    }

    file = root.openNextFile();
  }

  Serial.println(FPSTR(line));
  Serial.println();
  delay(1000);
}

void send_voicefile()
{     
      SPIFFSInit();
      i2sInit();
      //?adc
      xTaskCreate(i2s_adc, "i2s_adc", 1024 * 8, NULL, 1, NULL); // 根据需要修改堆栈深度，否则会无限重启....服了
      delay(20000);
}

//---------------*播音函数*-----------------//
void play_sound()
{
  digitalWrite(MOTOR,HIGH);
  Audio audio;
  audio.setPinout(SPK_I2S_BCLK, SPK_I2S_LRC, SPK_I2S_DOUT); // 初始化，配置所用引脚
  audio.setVolume(20); // 设置音量大小 0...21 
  audio.connecttohost("http://192.168.48.78:5000/uploads/ESP32s3UserB.wav");
  int i=0;
  while(i<1200){
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

void uploadFile() {
  File myfile = SPIFFS.open(filename, "r");
  if (!myfile) {
    Serial.println("Failed to open file for reading");
    return;
  }
  if (myfile.size() < 100){
    Serial.println(myfile.size());
    Serial.println("too small to upload");
    return;
  }
  digitalWrite(MEA_LED, HIGH);
  Serial.println((String)myfile.size());
  HTTPClient http;
  http.begin(httpserverName); // 初始化HTTP客户端
  http.addHeader("Content-Type", "audio/x-wav"); // 添加HTTP头部信息
  http.addHeader("Device","ESP32s3UserA");
  // 发送POST请求
  int httpResponseCode = http.sendRequest("POST", &myfile, myfile.size());

  if (httpResponseCode == 201) {
    Serial.printf("File uploaded successfully, server responded: %d\n", httpResponseCode);
    onceflag = 0;
  } else {
    Serial.printf("File upload failed, error: %d\n", httpResponseCode);
  }

  file.close(); // 关闭文件
  http.end(); // 关闭HTTP客户端
  digitalWrite(MEA_LED, LOW);
}

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
  if (accSensor.gyroX() > 80 || accSensor.gyroY() > 80 || accSensor.gyroZ() > 80)
  {
    myShakeFlag = 1;
    fan_blow(myShakeFlag);
  }
  //环境
  double myEnvData = 0;
  myEnvData = analogRead(ENV_SENSOR);
  //消息 tbc
  myMesFlag = analogRead(MICRO_ENABLE);
  if (myMesFlag == 4095){
    if (preMesFlag == 4095){
      if (onceflag ==1){
      send_voicefile();
      }
    }
  }
  preMesFlag = myMesFlag;
  
//  //检测播放tbc
//  now_play_flag = analogRead(PLAY_SPRING);
//  Serial.print("now");
//  Serial.println((String)now_play_flag);
//  if (now_play_flag > 4000){
//    mqttClient.disconnect();
//    play_sound();
//  }

  //连接MQTT服务器
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
