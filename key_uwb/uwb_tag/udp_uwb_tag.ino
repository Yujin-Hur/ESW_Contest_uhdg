/*

For ESP32 UWB or ESP32 UWB Pro

*/
#include <SPI.h>
#include <DW1000Ranging.h>
#include <WiFi.h>
#include "link.h"
#include <Wire.h>
#include "BluetoothSerial.h"

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
#define PIN_RST 27
#define PIN_IRQ 34

#define UWB_RST 27 // reset pin
#define UWB_IRQ 34 // irq pin
#define UWB_SS 21   // spi select pin

int gamma_trigger = 0; //for bluetooth send trigger

//bluetooth
const char *pin = "1234"; //change this to more secure PIN.

String device_name = "ESP32-BT-Slave";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

//data
double distance_width = 1.0;
double distance_height = 1.4;
int anomal_flag = 0;

struct MyLink *uwb_data;
int index_num = 0;
long runtime = 0;
String all_json = "";

void tag_pos(double a, double b, double c, double d, double *x, double *y)
{
    if (b == 0)
      b = 0.0001; //for division by 0
    double cos_a = (b*b + c*c - a*a) / (2*b*c);
    double sin_a = sqrt(1 - cos_a*cos_a);
  
    // 후보 1
    double x1 = b * cos_a;
    double y1 = b * sin_a;
  
    // 후보 2
    double x2 = b * cos_a;
    double y2= -b * sin_a;
  
    // 앵커 3와의 거리 구하기
    double a3_x = 0.0;
    double a3_y = -distance_height;
  
    //두 후보 좌표 중 앵커 3과의 거리에 맞는 태그 위치 정하기. 
    double distance1 = sqrt((x1 - a3_x) * (x1 - a3_x) + (y1 - a3_y) * (y1 - a3_y));
    double distance2 = sqrt((x2 - a3_x) * (x2 - a3_x) + (y2 - a3_y) * (y2 - a3_y));
    
    double ret_x = (fabs(distance1 - d) < fabs(distance2 - d)) ? x1:x2;
    double ret_y = (fabs(distance1 - d) < fabs(distance2 - d)) ? y1:y2;

    Serial.printf("ret : %f %f\n", ret_x, ret_y);
    if ((ret_x > 300.0) || (ret_y > 300.0) || (ret_x < -300.0) || (ret_y < -300.0) || isnan(ret_y))//x축 상에 존재하거나 값이 튈 때
    {
      if ((a - 1 < b && b < a + 3) || (b - 1 < a && a < b + 3))//x축 상에 존재할 때는 sin_a가 NaN이 나옴. 이럴 때는 (+-a1, 0)의 좌표로. a1 > a2이면 (a1, 0), a1 < a2이면 (-a1, 0)으로.
      {
        if (b > a)
        {
          ret_x = b;
          ret_y = 0;
        }
        else if (b < a)
        {
          ret_x = -b;
          ret_y = 0;
        }
      }
      else // 값이 튈 때
      {
        anomal_flag = 1;
      }
    }
    *x = ret_x;
    *y = ret_y;
}

void calculate_angle_distance(float x, float y, int *angle, int *distance)
{
    if (x == 0)
      x = 0.0001;
    if (y == 0)
      y = 0.0001;
    if (anomal_flag == 1)
    {
      *angle = 999;
      *distance = 0;
      return;
    }
    int angle_temp = atan2(fabs(y), fabs(x)) / PI * 180;
    *distance = sqrt(x * x + y * y);
    Serial.printf("x : %f, y ; %f, middle angle : %d middle_distance : %d\n", x, y, angle_temp, *distance);
    
    if (x > 0 && y > 0) // 1사분면
      *angle = (90 - angle_temp);
    else if (x > 0 && y < 0) // 2사분면
      *angle = (90 + angle_temp);
    else if (x < 0 && y < 0) // 3사분면
      *angle = (270 - angle_temp);
    else if (x < 0 && y > 0)// 4사분면
      *angle = (270 + angle_temp);
}

void setup()
{
    Serial.begin(115200);

    //bluetooth
    SerialBT.begin(device_name); //Bluetooth device name
    Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
    #ifdef USE_PIN
      SerialBT.setPin(pin);
      Serial.println("Using PIN");
    #endif

    //init the configuration
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(UWB_RST, UWB_SS, UWB_IRQ);
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);

    //we start the module as a tag
    //DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9B", DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
    DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:5E", DW1000.MODE_LONGDATA_RANGE_ACCURACY);
    delay(2000); //안정성을 위해 켜고 2초 기다림
    uwb_data = init_link();
}

void loop()
{
    char data;
    double a1_range = 0.0;
    double a2_range = 0.0;
    double a3_range = 0.0;
  
    double x = 0.0;
    double y = 0.0;
  
    int angle = 0;
    int distance = 0;
    anomal_flag = 0;

    if (gamma_trigger == 0)
    {
      if (SerialBT.available())
      {
        data = SerialBT.read();
        Serial.println(data);
        if (data == 'g')
        {
          gamma_trigger = 1;
        }
      }
    }
    if (gamma_trigger == 1)
    {
      if (SerialBT.available())
      {
        data = SerialBT.read();
        Serial.println(data);
        if (data == 's')
        {
          gamma_trigger = 0;
        }
      }
    }
    DW1000Ranging.loop();
    if ((millis() - runtime) > 500 && gamma_trigger == 1)
    {
        make_link_json(uwb_data, &all_json);
        //send_udp(&all_json);
        runtime = millis();
        int node_cnt = 0;
        int node_cnt2 = 0;
    
        // node가 3개
        struct MyLink *temp = uwb_data;
        while (temp != NULL)
        {
          //Serial.println(temp->anchor_addr);
          if (temp->anchor_addr == 0x1A)
          {
            //Serial.print('h');
            a1_range = double(temp->range[0]);
            Serial.println("a1:" + String(a1_range));
            node_cnt2++;
          }
          else if (temp->anchor_addr == 0x2B)
          {
            a2_range = double(temp->range[0]);
            Serial.println("a2:" + String(a2_range));
            node_cnt2++;
          }
          else if (temp->anchor_addr == 0x3C)
          {
            a3_range = double(temp->range[0]);
            Serial.println("a3:" + String(a3_range));
            node_cnt2++;
          }
          temp = temp->next;
        }
        if (node_cnt2 == 3)
        {
          tag_pos(a2_range, a1_range, distance_width, a3_range, &x, &y);
          calculate_angle_distance(x, y, &angle, &distance);
          if (angle >= 0 && angle <= 360)
          {
            if (gamma_trigger == 1)
            {
              SerialBT.print('g'); // gamma
              SerialBT.print(angle);
              Serial.printf("angle : %d\n", angle);
              SerialBT.print('d'); // distance
              SerialBT.print(distance);
              Serial.printf("distance : %d\n", distance);
            }
          }
        }
        else
        {
          if (gamma_trigger == 1)
          {
            SerialBT.print('g'); // gamma
            SerialBT.print('999');
            Serial.printf("angle : %d\n", 999);
            SerialBT.print('d'); // distance
            SerialBT.print('0');
            Serial.printf("distance : %d\n", 0);
          }
        }
    }
}

void newRange()
{
    char c[30];
    /*
    Serial.print("from: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
    Serial.print("\t Range: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRange());
    Serial.print(" m");
    
    Serial.print("\t RX power: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
    Serial.println(" dBm");
    */
    fresh_link(uwb_data, DW1000Ranging.getDistantDevice()->getShortAddress(), DW1000Ranging.getDistantDevice()->getRange(), DW1000Ranging.getDistantDevice()->getRXPower());
}
 
void newDevice(DW1000Device *device)
{
    Serial.print("ranging init; 1 device added ! -> ");
    Serial.print(" short:");
    Serial.println(device->getShortAddress(), HEX);

    add_link(uwb_data, device->getShortAddress());
}

void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);

    delete_link(uwb_data, device->getShortAddress());
}