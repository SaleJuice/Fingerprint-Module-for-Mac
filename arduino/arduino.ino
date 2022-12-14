/*
 * @FilePath: /Fingerprint-Module-for-Mac/arduino/arduino.ino
 * @Date: 2022-08-10 14:05:47
 * @LastEditTime: 2022-08-10 18:55:13
 * @Author: Xiaozhu Lin
 * @E-Mail: linxzh@shanghaitech.edu.cn
 * @Institution: MAgIC Lab, ShanghaiTech University, China
 * @SoftWare: VSCode
 */


#include "stdio.h"
#include "SoftwareSerial.h"       //注意添加这个软串口头文件

#include "USB.h"
#include "USBHIDKeyboard.h"
USBHIDKeyboard Keyboard;

char auth[] = "";   //输入你的点灯科技的项目密钥
char ssid[] = "";   //输入你的WiFi账号
char pswd[] = "";   //输入你的WiFi密码

SoftwareSerial mySerial(4,5);    //软串口引脚，RX：GPIO4    TX：GPIO5

char str[20];    //用于sprint函数的临时数组
int SearchID,EnrollID;    //搜索指纹的ID号和注册指纹的ID号
uint16_t ScanState = 0,WiFi_Connected_State = 1,ErrorNum = 0,PageID = 0;   //状态标志变量；WiFi是否连接状态标志位；扫描指纹错误次数标志位；输入ID号变量
uint8_t PS_ReceiveBuffer[20];   //串口接收数据的临时缓冲数组

/*******************************************************这里的数组都是手册里面的，有好几个用不到，看个人使用********************************************************/
uint8_t PS_RegMBBuffer[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x05,0x00,0x09};
uint8_t PS_SleepBuffer[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x33,0x00,0x37};
uint8_t PS_EmptyBuffer[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x0D,0x00,0x11};
uint8_t PS_GetImageBuffer[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x01,0x00,0x05};
uint8_t PS_CancelBuffer[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x30,0x00,0x34};
uint8_t PS_GetEnrollImageBuffer[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x29,0x00,0x2D};
uint8_t PS_GetChar1Buffer[13] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x04,0x02,0x01,0x00,0x08};
uint8_t PS_GetChar2Buffer[13] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x04,0x02,0x02,0x00,0x09};
uint8_t PS_GetChar3Buffer[13] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x04,0x02,0x03,0x00,0x0A};
uint8_t PS_GetChar4Buffer[13] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x04,0x02,0x04,0x00,0x0B};
uint8_t PS_StorMBBuffer[15] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x06,0x06,0x01,'\0','\0','\0','\0'};
uint8_t PS_BlueLEDBuffer[16] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x07,0x3C,0x03,0x01,0x01,0x00,0x00,0x49};
uint8_t PS_RedLEDBuffer[16] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x07,0x3C,0x02,0x04,0x04,0x02,0x00,0x50};
uint8_t PS_GreenLEDBuffer[16] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x07,0x3C,0x02,0x02,0x02,0x02,0x00,0x4C};
uint8_t PS_SearchMBBuffer[17] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x08,0x04,0x01,0x00,0x00,0xFF,0xFF,0x02,0x0C};
uint8_t PS_AutoEnrollBuffer[17] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x08,0x31,'\0','\0',0x04,0x00,0x16,'\0','\0'}; //PageID: bit 10:11，SUM: bit 15:16
uint8_t PS_DeleteBuffer[16] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x07,0x0C,'\0','\0',0x00,0x01,'\0','\0'}; //PageID: bit 10:11，SUM: bit 14:15


/********************************************************************以下是软串口接收发送函数的实现****************************************************************/
void delay_ms(long int ms)
{
  for(int i=0;i<ms;i++)
  {
    delayMicroseconds(1000);
  }
}

void FPM383C_SendData(int len,uint8_t PS_Databuffer[])
{
  mySerial.write(PS_Databuffer,len);
  mySerial.flush();
}

void FPM383C_ReceiveData(uint16_t Timeout)
{
  uint8_t i = 0;
  while(mySerial.available() == 0 && (--Timeout))
  {
    delay(1);
  }
  while(mySerial.available() > 0)
  {
    delay(2);
    PS_ReceiveBuffer[i++] = mySerial.read();
    if(i > 15) break; 
  }
}
/********************************************************************以上是软串口接收发送函数的实现****************************************************************/


/********************************************************************以下是指纹模块的功能函数的实现****************************************************************/
/*发送休眠指令，让FPM383C模块为下一次Touch中断做准备*/
void PS_Sleep()
{
  FPM383C_SendData(12,PS_SleepBuffer);
}

/*控制LED灯函数，参数为LED控制数组*/
void PS_ControlLED(uint8_t PS_ControlLEDBuffer[])
{
  FPM383C_SendData(16,PS_ControlLEDBuffer);
}

/*取消自动注册，返回应答包的位9确认码。*/
uint8_t PS_Cancel()
{
  FPM383C_SendData(12,PS_CancelBuffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}

/*获取搜索指纹用图像，返回应答包的位9确认码。*/
uint8_t PS_GetImage()
{
  FPM383C_SendData(12,PS_GetImageBuffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}

 /*获取注册指纹用图像，返回应答包的位9确认码。本例程里不需要用到这个函数，大家可以参考手册自行了解一下*/
// uint8_t PS_GetEnrollImage()
// {
//   FPM383C_SendData(12,PS_GetEnrollImageBuffer);
//   FPM383C_ReceiveData(2000);
//   return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
// }

/*生成特征，存储到缓冲区1，返回应答包的位9确认码。*/
uint8_t PS_GetChar1()
{
  FPM383C_SendData(13,PS_GetChar1Buffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}

/*生成特征，存储到缓冲区2，返回应答包的位9确认码。*/
uint8_t PS_GetChar2()
{
  FPM383C_SendData(13,PS_GetChar2Buffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}

/*生成特征，存储到缓冲区3，返回应答包的位9确认码。*/
// uint8_t PS_GetChar3()
// {
//   FPM383C_SendData(13,PS_GetChar3Buffer);
//   FPM383C_ReceiveData(2000);
//   return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
// }

/*生成特征，存储到缓冲区4，返回应答包的位9确认码。*/
// uint8_t PS_GetChar4()
// {
//   FPM383C_SendData(13,PS_GetChar4Buffer);
//   FPM383C_ReceiveData(2000);
//   return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
// }

/*合并模板，返回应答包的位9确认码。*/
// uint8_t PS_RegMB()
// {
//   FPM383C_SendData(12,PS_RegMBBuffer);
//   FPM383C_ReceiveData(2000);
//   return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
// }

/*存储模板，返回应答包的位9确认码。*/
// uint8_t PS_StorMB()
// {
//   FPM383C_SendData(15,PS_StorMBBuffer);
//   FPM383C_ReceiveData(2000);
//   return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
// }

/*搜索模板，返回应答包的位9确认码。*/
uint8_t PS_SearchMB()
{
  FPM383C_SendData(17,PS_SearchMBBuffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}

/*清空模板，返回应答包的位9确认码。*/
uint8_t PS_Empty()
{
  FPM383C_SendData(12,PS_EmptyBuffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}

/*自动注册模板，返回应答包的位9确认码。*/
uint8_t PS_AutoEnroll(uint16_t PageID)
{
  PS_AutoEnrollBuffer[10] = (PageID>>8);
  PS_AutoEnrollBuffer[11] = (PageID);
  PS_AutoEnrollBuffer[15] = (0x54+PS_AutoEnrollBuffer[10]+PS_AutoEnrollBuffer[11])>>8;
  PS_AutoEnrollBuffer[16] = (0x54+PS_AutoEnrollBuffer[10]+PS_AutoEnrollBuffer[11]);
  FPM383C_SendData(17,PS_AutoEnrollBuffer);
  FPM383C_ReceiveData(10000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}

/*删除指定模板，返回应答包的位9确认码。*/
uint8_t PS_Delete(uint16_t PageID)
{
  PS_DeleteBuffer[10] = (PageID>>8);
  PS_DeleteBuffer[11] = (PageID);
  PS_DeleteBuffer[14] = (0x15+PS_DeleteBuffer[10]+PS_DeleteBuffer[11])>>8;
  PS_DeleteBuffer[15] = (0x15+PS_DeleteBuffer[10]+PS_DeleteBuffer[11]);
  FPM383C_SendData(16,PS_DeleteBuffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}

/*自动注册指纹函数，返回应答包的位9确认码。*/
uint8_t PS_Enroll(uint16_t PageID)
{
  if(PS_AutoEnroll(PageID) == 0x00)
  {
    PS_ControlLED(PS_GreenLEDBuffer);
    return PS_ReceiveBuffer[9];
  }
  PS_ControlLED(PS_RedLEDBuffer);
  return 0xFF;
}

/*自动搜索指纹函数，返回应答包的位9确认码。*/
uint8_t PS_Identify()
{
  if(PS_GetImage() == 0x00)
  {
    if(PS_GetChar1() == 0x00)
    {
      if(PS_SearchMB() == 0x00)
      {
        if(PS_ReceiveBuffer[8] == 0x07 && PS_ReceiveBuffer[9] == 0x00)
        {
          PS_ControlLED(PS_GreenLEDBuffer);
          return PS_ReceiveBuffer[9];
        }
      }
    }
  }
  ErrorNum++;
  PS_ControlLED(PS_RedLEDBuffer);
  return 0xFF;
}

/*判断指纹是否已经注册过*/
void SEARCH_ACK_CHECK(uint8_t ACK)
{
	if(PS_ReceiveBuffer[6] == 0x07)
	{
		switch (ACK)
		{
			case 0x00:  //指令正确
        SearchID = (int)((PS_ReceiveBuffer[10] << 8) + PS_ReceiveBuffer[11]);
        sprintf(str,"Now Search ID: %d",(int)SearchID);
        Serial.println(str);
        if(SearchID == 0) WiFi_Connected_State = 0;  // 为什么要在这里判断一下 'SearchID' ?
        // 成功之后需要执行的操作:比如「解锁」
        digitalWrite(15,!digitalRead(15));
        Keyboard.write(KEY_BACKSPACE);
        delay(300);
        Keyboard.print("输入你自己的电脑密码\n");
        if(ErrorNum < 5) ErrorNum = 0;
				break;
		}
	}
  for(int i=0;i<20;i++) PS_ReceiveBuffer[i] = 0xFF;
}

/*判断指纹是否注册成功*/
void ENROLL_ACK_CHECK(uint8_t ACK)
{
	if(PS_ReceiveBuffer[6] == 0x07)
	{
		switch (ACK)
		{
			case 0x00:  //指令正确
        EnrollID = (int)((PS_AutoEnrollBuffer[10] << 8) + PS_AutoEnrollBuffer[11]);
        sprintf(str,"Now Enroll ID: %d",(int)EnrollID);
        Serial.println(str);
				break;
		}
	}
  for(int i=0;i<20;i++) PS_ReceiveBuffer[i] = 0xFF;
}
/********************************************************************以上是指纹模块的功能函数的实现****************************************************************/




/***********************************************************************以下是外部中断函数的实现******************************************************************/
ICACHE_RAM_ATTR void InterruptFun()
{
  detachInterrupt(digitalPinToInterrupt(14));
  PS_ControlLED(PS_BlueLEDBuffer);
  delay_ms(10);
  ScanState |= 1<<4;
}
/***********************************************************************以上是外部中断函数的实现******************************************************************/



/****************************************************************以下是点灯科技APP里面按键等组件的实现************************************************************/
void OneEnroll_callback()
{
  delay_ms(500);
  ScanState |= 1<<2;
  Serial.println("OneEnroll Fingerprint");
}

void Delete_callback()
{
  delay_ms(500);
  ScanState |= 1<<3;
  Serial.println("Delete Fingerprint");
}

void Identify_callback()
{
  delay_ms(500);
  ScanState &= ~(1<<0);
  Serial.println("MultSearch Fingerprint");
}

void Empty_callback()
{
  PageID = 0;
  delay_ms(500);
  Serial.println("Empty Fingerprint");
  if(PS_Empty() == 0x00)
  {
    PS_ControlLED(PS_GreenLEDBuffer);
  }
  else
  {
    PS_ControlLED(PS_RedLEDBuffer);
  }
}

void MultEnroll_callback()
{
  delay_ms(500);
  ScanState |= 0x01;
  Serial.println("MultEnroll Fingerprint");
}

void Reset_callback()
{
  delay_ms(500);
  Serial.println("Reset Fingerprint");
  PS_Cancel();
  delay(500);
  PS_Sleep();
  attachInterrupt(digitalPinToInterrupt(14),InterruptFun,RISING);
}

void disconnect_callback()
{
  ErrorNum = 0;
  delay_ms(500);
  Serial.println("WiFi Disable");
  WiFi_Connected_State = 1;
  delay_ms(500);
  Serial.println("WiFi Connected");
}

void ON_callback()
{
  delay_ms(500);
  Serial.println("ON Relay");
  digitalWrite(15,HIGH);
}

void OFF_callback()
{
  delay_ms(500);
  Serial.println("OFF Relay");
  digitalWrite(15,LOW);
}

void DataRead(const String & data)
{
  PageID = data.toInt();
  Serial.println(PageID);
  ScanState |= 1<<1;
}
/****************************************************************以上是点灯科技APP里面按键等组件的实现************************************************************/



void setup()
{  
  Serial.begin(115200);  // 和电脑连接的串口
  mySerial.begin(57600);  //软串口波特率，默认FPM383C指纹模块的57600，所以不需要动它

  pinMode(15,OUTPUT);  //ESP32S2 Builtin LED内置的灯引脚模式
  pinMode(14,INPUT);  //FPM383C的2脚TouchOUT引脚，用于外部中断

  delay_ms(200);  //用于FPM383C模块启动延时，不可去掉
  PS_Sleep();
  delay_ms(200);

  attachInterrupt(digitalPinToInterrupt(14),InterruptFun,RISING);  //外部中断初始化

  Keyboard.begin();
  USB.begin();

  delay_ms(3000);
  Serial.println("Initial Process Successed.");
}

void loop()
{
  if (Serial.available() > 0)  //判读是否串口有数据
  {
    String cmd = Serial.readStringUntil('\n');
    if (cmd == "OneEnroll") OneEnroll_callback();         //单次注册按钮
    else if (cmd == "Delete") Delete_callback();          //删除指纹按钮
    else if (cmd == "Identify") Identify_callback();      //搜索模式按钮
    else if (cmd == "Empty") Empty_callback();            //清空指纹按钮
    else if (cmd == "MultEnroll") MultEnroll_callback();  //连接注册按钮
    else if (cmd == "Reset") Reset_callback();            //复位模块按钮
    else if (cmd == "disconnect") disconnect_callback();  //断开WiFi按钮
    else if (cmd == "ON") ON_callback();                  //手动开启继电器按钮
    else if (cmd == "OFF") OFF_callback();                //手动关闭继电器按钮
    else DataRead(cmd);
  }
  switch (ScanState)
  {
    //第一步
    case 0x10:  //搜索指纹模式
        SEARCH_ACK_CHECK(PS_Identify());
        delay(1000);
        PS_Sleep();
        ScanState = 0x00;
        attachInterrupt(digitalPinToInterrupt(14),InterruptFun,RISING);
    break;

    //第二步
    case 0x11:  //指纹中断提醒输入指纹ID，执行完毕返回搜索指纹模式
        Serial.println("Please Enter ID First");
        PS_ControlLED(PS_RedLEDBuffer);
        delay(1000);
        PS_Sleep();
        ScanState = 0x00;
        attachInterrupt(digitalPinToInterrupt(14),InterruptFun,RISING);
    break;
    
    //第三步
    case 0x12:  //指纹中断提醒按下功能按键，执行完毕返回搜索指纹模式
        Serial.println("Please Press Enroll or Delete Key");
        PS_ControlLED(PS_RedLEDBuffer);
        delay(1000);
        PS_Sleep();
        ScanState = 0x00;
        attachInterrupt(digitalPinToInterrupt(14),InterruptFun,RISING);
    break;

    //第四步
    case 0x13:    //连续搜索指纹模式，每次搜索前都必须由APP发送指纹ID，由函数将ScanState bit1置位才进入下一次搜索，否则提醒输入指纹ID并返回搜索模式
        ENROLL_ACK_CHECK(PS_Enroll(PageID));
        delay(1000);
        PS_Sleep();
        ScanState = 0x01;
        attachInterrupt(digitalPinToInterrupt(14),InterruptFun,RISING);
    break;

    //第五步
    case 0x14:    //指纹中断提醒输入指纹ID，执行完毕返回搜索指纹模式
        ScanState = 0x11;   //返回第二步，提示输入指纹ID
    break;

    //第六步
    case 0x16:    //单次指纹注册模式，必须同时满足按下单次注册按键且已输入ID情况下才会执行
        ENROLL_ACK_CHECK(PS_Enroll(PageID));
        delay(1000);
        PS_Sleep();
        ScanState = 0x00;
        attachInterrupt(digitalPinToInterrupt(14),InterruptFun,RISING);
    break;

    //第七步
    case 0x08:    //指纹中断提醒输入指纹ID，执行完毕返回搜索指纹模式
        ScanState = 0x11;   //返回第二步，提示输入指纹ID
    break;

    //第八步
    case 0x0A:    //单独指纹删除模式
        if(PS_Delete(PageID) == 0x00)
        {
          Serial.println("Delete Success");
          PS_ControlLED(PS_GreenLEDBuffer);
        }
        ScanState = 0x00;
    break;

    default:    //输入错误次数大于等于5次，将重新开启WiFI功能。
        if(WiFi_Connected_State == 0 || ErrorNum >= 5)
        {
          ; 
        }
    break;
  }
}