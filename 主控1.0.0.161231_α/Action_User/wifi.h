#include "stm32f4xx.h"
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//用户配置区

////连接端口号:8086,可自行修改为其他端口.
//const u8* portnum=(u8*)"8086";
//const u8* ipp=(u8*)"192.168.4.2";

//WIFI STA模式,设置要去连接的路由器无线参数,请根据你自己的路由器设置,自行修改.
//const u8* wifista_ssid=(u8*)"ALIENTEK";			//路由器SSID号
//const u8* wifista_encryption=(u8*)"wpawpa2_aes";	//wpa/wpa2 aes加密方式
//const u8* wifista_password=(u8*)"15902020353"; 	//连接密码

////WIFI AP模式,模块对外的无线参数,可自行修改.
//const u8* wifiap_ssid=(u8*)"ATK-ESP8266";			//对外SSID号
//const u8* wifiap_encryption=(u8*)"wpawpa2_aes";	//wpa/wpa2 aes加密方式
//const u8* wifiap_password=(u8*)"12345678"; 		//连接密码 

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
////4个网络模式
//const u8 *ATK_ESP8266_CWMODE_TBL[3]={(u8*)"STA模式 ",(u8*)"AP模式 ",(u8*)"AP&STA模式 "};	//ATK-ESP8266,3种网络模式,默认为路由器(ROUTER)模式 
////4种工作模式
//const u8 *ATK_ESP8266_WORKMODE_TBL[3]={(u8*)"TCP服务器",(u8*)"TCP客户端",(u8*)" UDP 模式"};	//ATK-ESP8266,4种工作模式
////5种加密方式
//const u8 *ATK_ESP8266_ECN_TBL[5]={(u8*)"OPEN",(u8*)"WEP",(u8*)"WPA_PSK",(u8*)"WPA2_PSK",(u8*)"WPA_WAP2_PSK"};
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 

u8* atk_8266_check_cmd(u8 *str);
u8 atk_8266_send_cmd(u8 *cmd,u8 *ack,u16 waittime);
u8 atk_8266_send_data(u8 *data,u8 *ack,u16 waittime);
u8 atk_8266_quit_trans(void);
void atk_8266_at_response(u8 mode);
void atk_8266_init(void);
u8 atk_8266_wifiap_test(void);

