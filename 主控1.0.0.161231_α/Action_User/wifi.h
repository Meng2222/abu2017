#include "stm32f4xx.h"
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//�û�������

////���Ӷ˿ں�:8086,�������޸�Ϊ�����˿�.
//const u8* portnum=(u8*)"8086";
//const u8* ipp=(u8*)"192.168.4.2";

//WIFI STAģʽ,����Ҫȥ���ӵ�·�������߲���,��������Լ���·��������,�����޸�.
//const u8* wifista_ssid=(u8*)"ALIENTEK";			//·����SSID��
//const u8* wifista_encryption=(u8*)"wpawpa2_aes";	//wpa/wpa2 aes���ܷ�ʽ
//const u8* wifista_password=(u8*)"15902020353"; 	//��������

////WIFI APģʽ,ģ���������߲���,�������޸�.
//const u8* wifiap_ssid=(u8*)"ATK-ESP8266";			//����SSID��
//const u8* wifiap_encryption=(u8*)"wpawpa2_aes";	//wpa/wpa2 aes���ܷ�ʽ
//const u8* wifiap_password=(u8*)"12345678"; 		//�������� 

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
////4������ģʽ
//const u8 *ATK_ESP8266_CWMODE_TBL[3]={(u8*)"STAģʽ ",(u8*)"APģʽ ",(u8*)"AP&STAģʽ "};	//ATK-ESP8266,3������ģʽ,Ĭ��Ϊ·����(ROUTER)ģʽ 
////4�ֹ���ģʽ
//const u8 *ATK_ESP8266_WORKMODE_TBL[3]={(u8*)"TCP������",(u8*)"TCP�ͻ���",(u8*)" UDP ģʽ"};	//ATK-ESP8266,4�ֹ���ģʽ
////5�ּ��ܷ�ʽ
//const u8 *ATK_ESP8266_ECN_TBL[5]={(u8*)"OPEN",(u8*)"WEP",(u8*)"WPA_PSK",(u8*)"WPA2_PSK",(u8*)"WPA_WAP2_PSK"};
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 

u8* atk_8266_check_cmd(u8 *str);
u8 atk_8266_send_cmd(u8 *cmd,u8 *ack,u16 waittime);
u8 atk_8266_send_data(u8 *data,u8 *ack,u16 waittime);
u8 atk_8266_quit_trans(void);
void atk_8266_at_response(u8 mode);
void atk_8266_init(void);
u8 atk_8266_wifiap_test(void);

