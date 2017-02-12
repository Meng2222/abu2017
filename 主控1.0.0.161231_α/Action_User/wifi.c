#include "wifi.h"
#include "usart.h"
#include <stdio.h>
#include "timer.h"
#include <string.h>
#include "GET_SET.h"

//���Ӷ˿ں�:8086,�������޸�Ϊ�����˿�.
const u8* portnum=(u8*)"8086";
const u8* ipp=(u8*)"192.168.4.2";

//WIFI APģʽ,ģ���������߲���,�������޸�.
const u8* wifiap_ssid=(u8*)"ATK-ESP8266";			//����SSID��
const u8* wifiap_encryption=(u8*)"wpawpa2_aes";	//wpa/wpa2 aes���ܷ�ʽ
const u8* wifiap_password=(u8*)"12345678"; 		//�������� 

//ATK-ESP8266���������,�����յ���Ӧ��
//str:�ڴ���Ӧ����
//����ֵ:0,û�еõ��ڴ���Ӧ����
//    ����,�ڴ�Ӧ������λ��(str��λ��)
u8* atk_8266_check_cmd(u8 *str)
{
	
	char *strx=0;
	if(USART5_RX_STA&0X8000)		//���յ�һ��������
	{ 
		USART5_RX_BUF[USART5_RX_STA&0X7FFF]=0;//��ӽ�����
		strx=strstr((const char*)USART5_RX_BUF,(const char*)str);
	} 
	return (u8*)strx;
}

//��ATK-ESP8266��������
//cmd:���͵������ַ���
//ack:�ڴ���Ӧ����,���Ϊ��,���ʾ����Ҫ�ȴ�Ӧ��
//waittime:�ȴ�ʱ��(��λ:10ms)
//����ֵ:0,���ͳɹ�(�õ����ڴ���Ӧ����)
//       1,����ʧ��
u8 atk_8266_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	u8 res=0; 
	USART5_RX_STA=0;
	u5_printf("%s\r\n",cmd);	//��������
	if(ack&&waittime)		//��Ҫ�ȴ�Ӧ��
	{
		while(--waittime)	//�ȴ�����ʱ
		{
			TIM_Delayms(TIM5,10);//delay_ms(10);
			if(USART5_RX_STA&0X8000)//���յ��ڴ���Ӧ����
			{
				if(atk_8266_check_cmd(ack))
				{
					//printf("ack:%s\r\n",(u8*)ack);
					break;//�õ���Ч���� 
				}
					USART5_RX_STA=0;
			} 
		}
		if(waittime==0)res=1; 
	}
	return res;
}

//��ATK-ESP8266����ָ������
//data:���͵�����(����Ҫ��ӻس���)
//ack:�ڴ���Ӧ����,���Ϊ��,���ʾ����Ҫ�ȴ�Ӧ��
//waittime:�ȴ�ʱ��(��λ:10ms)
//����ֵ:0,���ͳɹ�(�õ����ڴ���Ӧ����)luojian
u8 atk_8266_send_data(u8 *data,u8 *ack,u16 waittime)
{
	u8 res=0; 
	USART5_RX_STA=0;
	u5_printf("%s",data);	//��������
	if(ack&&waittime)		//��Ҫ�ȴ�Ӧ��
	{
		while(--waittime)	//�ȴ�����ʱ
		{
			TIM_Delayms(TIM5,10);//delay_ms(10);
			if(USART5_RX_STA&0X8000)//���յ��ڴ���Ӧ����
			{
				if(atk_8266_check_cmd(ack))break;//�õ���Ч���� 
				USART5_RX_STA=0;
			} 
		}
		if(waittime==0)res=1; 
	}
	return res;
}

//ATK-ESP8266�˳�͸��ģʽ
//����ֵ:0,�˳��ɹ�;
//       1,�˳�ʧ��
u8 atk_8266_quit_trans(void)
{
	while((UART5->SR&0X40)==0);	//�ȴ����Ϳ�
	UART5->DR='+';      
	TIM_Delayms(TIM5,15);//delay_ms(15);					//���ڴ�����֡ʱ��(10ms)
	while((UART5->SR&0X40)==0);	//�ȴ����Ϳ�
	UART5->DR='+';      
	TIM_Delayms(TIM5,15);//delay_ms(15);					//���ڴ�����֡ʱ��(10ms)
	while((UART5->SR&0X40)==0);	//�ȴ����Ϳ�
	UART5->DR='+';      
	TIM_Delayms(TIM5,500);//delay_ms(500);					//�ȴ�500ms
	return atk_8266_send_cmd("AT","OK",20);//�˳�͸���ж�.
}

//usmart֧�ֲ���
//���յ���ATָ��Ӧ�����ݷ��ظ����Դ���
//mode:0,������USART3_RX_STA;
//     1,����USART3_RX_STA;
void atk_8266_at_response(u8 mode)
{
	if(USART5_RX_STA&0X8000)		//���յ�һ��������
	{ 
		USART5_RX_BUF[USART5_RX_STA&0X7FFF]=0;//��ӽ�����
		//printf("%s",USART5_RX_BUF);	//���͵�����
		if(mode)USART5_RX_STA=0;
	} 
}


//ATK-ESP8266ģ�����������
void atk_8266_init(void)
{
	u8 res=0;	
  char p[128];//����32�ֽ��ڴ�
	while(atk_8266_send_cmd("AT","OK",20))//���WIFIģ���Ƿ�����
	{
		atk_8266_quit_trans();//�˳�͸��
		atk_8266_send_cmd("AT+CIPMODE=0","OK",200);  //�ر�͸��ģʽ	
	} 
	while(atk_8266_send_cmd("ATE0","OK",20));//�رջ���
		
	atk_8266_send_cmd("AT+CWMODE=2","OK",20);
	atk_8266_send_cmd("AT+RST","OK",20);
	TIM_Delayms(TIM5,4000);//delay_ms(1000);//��ʱ2s�ȴ�ģ������
////	delay_ms(1000);//
////	delay_ms(1000);
////	delay_ms(1000);
	sprintf((char*)p,"AT+CWSAP=\"%s\",\"%s\",1,4",wifiap_ssid,wifiap_password);    //����ģ��APģʽ���߲���
	atk_8266_send_cmd(p,"OK",1000);

	atk_8266_send_cmd("AT+CIPMUX=0","OK",20);   //0�������ӣ�1��������
	sprintf((char*)p,"AT+CIPSTART=\"TCP\",\"%s\",%s",(u8*)ipp,(u8*)portnum);    //����Ŀ��TCP������
  while(atk_8266_send_cmd(p,"OK",200));

	atk_8266_send_cmd("AT+CIPMODE=1","OK",200);      //����ģʽΪ��͸��	
	
	atk_8266_quit_trans();
	atk_8266_send_cmd("AT+CIPSEND","OK",20);       //��ʼ͸��
	
	//atk_8266_wifiap_test();	//WIFI AP����
		
}

//ATK-ESP8266 WIFI AP����
//���ڲ���TCP/UDP����
//����ֵ:0,����
//    ����,�������
u8 atk_8266_wifiap_test(void)
{
	//u8 *p;
	//char p[32]="dddddd";  //����32�ֽ��ڴ�
	u8 res=0;		
  int a=(int)(GetPosX()*100);
  int b=(int)(GetPosY()*100);
	int c=(int)(100*GetAngle());
//	
//while(1)
//{
	if(res<100)res++;
	else res=0;
	
  u5_printf("s=%d\r\n",a);
	res++;
	u5_printf("s=%d\r\n",b);
	res++;
	u5_printf("s=%d\r\n",c);
//}
					
	return res;		
} 
