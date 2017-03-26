#include "wifi.h"
#include "usart.h"
#include <stdio.h>
#include "timer.h"
#include <string.h>

//连接端口号:8086,可自行修改为其他端口.
const u8* portnum=(u8*)"8086";
const u8* ipp=(u8*)"192.168.4.2";

//WIFI AP模式,模块对外的无线参数,可自行修改.
const u8* wifiap_ssid=(u8*)"ATK-ESP8266";			//对外SSID号
const u8* wifiap_encryption=(u8*)"wpawpa2_aes";	//wpa/wpa2 aes加密方式
const u8* wifiap_password=(u8*)"12345678"; 		//连接密码 

//ATK-ESP8266发送命令后,检测接收到的应答
//str:期待的应答结果
//返回值:0,没有得到期待的应答结果
//    其他,期待应答结果的位置(str的位置)
u8* atk_8266_check_cmd(u8 *str)
{
	
	char *strx=0;
	if(USART5_RX_STA&0X8000)		//接收到一次数据了
	{ 
		USART5_RX_BUF[USART5_RX_STA&0X7FFF]=0;//添加结束符
		strx=strstr((const char*)USART5_RX_BUF,(const char*)str);
	} 
	return (u8*)strx;
}

//向ATK-ESP8266发送命令
//cmd:发送的命令字符串
//ack:期待的应答结果,如果为空,则表示不需要等待应答
//waittime:等待时间(单位:10ms)
//返回值:0,发送成功(得到了期待的应答结果)
//       1,发送失败
u8 atk_8266_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	u8 res=0; 
	USART5_RX_STA=0;
	u5_printf("%s\r\n",cmd);	//发送命令
	if(ack&&waittime)		//需要等待应答
	{
		while(--waittime)	//等待倒计时
		{
			TIM_Delayms(TIM5,10);//delay_ms(10);
			if(USART5_RX_STA&0X8000)//接收到期待的应答结果
			{
				if(atk_8266_check_cmd(ack))
				{
					//printf("ack:%s\r\n",(u8*)ack);
					break;//得到有效数据 
				}
					USART5_RX_STA=0;
			} 
		}
		if(waittime==0)res=1; 
	}
	return res;
}

//向ATK-ESP8266发送指定数据
//data:发送的数据(不需要添加回车了)
//ack:期待的应答结果,如果为空,则表示不需要等待应答
//waittime:等待时间(单位:10ms)
//返回值:0,发送成功(得到了期待的应答结果)luojian
u8 atk_8266_send_data(u8 *data,u8 *ack,u16 waittime)
{
	u8 res=0; 
	USART5_RX_STA=0;
	u5_printf("%s",data);	//发送命令
	if(ack&&waittime)		//需要等待应答
	{
		while(--waittime)	//等待倒计时
		{
			TIM_Delayms(TIM5,10);//delay_ms(10);
			if(USART5_RX_STA&0X8000)//接收到期待的应答结果
			{
				if(atk_8266_check_cmd(ack))break;//得到有效数据 
				USART5_RX_STA=0;
			} 
		}
		if(waittime==0)res=1; 
	}
	return res;
}

//ATK-ESP8266退出透传模式
//返回值:0,退出成功;
//       1,退出失败
u8 atk_8266_quit_trans(void)
{
	while((UART5->SR&0X40)==0);	//等待发送空
	UART5->DR='+';      
	TIM_Delayms(TIM5,15);//delay_ms(15);					//大于串口组帧时间(10ms)
	while((UART5->SR&0X40)==0);	//等待发送空
	UART5->DR='+';      
	TIM_Delayms(TIM5,15);//delay_ms(15);					//大于串口组帧时间(10ms)
	while((UART5->SR&0X40)==0);	//等待发送空
	UART5->DR='+';      
	TIM_Delayms(TIM5,500);//delay_ms(500);					//等待500ms
	return atk_8266_send_cmd("AT","OK",20);//退出透传判断.
}

//usmart支持部分
//将收到的AT指令应答数据返回给电脑串口
//mode:0,不清零USART3_RX_STA;
//     1,清零USART3_RX_STA;
void atk_8266_at_response(u8 mode)
{
	if(USART5_RX_STA&0X8000)		//接收到一次数据了
	{ 
		USART5_RX_BUF[USART5_RX_STA&0X7FFF]=0;//添加结束符
		//printf("%s",USART5_RX_BUF);	//发送到串口
		if(mode)USART5_RX_STA=0;
	} 
}


//ATK-ESP8266模块测试主函数
void atk_8266_init(void)
{
	char p[128];//申请32字节内存
	while(atk_8266_send_cmd("AT","OK",20))//检查WIFI模块是否在线
	{
		atk_8266_quit_trans();//退出透传
		atk_8266_send_cmd("AT+CIPMODE=0","OK",200);  //关闭透传模式	
	} 
	while(atk_8266_send_cmd("ATE0","OK",20));//关闭回显
		
	atk_8266_send_cmd("AT+CWMODE=2","OK",20);
	atk_8266_send_cmd("AT+RST","OK",20);
	TIM_Delayms(TIM5,4000);//延时2s等待模块重启

	sprintf((char*)p,"AT+CWSAP=\"%s\",\"%s\",1,4",wifiap_ssid,wifiap_password);    //配置模块AP模式无线参数
	atk_8266_send_cmd(p,"OK",1000);

	atk_8266_send_cmd("AT+CIPMUX=0","OK",20);   //0：单连接，1：多连接
	sprintf((char*)p,"AT+CIPSTART=\"TCP\",\"%s\",%s",(u8*)ipp,(u8*)portnum);    //配置目标TCP服务器
    while(atk_8266_send_cmd(p,"OK",200));

	atk_8266_send_cmd("AT+CIPMODE=1","OK",200);      //传输模式为：透传	
	
	atk_8266_quit_trans();
	atk_8266_send_cmd("AT+CIPSEND","OK",20);       //开始透传
	
	//atk_8266_wifiap_test();	//WIFI AP测试
		
}

//ATK-ESP8266 WIFI AP测试
//用于测试TCP/UDP连接
//返回值:0,正常
//    其他,错误代码
u8 atk_8266_wifiap_test(void)
{

	u5_printf("s=%d\r\n",100);
	u5_printf("s=%d\r\n",101);
	u5_printf("s=%d\r\n",102);
		
	return 0;		
} 
