#include "gps.h"
#include "FIFOBuffer.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

typedef struct{
	//原始資料欄位
	float time;	
	float oriLat;
	char dirLat;
	float oriLng;
	char dirLng;
	
	uint8_t quality;
	uint8_t satNum;
	float HDOP;
	float altitude;
	char altitudeUnit;
	float geoHeight;
	char geoHeightUnit;
	float sinceLastUpdate;
	char stationID[10];
	char checkSum[2];	
	
	//依原始資料運算後的經緯度
	float lat;
	float lng;
	char valid;
	
}GPGGA_t;


extern UART_HandleTypeDef huart3;
static FIFOBufferInstance g_GPSBuffer;
static GPGGA_t g_GPSInfo;
static unsigned char g_GPSStart = 0;


enum MessageType{
	GNGGA,
	GNRMC,
	UNKNOWN
};

static unsigned char g_GPSInData;

void StartGPSReceive(){
	HAL_UART_Receive_IT(&huart3,&g_GPSInData,1);
	g_GPSStart = 1;
}

void StopGPSReceive(){
	HAL_UART_AbortReceive_IT(&huart3);
	g_GPSStart = 0;
}


void ConvertLatLng(){
	double latDeg,latMin;
	double lngDeg,lngMin;
 
  latMin = fmod((double)g_GPSInfo.oriLat, 100.0);
	lngMin = fmod((double)g_GPSInfo.oriLng, 100.0);
	latDeg = (int)(g_GPSInfo.oriLat*0.01f);
	lngDeg = (int)(g_GPSInfo.oriLng*0.01f);
	
	g_GPSInfo.lat = latDeg+latMin/60;
	if(g_GPSInfo.dirLat == 'S') g_GPSInfo.lat = -g_GPSInfo.lat;
	
	g_GPSInfo.lng = lngDeg+lngMin/60;
	if(g_GPSInfo.dirLng == 'W') g_GPSInfo.lng = -g_GPSInfo.lng;
 
	g_GPSInfo.valid = 1;
	
}

unsigned char ComparePrefix(char* str, char* prefix, int len){
	int i=0;
	for(i=0;i<len;i++){
		if(str[i] != prefix[i]) return 0;
	}
	return 1;
}

unsigned char ParseGPSInfo(char* data, int len){
	int i=0,termID=0,termStart=0,termEnd=0;
	char term[32] = {0};
	unsigned char sum = 0,checkSum=0;
	enum MessageType type = UNKNOWN;
	
	for(i=0;i<len;i++){
		if(data[i] == '*') break;
		sum ^= (unsigned char)data[i];
	}
	
	if(ComparePrefix(data,"GNGGA",5)){
		type = GNGGA;
	}
	else if(ComparePrefix(data,"GNRMC",5)){
		type = GNRMC;
	}
	switch(type){
		case GNGGA:
			for(i=0;i<len;i++){
				if(data[i] == ','){
					termEnd = i;
					if(termEnd == termStart){
						strcpy(term,"");
					}
					else{
						strncpy(term,data+termStart,termEnd-termStart);
					}
					switch(termID){
						case 0:
							break;
						case 1:
							g_GPSInfo.time = atof(term);
							break;
						case 2:
							if(strcmp(term,"") == 0){
								g_GPSInfo.oriLat = -9999;
							}
							else g_GPSInfo.oriLat = atof(term);
							break;
						case 3:
							g_GPSInfo.dirLat = data[termStart];
							break;
						case 4:
							if(strcmp(term,"") == 0){
								g_GPSInfo.oriLng = -9999;
							}
							else g_GPSInfo.oriLng = atof(term);
							break;
						case 5:
							g_GPSInfo.dirLng = data[termStart];
							break;
						case 6:
							g_GPSInfo.quality = atoi(term);
							break;
						case 7:
							g_GPSInfo.satNum = atoi(term);
							break;
						case 8:
							g_GPSInfo.HDOP = atof(term);
							break;
						case 9:
							g_GPSInfo.altitude = atof(term);
							break;
						case 10:
							g_GPSInfo.altitudeUnit = data[termStart];
							break;
						case 11:
							g_GPSInfo.geoHeight = atof(term);
							break;
						case 12:
							g_GPSInfo.geoHeightUnit = data[termStart];
							break;
						case 13:
							g_GPSInfo.sinceLastUpdate = atof(term);
							break;
						case 14:
							strcpy(g_GPSInfo.stationID,term);
							break;
					}
					termID++;
					termStart = i+1;
				}
				else if(data[i] == '*'){
					strncpy(g_GPSInfo.checkSum,data+i+1,2);
				}
			}
			checkSum = strtol(g_GPSInfo.checkSum, NULL, 16);
			if(g_GPSInfo.quality != 0 && sum == checkSum){
				ConvertLatLng();
				return 1;
			}
			break;
		case GNRMC:
			break;
		default:
			break;
	}

	return 0;
}

void ProcessGPS(){
	int i=0, dataLen = 0, processed = -1;
	int len = FIFOBufferGetDataSize(&g_GPSBuffer);
	char data[256] = {0};
	unsigned char d,parseStatus = 0;
	
	//取得gps指令頭尾位置
	for(i=0;i<len;i++){
		FIFOBufferPeekData(&g_GPSBuffer,&d,i);
		
		switch(parseStatus){
			case 0:	//尋找gps指令開頭
				if(d == '$'){
					parseStatus = 1;
				}
				break;
			case 1:	//尋找gps指令結尾
				data[dataLen++] = d;
			
				if(d == '\n'){
					processed = i;
					parseStatus = 0;
					ParseGPSInfo(data,dataLen);
				}
				break;
		}
	}
	//清空已處理的部分
	if(processed > 0){
		FIFOBufferClear(&g_GPSBuffer,processed);
	}
	
	//有時候接收會斷掉，這邊定時重啟接收
	if(g_GPSStart){
		HAL_UART_Receive_IT(&huart3,&g_GPSInData,1);
	}

}

void ReceiveGPSInfo(UART_HandleTypeDef *UartHandle){
	//繼續等下一筆資料
	HAL_UART_Receive_IT(&huart3,&g_GPSInData,1);

	if(UartHandle->Instance != USART3) return;
	//將讀到的資料存到gps buffer
	FIFOBufferPutData(&g_GPSBuffer,&g_GPSInData,1);
	
	//echo回去
	//HAL_UART_Transmit_IT(&huart1, &g_InData,1);
}

unsigned char GetGPSPos(float* lat, float* lng){
	if(g_GPSInfo.valid){
		*lat = g_GPSInfo.lat;
		*lng = g_GPSInfo.lng;
	}
	else{
		*lat = -9999;
		*lng = -9999;
	}
	return g_GPSInfo.valid;
}
