#include "FIFOBuffer.h"
#include "string.h"

void FIFOBufferInit(FIFOBufferInstance* instance){
	instance->first = 0;
	instance->last = 0;
	memset(instance->buffer,0,BUFFER_SIZE);
}

unsigned short FIFOBufferGetDataSize(FIFOBufferInstance* instance){
	return (instance->last-instance->first+BUFFER_SIZE)%BUFFER_SIZE;
}

unsigned char FIFOBufferPeekData(FIFOBufferInstance* instance, unsigned char* dest, unsigned short index){
	unsigned short size = FIFOBufferGetDataSize(instance);
	if(index>=size) return 0;
	*dest = instance->buffer[(instance->first+index)%BUFFER_SIZE];
	return 1;
}

unsigned char FIFOBufferGetData(FIFOBufferInstance* instance, unsigned char* dest, unsigned short num){
	unsigned short size = FIFOBufferGetDataSize(instance);
	int i=0;
	if(size<num) return 0;

	for(i=0;i<num;i++){
		dest[i] = instance->buffer[instance->first];
		instance->first = (instance->first+1)%BUFFER_SIZE;
	}
	return num;
}

unsigned char FIFOBufferPutData(FIFOBufferInstance* instance, unsigned char* src, unsigned short num){
	unsigned short size = FIFOBufferGetDataSize(instance);
	int i=0;
	//�����ɭ�size�OBUFFER_SIZE-1�A(last+1)%MAX_BUFFER == first
	//���M�Ÿ򺡪��ɭԳ��Ofirst==last�A�L�k�Ϥ�
	if(size+num>=BUFFER_SIZE) return 0;

	for(i=0;i<num;i++){
		instance->buffer[instance->last] = src[i];
		instance->last = (instance->last+1)%BUFFER_SIZE;
	}
	return num;
}

void FIFOBufferClear(FIFOBufferInstance* instance, unsigned short num){
	if(num <= 0) return;
	unsigned short size = FIFOBufferGetDataSize(instance);
	if(num > size) num = size;
	instance->first = (instance->first+num)%BUFFER_SIZE;
}
