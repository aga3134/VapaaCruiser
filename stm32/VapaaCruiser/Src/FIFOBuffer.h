#ifndef FIFO_BUFFER_H
#define FIFO_BUFFER_H

#define BUFFER_SIZE 256

//buffer�s��ơAfirst in first out�Acircular buffer
//first����ƶ}�Y��index�Alast����Ƶ�����index+1
typedef struct{
	unsigned char buffer[BUFFER_SIZE];
	short first, last;
}FIFOBufferInstance;

//��l��instance
void FIFOBufferInit(FIFOBufferInstance* instance);

//���o�ثe�s�bbuffer�����size
unsigned short FIFOBufferGetDataSize(FIFOBufferInstance* instance);

//Ū��buffer����index�Ӹ�ƨ�dest�A���O���N��Ʊqbuffer���X
//�Yindex�W�Xbuffer��ƽd��^��0�A�Ϥ��^��1
unsigned char FIFOBufferPeekData(FIFOBufferInstance* instance, unsigned char* dest,unsigned short index);

//Ū��num�j�p����ƨ�dest�A�åB�N��Ʊqbuffer���X
//�Ynum�j��buffer��������Ƥj�p�^��0(�B��Ū�X���)�A�Ϥ��^��num
unsigned char FIFOBufferGetData(FIFOBufferInstance* instance, unsigned char* dest, unsigned short num);

//�qsrc���g�Jnum�j�p����ƨ�dest
//�Y�g���U(buffer�w��)�h���g�J�æ^��0�A�Ϥ��^��num
unsigned char FIFOBufferPutData(FIFOBufferInstance* instance, unsigned char* src, unsigned short num);

//�M��buffer��num�Ӹ��
void FIFOBufferClear(FIFOBufferInstance* instance, unsigned short num);

#endif
