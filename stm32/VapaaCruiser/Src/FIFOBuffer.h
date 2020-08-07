#ifndef FIFO_BUFFER_H
#define FIFO_BUFFER_H

#define BUFFER_SIZE 256

//buffer存資料，first in first out，circular buffer
//first為資料開頭的index，last為資料結尾的index+1
typedef struct{
	unsigned char buffer[BUFFER_SIZE];
	short first, last;
}FIFOBufferInstance;

//初始化instance
void FIFOBufferInit(FIFOBufferInstance* instance);

//取得目前存在buffer的資料size
unsigned short FIFOBufferGetDataSize(FIFOBufferInstance* instance);

//讀取buffer中第index個資料到dest，但是不將資料從buffer取出
//若index超出buffer資料範圍回傳0，反之回傳1
unsigned char FIFOBufferPeekData(FIFOBufferInstance* instance, unsigned char* dest,unsigned short index);

//讀取num大小的資料到dest，並且將資料從buffer取出
//若num大於buffer中有的資料大小回傳0(且不讀出資料)，反之回傳num
unsigned char FIFOBufferGetData(FIFOBufferInstance* instance, unsigned char* dest, unsigned short num);

//從src中寫入num大小的資料到dest
//若寫不下(buffer已滿)則不寫入並回傳0，反之回傳num
unsigned char FIFOBufferPutData(FIFOBufferInstance* instance, unsigned char* src, unsigned short num);

//清空buffer中num個資料
void FIFOBufferClear(FIFOBufferInstance* instance, unsigned short num);

#endif
