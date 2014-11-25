/******************************************************************************
RingBuffer.h

This contains all the functions needed to implement a RingBuffer

Dependencies:
	-none
******************************************************************************/

#ifndef __RingBuffer_H
#define __RingBuffer_H


struct RingBufferData
{
    unsigned char writePointer; // write pointer 
    unsigned char readPointer;  // read pointer
    unsigned char size; // size (max num of elements) of ring buffer 
    unsigned char data[BUFFER_SIZE]; // data array of ring buffer 
};

typedef struct RingBufferData* RingBuffer;

void RingBufferInit(RingBuffer queue);

int RingBufferEnqueue(RingBuffer queue, unsigned char data);
int RingBufferEnqueueArray(RingBuffer queue, unsigned char *array, int size);
int RingBufferDequeue(RingBuffer queue, unsigned char *data);
int RingBufferDequeueArray(RingBuffer queue, unsigned char *array, int size);
int RingBufferMultDelete(RingBuffer queue, int numToDelete);
int RingBufferPeek(RingBuffer queue, unsigned char *data);
int RingBufferIsEmpty(RingBuffer queue);
int RingBufferIsFull(RingBuffer queue);
int RingBufferFillCount(RingBuffer queue);
int RingBufferHasRoom(RingBuffer queue, int packetSize);
int RingBufferSpaceFree(RingBuffer queue);

#endif