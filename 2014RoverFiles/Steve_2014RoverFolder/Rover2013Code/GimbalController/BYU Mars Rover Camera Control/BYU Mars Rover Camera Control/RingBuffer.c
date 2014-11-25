/****************************************************************************************
RingBuffer.c

Stuff for the Buffer
****************************************************************************************/

#include "RingBuffer.h"	
/****************************************************************************************
RingBufferInit takes a pointer to a ringBuffer and a size which is usually equal to 
BUFFER_SIZE, and initialized the ringBuffer.  The data array is not initialized.
****************************************************************************************/
void RingBufferInit(RingBuffer queue){
	queue->readPointer = 0;
	queue->writePointer = 0;
}

int RingBufferIsEmpty(RingBuffer queue){
	return (queue->writePointer == queue->readPointer);
}

int RingBufferIsFull(RingBuffer queue){
	return (((queue->writePointer + 1) % BUFFER_SIZE) == queue->readPointer);
}

int RingBufferFillCount(RingBuffer queue){
	return (((queue->writePointer + BUFFER_SIZE) - queue->readPointer) % BUFFER_SIZE);
}

int RingBufferHasRoom(RingBuffer queue, int packetSize){
	//we fill the buffer with a certain # of byte chunks, so there must be room to add them and leave an empty space
	int fillCount = RingBufferFillCount(queue);
	return (fillCount < (BUFFER_SIZE - packetSize)); 
}	
	
int RingBufferSpaceFree(RingBuffer queue){
	int fillCount = RingBufferFillCount(queue);
	return (BUFFER_SIZE - (fillCount + 1));
}

int RingBufferPeek(RingBuffer queue, unsigned char *data){
	//return the char stored at the "front" of the queue, but leave it in the array.
	if (!RingBufferIsEmpty(queue)){
		*data = queue->data[queue->readPointer];
		return 0;
	}
	else
		return 1;	
}

int RingBufferDequeueArray(RingBuffer queue, unsigned char *array, int size){
	int fillCount = RingBufferFillCount(queue);
	if (fillCount >= size){
		int i;		
		for(i = 0; i < size; i++)
			RingBufferDequeue(queue, &(array[i]));
		return 0;
	}
	else
		return 1;	
}		
		
int RingBufferDequeue(RingBuffer queue, unsigned char *data){
	//return the char stored at the "front" of the queue, and move the pointer forward
	if (!RingBufferIsEmpty(queue)){
		*data = queue->data[queue->readPointer];
		queue->readPointer++;
		queue->readPointer %= BUFFER_SIZE;
		return 0;
	}
	else
		return 1;	
}

int RingBufferMultDelete(RingBuffer queue, int numToDelete){
	int fillCount = RingBufferFillCount(queue);
	if (fillCount >= numToDelete){
		queue->readPointer += numToDelete;
		queue->readPointer %= BUFFER_SIZE;
		return 0;
	}		
	else
		return 1;
}

int RingBufferEnqueue(RingBuffer queue, unsigned char data){
	if (!RingBufferIsFull(queue)){
		queue->data[queue->writePointer] = data;
		queue->writePointer++;
		queue->writePointer %= BUFFER_SIZE;
		return 0;
	}
	else
		return 1;
}

int RingBufferEnqueueArray(RingBuffer queue, unsigned char *array, int size){
	if (RingBufferHasRoom(queue, size)){
		int i;		
		for(i = 0; i < size; i++)
			RingBufferEnqueue(queue, array[i]);
		return 0;
	}
	else
		return 1;	
}
