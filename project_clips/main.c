#define F_CPU 7372800UL
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#include "myLCD_new.h"


int prevState=0, currentState=0; //prev prodtype and current prodtype

uint16_t adc_val0;

uint16_t read_adc(unsigned int adc_channel)
{
	ADMUX &= 0xE0;

	ADMUX |= adc_channel;

	ADCSRA |= (1 << ADSC);

	while (bit_is_clear(ADCSRA, ADIF))
	{
		; //idk do nothing 
	}

	ADCSRA |= (1 << ADIF);

	return ADCW;
}

/* 
SETTING: NHAP GIA TRI ADC TUONG UNG VOI SAN PHAM VAO DAY
	range:
	size 3: 1-> 128
	size 2: 129-> 213
	size 1: 214-> 255
*/
int prod_type(int adc_val){
	if (adc_val<64) return 0; //empty
	else if (adc_val<128) return 3;
	else if (adc_val<213) return 2;
	else return 1;
}

// enqueue - dequeue product

typedef struct Node {
	int prodType;   // product type number (1-3)
	int id;      // object number
	struct Node* next;
} Node;

Node* front = NULL;
Node* rear  = NULL;

int objectCounter = 0;

//add product type to queue
void enqueue(int val) {
	Node* newNode = (Node*)malloc(sizeof(Node));
	if (newNode == NULL)
	{
		return;
	}

	objectCounter++;
	newNode->id = objectCounter; //object number... how many
	newNode->prodType = val; // type of product
	newNode->next = NULL; //points to next object (null)

	if (rear == NULL) { 
		front = rear = newNode;
		} else {
		rear->next = newNode;
		rear = newNode;
	}
}


//remove product type from queue
int dequeue() {
	if (front == NULL) return -1;

	Node* temp = front;

	int val = temp->prodType;
	int id  = temp->id;

	front = front->next;
	
	if (front == NULL) rear = NULL;

	free(temp);
	return val;
}





int main(void)
{
	init_LCD();
	clr_LCD();
	
	ADMUX |= (1 << REFS0);
	ADCSRA |= (1 << ADEN) |	(1 << ADPS2) |	(1 << ADPS1);
	while (1)
	{
		clr_LCD();

		adc_val0 = read_adc(0); //read adc value on pin 0

		char adc_str[16];
		move_LCD(1,1);
		prevState=currentState;
		currentState=prod_type((adc_val0));
		
		if (prevState == 0 && currentState != 0){
			if (currentState == 0)
			{
				sprintf(adc_str, "Empty      ");
			}
			else { 
				enqueue(prod_type(adc_val0)); // if detect new object, put in queue
				sprintf(adc_str, "Obj no: %4d", objectCounter);
				//update prev and current
				
			}
		putStr_LCD(adc_str);
		_delay_ms(100);
		/*
		if(prod_type(adc_val0)==0){
			sprintf(adc_str, "Empty");
		}
		else{
			sprintf(adc_str, "Type: %4d", prod_type(adc_val0));
		}
		putStr_LCD(adc_str);

		_delay_ms(100);
		*/
		}
	}
}