#include "ws2812b.h"

void ws2812b_rotate( WS2812B_t *base, uint8_t num_leds) {
	uint8_t i = 0;
	
	for(i = num_leds - 1; i > 0; i--) {
		base[i] = base[i - 1];
	}
	base[0] = base[num_leds - 1];
	return;
}

void ws2812b_pulse(WS2812B_t *base, uint8_t num_leds) {
	
	// Value: 
	//				0 if decrementing
	//				1 if incrementing
	static uint8_t direction = 0;
	int i;
	
	
	WS2812B_t *currLED = base;
	
	/*
	*   If the direction is incrementing AND the value of the red field is less than
  *   0xFF, the function will increment the red filed by 1 for each structure in 
  *   the array.
	*/
	if( (direction == 1) && (currLED->red < 0xFF) ) {
		for(i = 0; i < num_leds; i++) {
			currLED[i].red += 0x01;
		}
	}
	
	/*
	*   If the direction is incrementing and the value of the red filed is equal to
	*   0xFF, the function will change the direction to be counting down and decrement
	*   the red filed by 1 for each structure in the array.  
	*/
	else if( (direction == 1) && (currLED->red == 0xFF) ) {
		// Change direction to counting down
		direction = 0;
		
		for(i = 0; i < num_leds; i++) {
			currLED[i].red -= 0x01;
		}
	}
	
	/*
	*   If the direction is decrementing AND the value of the red field is greater than
	*   0x00, the function will decrement the red filed by 1 for each structure in 
	*   the array.
	*/
	else if( (direction == 0) && (currLED->red > 0x00) ) {
		
		// Decrement each red value in the LED array by 0x01
		for(i = 0; i < num_leds; i++) {
			currLED[i].red -= 0x01;
		}
	}
	
	/*
	*   If the direction is decrementing and the value of the red filed is equal to
	*   0x00, the function will change the direction to be counting up and increment
	*   the red filed by 1 for each structure in the array. 
	*/
	else if( (direction == 0) && (currLED->red == 0x00) ) {
		// Change direction to counting up
		direction = 1;
		for(i = 0; i < num_leds; i++) {
			currLED[i].red += 0x01;
		}
	}
	return;
}
