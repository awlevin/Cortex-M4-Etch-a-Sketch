// Copyright (c) 2015-16, Joe Krachey
// All rights reserved.
//
// Redistribution and use in source or binary form, with or without modification,
// are permitted provided that the following conditions are met:
//
// 1. Redistributions in source form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "main.h"
#include "timers.h"
#include "ps2.h"
#include "launchpad_io.h"
#include "lcd.h"

//***********VARIABLES********************//
char group[] = "Group36";
char individual_1[] = "Shyamal Anadkat";
char individual_2[] = "Aaron Levin";
char individual_3[] = "Sneha Patri";

extern volatile bool Alert_Timer0A;
extern volatile bool Alert_Timer0B;
extern volatile bool Alert_ADC0_Conversion;

uint32_t curr_lcdX = 119;
uint32_t curr_lcdY = 159;
uint32_t prev_lcdX = 119;
uint32_t prev_lcdY = 159;
uint16_t curr_x_px;
uint16_t curr_y_px;

static int timer0A_count = 0;
static int timer0B_count = 0;
uint16_t draw_color = LCD_COLOR_GREEN;
uint16_t move_color = LCD_COLOR_RED;
uint32_t pixels[10][240];

uint16_t x_left_threshold = (0xFFF / 4) * 3;
uint16_t y_up_threshold = (0xFFF / 4) * 3;
uint16_t x_right_threshold = (0xFFF / 4);
uint16_t y_down_threshold = (0xFFF / 4);

volatile bool right;
volatile bool up;
volatile bool down; 
volatile bool left; 

//************ENUMS********************//
//@Deprecated
typedef enum {
  DEBOUNCE_ONE,
  DEBOUNCE_1ST_ZERO,
  DEBOUNCE_2ND_ZERO,
	DEBOUNCE_3RD_ZERO,
	DEBOUNCE_4TH_ZERO,
	DEBOUNCE_5TH_ZERO,
  DEBOUNCE_PRESSED
}
DEBOUNCE_STATES;

typedef enum {
  MOVE,
  DRAW
}
MODE_STATES;

static MODE_STATES mode = DRAW;
static DEBOUNCE_STATES state = DEBOUNCE_ONE;

//*****************************************************************************
//*****************************************************************************
bool sw1_debounce_fsm() {
	
	bool logic_level = lp_io_read_pin(SW1_BIT);

	switch(state) {
		case DEBOUNCE_ONE:
			if(logic_level == 0) state = DEBOUNCE_2ND_ZERO;
			else state = DEBOUNCE_ONE;
		case DEBOUNCE_1ST_ZERO:
			if(logic_level == 0) state = DEBOUNCE_3RD_ZERO;
			else state = DEBOUNCE_ONE;
		case DEBOUNCE_2ND_ZERO:
			if(logic_level == 0) state = DEBOUNCE_4TH_ZERO;
			else state = DEBOUNCE_ONE;
		case DEBOUNCE_3RD_ZERO:
			if(logic_level == 0) state = DEBOUNCE_4TH_ZERO;
			else state = DEBOUNCE_ONE;
		case DEBOUNCE_4TH_ZERO:
			if(logic_level == 0) state = DEBOUNCE_5TH_ZERO;
			else state = DEBOUNCE_ONE;
		case DEBOUNCE_5TH_ZERO:
			if(logic_level == 0) state = DEBOUNCE_PRESSED;
			else state = DEBOUNCE_ONE;
		case DEBOUNCE_PRESSED:
			if(logic_level == 0) {
				state = DEBOUNCE_PRESSED;
				return true;
			}
			else state = DEBOUNCE_ONE;
	}
	
	return false;
}




void initialize_hardware(void) {
  initialize_serial_debug();

  // Initialize switches and LEDs
  lp_io_init();

  // Enable PS2 joystick
  ps2_initialize_hw3();

  // Enable TIMER0 A and B for HW3
  timer_config_hw3();

  // Start TIMER0 A and B for HW3
  timer_start_hw3();

  // Configure the LCD and set screen to be black
  lcd_config_gpio();
  lcd_config_screen();
  lcd_clear_screen(LCD_COLOR_BLACK);

  memset(pixels, 0, 600);
}

// updates the pixel map
void update_lcd_shadow_map(uint32_t xpx, uint32_t ypx) {
  pixels[xpx / 32][ypx] |= (1 << (xpx % 32));
}

bool read_lcd(uint32_t xpx, uint32_t ypx) {
  if ((pixels[xpx / 32][ypx] & (1 << (xpx % 32))) == 1)
    return true;
  else
    return false;
}

bool move_x_pixels_left(uint32_t * prev, uint32_t * curr) {

  * prev = * curr;
  if (curr_x_px >= x_left_threshold  && (*curr <= 239)) 
		{ * curr += 1;
    return true;
  } else {
    return false;
  }
}

bool move_y_pixels_up(uint32_t * prev, uint32_t * curr) {

  * prev = * curr;
  if (curr_y_px >= y_up_threshold && (*curr <= 319) ) 
		{ * curr += 1;
    return true;
  } else if (*curr >= 319) {
		curr_lcdX = 0; 
	}
   else {
    return false;
  }
}


bool move_y_pixels_down(uint32_t * prev, uint32_t * curr) {
	  if (curr_y_px <= y_down_threshold) 
			{ * curr -= 1;
    return true;
  } else {
    return false;
  }
	
}


bool move_x_pixels_right(uint32_t * prev, uint32_t * curr){
	if (curr_x_px <= x_right_threshold) 
		{ * curr -= 1;
    return true;
  } else {
    return false;
  }
}

void print_ps2(uint16_t x_data, uint16_t y_data) {
  uint32_t i;
  char msg[80];

  sprintf(msg, "X Dir value : 0x%03x Y Dir value : 0x%03x\r", x_data, y_data);
  put_string(msg);
}

//*****************************************************************************
//*****************************************************************************
int
main(void) {
  initialize_hardware();

  put_string("\n\r");
  put_string("************************************\n\r");
  put_string("ECE353 - Fall 2016 HW3\n\r  ");
  put_string(group);
  put_string("\n\r     Name:");
  put_string(individual_1);
  put_string("\n\r     Name:");
  put_string(individual_2);
  put_string("\n\r     Name:");
  put_string(individual_3);
  put_string("\n\r");
  put_string("************************************\n\r");

  lcd_draw_pixel(curr_lcdX, 1, curr_lcdY, 1, draw_color);
	
	
  // Reach infinite loop
  while (1) {

		// TIMER0A HANDLER
    if (Alert_Timer0A) {
      timer0A_count++;
      //if( sw1_debounce_fsm() ) {
			//	mode = ~mode;
			//}
      Alert_Timer0A = false;
    }

		// TIMER0B HANDLER
    if (Alert_Timer0B) {
      timer0B_count++;
			  
			// Start SS2 conversion
      ADC0 -> PSSI |= ADC_PSSI_SS2;
			
      Alert_Timer0B = false;
    }
		
		// ADC COMPUTATION HANDLER (UPDATES PS2 X/Y VALUES)
    if (Alert_ADC0_Conversion) {
			
			// Toggle ADC0 Conversion notifier
			Alert_ADC0_Conversion = false;
      
      // Update current x position with current PS2 joystick ADC value
      curr_y_px = ADC0 -> SSFIFO2 & ADC_SSFIFO2_DATA_M;

      // Update current y position with current PS2 joystick ADC value
      curr_x_px = ADC0 -> SSFIFO2 & ADC_SSFIFO2_DATA_M;

      // TESTING PS2 values. WORKS NOW >>>
      print_ps2(curr_x_px, curr_y_px);
    }

    // If interrupt A has occurred 10 times -> TOGGLE BLUE LED
    if (timer0A_count == 10) {

      // Toggle Blue LED
      if (!lp_io_read_pin(BLUE_BIT)) {
        lp_io_set_pin(BLUE_BIT);
      } else {
        lp_io_clear_pin(BLUE_BIT);
      }

      // Reset timer0A count
      timer0A_count = 0;
    }

    // If interrupt B has occurred 10 times (TOGGLE GREEN LED)
    if (timer0B_count == 10) {

      // Reset timer0B alert
      Alert_Timer0B = false;

      // Toggle Green LED
      if (!lp_io_read_pin(GREEN_BIT)) {
        lp_io_set_pin(GREEN_BIT);
      } else {
        lp_io_clear_pin(GREEN_BIT);
      }

      // Reset timer0B count
      timer0B_count = 0;
    }
		
	//if(mode == DRAW){
				up = move_y_pixels_up( &prev_lcdY, &curr_lcdY);
				down = move_y_pixels_down( &prev_lcdY, &curr_lcdY);
				right = move_x_pixels_right( &prev_lcdX, &curr_lcdX);
				left = move_x_pixels_left( &prev_lcdX, &curr_lcdX); 
			
			  if (left) {
          lcd_draw_pixel(curr_lcdX, 1, curr_lcdY, 1, draw_color);
          update_lcd_shadow_map(curr_lcdX, curr_lcdY);			
		    }
				
			  if (right) {
          lcd_draw_pixel(curr_lcdX, 1, curr_lcdY, 1, draw_color);
          update_lcd_shadow_map(curr_lcdX, curr_lcdY);			
		    }
								
        if (up) {
          lcd_draw_pixel(curr_lcdX, 1, curr_lcdY, 1, draw_color);
          update_lcd_shadow_map(curr_lcdX, curr_lcdY);			
		    }
				
        if (down) {
          lcd_draw_pixel(curr_lcdX, 1, curr_lcdY, 1, draw_color);
          update_lcd_shadow_map(curr_lcdX, curr_lcdY);			
		    }
					
				lcd_draw_pixel(curr_lcdX, 1, curr_lcdY, 1, draw_color);
        update_lcd_shadow_map(curr_lcdX, curr_lcdY);			





		/*
	//}	else {
			
		if (move_x_pixels( &prev_lcdX, &curr_lcdX) || move_y_pixels( &prev_lcdY, &curr_lcdY)) {
			if(read_lcd(prev_lcdX, prev_lcdY)) {
				lcd_draw_pixel(prev_lcdX, 1, prev_lcdY, 1, draw_color);
				
			} else {
					lcd_draw_pixel(prev_lcdX, 1, prev_lcdY, 1, LCD_COLOR_BLACK);
			}
		}
	}*/

	}
			

}
