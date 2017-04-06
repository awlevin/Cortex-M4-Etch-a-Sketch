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

char group[] = "Group02";
char individual_1[] = "Shyamal Anadkat";
char individual_2[] = "Aaron Levin";
char individual_3[] = "Sneha Patri";

extern volatile bool Alert_Timer0A;
extern volatile bool Alert_Timer0B;
extern volatile bool Alert_ADC0_Conversion;

static int sw1_debounce_counter = 0; 

static int timer0A_count = 0;
static int timer0B_count = 0;

static uint16_t curr_x_px = 319/2;
static uint16_t curr_y_px = 239/2; 


typedef enum 
{
  DEBOUNCE_ONE,
  DEBOUNCE_1ST_ZERO,
  DEBOUNCE_2ND_ZERO,
  DEBOUNCE_PRESSED
} DEBOUNCE_STATES;


typedef enum
{
	MOVE,
	DRAW
} FSM_STATES;


//*****************************************************************************
//*****************************************************************************
void initialize_hardware(void)
{
  initialize_serial_debug();
	
	// Initialize switches and LEDs
	lp_io_init();
	
	// Enable PS2 joystick
	ps2_initialize_hw3();
	
	// Configure adc sample sequencer to sample from x and y separately
	//ps2_adc_init_hw3();
	
	// Enable TIMER0 A and B for HW3
	timer_config_hw3();
	
	// Start TIMER0 A and B for HW3
	timer_start_hw3();
	
	// Configure the LCD and set screen to be black
	lcd_config_gpio();
	lcd_config_screen();
	lcd_clear_screen(LCD_COLOR_BLACK);	
}


// Debounce FSM for SW1 
bool sw1_debounce_fsm(void)
{
  static DEBOUNCE_STATES state = DEBOUNCE_ONE;
  bool pin_logic_level;
  
  pin_logic_level = lp_io_read_pin(SW1_BIT);
  
  switch (state)
  {
    case DEBOUNCE_ONE:
    {
      if(pin_logic_level)
      {
        state = DEBOUNCE_ONE;
      }
      else
      {
        state = DEBOUNCE_1ST_ZERO;
      }
      break;
    }
    case DEBOUNCE_1ST_ZERO:
    {
      if(pin_logic_level)
      {
        state = DEBOUNCE_ONE;
      }
      else
      {
        state = DEBOUNCE_2ND_ZERO;
      }
      break;
    }
    case DEBOUNCE_2ND_ZERO:
    {
      if(pin_logic_level)
      {
        state = DEBOUNCE_ONE;
      }
      else
      {
        state = DEBOUNCE_PRESSED;
      }
      break;
    }
    case DEBOUNCE_PRESSED:
    {
      if(pin_logic_level)
      {
        state = DEBOUNCE_ONE;
      }
      else
      {
        state = DEBOUNCE_PRESSED;
      }
      break;
    }
    default:
    {
      while(1){};
    }
  }
  
  if(state == DEBOUNCE_2ND_ZERO )
  {
    return true;
  }
  else
  {
    return false;
  }
}


void debounce_wait(void) 
{
  int i = 10000;
  // Delay
  while(i > 0)
  {
    i--;
  }
}

// Implementing FSM for DRAW/MOVE Mode SW1
// De-bounced on initial button press where button alue has been 
// low for 60ms. 
void sw1_fsm(void) {
	
	  static FSM_STATES fsm_state = DRAW;
	
	  // Delay before entering the code to determine which FSM state to 
    // transistion to.
    //debounce_wait();
	  if (sw1_debounce_fsm() ) {
	
		switch(fsm_state) {
			
		case DRAW:
		fsm_state = MOVE; 
		put_string("\n\r IN DRAW MODE");
		break;
			
			
		case MOVE: 
		fsm_state = DRAW;
		put_string("\n\r IN MOVE MODE");
		break; 
			
		default:
			break;
		}

	}

}

void print_ps2(uint16_t x_data, uint16_t y_data)
{
  uint32_t i;
  char msg[80];
  
	//sprintf(msg,"X Dir value : 0x%03x Y Dir value : 0x%03x\r",x_data, y_data);
  put_string(msg);
    
}


//*****************************************************************************
//*****************************************************************************
int 
main(void)
{
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
	
  // Reach infinite loop
  while(1){
		
		//print_ps2(curr_x_px, curr_y_px);
		
		if(Alert_Timer0A) {
			timer0A_count++;
			sw1_debounce_counter++;
			Alert_Timer0A = false;
		}
		
		if(Alert_Timer0B) {
			timer0B_count++;
			Alert_Timer0B = false;
		}
		
		// If interrupt A has occurred 10 times
		if(timer0A_count == 10){
						
			// Toggle Blue LED
			if(!lp_io_read_pin(BLUE_BIT)) {
				lp_io_set_pin(BLUE_BIT);
			} else {
				lp_io_clear_pin(BLUE_BIT);
			}
			
			// Reset timer0A count
			timer0A_count = 0;
		}
		
		// If interrupt B has occurred 10 times
		if(timer0B_count == 10) {
			
			// Reset timer0B alert
			Alert_Timer0B = false;
			
			// Toggle Green LED
			if(!lp_io_read_pin(GREEN_BIT)) {
				lp_io_set_pin(GREEN_BIT);
			} else {
				lp_io_clear_pin(GREEN_BIT);
			}
			
			// Start SS2 conversion
			ADC0->PSSI |= ADC_PSSI_SS2;
			
			// Reset timer0B count
			timer0B_count = 0;
		}
		
		if(sw1_debounce_counter == 3) {
			sw1_fsm();
			sw1_debounce_counter = 0;
		}
		
		
		if(!lp_io_read_pin(SW2_BIT)) {
			put_string("\n\r ERASE MODE ON");
		} 
		
		
		// if ADC interrupt has occured 
		
		if(Alert_ADC0_Conversion) {
			Alert_ADC0_Conversion = false; 
			
			//used to examine curr position of PS2 joystick
			
			// Update current x position with current PS2 joystick ADC value
			curr_x_px = ADC0->SSFIFO2 & ADC_SSFIFO2_DATA_M;
			
			// Update current y position with current PS2 joystick ADC value
			curr_y_px = ADC0->SSFIFO2 & ADC_SSFIFO2_DATA_M;
			
			// UPDATE LCD ACCORDINGLY
			
			
			
			
			
			// TESTING Ps2 values 
	    print_ps2(curr_x_px, curr_y_px);
			
			
		}
		
		
	}
}