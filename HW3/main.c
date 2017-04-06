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

char group[] = "Group00";
char individual_1[] = "Shyamal Anadkat";
char individual_2[] = "Aaron Levin";

extern volatile bool Alert_Timer0A;
extern volatile bool Alert_Timer0B;


//*****************************************************************************
//*****************************************************************************
void initialize_hardware(void)
{
  initialize_serial_debug();
	
	// Initialize switches and LEDs
	lp_io_init();
	
	// Enable PS2 joystick
	ps2_initialize();
	
	// Enable TIMER0 A and B for HW3
	timer_config_hw3();
	
	// Start TIMER0 A and B for HW3
	timer_start_hw3();
	
	// Configure the LCD and set screen to be black
	lcd_config_gpio();
	lcd_config_screen();
	lcd_clear_screen(LCD_COLOR_BLACK);	
}




void print_ps2(void)
{
  uint16_t x_data, y_data;
  uint32_t i;
  char msg[80];
  while(1)
  {

    x_data = ps2_get_x();
    y_data = ps2_get_y();
    sprintf(msg,"X Dir value : 0x%03x        Y Dir value : 0x%03x\r",x_data, y_data);
    put_string(msg);
    for(i=0;i<100000; i++){}
    
  }
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
  put_string("\n\r");  
  put_string("************************************\n\r");
  
  // Reach infinite loop
  while(1){
		if(Alert_Timer0A){
			Alert_Timer0A = false;
			if(!lp_io_read_pin(BLUE_BIT)) {
				lp_io_set_pin(BLUE_BIT);
			} else {
				lp_io_clear_pin(BLUE_BIT);
			}
		}
		if(Alert_Timer0B) {
			Alert_Timer0B = false;
			if(!lp_io_read_pin(GREEN_BIT)) {
				lp_io_set_pin(GREEN_BIT);
			} else {
				lp_io_clear_pin(GREEN_BIT);
			}
		}
  }
}
