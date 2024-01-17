#include <avr/io.h>
#include "i2c.h"
#include "SSD1306.h"
#include <util/delay.h>
#define MYDELAY 1000
#define TRIGenter PD6 //pin of trig pulse enter
#define ECHOenter PD5 //pin of receive pulse enter
#define TRIGexit PD3//pin of trig pulse exit
#define ECHOexit PD4 //pin of receive pulse exit
#define ButtonPin PC0 //button

//FURQUAAN SYED

#define R_clk 1.098 //distance covered when clk up by one/2


// DDRB|=(1<<5); // DDRB5 high. No impact on other DDRB bits
// PORTB|=(1<<5); // PB5 high. No impact on other PORTB bits
// PORTB&=~(1<<5); // PB5 low. No impact on other PORTB bits
// PORTB^=(1<<5); // Toggle PB5. No impact on other PORTB bits.

//if ((PINC & (1 << PINC0)) == 0) { this is for your button


void init_timer(void);
int main(void){
    int occupancy = 0;
    int capacity = 10;
    int inc = (capacity/4);

    unsigned char rise_clk_enter, fall_clk_enter, echo_width_enter;
    float distance_enter, dist_inc_enter; //distance for enter in cm and inch

    unsigned char rise_clk_exit, fall_clk_exit, echo_width_exit;
    float distance_exit, dist_inc_exit; //distance for enter in cm and inch

    DDRB |= 1<<PB5;
    DDRB |= 1<<PB4;     //SETTING LED PINS AS AOUTPUT

    PORTC |= (1 << ButtonPin); // enable internal pullup resistors for button

    DDRD |= 1<<TRIGenter; // set trig pin as output
    PORTD |= ~(1<<TRIGenter); // set 0 voltage

    DDRD |= 1<<TRIGexit; // set trig pin as output
    PORTD |= ~(1<<TRIGexit); // set 0 voltage

    OLED_Init();  //initialize the OLED
    init_timer();

    while(1){
       // OLED_Clear();
       
       

        TCNT0=0;                            //setting up entrance ultrasonic sensor 
        PORTD |= 1<<TRIGenter; 
        _delay_us(10); 
        PORTD = ~(1<<TRIGenter); 

//CODE FOR ULTRASONIC SENSORS TAKEN FROM DAVID MCLAUGHLIN 231 CLASS

        while ((PIND & (1<<ECHOenter))==0); //sent pulse not recorded yet by echo
        rise_clk_enter = TCNT0; // record start time when received start pulse in echo
        while (!(PIND & (1<<ECHOenter))==0); //wait for return pulse in echo
        fall_clk_enter = TCNT0; //record end time when received return pulse in echo

        echo_width_enter = fall_clk_enter - rise_clk_enter; //total increment of clock during echo
        distance_enter = echo_width_enter*R_clk;

        if (fall_clk_enter > rise_clk_enter){ //no calculations needed when fall=rise,not interested
           echo_width_enter = fall_clk_enter - rise_clk_enter; //total increment of clock during echo
           distance_enter = echo_width_enter*R_clk;

        } //distance in cm of return signal only


        //EXIT EXIT  EXIT EXIT  EXIT EXIT  EXIT EXIT  EXIT EXIT  EXIT EXIT  EXIT EXIT  EXIT EXIT  EXIT EXIT  EXIT EXIT  EXIT EXIT  EXIT EXIT  EXIT EXIT
        TCNT0=0; //
        PORTD |= 1<<TRIGexit; 
        _delay_us(10); 
        PORTD = ~(1<<TRIGexit); 

        while ((PIND & (1<<ECHOexit))==0); 
        rise_clk_exit = TCNT0; // 
        while (!(PIND & (1<<ECHOexit))==0); 
        fall_clk_exit = TCNT0; //

        echo_width_exit = fall_clk_exit - rise_clk_exit; //total increment of clock during echo
        distance_exit = echo_width_exit*R_clk;


        if (fall_clk_exit > rise_clk_exit){ //
           echo_width_exit = fall_clk_exit - rise_clk_exit; 
           distance_exit = echo_width_exit*R_clk;
           } //distance in cm of return signal only




        //ALL the different if statements for displaying occupancy level
        //Using inc so that occupancy level will be scalable depending on capacity input

        if (occupancy == capacity) {
            OLED_GoToLine(7);
            OLED_DisplayString("OCCUPANCY LEVEL: ");
            OLED_DisplayString("FULL           ");
        }
        else if (occupancy == 0){
            OLED_GoToLine(7);
            OLED_DisplayString("OCCUPANCY LEVEL:");
            OLED_DisplayString("EMPTY          ");
        }
        else if (occupancy > 0 && occupancy <= (inc*2)) {
             OLED_GoToLine(7);
            OLED_DisplayString("OCCUPANCY LEVEL: ");
            OLED_DisplayString("LOW         ");
        }
        else if (occupancy > (inc*2) && occupancy <= (inc*4)) {
            OLED_GoToLine(7);
            OLED_DisplayString("OCCUPANCY LEVEL: ");
            OLED_DisplayString("MED          ");
        }
        else if (occupancy > (inc*4) && occupancy < capacity) {
            OLED_GoToLine(7);
            OLED_DisplayString("OCCUPANCY LEVEL: ");
            OLED_DisplayString("HIGH          ");
        }

        if (!(PINC & (1 << ButtonPin)) && (occupancy < capacity)){            //if button is pressed and the room is not at capactiy
            PORTB &= ~(1<<PB4);
            PORTB |= (1<<PB5); // throw on the green lights

            for (int i = 0; i <= 5; i++){                      //after subject self-certifies, unlock door for 5 seconds

            //Make sure distance is being measured

                TCNT0=0; //start clock
                PORTD |= 1<<TRIGenter; //trig pulse on
                _delay_us(10); //10 us pulse
                PORTD = ~(1<<TRIGenter); //trig pulse off


                while ((PIND & (1<<ECHOenter))==0); //
                rise_clk_enter = TCNT0; // 
                while (!(PIND & (1<<ECHOenter))==0); 
                fall_clk_enter = TCNT0; 

                echo_width_enter = fall_clk_enter - rise_clk_enter; 
                distance_enter = echo_width_enter*R_clk;

                if (fall_clk_enter > rise_clk_enter){ 
                echo_width_enter = fall_clk_enter - rise_clk_enter; 
                distance_enter = echo_width_enter*R_clk;

                } 

                if (distance_enter < 15) {                  //15 cm is enough to say that person is entering after they are self certified
                    _delay_ms(500);
                    occupancy += 1;
                    if (occupancy > 10) {
                        occupancy = 10;
                    }
                }

                OLED_GoToLine(0);
                OLED_DisplayString("DOOR UNLOCKED FOR:");
                OLED_DisplayNumber(10, 5-i, 2);
                OLED_DisplayString("s");

                OLED_GoToLine(6);
                OLED_DisplayString("OCCUPANCY: ");
                OLED_DisplayNumber(10, occupancy, 2);




                _delay_ms(1000);
            }
            PORTB &= ~(1<<PB5);         ///signify that door is locked and unlocked
            PORTB |= (1<<PB4); // throw on the green light

        }
        else{


            if (distance_exit < 15){ // if person is exiting
            OLED_GoToLine(0);
            OLED_DisplayString("PERSON EXITING");

            PORTB &= ~(1<<PB4);
            PORTB |= (1<<PB5); // throw on the green light
            occupancy -= 1; //one less person in the room
            if (occupancy < 0) {
                occupancy = 0;
            }
            _delay_ms(1000);


        }
        else{
            PORTB&= ~(1<<PB5);
            PORTB |= (1<<PB4);
            OLED_GoToLine(0);
            OLED_DisplayString("LOCKED               ");    ///lots of spaces cause I don't want this OLED to clear that much it blinks and is ugly
            OLED_GoToLine(6);
            OLED_DisplayString("OCCUPANCY: ");
            OLED_DisplayNumber(10, occupancy, 2);


            OLED_GoToLine(1);                                                           //health inquiry prompt
            OLED_DisplayString("PUSH BUTTON IF YOU DONOT HAVE SYMPTOMS OF COVID-19");
            OLED_GoToNextLine();
            OLED_DisplayString("ELSE, WALK AWAY");
            }
        }




        _delay_ms(500);
    }
    return 0;
}


void init_timer(void){
   TCCR0A = 0x00; //normal mode
   TCCR0B = 0x05; //1024 pre-scaler
}

