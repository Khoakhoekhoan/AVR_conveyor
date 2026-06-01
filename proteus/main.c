/*
 * main.c
 *
 * Created: 5/27/2026 4:30:42 PM
 * Author: Nhom6 & AI Collaborator
 */ 

#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "myLCD_new.h"

#define KEYPAD_PORT PORTC
#define KEYPAD_PIN   PINC
#define KEYPAD_DDR   DDRC

// Define L298 Direction control pins on PORTB
#define MOTOR_DDR  DDRB
#define MOTOR_PORT PORTB
#define IN1        PB0
#define IN2        PB1

// Define Hardware PWM and Servo pins on PORTE
#define PORTE_DDR  DDRE
#define PORTE_PORT PORTE
#define DC_PWM_PIN PE4  // OC3B Hardware PWM for L298 ENA
#define SERVO_PIN  PE3  // Software PWM for Directional Servo

// Define IR Obstacle Sensor pins on PORTF (Active LOW inputs)
#define IR_PIN     PINF
#define IR_DDR     DDRF
#define IR_COUNT   PF0
#define IR1_S      PF1
#define IR2_M      PF2
#define IR3_L      PF3

// Servo position timing ticks based on Timer 0 ISR (1 tick = ~200us)
#define SERVO_POS_MINUS_90 5   // 1.0 ms pulse -> -90 degrees
#define SERVO_POS_0        7   // 1.5 ms pulse -> 0 degrees
#define SERVO_POS_PLUS_90  10  // 2.0 ms pulse -> +90 degrees

// Global status flags for cross-functional accessibility
volatile uint8_t size_S_flag = 0;
volatile uint8_t size_M_flag = 0;
volatile uint8_t size_L_flag = 0;
volatile int max_num = 0;

// Runtime counter variables
volatile uint16_t current_sum = 0;
volatile uint16_t count_S = 0;
volatile uint16_t count_M = 0;
volatile uint16_t count_L = 0;

// System state tracking flags
volatile uint8_t sys_rejected_state = 0; 
volatile uint8_t detected_S_temp = 0;
volatile uint8_t detected_M_temp = 0;
volatile uint8_t detected_L_temp = 0;
volatile uint8_t servo_pulse_target = SERVO_POS_0; // Default center position

uint8_t scan_code[4]={0x0E, 0x0D, 0x0B, 0x07};
uint8_t ascii_code[4][4] = {
    {'7', '8', '9', 'A'},
    {'4', '5', '6', 'B'},
    {'1', '2', '3', 'C'},
    {'*', '0', '#', 'D'}
};

// Prototypes for sub-functional routines
uint8_t checkpad(void);
void wait_for_release(void);
void setup_max_num(void);
void setup_product_size(void);
void update_running_display(void);
void process_ir_sensors(void);
void trigger_rejection_mode(const char* error_msg);
void set_dc_motor_speed(uint8_t duty);

uint8_t checkpad(){
    uint8_t i, j, keyin;
    for(i=0; i<4; i++){
        KEYPAD_PORT = 0xFF - (1<<(i+4));
        _delay_ms(1);
        keyin = KEYPAD_PIN & 0X0F;
        if(keyin != 0x0F) {
            for(j=0; j<4; j++){
                if(keyin == scan_code[j]) return ascii_code[j][i];
            }
        }
    }
    return 0;
}

void wait_for_release() {
    while (checkpad() != 0);
    _delay_ms(10);
}

// --- TIMER 0 OVERFLOW INTERRUPT SERVICE ROUTINE ---
// Handles background sensor scanning and Software PWM for the PE3 Servo
ISR(TIMER0_OVF_vect) {
    TCNT0 = 55; // Tweak reload value to achieve exactly 200us per interrupt tick
    static uint8_t servo_tick_counter = 0;
    
    // --- Software PWM Generation for PE3 Servo (50Hz / 20ms Frame) ---
    servo_tick_counter++;
    if (servo_tick_counter >= 100) { // 100 ticks * 200us = 20ms period
        servo_tick_counter = 0;
    }
    
    if (servo_tick_counter < servo_pulse_target) {
        PORTE_PORT |= (1 << SERVO_PIN);  // Drive Servo Pin HIGH
    } else {
        PORTE_PORT &= ~(1 << SERVO_PIN); // Pull Servo Pin LOW
    }
    
    // --- Background Sensor Scanning Line ---
    if (sys_rejected_state == 0 && max_num > 0) {
        process_ir_sensors();
    }
}

// --- SUB-ROUTINE: EVALUATE SENSOR REGISTER TRANSITIONS ---
void process_ir_sensors(void) {
    static uint8_t last_ir1 = 1, last_ir2 = 1, last_ir3 = 1, last_ircount = 1;
    
    uint8_t current_ir1 = (PINF & (1 << IR1_S)) ? 1 : 0;
    uint8_t current_ir2 = (PINF & (1 << IR2_M)) ? 1 : 0;
    uint8_t current_ir3 = (PINF & (1 << IR3_L)) ? 1 : 0;
    uint8_t current_ircount = (PINF & (1 << IR_COUNT)) ? 1 : 0;
    
    // 1. Size S Detection Line
    if (current_ir1 == 0 && last_ir1 == 1) {
        if (size_S_flag == 1) {
            detected_S_temp = 1;
            servo_pulse_target = SERVO_POS_MINUS_90; // Actuate sorting flap to -90 deg
        } else {
            trigger_rejection_mode("UNWANTED SIZE S");
            return;
        }
    }
    
    // 2. Size M Detection Line
    if (current_ir2 == 0 && last_ir2 == 1) {
        if (size_M_flag == 1) {
            detected_M_temp = 1;
            servo_pulse_target = SERVO_POS_0;        // Keep sorting flap at 0 deg
        } else {
            trigger_rejection_mode("UNWANTED SIZE M");
            return;
        }
    }
    
    // 3. Size L Detection Line
    if (current_ir3 == 0 && last_ir3 == 1) {
        if (size_L_flag == 1) {
            detected_L_temp = 1;
            servo_pulse_target = SERVO_POS_PLUS_90;  // Actuate sorting flap to +90 deg
        } else {
            trigger_rejection_mode("UNWANTED SIZE L");
            return;
        }
    }
    
    // 4. Global Counter Drop Sensor Line
    if (current_ircount == 0 && last_ircount == 1) {
        if (detected_S_temp)      { count_S++; current_sum++; detected_S_temp = 0; }
        else if (detected_M_temp) { count_M++; current_sum++; detected_M_temp = 0; }
        else if (detected_L_temp) { count_L++; current_sum++; detected_L_temp = 0; }
        update_running_display();
    }
    
    last_ir1 = current_ir1;
    last_ir2 = current_ir2;
    last_ir3 = current_ir3;
    last_ircount = current_ircount;
}

// --- SUB-ROUTINE: DIRECT HARDWARE PWM DUTY REGISTER ---
void set_dc_motor_speed(uint8_t duty) {
    OCR3BL = duty; // Load 8-bit duty values straight into Timer 3 Output Compare Registry B
}

// --- SUB-ROUTINE: TRIGGER SLOW REVERSE REJECTION SYSTEM ---
void trigger_rejection_mode(const char* error_msg) {
    sys_rejected_state = 1; 
    
    // Force L298 H-Bridge to REVERSE Direction (IN1=0, IN2=1)
    MOTOR_PORT &= ~(1 << IN1);
    MOTOR_PORT |= (1 << IN2);
    
    // Throttle to SLOW PWM speed execution profiles directly via PE4 (approx 45% duty)
    set_dc_motor_speed(115); 
    
    clr_LCD();
    move_LCD(1, 1);
    putStr_LCD("REJECT RUNNING");
    move_LCD(2, 1);
    putStr_LCD(error_msg);
    
    detected_S_temp = 0;
    detected_M_temp = 0;
    detected_L_temp = 0;
}

// --- SUB-ROUTINE FOR STEP 1: MAX QUANTITY CONFIGURATION ---
void setup_max_num(void) {
    uint8_t b[5];
    uint8_t idx = 0;
    uint8_t key;

    ENTRY_MAX_NUM:
    clr_LCD();
    move_LCD(1, 1);
    putStr_LCD("Enter max (1-20):");
    move_LCD(2, 1);
    idx = 0;

    while (1) {
        key = checkpad();
        if (key != 0) {
            if (key >= '0' && key <= '9') {
                if (idx < 2) {
                    b[idx] = key;
                    putChar_LCD(key);
                    idx++;
                }
            }
            else if (key == '#') {
                wait_for_release();
                b[idx] = '\0';
                max_num = atoi((char*)b);
                
                if (max_num >= 1 && max_num <= 20) {
                    clr_LCD();
                    move_LCD(1, 1);
                    putStr_LCD("Max Num Set!");
                    _delay_ms(1000);
                    return;
                } else {
                    clr_LCD();
                    move_LCD(1, 1);
                    putStr_LCD("Invalid! (1-20)");
                    _delay_ms(1500);
                    goto ENTRY_MAX_NUM;
                }
            }
            wait_for_release();
        }
    }
}

// --- SUB-ROUTINE FOR STEP 2: MULTI-SIZE CONFIGURATION ---
void setup_product_size(void) {
    uint8_t b[5];
    uint8_t idx = 0;
    uint8_t key;

    ENTRY_SIZE:
    clr_LCD();
    move_LCD(1, 1);
    putStr_LCD("Size (1, 2 or 3)");
    move_LCD(2, 1);
    
    idx = 0;
    b[0] = '\0';

    while (1) {
        key = checkpad();
        if (key != 0) {
            if (key >= '0' && key <= '9') {
                if (idx < 3) {
                    b[idx] = key;
                    idx++;
                    
                    clr_LCD();
                    move_LCD(1, 1);
                    putStr_LCD("Size (1, 2 or 3)");
                    move_LCD(2, 1);
                    
                    for (uint8_t k = 0; k < idx; k++) {
                        putChar_LCD(b[k]);
                        if (k < idx - 1) {
                            putChar_LCD(' ');
                        }
                    }
                }
            }
            else if (key == '#') {
                wait_for_release();
                
                if (idx == 0) {
                    clr_LCD();
                    move_LCD(1, 1);
                    putStr_LCD("Error: Empty!");
                    move_LCD(2, 1);
                    putStr_LCD("Choose at least 1");
                    _delay_ms(1500);
                    goto ENTRY_SIZE;
                }
                
                uint8_t data_invalid = 0;
                size_S_flag = 0;
                size_M_flag = 0;
                size_L_flag = 0;
                
                for (uint8_t k = 0; k < idx; k++) {
                    if (b[k] == '1')      size_S_flag = 1;
                    else if (b[k] == '2') size_M_flag = 1;
                    else if (b[k] == '3') size_L_flag = 1;
                    else {
                        data_invalid = 1;
                    }
                }
                
                if (data_invalid == 1) {
                    clr_LCD();
                    move_LCD(1, 1);
                    putStr_LCD("Invalid Choice!");
                    move_LCD(2, 1);
                    putStr_LCD("Retrying...");
                    _delay_ms(1500);
                    goto ENTRY_SIZE;
                }
                
                clr_LCD();
                move_LCD(1, 1);
                putStr_LCD("Confirmed!");
                move_LCD(2, 1);
                
                if (size_S_flag && size_M_flag && size_L_flag)      putStr_LCD("S, M, L chosen");
                else if (size_S_flag && size_M_flag)                putStr_LCD("S, M chosen");
                else if (size_S_flag && size_L_flag)                putStr_LCD("S, L chosen");
                else if (size_M_flag && size_L_flag)                putStr_LCD("M, L chosen");
                else if (size_S_flag)                               putStr_LCD("Size S chosen");
                else if (size_M_flag)                               putStr_LCD("Size M chosen");
                else if (size_L_flag)                               putStr_LCD("Size L chosen");
                
                _delay_ms(2000);
                return;
            }
            wait_for_release();
        }
    }
}

// --- SUB-ROUTINE FOR STEP 3: RE-RENDER RUNTIME COUNTERS ON LCD ---
void update_running_display(void) {
    char line1_buffer[17];
    char line2_buffer[17];
    
    sprintf(line1_buffer, "Max:%-3dSum:%d", max_num, current_sum);
    move_LCD(1, 1);
    putStr_LCD(line1_buffer);
    
    if (size_S_flag && size_M_flag && size_L_flag) {
        sprintf(line2_buffer, "S:%-3dM:%-3dL:%d", count_S, count_M, count_L);
    }
    else if (size_S_flag && size_M_flag) {
        sprintf(line2_buffer, "S:%-3dM:%d", count_S, count_M);
    }
    else if (size_S_flag && size_L_flag) {
        sprintf(line2_buffer, "S:%-3dL:%d", count_S, count_L);
    }
    else if (size_M_flag && size_L_flag) {
        sprintf(line2_buffer, "M:%-3dL:%d", count_M, count_L);
    }
    else if (size_S_flag) {
        sprintf(line2_buffer, "S:%d", count_S);
    }
    else if (size_M_flag) {
        sprintf(line2_buffer, "M:%d", count_M);
    }
    else if (size_L_flag) {
        sprintf(line2_buffer, "L:%d", count_L);
    }
    
    move_LCD(2, 1);
    putStr_LCD(line2_buffer);
}

// --- MAIN CONTROL PROGRAM ---
int main(void)
{
    // Configure Matrix Keypad Ports
    KEYPAD_DDR = 0xF0;
    KEYPAD_PORT = 0xFF;
    
    // Configure L298 Direction controller pins as OUTPUT
    MOTOR_DDR |= (1 << IN1) | (1 << IN2);
    
    // Configure PE4 (Hardware PWM) and PE3 (Software PWM Servo) as OUTPUT
    PORTE_DDR |= (1 << DC_PWM_PIN) | (1 << SERVO_PIN);
    
    // Configure IR Sensors as INPUT pins with internal pull-ups enabled
    IR_DDR &= ~((1 << IR_COUNT) | (1 << IR1_S) | (1 << IR2_M) | (1 << IR3_L));
    PORTF |= (1 << IR_COUNT) | (1 << IR1_S) | (1 << IR2_M) | (1 << IR3_L);
    
    // --- TIMER 3 CONFIGURATION FOR HARDWARE FAST PWM ON PE4 (OC3B) ---
    // WGM33:0 = 0101 (Fast PWM, 8-bit), COM3B1:0 = 10 (Clear OC3B on Compare Match, Non-Inverting)
    TCCR3A = (1 << COM3B1) | (1 << WGM30);
    TCCR3B = (1 << WGM32) | (1 << CS30); // Prescaler = 1 (F_PWM = 1MHz / 256 = ~3.9kHz)
    OCR3BL = 0; // Initialize DC Motor speed at 0% duty
    
    // --- TIMER 0 ARCHITECTURE CONFIGURATION FOR SENSOR WORKLOAD & SERVO INTERRUPTS ---
    TCCR0 = (1 << CS01); // Prescaler = 8 (1MHz / 8 = 125kHz tick rate)
    TIMSK |= (1 << TOIE0);
    TCNT0 = 55;          // (~256 - 55) * 8us = ~1.6ms loop window total
    
    init_LCD();
    clr_LCD();
    
    uint8_t key;

    // GLOBAL SYSTEM RESET ENTRY POINT
    START_SETUP:
    cli(); 
    max_num = 0; 
    current_sum = 0; count_S = 0; count_M = 0; count_L = 0;
    sys_rejected_state = 0;
    servo_pulse_target = SERVO_POS_0; // Center the servo flap
    
    // Hard stop on H-Bridge outputs and reset PWM registers
    MOTOR_PORT &= ~((1 << IN1) | (1 << IN2));
    set_dc_motor_speed(0);
    
    setup_max_num();
    setup_product_size();

    clr_LCD();
    update_running_display();
    
    sei(); 

    // Fire H-Bridge Driver to FORWARD Mode (IN1=1, IN2=0)
    MOTOR_PORT |= (1 << IN1);
    MOTOR_PORT &= ~(1 << IN2);
    
    // Throttle DC Motor to FAST speed profile directly via hardware PWM (approx 85% duty)
    set_dc_motor_speed(217); 

    // Primary Operation Monitoring Loop
    while (1) {
        key = checkpad();
        if (key != 0) {
            if (key == 'A') {
                wait_for_release();
                clr_LCD();
                move_LCD(1, 1);
                putStr_LCD("EMERGENCY STOP");
                _delay_ms(1500);
                goto START_SETUP; 
            }
            else if (key == '#' && sys_rejected_state == 1) {
                wait_for_release();
                
                clr_LCD();
                move_LCD(1, 1);
                putStr_LCD("Resuming System");
                _delay_ms(1000);
                clr_LCD();
                update_running_display();
                
                // Return Servo to center position
                servo_pulse_target = SERVO_POS_0;
                _delay_ms(200); // Give servo time to physical swing back
                
                // Switch L298 H-Bridge back to FORWARD direction
                MOTOR_PORT |= (1 << IN1);
                MOTOR_PORT &= ~(1 << IN2);
                
                // Restore Fast Speed Profile via Hardware PWM
                set_dc_motor_speed(217); 
                
                sys_rejected_state = 0; 
            }
            wait_for_release();
        }
        
        // Automated Target Capacity Cap Check Intercept
        if (current_sum >= max_num && sys_rejected_state == 0) {
            cli(); 
            MOTOR_PORT &= ~((1 << IN1) | (1 << IN2));
            set_dc_motor_speed(0); // Cut PWM line completely
            clr_LCD();
            move_LCD(1, 1);
            putStr_LCD("TARGET REACHED!");
            move_LCD(2, 1);
            putStr_LCD("Batch Completed");
            
            while (checkpad() != 'A');
            wait_for_release();
            goto START_SETUP;
        }
    }
}