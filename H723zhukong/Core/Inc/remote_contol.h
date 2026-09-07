#ifndef  REMOTE_CONTOL_H
#define  REMOTE_CONTOL_H
#include "main.h"
#include "usart.h"

#define My_constrain(amt, low, high)   ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) 
#define SBUS_FRAME_SIZE 25
/* Bluetooth controls are converted to foot excursion (cm), not cm/s.
 * Translation: byte / 50; turn: byte / 32. Hysteresis prevents chatter.
 */
#define BT_MOTION_START_CM 0.20f
#define BT_MOTION_STOP_CM  0.10f
extern  int Calibrate_flag ;

extern uint8_t sbus_buf[SBUS_FRAME_SIZE];
extern volatile uint8_t sbus_new_data ;
extern int16_t sbus_ch[6] ;  
extern char x ;
void  REMOTE_MOVE( void  );
void  REMOTE_V_Set( float  REMOTE_BASIC_L_R_V, float  REMOTE_BASIC_F_B_V, float  RE_turn_V ,int Remote_V_Limit );
void  get_SBUS_data(void);
void get_blue_tooth (void );
#endif
