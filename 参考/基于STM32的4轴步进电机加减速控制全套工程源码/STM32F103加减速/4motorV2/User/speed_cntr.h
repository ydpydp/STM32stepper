
#ifndef SPEED_CNTR_H
#define SPEED_CNTR_H

// Direction of stepper motor movement
#define CW  0
#define CCW 1
/*! \brief Holding data used by timer interrupt for speed ramp calculation.
 *
 *  Contains data used by timer interrupt to calculate speed profile.
 *  Data is written to it by move(), when stepper motor is moving (timer
 *  interrupt running) data is read/updated when calculating a new step_delay
 */
typedef struct {
  //! What part of the speed ramp we are in.
  u8 run_state ;
  //! Direction stepper motor should move.
  u8 dir ;
  //! Peroid of next timer delay. At start this value set the accelration rate.
  s32 step_delay;
  //! What step_pos to start decelaration
  u32 decel_start;
  //! Sets deceleration rate.
  s32 decel_val;
  //! Minimum time delay (max speed)
  s32 min_delay;
  //! Counter used when accelerateing/decelerateing to calculate step_delay.
  s32 accel_count;
} speedRampData;


/*! \Brief Frequency of timer1 in [Hz].
 *
 * Modify this according to frequency used. Because of the prescaler setting,
 * the timer1 frequency is the clock frequency divided by 8.
 */
// Timer/Counter 1 running on 3,686MHz / 8 = 460,75kHz (2,17uS). (T1-FREQ 460750)
#define T1_FREQ 1000000
#define SPR 1600
// Maths constants. To simplify maths when calculating in speed_cntr_Move().
#define ALPHA (2*3.14159/SPR)                    // 2*pi/spr
#define A_T_x100 ((long)(ALPHA*T1_FREQ*100))     // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((T1_FREQ*0.676)/100)) // divided by 100 and scaled by 0.676
#define A_SQ (long)(ALPHA*2*100000*100000)         // 
#define A_x20000 (int)(ALPHA*20000)              // ALPHA*20000
// Speed ramp states
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3

extern void speed_cntr_Move(unsigned int axis,signed int step, unsigned int accel, unsigned int decel, unsigned int speed);
extern u32 sqrt(u32 x);
extern bool status;


//extern void speed_cntr_Move(signed int step, unsigned int accel, unsigned int decel, unsigned int speed);
//extern void speed_cntr_Init_Timer1(void);
//extern unsigned long sqrt(unsigned long v);
//extern unsigned int min(unsigned int x, unsigned int y);

//! Global status flags
#endif
