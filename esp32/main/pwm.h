// pwm definitions and globals

// Adjust thrust with battery voltage
//#define ESC_ADJUST_THRUST_WITH_VOLTAGE

#define NUM_CHANNELS 8

#define MIN_WIDTH 800
#define LO_WIDTH 1100
#define HI_WIDTH 1900
#define MID_WIDTH 1500

#define NUM_MOTORS 4
//#define MOTOR_ORDER_CW
#undef MOTOR_ORDER_CW

#define PWM_STARTUP_COUNT 10

#define DISARM_ON_INVERSION 1
#define INVERSION_WM 50

#define PACKET_SIZE 32
extern volatile uint16_t pwmbuf[PACKET_SIZE / sizeof(uint16_t)];

// Indecies for other values in pwm packet
#define RC_RATE  8
#define BAT_IDX  9
#define POSZ_IDX 11
#define OFQ_IDX 12
#define OFX_IDX 13
#define OFY_IDX 14

extern bool prepare_failsafe;
extern bool in_failsafe;
extern bool in_config;
extern bool in_arm;
extern uint32_t pwm_count;
extern float last_width[NUM_CHANNELS];
extern bool pwm_stopped;

extern void pwm_output(uint16_t *wd);
extern void pwm_shutdown(void);
