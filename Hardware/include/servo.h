#ifndef _SERVO_H
#define _SERVO_H

#include "sys.h"

extern u16 _pwm[12];
extern const char Servo_Err_Info[][255];
extern const u8 SERVO_NUM;
extern u8 ctrl_mode;
#define CLR_ERR clear_err_msg()

typedef enum ERROR_LIST
{
	NO_ERROR = 0,
	ERR_OUT_OF_RANGE,
	ERR_INIT_FAILED,
	
}ERROR_LIST;
typedef enum LOCO_MODE
{
	MODE_STILL = 0,
	MODE_CRAWLING,
}LOCO_MODE;



void pwm_init(void);
void servo_init(enum LOCO_MODE mode);
int servo_ctrl(u16* _pos);
int servo_ctrl_degree(double* _buf);
u16 pos2pwm(u16 pos);
bool is_over_range(u16* _p);
void err_handle(ERROR_LIST err);
void clear_err_msg(void);
#endif
