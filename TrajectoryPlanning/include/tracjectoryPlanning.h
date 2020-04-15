//#ifndef _TRACJECTORY_PLANNING_H
//#define _TRACJECTORY_PLANNING_H

//#include "sys.h"



//typedef enum GAIT
//{
//	GAIT_BI,
//	GAIT_TRI,
//}GAIT;

//enum LEG{
//	LF = 0,
//	RF,
//	LH,
//	RH,
//};

//typedef struct {
//	
//	float j1;
//	float j2;
//	float j3;
//	
//}Joints;

//typedef struct Coor3d{
//	
//	float x;
//	float y;
//	float z;
//	
//}Coor3d;

//typedef struct State{
//	
//	GAIT gait;					//步态
//	int speed;					//速度（-10 - 10）
//	int t;						//变量t
//	u8 step;					//t增加或减少的步长（1 - T/40）
//	double step_len;			//步长（1 - 80）
//	double step_height;			//抬腿高度（0 - 35）
//	double T;					//周期点数（100 - 500）
//}State;
//	


//u8 gecko_init(LOCO_MODE mode, State s);
//void single_step(void);

///**************************步态生成及角度填充设置**************************/
//void joints_update(void);
//void gait_gen(void);
//void bipod_gait(int t);
//void tripod_gait(int t);
//void update_angle_buf(void);
//void set_angle_buf(double val, u8 index);
///***********************************end***********************************/


///*******************************运动参数调节******************************/
//void set_gait(GAIT gait);
//u8 set_speed(int speed);
//void reset_t(void);
//void t_update(void);
//u8 set_step(u8 step);
//u8 set_peroid(double peroid);
//u8 set_step_len(double len);
//u8 set_step_height(double h);
///***********************************end***********************************/


///*******************************逆运动学计算******************************/
//Joints ikine(Coor3d _coord);
//double sqr(double x);
///***********************************end***********************************/

//#endif

