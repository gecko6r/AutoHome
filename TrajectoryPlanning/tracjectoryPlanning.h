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
//	GAIT gait;					//��̬
//	int speed;					//�ٶȣ�-10 - 10��
//	int t;						//����t
//	u8 step;					//t���ӻ���ٵĲ�����1 - T/40��
//	double step_len;			//������1 - 80��
//	double step_height;			//̧�ȸ߶ȣ�0 - 35��
//	double T;					//���ڵ�����100 - 500��
//}State;
//	


//u8 gecko_init(LOCO_MODE mode, State s);
//void single_step(void);

///**************************��̬���ɼ��Ƕ��������**************************/
//void joints_update(void);
//void gait_gen(void);
//void bipod_gait(int t);
//void tripod_gait(int t);
//void update_angle_buf(void);
//void set_angle_buf(double val, u8 index);
///***********************************end***********************************/


///*******************************�˶���������******************************/
//void set_gait(GAIT gait);
//u8 set_speed(int speed);
//void reset_t(void);
//void t_update(void);
//u8 set_step(u8 step);
//u8 set_peroid(double peroid);
//u8 set_step_len(double len);
//u8 set_step_height(double h);
///***********************************end***********************************/


///*******************************���˶�ѧ����******************************/
//Joints ikine(Coor3d _coord);
//double sqr(double x);
///***********************************end***********************************/

//#endif

