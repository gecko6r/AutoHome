//#include "tracjectoryPlanning.h"

//const u16 DELAY_CONST = 20;											//���������ʱ
//const double BASE_X = 70.0, BASE_Y = 40.0, BASE_Z = -35.0;			//��ʼλ��(mm)
//const double link1 = 70.0, link2 = 40.0, link3 = 35.0;				//���˳���(mm)
//const double pi = 3.14159265359;									//��
//Coor3d C[4];														//�ĸ�����ĩ��λ��
//Joints J[4];														//�����ȹؽڽǶ�
//double AngleBuf[12] = {0.0};										//�Ƕ�����
//State state;														//�˶�״̬����

///*
////#define IN_DEBUG

//////////////////////////////////////////////////////////////////////////////////
///// @���� ϵͳ��ʼ��
///// @���� init_mode:��ʼ��ģʽ��s���˶�ѧ�����ṹ��
///// @���� 0����ʼ���ɹ���LED��������0��ʧ�ܣ�LED��˸
//////////////////////////////////////////////////////////////////////////////////
//u8 gecko_init(LOCO_MODE init_mode, State s)
//{
//	u8 ret = 0;
//	delay_init(168);  							//��ʼ����ʱ����
//	usart_init(115200);							//��ʼ�����ڲ�����Ϊ115200
//    servo_init(init_mode);						//��ʼ����������˶�����ʼ״̬
//	LED_Init();									//��ʼ��Ӧ��ָʾ��
//		
//	set_gait(s.gait);
//	reset_t();
//	
//	ret += set_speed(s.speed);
//	ret += set_peroid(s.T);
//	ret += set_step(s.step);
//	ret += set_step_len(s.step_len);
//	ret += set_step_height(s.step_height);

//	//�����ʼ������
//	if(ret)
//	{
//		err_handle(ERR_INIT_FAILED);
//		CLR_ERR;
//		return ret;
//	}
//	
//	//�����ʼ���ɹ���LED����
//	else
//		LED = 0;
//	
//	
//	return 0;

//}


//////////////////////////////////////////////////////////////////////////////////
///// @���� �˶�����һ��λ�õ�
///// @���� ��
///// @���� ��
//////////////////////////////////////////////////////////////////////////////////
//void single_step(void)
//{
//	u8 speed = abs(state.speed);
//	
//	u8 i;
//	
//	//��ֹ
//	if(state.speed == 0)
//		return;
//	
//	//����ؽڽǶ�ֵ��д������
//	joints_update();
//	update_angle_buf();
//	
//	//���ö������
//	servo_ctrl_degree(AngleBuf);
//	
//#ifdef IN_DEBUG
//		
//	printf("%d \r\n", state.t);
//	
//	for(i=0; i<12; i++)
//	{
//		printf("%3.2f  ", AngleBuf[i]);
//	}
//	
//	printf("\r\n");

//#endif	
//	
//	//����t
//	t_update();
//	
//	//���ڵ��ڲ�Ƶ
////	delay_ms(DELAY_CONST / speed);
//	
//}


//////////////////////////////////////////////////////////////////////////////////
///// @���� ���ݲ�̬�������λ�ã��������˶�ѧ���
///// @���� ��
///// @���� ��
//////////////////////////////////////////////////////////////////////////////////
//void joints_update(void)
//{
//	u8 i;
//	
//	//���ݲ�̬���ɵ�ǰ���λ��
//	gait_gen();
//	
//	//�˶�ѧ���õ��ؽڽǶ�ֵ
//	for(i=0; i<4; i++)
//		J[i] = ikine(C[i]);
//	
//}

//////////////////////////////////////////////////////////////////////////////////
///// @���� ���ɲ�̬ɢ��
///// @���� ��
///// @���� ��
//////////////////////////////////////////////////////////////////////////////////
//void gait_gen(void)
//{
//	//�Խǲ�̬
//	if(state.gait == GAIT_BI)
//		bipod_gait(state.t);
//	
//	//���ǲ�̬
//	if(state.gait == GAIT_TRI)
//		tripod_gait(state.t);
//	
//}

//////////////////////////////////////////////////////////////////////////////////
///// @���� ���ɶԽǲ�̬��һϵ��ɢ��
///// @���� t����ǰ����ֵ��t��0��TΪһ�����ڣ�
///// @���� ��
//////////////////////////////////////////////////////////////////////////////////
//void bipod_gait(int t)
//{
//	double T = state.T;								//���ڵ���
//	double L = state.step_len;						//����
//	double H  = state.step_height;					//̧�ȸ߶�
//	t = (double)t;									//ת����double
//	
//	if(t < 0.25*T)
//	{
//		//��ǰ��
//		C[LF].x = BASE_X;
//		C[LF].y = -2.0*L*t / T + BASE_Y;
//		C[LF].z = BASE_Z;
//		
//		//��ǰ��
//		C[RF].x = BASE_X;
//		C[RF].y = 2.0*L*t / T + BASE_Y;
//		C[RF].z = H*sin(0.5*pi*(4*t / T + 1)) + BASE_Z;
//		
//		//�����
//		C[LH].x = BASE_X;
//		C[LH].y = -2.0*L*t / T + BASE_Y;
//		C[LH].z = H*sin(0.5*pi*(4*t / T + 1)) + BASE_Z;
//		
//		//�Һ���
//		C[RH].x = BASE_X;
//		C[RH].y = 2.0*L*t / T + BASE_Y;
//		C[RH].z = BASE_Z;
//		
//		
//	}
//	
//	else if(t < 0.75*T)
//	{
//		//��ǰ��
//		C[LF].x = BASE_X;
//		C[LF].y = 2.0*L*(t-T/4.0) / T - L/2.0 + BASE_Y;
//		C[LF].z = H*sin(2.0*pi*(t-T/4.0) / T) + BASE_Z;
//		
//		//��ǰ��
//		C[RF].x = BASE_X;
//		C[RF].y = -2.0*L*(t - T/4)/T + L/2.0 + BASE_Y;
//		C[RF].z = BASE_Z;;
//		
//		//�����
//		C[LH].x = BASE_X;
//		C[LH].y = 2.0*L*(t - T/4)/T - L/2.0 + BASE_Y;
//		C[LH].z = BASE_Z;;
//		
//		//�Һ���
//		C[RH].x = BASE_X;
//		C[RH].y = -2.0*L*(t-T/4.0) / T + L/2.0 + BASE_Y;
//		C[RH].z = H*sin(2.0*pi*(t-T/4.0) / T) + BASE_Z;
//	}
//	
//	else if(t <= T)
//	{
//		//��ǰ��
//		C[LF].x = BASE_X;
//		C[LF].y = L/2.0*(1 - (4*t - 3*T)/T) + BASE_Y;
//		C[LF].z = BASE_Z;
//		
//		//��ǰ��
//		C[RF].x = BASE_X;
//		C[RF].y = -2.0*L*(T - t)/T + BASE_Y;
//		C[RF].z = H*sin(2.0*pi * ((t - 3.0*T/4.0) / T)) + BASE_Z;
//		
//		//�����
//		C[LH].x = BASE_X;
//		C[LH].y = 2.0*L*(T - t)/T + BASE_Y;
//		C[LH].z = H*sin(2.0*pi * ((t - 3.0*T/4.0) / T)) + BASE_Z;
//		
//		//�Һ���
//		C[RH].x = BASE_X;
//		C[RH].y = -L/2.0*(1 - (4*t - 3*T)/T) + BASE_Y;
//		C[RH].z = BASE_Z;
//		
//	}
//	

//}

//////////////////////////////////////////////////////////////////////////////////
///// @���� �������ǲ�̬��һϵ��ɢ��
///// @���� t����ǰ����ֵ��t��0��TΪһ�����ڣ�
///// @���� ��
//////////////////////////////////////////////////////////////////////////////////
//void tripod_gait(int t)
//{
//	
//}


//////////////////////////////////////////////////////////////////////////////////
///// @���� �������ȵĸ��ؽڽǶ�ֵ������ؽڽǶ�����
///// @���� ��
///// @���� ��
//////////////////////////////////////////////////////////////////////////////////
//void update_angle_buf(void)
//{
//	u8 i = 0;
//	
//	for(i=0; i<4; i++)
//	{
//		
//		set_angle_buf(J[i].j1, (3*i + 0));
//		set_angle_buf(J[i].j2, (3*i + 1));
//		set_angle_buf(J[i].j3, (3*i + 2));
//	}
//	
//}

//////////////////////////////////////////////////////////////////////////////////
///// @���� ����Ԫ�������ı���ֵ
///// @���� val���µĸ���ֵ�� index��Ԫ��λ��
///// @���� ��
//////////////////////////////////////////////////////////////////////////////////
//void set_angle_buf(double val, u8 index)
//{
//	AngleBuf[index] = val;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @���� �趨�˶��������е��ٶȣ��ٶȾ���������λ�õ�֮���ʱ����
///// @���� speed;�ٶ�ֵ������ʾ���򡣸�ֵ����0��ʾ��ֹ
///// @���� ��
//////////////////////////////////////////////////////////////////////////////////
//u8 set_speed(int speed)
//{
//	//�ٶȷ�Χ ��-10 - 10��
//	if(speed > 10)
//		return 1;
//	
//	//�ٶȷ�Χ ��-10 - 10��
//	if(speed < -10)
//		return 1;
//	
//	state.speed = speed;
//	
//	return 0;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @���� ���˶�ѧ�����е�tֵ����
///// @���� ��
///// @���� ��
//////////////////////////////////////////////////////////////////////////////////
//void reset_t(void)
//{
//	state.t = 0;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @���� �趨�˶��������еĲ�̬
///// @���� gait����̬��Ӧ����Ҫ���μ�GAITö��
///// @���� ��
//////////////////////////////////////////////////////////////////////////////////
//void set_gait(GAIT gait)
//{
//	state.gait = gait;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @���� �����˶�ѧ�����е�speed��step������t
///// @���� ��
///// @���� ��
//////////////////////////////////////////////////////////////////////////////////
//void t_update(void)
//{
//	//�����˶�
//	if(state.speed > 0)
//	{
//		state.t += state.step;
//		if(state.t >= state.T)
//			state.t -= state.T;
//	}
//	
//	//�����˶�
//	if(state.speed < 0)
//	{
//		state.t -= state.step;
//		if(state.t <= 0)
//			state.t += state.T;
//	}
//	
//}

//////////////////////////////////////////////////////////////////////////////////
///// @���� �趨�˶�ѧ�����е�step
///// @���� step��tÿ�����ӻ���ٵ�ֵ
///// @���� 0���趨��ɣ� ��0������������Χ
//////////////////////////////////////////////////////////////////////////////////
//u8 set_step(u8 step)
//{
//	if(step < 1)
//		return 1;
//	if(step > state.T/40)
//		return 1;
//	
//	state.step = step;
//	return 0;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @���� �趨�˶�ѧ�����е�����
///// @���� peroid��ÿ����̬���ڵ�λ�õ���
///// @���� 0���趨��ɣ� ��0������������Χ
//////////////////////////////////////////////////////////////////////////////////
//u8 set_peroid(double peroid)
//{
//	if(peroid < 100)
//		return 1;
//	
//	if(peroid > 500)
//		return 1;
//	
//	state.T = peroid;
//	return 0;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @���� �趨�˶�ѧ�����еĲ���step_len
///// @���� len��һ�������˶��Ĳ�������λmm
///// @���� 0���趨��ɣ� ��0������������Χ
//////////////////////////////////////////////////////////////////////////////////
//u8 set_step_len(double len)
//{
//	if(len <=0)
//		return 1;
//	
//	if(len > 80)
//		return 1;
//	
//	state.step_len = len;
//	return 0;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @���� �趨�˶�ѧ�����е�̧�ȸ߶ȣ���ԣ���step_height
///// @���� h��ÿ��̧�ȵ����߶ȣ���λmm
///// @���� 0���趨��ɣ� ��0������������Χ
//////////////////////////////////////////////////////////////////////////////////
//u8 set_step_height(double h)
//{
//	if(h <= 0)
//		return 1;
//	
//	if(h > 35.0)
//		return 1;
//	
//	state.step_height = h;
//	return 0;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @���� ���������ά��������˶�ѧ���
///// @���� c:�����ά����
///// @���� ���ȵ������ؽڽǶ�ֵ
//////////////////////////////////////////////////////////////////////////////////
//Joints ikine(Coor3d c)
//{
//	double x = c.x, y = c.y, z = c.z;
//	double exp1, exp2, exp3;
//	Joints j;
//	
//	exp1 = sqr(x) + sqr(y) + sqr(z) + sqr(link1) - sqr(link2) - sqr(link3);
//	exp2 = 2*link1*(sqrt(sqr(x) + sqr(y) + sqr(z) - sqr(link3)));
//	exp3 = y / sqrt(sqr(x) + sqr(y) + sqr(z) - sqr(link3));
//	
//	//����ؽڽǶ�ֵ�����ȣ�
//	j.j1 = asin(-link3 / sqrt(sqr(x) + sqr(z))) - atan(z/x);
//	j.j2 = asin(exp1 / exp2) - acos(exp3);
//	j.j3 = asin((sqr(link1)+sqr(link2)+sqr(link3)-sqr(x)-sqr(y)-sqr(z)) / (2*link1*link2));
//	
//	//����ת�Ƕ�
//	j.j1 *= -180.0/pi;
//	j.j2 *= 180.0/pi;
//	j.j3 *= 180.0/pi;
////	
//	return j;
//}

//////////////////////////////////////////////////////////////////////////////////
///// @���� ���㸡������ƽ��
///// @���� x:����
///// @���� ���
//////////////////////////////////////////////////////////////////////////////////
//double sqr(double x)
//{
//	return x*x;
//}

//*/