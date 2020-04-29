/**
  ***********************************UTF-8**************************************
  * @file    matrix.c
  * @author  Xiong
  * @version V0.1
  * @date    31-July-2020
  * @brief   此文件用户实现C下的矩阵运算库，矩阵全部定义为double型，如果仅仅执行
			 了创建矩阵或创建向量函数（没有调用Zeros、Ones等函数），那么矩阵也就
			 只完成了内存空间的分配，矩阵的内容并没有确定，需要调用MATRIX_Set函
			 数初始化矩阵的内容
  * @notice	 ！！不要在中断中进行矩阵操作，浮点计算会消耗更多的任务栈资源和时间
			 ！！矩阵使用完毕后，一定要调用MATRIX_Destroy（）销毁，否则堆空间得不到释放
  ******************************************************************************  
  */ 
  
#include "stdlib.h"
  
#include "matrix.h"



/* 矩阵和向量的创建和销毁函数定义----------------------------------------------------------*/		
											   
/****
	* @brief	创建一个row行，col列的矩阵，通过malloc为其开辟空间，所以在使用完
				毕后，务必调用MATRIX_Destroy（）函数销毁矩阵以释放堆空间。矩阵的
				类型默认为void型，使用时需要通过类型转换将其转换成想要的类型
	* @param  	row，col：矩阵的行和列
	* @param  	pMatrix：矩阵的指针
	* @retval 	0：成功创建矩阵；
				1：参数有误（包括：行数无效、列数无效）；
				2：矩阵的大小超出边界
				3：malloc返回空指针，空间分配失败。
	*/
int MATRIX_CreateMatrix( int row, int col, Matrix_t *pMatrix )
{
	void *p = NULL;
	
	
	//矩阵大小
	if( row <= 0 || col <= 0 ) { return MatrixErr_InvalidSize; }
	if( row > matrixMAX_ROW || col > matrixMAX_COL ) { return MatrixErr_Oversize; }
	
	*(uint32_t*)&pMatrix->row = row;
	*(uint32_t*)&pMatrix->col = col;
	
	p = malloc( row * col * sizeof( double ) );
	pMatrix->matrix = p;
	if ( p == NULL ) { return MatrixErr_MemAllocFailed; }
	
	return 0;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	创建一个行向量
	* @param  	col：向量的列数
	* @param  	pMatrix：矩阵结构的指针
	* @retval 	0：成功创建矩阵；
				1：参数有误（包括：行数无效、列数无效、元素大小无效）；
				2：矩阵的大小超出边界
				3：malloc返回空指针，空间分配失败。
	*/
int MATRIX_CreateRowVector( int col, Matrix_t *pMatrix )
{
	// 调用MATRIX_Create()
	return MATRIX_CreateMatrix( 1, col, pMatrix );
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	创建一个列向量
	* @param  	row：向量的行数
	* @param  	size：矩阵元素的大小（单位字节，比如int：4）
	* @param  	pMatrix：矩阵结构的指针
	* @retval 	0：成功创建矩阵；
				1：参数有误（包括：行数无效、列数无效、元素大小无效）；
				2：矩阵的大小超出边界
				3：malloc返回空指针，空间分配失败。
	*/
int MATRIX_CreateColVextor( int row, Matrix_t *pMatrix )
{
	//调用 MATRIX_CreateMatrix()
	return MATRIX_CreateMatrix( row, 1, pMatrix );
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	销毁矩阵，将矩阵的行和列置零，调用free（）释放堆上分配的空间，为
				避免重复销毁导致的堆内容损坏，释放空间后将矩阵的指针置为NULL
	* @param  	row, col：向量的行数和列数
	* @param  	pMatrix：矩阵结构的指针
	* @retval 	0：成功创建矩阵；
	* @retval 	0：成功创建矩阵；
				1：参数有误（包括：行数无效、列数无效、元素大小无效）；
				2：矩阵的大小超出边界
				3：malloc返回空指针，空间分配失败。
	*/
int MATRIX_Destroy( Matrix_t *pMatrix )
{
	if( pMatrix->matrix == NULL ) return 0;
	
	*(uint32_t*)&pMatrix->row = 0;
	*(uint32_t*)&pMatrix->col = 0;
	free( pMatrix->matrix );
	pMatrix->matrix = NULL;
	
	return 0;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	创建指定大小的矩阵，并将矩阵全部置零
	* @param  	pMatrix：矩阵结构的指针
	* @retval 	0：成功创建矩阵；
				1：参数有误（包括：行数无效、列数无效、元素大小无效）；
				2：矩阵的大小超出边界
				3：malloc返回空指针，空间分配失败。
	*/
int MATRIX_Zeros( int row, int col, Matrix_t *pMatrix )
{
	int i, j;
	int res = MATRIX_CreateMatrix( row, col, pMatrix );
	
	if( res ) { return res; }
	
	while( row-- )
	{
		while( col-- )
		{
			*pMatrix->matrix++ = 0; 
		}
	}
	
	return 0;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	创建指定大小的矩阵，并将矩阵全部置1
	* @param  	pMatrix：矩阵结构的指针
	* @retval 	0：成功创建矩阵；
				1：参数有误（包括：行数无效、列数无效、元素大小无效）；
				2：矩阵的大小超出边界
				3：malloc返回空指针，空间分配失败。
	*/
int MATRIX_Ones( int row, int col, Matrix_t *pMatrix )
{
	int i, j;
	
	int res = MATRIX_CreateMatrix( row, col, pMatrix );
	if( res ) { return res; }
	
	while( row-- )
	{
		while( col-- )
		{
			*pMatrix->matrix++ = 1; 
		}
	}
	
	return 0;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	创建指定大小的对角矩阵，对角元素的值即为传入的值
	* @param  	pMatrix：矩阵结构的指针
	* @retval 	0：成功创建矩阵；
				1：参数有误（包括：行数无效、列数无效、元素大小无效）；
				2：矩阵的大小超出边界
				3：malloc返回空指针，空间分配失败。
	*/	
int MATRIX_Diag( int n /*row or col*/, double value, Matrix_t *pMatrix )	
{
	int i = 0;

	int result = MATRIX_CreateMatrix( n, n, pMatrix );	//分配内存空间
	if( result ) { return result; }
	
	//初始化对角线元素的值
	for( i = 0; i < n; i++ )
		if( i % ( n + 1 ) == 0 ) *( pMatrix->matrix + i ) = value;
	
	return 0;	
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	创建指定大小的矩阵，并将矩阵全部置1
	* @param  	pMatrix：矩阵结构的指针
	* @retval 	0：成功创建矩阵；
				1：参数有误（包括：行数无效、列数无效、元素大小无效）；
				2：矩阵的大小超出边界
				3：malloc返回空指针，空间分配失败。
	*/
int MATRIX_Constants( int row, int col, double value, Matrix_t *pMatrix )
{
	int i, j;
	
	int result = MATRIX_CreateMatrix( row, col, pMatrix );	//分配内存空间
	if( result ) { return result; }
	
	while( row-- )
	{
		while( col-- )
		{
			*pMatrix->matrix = 1; 
		}
	}
	
	return 0;
}


