/**
  ***********************************UTF-8**************************************
  * @file    matrix.h
  * @author  Xiong
  * @version V0.1
  * @date    31-July-2020
  * @brief   此文件用户实现C下的矩阵运算库
  ******************************************************************************  
  */ 
#ifndef _MATRIX_H
#define _MATRIX_H

#include "stdint.h"

#define matrixMAX_ROW			( ( uint32_t ) 200 )
#define matrixMAX_COL			( ( uint32_t ) 200 )

/* 矩阵类型定义---------------------------------------------------------------*/
/* 定义矩阵类型结构体*/
typedef struct 
{
	
	const uint32_t row;	//定义为只读，但可以通过矩阵变形函数改变
	const uint32_t col;
	double* matrix;
}Matrix_t;

/* 定义错误枚举 */
typedef enum 
{
	MatrixErr_NoError,
	MatrixErr_InvalidSize,
	MatrixErr_Oversize,
	MatrixErr_MemAllocFailed,
	MatrixErr_SizeNotMatch,
}MatrixErr_t;

/* 矩阵和向量的创建和销毁函数-------------------------------------------------*/
int MATRIX_CreateMatrix( int row, int col, Matrix_t *pMatrix );
int MATRIX_CreateRowVector( int col, Matrix_t *pMatrix );
int MATRIX_CreateColVector( int row, Matrix_t *pMatrix );
int MATRIX_Destroy( Matrix_t *pMatrix );

/* 创建矩阵（向量）并赋初值函数*/
int MATRIX_Zeros( int row, int col, Matrix_t *pMatrix );
int MATRIX_Ones( int row, int col, Matrix_t *pMatrix );
int MATRIX_Constants( int row, int col, double value, Matrix_t *pMatrix );
int MATRIX_Diag( int n /*row or col*/, double value, Matrix_t *pMatrix );

/* 矩阵变形函数 */
int MATRIX_Resize( int newRow, int newCol, Matrix_t *pMatrix );
int MATRIX_AppendOneColumn( Matrix_t *pMatrix );
int MATRIX_AppendOneRow( Matrix_t *pMatrix );

/* 矩阵运算函数定义 ----------------------------------------------------------*/
Matrix_t MATRIX_Plus( Matrix_t, Matrix_t );
Matrix_t MATRIX_Minus( Matrix_t mSubstahend, Matrix_t );
Matrix_t MATRIX_Multiply( Matrix_t leftMatrix, Matrix_t rightMatrix);
Matrix_t MATRIX_Devide( Matrix_t mDevidend, Matrix_t mDevisor);
Matrix_t MATRIX_Inverse( Matrix_t );
Matrix_t MATRIX_Transpose( Matrix_t );

/* 矩阵取值与赋值函数定义 ----------------------------------------------------*/
int MATRIX_GetRows(int startRow, int rowCount, Matrix_t *pSrc, Matrix_t *pDst );
int MATRIX_GetRows(int startRow, int rowCount, Matrix_t *pSrc, Matrix_t *pDst );
double MATRIX_GetElement( int row, int col, Matrix_t *pMatrix );
void MATRIX_Set( Matrix_t *pMatrix, double *pbSrc );

#endif
