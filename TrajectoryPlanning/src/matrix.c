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
#include "string.h"

#include "matrix.h"
#include "usart.h"



/************************* 矩阵和向量的创建和销毁函数 *************************/	
											   
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
	//矩阵大小
	if( row <= 0 || col <= 0 ) { return MatrixErr_InvalidSize; }
	if( row > matrixMAX_ROW || col > matrixMAX_COL ) { return MatrixErr_Oversize; }
	
	*(uint32_t*)&pMatrix->row = row;
	*(uint32_t*)&pMatrix->col = col;

	
	pMatrix->matrix = ( double* )malloc( row * col * sizeof( double ) );
	if ( pMatrix->matrix == NULL ) { return MatrixErr_MemAllocFailed; }
	
	pMatrix->isNamed = 0;
	return 0;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	创建一个行向量
	* @param  	col：向量的列数
	* @param  	pMatrix：矩阵结构的指针
	* @retval 	0：成功创建矩阵；
				1：参数有误（包括：行数无效、列数无效）；
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
				1：参数有误（包括：行数无效、列数无效）；
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
				1：参数有误（包括：行数无效、列数无效）；
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
	* @brief	为矩阵命名，以便于在打印矩阵时清晰的分辨不同矩阵
	* @param  	pMatrix：矩阵结构的指针
	* @param  	name：矩阵名字，长度不大于matrixMAX_NAME_LEN
	* @retval 	无
	*/
void MATRIX_SetName( Matrix_t *pMatrix, const char * const pcName )
{
	int i = 0;
	if( pcName != NULL )
	{
		for( i = 0; i < matrixMAX_NAME_LEN; i++ )
		{
			pMatrix->name[i] = pcName[i];
	
			if( pcName[i] == ( char ) 0 )
			{
				break;
			}
				
		}
		
		//确保字符串正确结束
		pMatrix->name[ matrixMAX_NAME_LEN - 1 ] = '\0';
		
		pMatrix->isNamed = 1;
	}
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	创建指定大小的矩阵，并将矩阵全部置零
	* @param  	pMatrix：矩阵结构的指针
	* @retval 	0：成功创建矩阵；
				1：参数有误（包括：行数无效、列数无效）；
				2：矩阵的大小超出边界
				3：malloc返回空指针，空间分配失败。
	*/
int MATRIX_Zeros( int row, int col, Matrix_t *pMatrix )
{
	int i, j;
	double *p = pMatrix->matrix;
	int res = MATRIX_CreateMatrix( row, col, pMatrix );
	
	if( res ) { return res; }
	
	for( i = 0; i < row; i++ )
	{
		for( j = 0; j < col; j++ )
		{
			pMatrix->matrix[ i * col + j ] = 0; 
		}
	}
	
	return 0;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	创建指定大小的矩阵，并将矩阵全部置1
	* @param  	pMatrix：矩阵结构的指针
	* @retval 	0：成功创建矩阵；
				1：参数有误（包括：行数无效、列数无效）；
				2：矩阵的大小超出边界
				3：malloc返回空指针，空间分配失败。
	*/
int MATRIX_Ones( int row, int col, Matrix_t *pMatrix )
{
	uint32_t i = 0, j = 0;
	
	int res = MATRIX_CreateMatrix( row, col, pMatrix );
	if( res ) { return res; }
	
	for( i = 0; i < row; i++ )
	{
		for( j = 0; j < col; j++ )
		{
			pMatrix->matrix[ i * col + j ] = 1; 
		}
	}
	
	return 0;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	创建指定大小的对角矩阵，对角元素的值即为传入的值
	* @param  	pMatrix：矩阵结构的指针
	* @retval 	0：成功创建矩阵；
				1：参数有误（包括：行数无效、列数无效）；
				2：矩阵的大小超出边界
				3：malloc返回空指针，空间分配失败。
	*/	
int MATRIX_Diag( int n /*row or col*/, double value, Matrix_t *pMatrix )	
{
	int i = 0;

	int result = MATRIX_CreateMatrix( n, n, pMatrix );	//分配内存空间
	if( result ) { return result; }
	
	//初始化对角线元素的值
	for( i = 0; i < n*n; i++ )
		if( i % ( n + 1 ) == 0 ) { pMatrix->matrix[i] = value; }
	
	return 0;	
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	创建指定大小的矩阵，并将矩阵全部置1
	* @param  	pMatrix：矩阵结构的指针
	* @retval 	0：成功创建矩阵；
				1：参数有误（包括：行数无效、列数无效）；
				2：矩阵的大小超出边界
				3：malloc返回空指针，空间分配失败。
	*/
int MATRIX_Constants( int row, int col, double value, Matrix_t *pMatrix )
{
	uint32_t i, j;
	
	int result = MATRIX_CreateMatrix( row, col, pMatrix );	//分配内存空间
	if( result ) { return result; }
	
	for( i = 0; i < row; i++ )
	{
		for( j = 0; j < col; j++ )
		{
			pMatrix->matrix[ i * col + j ] = 1; 
		}
	}
	
	return 0;
}
/* ---------------------------------------------------------------------------*/
/******************************** 矩阵判断函数 ********************************/
/****
	* @brief	判断矩阵是否为空
	* @param  	matrix：传入矩阵
	* @retval 	true：空， false：非空
	*/	
bool MATRIX_IsEmpty( Matrix_t matrix )
{
	if( matrix.matrix == NULL 
		|| matrix.col == 0
		|| matrix.row == 0) { return true; }
	return false;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	判断矩阵行列数是否匹配,如果是转置，两个输入都为需要转置的矩阵即可
	* @param  	matrix：传入矩阵
	* @param  	matrix：传入矩阵
	* @retval 	true：匹配， false：不匹配
	*/	
bool MATRIX_SizeMatched( Matrix_t leftMatrix, Matrix_t rightMatrix, MatrixOprt_t operation)
{
	//如果是矩阵加减，则行列分别相等
	if( operation == MatrixOprt_Plus || operation == MatrixOprt_Minus )
	{
		if( leftMatrix.row == rightMatrix.row && leftMatrix.col == rightMatrix.col )
			return true;
		else
			return false;
	}
	//如果是矩阵乘除，左矩阵的列数等于又矩阵的行数
	if( operation == MatrixOprt_Multiply || operation == MatrixOprt_Devide )
	{
		if( leftMatrix.col == rightMatrix.row )
			return true;
		else
			return false;
	}
	//如果是矩阵求逆，矩阵的行数等于列数
	if( operation == MatrixOprt_Inverse )
	{
		if( leftMatrix.col == leftMatrix.row && rightMatrix.col == rightMatrix.row )
			return true;
		else
			return false;
	}
}
			
/******************************** 矩阵变形函数 ********************************/


/******************************** 矩阵运算函数 ********************************/

/****
	* @brief	计算两个矩阵的和
	* @param  	leftMatrix：左矩阵
	* @param  	leftMatrix：右矩阵
	* @param  	pmResult：保存结果的矩阵指针
	* @retval 	0：计算过程无误；
				4：矩阵行数或列数不匹配
				5：输入矩阵为空
	*/
int MATRIX_Plus( Matrix_t leftMatrix, Matrix_t rightMatrix, Matrix_t *pmResult )
{
	int i, j, row, col;
	//检查矩阵是否为空
	if( MATRIX_IsEmpty( leftMatrix ) || MATRIX_IsEmpty( rightMatrix ) )
		return MatrixErr_EmptyMatrix;
	//检查矩阵大小是否匹配
	if( !MATRIX_SizeMatched( leftMatrix, rightMatrix, MatrixOprt_Plus ) )
		return MatrixErr_SizeNotMatch;
	
	row = leftMatrix.row;
	col = leftMatrix.col;
	
	for( i = 0; i < row; i++ )
	{
		for( j = 0; j < col; j++ )
		{
			pmResult->matrix[ i * col + j ] = leftMatrix.matrix[ i * col + j ] 
											+ rightMatrix.matrix[ i * col + j ];
		}
	}
	
	return MatrixErr_NoError;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	计算两个矩阵的差
	* @param  	leftMatrix：左矩阵
	* @param  	leftMatrix：右矩阵
	* @param  	pmResult：保存结果的矩阵指针
	* @retval 	0：计算过程无误；
				4：矩阵行数或列数不匹配
				5：输入矩阵为空
	*/
int MATRIX_Minus( Matrix_t leftMatrix, Matrix_t rightMatrix, Matrix_t *pmResult )
{
	int i, j, row, col;
	//检查矩阵是否为空
	if( MATRIX_IsEmpty( leftMatrix ) || MATRIX_IsEmpty( rightMatrix ) )
		return MatrixErr_EmptyMatrix;
	//检查矩阵大小是否匹配
	if( !MATRIX_SizeMatched( leftMatrix, rightMatrix, MatrixOprt_Minus ) )
		return MatrixErr_SizeNotMatch;
	
	row = leftMatrix.row;
	col = leftMatrix.col;
	
	for( i = 0; i < row; i++ )
	{
		for( j = 0; j < col; j++ )
		{
			pmResult->matrix[ i * col + j ] = leftMatrix.matrix[ i * col + j ] 
											- rightMatrix.matrix[ i * col + j ];
		}
	}
	
	return MatrixErr_NoError;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	计算两个矩阵的乘积
	* @param  	leftMatrix：左矩阵
	* @param  	leftMatrix：右矩阵
	* @param  	pmResult：保存结果的矩阵指针
	* @retval 	0：计算过程无误；
				4：矩阵行数或列数不匹配
				5：输入矩阵为空
	*/
int MATRIX_Multiply( Matrix_t leftMatrix, Matrix_t rightMatrix, Matrix_t *pmResult )
{
	int i, j, k, l, row1, col1, row2, col2;
	double sum;
	//检查矩阵是否为空
	if( MATRIX_IsEmpty( leftMatrix ) || MATRIX_IsEmpty( rightMatrix ) )
		return MatrixErr_EmptyMatrix;
	//检查矩阵大小是否匹配
	if( !MATRIX_SizeMatched( leftMatrix, rightMatrix, MatrixOprt_Multiply ) )
		return MatrixErr_SizeNotMatch;
	
	row1 = leftMatrix.row;
	col1 = leftMatrix.col;
	row2 = rightMatrix.row;
	col2 = rightMatrix.col;
	
	//结果矩阵的行数为左矩阵的行数
	for( i = 0; i < row1; i++ )
	{
		//结果矩阵的列数为右矩阵的列数
		for( j = 0; j < col2; j++ )
		{
			sum = 0;
			for( k = 0; k < col1; k++ )
			{
				sum += leftMatrix.matrix[ i * col1 + k ] 
					   * rightMatrix.matrix[ k * col2 + j ];
			}
			pmResult->matrix[ i * col2 + j ] = sum;
		}
	}
	
	return MatrixErr_NoError;
}
	
/***************************** 矩阵取值与赋值函数 *****************************/
/****
	* @brief	将数组的值赋给矩阵，如果数组长度小于矩阵长度，则只赋值矩阵前len
				个元素，否则，按顺序赋值全部矩阵元素
	* @param  	pMatrix：矩阵结构的指针
	* @param	pbSrc：数组
	* @param	len：数组长度
	* @retval 	0：无错误
				1：参数有误（包括：行数无效，列数无效，数组长度无效）；
				5：空矩阵
	*/
int MATRIX_Set( Matrix_t *pMatrix, double *pbSrc, int len)
{
	uint32_t i, j;
	uint32_t count = 0;
	
	//检查矩阵和数组的大小是否有效
	if( pMatrix->row <=0 || pMatrix->col <= 0 || len <= 0 ) 
		return MatrixErr_InvalidSize;
	
	//检查矩阵数组头地址是否为NULL
	if( pMatrix->matrix == NULL ) { return MatrixErr_EmptyMatrix; }
	
	for( i = 0; i < pMatrix->row; i++ )
	{
		if( !len ) { break; }
		for( j = 0; j < pMatrix->col; j++ )
		{
			if( !len-- ) { break; }
			pMatrix->matrix[ i * pMatrix->col+ j ] = *pbSrc++; 
		}
	}

	return MatrixErr_NoError;
}




/******************************** 矩阵输出函数 ********************************/
void MATRIX_Print( Matrix_t xMatrix)
{
	uint32_t i, j;
	double *p = xMatrix.matrix;
	
	if( xMatrix.matrix == NULL ) { return ; }
	
	if( xMatrix.isNamed ) { printf( "%s\t", xMatrix.name ); }
	else { printf( "Unamed Matrix\t\t" ); }
	for( j = 0; j < xMatrix.col; j++ ) { printf("%d\t", j ); }
	printf( "\r\n\r\n" );
	for( i = 0; i < xMatrix.row; i++ )
	{
		printf( "\t%d\t", i );
		for( j = 0; j < xMatrix.col; j++ ) { printf("%4.3f\t", *p++ ); }
		printf( "\r\n" );
	}
	printf( "\r\n" );
			
}
