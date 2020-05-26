// OpenMPTest.cpp : 定义控制台应用程序的入口点。
//
 
#include <stdio.h>
#include <omp.h>
#include <iostream>
 
using namespace std;
 
//void main()
//{
//#pragma omp parallel for num_threads(6)
//	//上面程序指定了6个线程，迭代量为12，每个线程都分配到了2次的迭代量
//	//备注：如果for循环比较简单（执行时间短），不建议使用多线程并发，因为线程
//	//间的调度也会比较耗时，是一个不小的开销。
//	for (int i = 0; i < 12;i++)
//	{	
//		printf("OpenMP Test,线程编号为：%d\n", omp_get_thread_num());
//	}	
//	system("pause");
//    
//}
 
//OpenMP效率提升以及不同线程数效率对比
 
void test()
{
	for (int i = 0; i < 8000; i++)
	{
 
	}
}
 
int main()
{
	float startTime = omp_get_wtime();
 
	//指定两个线程
#pragma omp parallel for num_threads(2)
	for (int i = 0; i < 80000; i++)
	{
		test();
	}
	float endTime = omp_get_wtime();
	printf("指定 2 个线程，执行时间: %f\n", endTime - startTime);
	startTime = endTime;
 
	//指定4个线程
#pragma omp parallel for num_threads(4)
	for (int i = 0; i < 80000; i++)
	{
		test();
	}
	endTime = omp_get_wtime();
	printf("指定 4 个线程，执行时间: %f\n", endTime - startTime);
	startTime = endTime;
 
	//指定8个线程  
#pragma omp parallel for num_threads(8)
	for (int i = 0; i < 80000; i++)
	{
		test();
	}
	endTime = omp_get_wtime();
	printf("指定 8 个线程，执行时间: %f\n", endTime - startTime);
	startTime = endTime;
 
	//指定12个线程
#pragma omp parallel for num_threads(12)
	for (int i = 0; i < 80000; i++)
	{
		test();
	}
	endTime = omp_get_wtime();
	printf("指定 12 个线程，执行时间: %f\n", endTime - startTime);
	startTime = endTime;
 
	//不使用OpenMP
	for (int i = 0; i < 80000; i++)
	{
		test();
	}
	endTime = omp_get_wtime();
	printf("不使用OpenMP多线程，执行时间: %f\n", endTime - startTime);
	startTime = endTime;

	return 0;
}
