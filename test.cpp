#include<stdio.h>

#define EXAMPLE0 0
#define EXPAMLE1 1
#define EXAMPLE2 0

int main(void)
{
    volatile int i;
	
	for(i=0 ; i < 100 ; i++)
	{
		printf("%d\n",i);
	}
	printf("%d\n", i);	

}
