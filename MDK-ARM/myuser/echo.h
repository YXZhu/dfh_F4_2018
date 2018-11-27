#ifndef __ECHO_H
#define __ECHO_H


void UDelayUS(unsigned int ulCount);
void EchoFreertosInit(void);
//void Echo_Init(void);
//void echo_1(void);
//void echo_2(void);
//void echo_3(void);
//void echo_4(void);
//void echo_5(void);
//void echo_6(void);
void Echo_1task(void const * argument);
void Echo_2task(void const * argument);
void Echo_3task(void const * argument);
void Echo_4task(void const * argument);
void Echo_5task(void const * argument);
void Echo_6task(void const * argument);

#endif 
