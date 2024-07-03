#ifndef _INTERRUPT_FUNC_H_
#define _INTERRUPT_FUNC_H_

#if __cplusplus
extern "C"{
#endif

void int_spi1();
void int_spi2();
void int_1ms();
void int_10ms();
void int_125ums();

#if __cplusplus
};
#endif

#endif //_INTERRUPT_FUNC_H_
