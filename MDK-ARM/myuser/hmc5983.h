#ifndef __HMC5983_H
#define __HMC5983_H

void hmc5983_init(void);
void hmc5983task(void const * argument);
void hmc5983_read(void);
void hmc5983jzinit(void);

#endif
