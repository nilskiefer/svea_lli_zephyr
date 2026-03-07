#ifndef PROJECT_APP_THREADS_H_
#define PROJECT_APP_THREADS_H_

void lsm6dsox_stub_start(void);
void ads1115_stub_start(void);
void ina3221_stub_start(void);
void bq76942_stub_start(void);
void ina226_stub_start(void);
void heartbeat_stub_start(void);
void rc_command_stub_start(void);
int uart_publisher_start(void);

#endif
