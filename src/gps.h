#ifndef _GPS_H_
#define _GPS_H_

void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka,
                      uint8_t* ckb);
uint8_t _gps_verify_checksum(uint8_t* data, uint8_t len);
void sendUBX(uint8_t *MSG, uint8_t len);
void gps_on(void);
void gps_off(void);
void setupGPS(void);
uint8_t gps_check_nav(void);
void gps_get_data(void);
void gps_check_lock(void);
void gps_get_position(void);

extern int32_t lat;
extern int32_t lon;
extern int32_t alt;
extern uint8_t lock;
extern uint8_t sats;

#endif