/*
 * Config.h
 *
 * Created: 11/15/2019 2:09:43 PM
 *  Author: NEO
 */ 

#ifndef CONFIG_H#define CONFIG_H//#define HW_3v4//#define HW_4v0#define HW_4v1
#ifdef HW_3v4
uint16_t HW_VER = 304;
#elif defined(HW_4v0)
uint16_t HW_VER = 400;
#elif defined(HW_4v1)
uint16_t HW_VER = 401;
#endif
uint16_t TTL_FW = 7; // Format: v12.34 = 1234 | v0.5 = 0005#define BOOT_BTN PIN_PA15#endif