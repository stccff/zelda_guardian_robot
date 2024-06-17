#ifndef __STATUS_MACHINE_H__
#define __STATUS_MACHINE_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

enum LightStatus {
    SM_NO_LIGHT,
    SM_LIGHT_SCREEN,
    SM_LIGHT_SCREEN_BLUE,
    SM_LIGHT_SCREEN_RED,
    SM_LIGHT_SCREEN_ARGB,
    
    SM_LIGHT_NUN,
};

enum BluetoothStatus {
    SM_BL_SCAN_OFF,
    SM_BL_SCAN_ON,
    
    SM_BLUETOOTH_NUN,
};

enum OledDisplayType {
    OLED_CIRCLE,
    OLED_BAT,
    OLED_ATTACK,
    OLED_VOL,
    OLED_OFF,
    
    OLED_DISPLAY_NUN,
};

enum ActiveMod {
    SM_NORMAL,
    SM_DEMO,
    
    SM_ACTIVE_MOD_NUN,
};

extern void status_machine_init(void);
extern void sm_set_light_status(enum LightStatus status);
extern enum LightStatus sm_get_light_status(void);
extern void sm_set_bluetooth_status(enum BluetoothStatus status);
extern enum BluetoothStatus sm_get_bluetooth_status(void);
extern void oled_set_display(enum OledDisplayType type);
extern enum OledDisplayType oled_get_display(void);
extern void sm_set_active_mod(enum ActiveMod mod);
extern enum ActiveMod sm_get_active_mod(void);

#endif  // __STATUS_MACHINE_H__

