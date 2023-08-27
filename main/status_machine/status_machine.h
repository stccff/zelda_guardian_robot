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
    SM_BL_SCAN_ON = 6,
    
    SM_BLUETOOTH_NUN,
};

extern void status_machine_init(void);
extern void sm_set_light_status(enum LightStatus status);
extern enum LightStatus sm_get_light_status(void);
extern void sm_set_bluetooth_status(enum BluetoothStatus status);
extern enum BluetoothStatus sm_get_bluetooth_status(void);



#endif  // __STATUS_MACHINE_H__

