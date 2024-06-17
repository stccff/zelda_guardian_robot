#ifndef _HID_HOST_GAMEPAD_
#define _HID_HOST_GAMEPAD_


#define XBOX_TRIGGER_MAX    (0x3ff)
#define XBOX_JOYSTICK_MAX   (0xffff)


/* 结构体定义 */
struct Button1 {
    uint8_t btnA : 1;
    uint8_t btnB : 1;
    uint8_t : 1;
    uint8_t btnX : 1;
    uint8_t btnY : 1;
    uint8_t : 1;
    uint8_t btnL : 1;
    uint8_t btnR : 1;
};

struct Button2 {
    uint8_t : 2;
    uint8_t btnSel : 1;
    uint8_t btnStart : 1;
    uint8_t btnXbox : 1;
    uint8_t btnLs : 1;
    uint8_t btnRs : 1;
    uint8_t : 1;    
};

struct Button3 {
    uint8_t share : 1;
    uint8_t : 7;    
};

struct XboxData {
    uint16_t joyLx;
    uint16_t joyLy;
    uint16_t joyRx;
    uint16_t joyRy;
    uint16_t trigLT;
    uint16_t trigRT;
    uint8_t DPad;           // ↑-1,↗-2,...,↖-8
    struct Button1 bnt1;    // A-1,B-2,X-8,Y-10,L-40,R-80
    struct Button2 bnt2;    // sel-4,start-8,X-10,Ls-20,Rs-40,
    struct Button3 bnt3;    // share-1
};

/* 函数声明 */
extern void hid_host_init(void);
extern const struct XboxData* get_xbox_pad_data(void);
extern void set_xbox_pad_data_virtual(struct XboxData* xbox);



#endif