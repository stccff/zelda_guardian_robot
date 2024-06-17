#ifndef _I2S_STD_SOUND_H_
#define _I2S_STD_SOUND_H_

enum SoundType {
    SOUND_EYE_ON,
    SOUND_SEARCH,
    SOUND_LOCKING,
    SOUND_LOCKED,
    SOUND_BEAM,

    SOUND_MAX_NUM,
};


extern void sound_init(void);
extern uint32_t get_volume(void);
extern void sound_play_eyeon(bool isPlay);

#endif

