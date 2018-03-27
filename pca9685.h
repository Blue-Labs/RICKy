#define PCA9685_1 0x80 // address is documented as 0x40 but the device responds with 0x80
#define PCA9685_MODE1 0x0
#define PCA9685_MODE2 0x1
#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4
#define PCA9685_ALLCALL 0x5
#define PCA9685_CH0L 0x6
#define PCA9685_ALLONL 0xfa
#define PCA9685_ALLOFFL 0xfc
#define PCA9685_PRESCALE 0xfe

void PCA9685_init(void);
void PCA9685_set_ch(uint8_t, uint16_t, uint16_t);
void PCA9685_set_ch_percent(uint8_t, uint8_t);
