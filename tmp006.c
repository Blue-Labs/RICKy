void TMP006_init(void) {
    i2c_writeReg(TMP006_I2CADDR, TMP006_CONFIG, (uint8_t []){0x70 | 0x01, 0x00}, 2);
}

