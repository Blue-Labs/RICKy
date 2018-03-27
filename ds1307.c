void DS1307_init(void) {
    i2c_start(DS1307_I2CADDR);
}

int DS1307_printsec(void) {
    int sec,min,hour,weekday,date,month,year;

    i2c_start(DS1307_I2CADDR);
    i2c_write(2);
    i2c_stop();

    while(1) {
        i2c_start(DS1307_I2CADDR+1);
        sec=i2c_read_ack();
        min=i2c_read_ack();
        hour=i2c_read_ack();
        weekday=i2c_read_ack();
        date=i2c_read_ack();
        month=i2c_read_ack();
        year=i2c_read_nack();
        i2c_stop();

        printf("DS1307: %i/%i/%i %i %i:%i:%i\r\n", year, month, date, weekday, hour, min, sec);
    }
}

