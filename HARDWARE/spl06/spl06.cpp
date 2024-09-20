#include "spl06.hpp"
#include "delay.h"
// Register list
#define REG_PSR_B2 (0x00)
#define REG_PSR_B1 (0x01)
#define REG_PSR_B0 (0x02)
#define REG_TMP_B2 (0x03)
#define REG_TMP_B1 (0x04)
#define REG_TMP_B0 (0x05)
#define REG_PSR_CFG (0x06)
#define REG_TMP_CFG (0x07)
#define REG_MEAS_CFG (0x08)
#define REG_CFG_REG (0x09)
#define REG_INT_STS (0x0A)
#define REG_FIFO_STS (0x0B)
#define REG_RESET (0x0C)
#define REG_ID (0x0D)
// Calibration Coefficients (COEF)
#define REG_COEF_C0 (0x10)
#define REG_COEF_C0C1 (0x11)
#define REG_COEF_C1 (0x12)
#define REG_COEF_C00a (0x13)
#define REG_COEF_C00b (0x14)
#define REG_COEF_C00C10 (0x15)
#define REG_COEF_C10a (0x16)
#define REG_COEF_C10b (0x17)
#define REG_COEF_C01a (0x18)
#define REG_COEF_C01b (0x19)
#define REG_COEF_C11a (0x1A)
#define REG_COEF_C11b (0x1B)
#define REG_COEF_C20a (0x1C)
#define REG_COEF_C20b (0x1D)
#define REG_COEF_C21a (0x1E)
#define REG_COEF_C21b (0x1F)
#define REG_COEF_C30a (0x20)
#define REG_COEF_C30b (0x21)

ArtronShop_SPL06_001::ArtronShop_SPL06_001(const char* name, mDev::mI2c *wire) :mDev::mBarometor(name) , _wire(wire) {
    // -----
}

bool ArtronShop_SPL06_001::write_reg(uint8_t reg, uint8_t *value, size_t len) {

    return this->_wire->writeReg(_addr, reg, value, len) == M_RESULT_EOK;
}

bool ArtronShop_SPL06_001::write_reg(uint8_t reg, uint8_t value) {
    return this->write_reg(reg, &value, 1);
}

bool ArtronShop_SPL06_001::read_reg(uint8_t reg, uint8_t *value, size_t len) {
    return this->_wire->readReg(_addr, reg, value, len) == M_RESULT_EOK;
}

uint8_t ArtronShop_SPL06_001::read_reg(uint8_t reg) {
    uint8_t value = 0;
    this->read_reg(reg, &value, 1);
    return value;
}

#define bitRead(x, n)  ((x >> n) & (0x01))

bool ArtronShop_SPL06_001::begin() {
    this->write_reg(REG_RESET, 0b1001); // soft reset
    //delay_ms(100); // wait sensor ready and coefficients are available 

    Sensor_Status_t status;
    while(true)
    {
        delay_ms(50);
        this->status(&status);
        if((status.SENSOR_RDY) && (status.COEF_RDY)) // Check sensor ready and coefficients are available flag
        {
            printf("Sensor ready!\r\n");
            printf("read_reg(REG_ID) = %x\r\n",read_reg(REG_ID));
            break;
        }
    }

    this->write_reg(REG_MEAS_CFG, 0b111); // Set measurement mode: Continuous pressure and temperature measurement
    this->write_reg(REG_PSR_CFG, 0b01000000 | (this->p_oversampling_rate & 0x07)); // Pressure measurement rate: 100 - 16 measurements pr. sec., Pressure oversampling rate: ....
    this->write_reg(REG_TMP_CFG, 0b11000000 | (this->t_oversampling_rate & 0x07)); // Temperature measurement rate: 100 - 16 measurements pr. sec., Temperature oversampling rate: ...
    this->write_reg(REG_CFG_REG, (this->t_oversampling_rate >= _16_TIMES ? (1<<3) : 0) | (this->p_oversampling_rate >= _16_TIMES ? (1<<2) : 0)); // measurement data shift

    // Read Calibration Coefficients
    uint8_t buff[18];
    this->read_reg(REG_COEF_C0, buff, sizeof(buff));
    calibration_coefficients.c0 = ((int16_t)(buff[0]) << 4) | (buff[1] >> 4);
    calibration_coefficients.c1 = ((int16_t)(buff[1] & 0x0F) << 8) | buff[2];
    calibration_coefficients.c00 = (((int32_t)(buff[3]) << 12) | ((int32_t)(buff[4]) << 4) | (buff[5] >> 4));
    calibration_coefficients.c10 = ((int32_t)(buff[5] & 0x0F) << 16) | ((int32_t)(buff[6]) << 8) | buff[7];
    calibration_coefficients.c01 = ((int16_t)(buff[8]) << 8) | buff[9];
    calibration_coefficients.c11 = ((int16_t)(buff[10]) << 8) | buff[11];
    calibration_coefficients.c20 = ((int16_t)(buff[12]) << 8) | buff[13];
    calibration_coefficients.c21 = ((int16_t)(buff[14]) << 8) | buff[15];
    calibration_coefficients.c30 = ((int16_t)(buff[16]) << 8) | buff[17];

    if (bitRead(calibration_coefficients.c0, 11)) {
        calibration_coefficients.c0 |= 0xF000;
    }
    if (bitRead(calibration_coefficients.c1, 11)) {
        calibration_coefficients.c1 |= 0xF000;
    }
    if (bitRead(calibration_coefficients.c00, 19)) {
        calibration_coefficients.c00 |= 0xFFF00000;
    }
    if (bitRead(calibration_coefficients.c10, 19)) {
        calibration_coefficients.c10 |= 0xFFF00000;
    }

    delay_ms(5); // wait first measurement

    return true;
}

bool ArtronShop_SPL06_001::status(Sensor_Status_t *status) {
    this->read_reg(REG_MEAS_CFG, (uint8_t*) status); // Sensor Operating Mode and Status (MEAS_CFG)

    return true;
}

bool ArtronShop_SPL06_001::measure() {
    this->Pcomp = this->Tcomp = -1.0f;

    Sensor_Status_t status;
    this->status(&status);

    uint8_t buff[6];
    this->read_reg(REG_PSR_B2, buff, 6);
    int32_t Praw = (((int32_t) buff[0]) << 16) | (((int32_t) buff[1]) << 8) | buff[2];
    if (bitRead(Praw, 23)) {
        Praw |= 0xFF000000; // Set left bits to one for 2's complement conversion of negative number
    }
    int32_t Traw = (((int32_t) buff[3]) << 16) | (((int32_t) buff[4]) << 8) | buff[5];
    if (bitRead(Traw, 23)) {
        Traw |= 0xFF000000; // Set left bits to one for 2's complement conversion of negative number
    }

    /*
    Serial.println("----------");
    Serial.print("C0: "); Serial.println(calibration_coefficients.c0);
    Serial.print("C1: "); Serial.println(calibration_coefficients.c1);
    Serial.print("C00: "); Serial.println(calibration_coefficients.c00);
    Serial.print("C10: "); Serial.println(calibration_coefficients.c10);
    Serial.print("C01: "); Serial.println(calibration_coefficients.c01);
    Serial.print("C11: "); Serial.println(calibration_coefficients.c11);
    Serial.print("C20: "); Serial.println(calibration_coefficients.c20);
    Serial.print("C21: "); Serial.println(calibration_coefficients.c21);
    Serial.print("C30: "); Serial.println(calibration_coefficients.c30);
    Serial.print("Traw: "); Serial.println(Traw);
    Serial.print("Praw: "); Serial.println(Praw);
    Serial.println("----------");
    */

    float Traw_sc = Traw / (float) scale_factor[this->t_oversampling_rate];
    float Praw_sc = Praw / (float) scale_factor[this->p_oversampling_rate];

    this->Pcomp = calibration_coefficients.c00 + Praw_sc * (calibration_coefficients.c10 + Praw_sc *(calibration_coefficients.c20 + Praw_sc * calibration_coefficients.c30)) + Traw_sc * calibration_coefficients.c01 + Traw_sc * Praw_sc * (calibration_coefficients.c11 + Praw_sc * calibration_coefficients.c21);
    this->Tcomp = calibration_coefficients.c0 * 0.5 + calibration_coefficients.c1 * Traw_sc;

    return true;
}

float ArtronShop_SPL06_001::pressure() {
    return this->Pcomp;
}

float ArtronShop_SPL06_001::temperature() {
    return this->Tcomp;
}