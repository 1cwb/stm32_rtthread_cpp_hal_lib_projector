#pragma once
#include "mbarometordrv.hpp"
#include "mi2cdrv.hpp"

#define SPL06_001_ADDR0 (0x77)
#define SPL06_001_ADDR1 (0x76)

typedef struct {
    uint32_t MEAS_CTRL : 3;
    uint32_t RESERVED1 : 1;
    uint32_t PRS_RDY : 1;
    uint32_t TMP_RDY : 1;
    uint32_t SENSOR_RDY : 1;
    uint32_t COEF_RDY : 1;
} Sensor_Status_t;

enum Oversampling_Rate_t {
    _1_TIMES = 0,
    _2_TIMES,
    _4_TIMES,
    _8_TIMES,
    _16_TIMES,
    _32_TIMES,
    _64_TIMES,
    _128_TIMES,
};

class ArtronShop_SPL06_001 : public mDev::mBarometor{
    private:
        const uint8_t _addr = SPL06_001_ADDR0;
        mDev::mI2c *_wire = nullptr;
        struct {
            int16_t c0, c1, c01, c11, c20, c21, c30;
            int32_t c00, c10;
        } calibration_coefficients;
        uint8_t p_oversampling_rate = _16_TIMES;
        uint8_t t_oversampling_rate = _16_TIMES;
        const uint32_t scale_factor[8] = {
            524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960 // Ref. Table 7 : Compensation Scale Factors
        };
        float Pcomp = -1, Tcomp = -1;

        bool write_reg(uint8_t reg, uint8_t *value, size_t len) ;
        bool write_reg(uint8_t reg, uint8_t value) ;
        bool read_reg(uint8_t reg, uint8_t *value, size_t len = 1) ;
        uint8_t read_reg(uint8_t reg) ;

        bool status(Sensor_Status_t *status) ;

    public:
        ArtronShop_SPL06_001(const char* name, mDev::mI2c *wire) ;
        virtual ~ArtronShop_SPL06_001() = default;
        bool begin() ;
        bool measure() ;
        float pressure() ;
        float temperature() ;
        virtual float getTemp() override {return temperature();};
        virtual bool updateData() override {return measure();}
        virtual float getPressure() override {return pressure();}
};