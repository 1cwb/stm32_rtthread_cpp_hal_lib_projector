#include "icm42688.h"
#include "mbase.h"
#if defined(ICM_USE_HARD_SPI)
#include "spi.h"

#elif defined(ICM_USE_HARD_I2C)
#include "bsp_cpu_i2c2.h"
#endif
#include "stdio.h"
#include "delay.h"

static float accSensitivity   = 0.244f;
static float gyroSensitivity  = 32.8f;

#define ICM42688DelayMs(_nms)  delay_ms(_nms)

#if defined(ICM_USE_HARD_SPI)
#define ICM_RCC_SPIX_CS()    __HAL_RCC_GPIOE_CLK_ENABLE()
#define ICM_PORT_SPIX_CS		 GPIOE
#define ICM_PIN_SPIX_CS	     GPIO_PIN_11
#define ICM_SPI_CS_LOW()     HAL_GPIO_WritePin(ICM_PORT_SPIX_CS, ICM_PIN_SPIX_CS, GPIO_PIN_RESET)
#define ICM_SPI_CS_HIGH()    HAL_GPIO_WritePin(ICM_PORT_SPIX_CS, ICM_PIN_SPIX_CS, GPIO_PIN_SET)
#if 0
static void Icm_Spi_ReadWriteNbytes(uint8_t* pBuffer, uint8_t len)
{
    uint8_t i = 0;
	uint8_t txdata[10]={0};
	uint8_t rxdata[10]={0};

#if defined(ICM_USE_HARD_SPI)
    for(i = 0; i < len; i ++)
    {
        txdata[0] = *pBuffer;
        HAL_SPI_TransmitReceive(getSpi4Handler(),txdata,rxdata,1,0xffff);
        *pBuffer = rxdata[0];
        pBuffer++;
    }
#endif
}
#endif
#endif

static uint8_t icm42688_read_reg(uint8_t reg)
{
    uint8_t regval = 0;
#if defined(ICM_USE_HARD_SPI)
    ICM_SPI_CS_LOW();
    reg |= 0x80;
    HAL_SPI_Transmit(getSpi4Handler(), &reg, 1, 0xffff);
    HAL_SPI_Receive(getSpi4Handler(), &regval, 1, 0xffff);
    ICM_SPI_CS_HIGH();
#elif defined(ICM_USE_HARD_I2C)

#endif

    return regval;
}

static void icm42688_read_regs(uint8_t reg, uint8_t* buf, uint16_t len)
{
#if defined(ICM_USE_HARD_SPI)
    reg |= 0x80;
    ICM_SPI_CS_LOW();
    if(HAL_SPI_Transmit(getSpi4Handler(), &reg, 1, 0xffff)!=HAL_OK)
    {
        printf("Error %s%d\r\n",__FUNCTION__,__LINE__);
    }
    if(HAL_SPI_Receive(getSpi4Handler(), buf, len, 0xffff) != HAL_OK)
    {
        printf("Error %s%d\r\n",__FUNCTION__,__LINE__);
    }
    ICM_SPI_CS_HIGH();
#elif defined(ICM_USE_HARD_I2C)
#endif
}

static uint8_t icm42688_write_reg(uint8_t reg, uint8_t value)
{
#if defined(ICM_USE_HARD_SPI)
    ICM_SPI_CS_LOW();
    if(HAL_SPI_Transmit(getSpi4Handler(), &reg, 1, 0xffff) != HAL_OK)
    {
        printf("Error %s%d\r\n",__FUNCTION__,__LINE__);
    }
    if(HAL_SPI_Transmit(getSpi4Handler(), &value, 1, 0xffff) != HAL_OK)
    {
        printf("Error %s%d\r\n",__FUNCTION__,__LINE__);
    }
    ICM_SPI_CS_HIGH();
#elif defined(ICM_USE_HARD_I2C)
#endif
    return 0;
}

float bsp_Icm42688GetAres(uint8_t Ascale)
{
    switch(Ascale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (11), 4 Gs (10), 8 Gs (01), and 16 Gs  (00).
    case AFS_2G:
        accSensitivity = 2000 / 32768.0f;
        break;
    case AFS_4G:
        accSensitivity = 4000 / 32768.0f;
        break;
    case AFS_8G:
        accSensitivity = 8000 / 32768.0f;
        break;
    case AFS_16G:
        accSensitivity = 16000 / 32768.0f;
        break;
    }

    return accSensitivity;
}

float bsp_Icm42688GetGres(uint8_t Gscale)
{
    switch(Gscale)
    {
    case GFS_15_125DPS:
        gyroSensitivity = 15.125f / 32768.0f;
        break;
    case GFS_31_25DPS:
        gyroSensitivity = 31.25f / 32768.0f;
        break;
    case GFS_62_5DPS:
        gyroSensitivity = 62.5f / 32768.0f;
        break;
    case GFS_125DPS:
        gyroSensitivity = 125.0f / 32768.0f;
        break;
    case GFS_250DPS:
        gyroSensitivity = 250.0f / 32768.0f;
        break;
    case GFS_500DPS:
        gyroSensitivity = 500.0f / 32768.0f;
        break;
    case GFS_1000DPS:
        gyroSensitivity = 1000.0f / 32768.0f;
        break;
    case GFS_2000DPS:
        gyroSensitivity = 2000.0f / 32768.0f;
        break;
    }
    return gyroSensitivity;
}

int8_t bsp_Icm42688RegCfg(void)
{
    uint8_t reg_val = 0;

    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0); 
    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x01);
    ICM42688DelayMs(100);
    reg_val = icm42688_read_reg(ICM42688_WHO_AM_I);

    if(reg_val == ICM42688_ID)
    {
        icm42688_write_reg(ICM42688_REG_BANK_SEL, 1); 
        icm42688_write_reg(ICM42688_INTF_CONFIG4, 0x02);

        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0);
        icm42688_write_reg(ICM42688_FIFO_CONFIG, 0x40);


        reg_val = icm42688_read_reg(ICM42688_INT_SOURCE0);
        icm42688_write_reg(ICM42688_INT_SOURCE0, 0x00);
        icm42688_write_reg(ICM42688_FIFO_CONFIG2, 0x00);
        icm42688_write_reg(ICM42688_FIFO_CONFIG3, 0x02);
        icm42688_write_reg(ICM42688_INT_SOURCE0, reg_val);
        icm42688_write_reg(ICM42688_FIFO_CONFIG1, 0x63); // Enable the accel and gyro to the FIFO

        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        icm42688_write_reg(ICM42688_INT_CONFIG, 0x36);

        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        reg_val = icm42688_read_reg(ICM42688_INT_SOURCE0);
        reg_val |= (1 << 2); //FIFO_THS_INT1_ENABLE
        icm42688_write_reg(ICM42688_INT_SOURCE0, reg_val);

        bsp_Icm42688GetAres(AFS_8G);
        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        reg_val = icm42688_read_reg(ICM42688_ACCEL_CONFIG0);//page74
        reg_val |= (AFS_8G << 5);
        reg_val |= (AODR_50Hz);
        icm42688_write_reg(ICM42688_ACCEL_CONFIG0, reg_val);

        bsp_Icm42688GetGres(GFS_1000DPS);
        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        reg_val = icm42688_read_reg(ICM42688_GYRO_CONFIG0);//page73
        reg_val |= (GFS_1000DPS << 5);
        reg_val |= (GODR_50Hz);
        icm42688_write_reg(ICM42688_GYRO_CONFIG0, reg_val);

        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        reg_val = icm42688_read_reg(ICM42688_PWR_MGMT0); //ֵ(page72)
        reg_val &= ~(1 << 5);//
        reg_val |= ((3) << 2);//
        reg_val |= (3);//
        icm42688_write_reg(ICM42688_PWR_MGMT0, reg_val);
        ICM42688DelayMs(1); //

        return 0;
    }
    return -1;
}

int bsp_Icm42688Init(void)
{
    return(bsp_Icm42688RegCfg());
}
INIT_EXPORT(bsp_Icm42688Init, "1");
int8_t bsp_IcmGetTemperature(int16_t* pTemp)
{
    uint8_t buffer[2] = {0};

    icm42688_read_regs(ICM42688_TEMP_DATA1, buffer, 2);

    *pTemp = (int16_t)(((int16_t)((buffer[0] << 8) | buffer[1])) / 132.48 + 25);
    return 0;
}
int8_t bsp_IcmGetAccelerometer(icm42688RawData_t* accData)
{
    uint8_t buffer[6] = {0};

    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 6);

    accData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
    accData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
    accData->z = ((uint16_t)buffer[4] << 8) | buffer[5];

    accData->x = (int16_t)(accData->x * accSensitivity);
    accData->y = (int16_t)(accData->y * accSensitivity);
    accData->z = (int16_t)(accData->z * accSensitivity);

    return 0;
}

int8_t bsp_IcmGetGyroscope(icm42688RawData_t* GyroData)
{
    uint8_t buffer[6] = {0};

    icm42688_read_regs(ICM42688_GYRO_DATA_X1, buffer, 6);

    GyroData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
    GyroData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
    GyroData->z = ((uint16_t)buffer[4] << 8) | buffer[5];

    GyroData->x = (int16_t)(GyroData->x * gyroSensitivity);
    GyroData->y = (int16_t)(GyroData->y * gyroSensitivity);
    GyroData->z = (int16_t)(GyroData->z * gyroSensitivity);
    return 0;
}

int8_t bsp_IcmGetRawData(icm42688RawData_t* accData, icm42688RawData_t* GyroData)
{
    uint8_t buffer[12] = {0};

    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 12);

    accData->x  = ((uint16_t)buffer[0] << 8)  | buffer[1];
    accData->y  = ((uint16_t)buffer[2] << 8)  | buffer[3];
    accData->z  = ((uint16_t)buffer[4] << 8)  | buffer[5];
    GyroData->x = ((uint16_t)buffer[6] << 8)  | buffer[7];
    GyroData->y = ((uint16_t)buffer[8] << 8)  | buffer[9];
    GyroData->z = ((uint16_t)buffer[10] << 8) | buffer[11];


    accData->x = (int16_t)(accData->x * accSensitivity);
    accData->y = (int16_t)(accData->y * accSensitivity);
    accData->z = (int16_t)(accData->z * accSensitivity);

    GyroData->x = (int16_t)(GyroData->x * gyroSensitivity);
    GyroData->y = (int16_t)(GyroData->y * gyroSensitivity);
    GyroData->z = (int16_t)(GyroData->z * gyroSensitivity);

    return 0;
}

uint8_t bsp_WhoAmi()
{
    return icm42688_read_reg(ICM42688_WHO_AM_I);
}