#include <stdint.h>

#define INA228_16BREG	2
#define INA228_24BREG	3
#define INA228_40BREG	5

/* INA228s the driver tracks at once, each by I2C address with its own
 * CURRENT_LSB and ADCRANGE */
#define INA228_MAX_DEVICES	4

typedef enum {
	CONFIG = 0,
	ADC_CONFIG,
	SHUNT_CAL,
	SHUNT_TEMPCO,
	VSHUNT,
	VBUS,
	DIETEMP,
	CURRENT,
	POWER,
	ENERGY,
	CHARGE,
	DIAG_ALRT,
	SOVL,
	SUVL,
	BOVL,
	BUVL,
	TEMP_LIMIT,
	PWR_LIMIT,
	MANUFACTURER_ID = 0x3e,
	DEVICE_ID = 0x3f
} INA228_REGS;

int32_t INA228_Init  (uint8_t *dev_addr, uint8_t dev_count);
int32_t INA228_Deinit(uint8_t *dev_addr, uint8_t dev_count);
int32_t INA228_SetCurrentLSB(uint8_t dev_addr, uint32_t current_lsb);
int32_t INA228_Write (uint8_t  dev_addr, uint8_t reg_addr, uint16_t  reg_data);
int32_t INA228_Read16(uint8_t  dev_addr, uint8_t reg_addr, uint16_t *reg_data);
int32_t INA228_Read32(uint8_t  dev_addr, uint8_t reg_addr, uint32_t *reg_data);
int32_t INA228_Read64(uint8_t  dev_addr, uint8_t reg_addr, uint64_t *reg_data);
int32_t INA228_WriteCONFIG(uint8_t  dev_addr, uint16_t *reg_data);
int32_t INA228_WriteSHUNT_CAL(uint8_t dev_addr, uint16_t shunt_cal);
int32_t INA228_ReadVSHUNT (uint8_t dev_addr, int32_t *reg_data);
int32_t INA228_ReadVBUS   (uint8_t dev_addr, uint32_t *reg_data);
int32_t INA228_ReadDIETEMP(uint8_t dev_addr, int32_t *reg_data);
int32_t INA228_ReadCURRENT(uint8_t dev_addr, int32_t *reg_data);
int32_t INA228_ReadPOWER  (uint8_t dev_addr, uint64_t *reg_data);
int32_t INA228_ReadENERGY (uint8_t dev_addr, uint64_t *reg_data);
int32_t INA228_ReadCHARGE (uint8_t dev_addr, int64_t *reg_data);
int32_t INA228_ClearAccumulators(uint8_t dev_addr);