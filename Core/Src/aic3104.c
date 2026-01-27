#include "aic3104.h"
#include "string.h"

/* ============================================================
 * Register Write/Read Functions
 * ============================================================ */

/**
 * @brief  Write a single register to AIC3104
 * @param  config: Pointer to AIC3104 configuration structure
 * @param  reg: Register address
 * @param  val: Register value
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef AIC3104_WriteReg(AIC3104_Config_t *config, uint8_t reg, uint8_t val)
{
    if (!config || !config->hi2c) {
        return HAL_ERROR;
    }
    
    uint8_t i2c_addr = (AIC3104_I2C_ADDR << 1); /* 7-bit to 8-bit (with R/W bit) */
    return HAL_I2C_Mem_Write(config->hi2c, i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

/**
 * @brief  Read a single register from AIC3104
 * @param  config: Pointer to AIC3104 configuration structure
 * @param  reg: Register address
 * @param  val: Pointer to store register value
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef AIC3104_ReadReg(AIC3104_Config_t *config, uint8_t reg, uint8_t *val)
{
    if (!config || !config->hi2c || !val) {
        return HAL_ERROR;
    }
    
    uint8_t i2c_addr = (AIC3104_I2C_ADDR << 1);
    return HAL_I2C_Mem_Read(config->hi2c, i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, val, 1, 100);
}

/* ============================================================
 * Hardware Reset Function
 * ============================================================ */

/**
 * @brief  Perform hardware reset on AIC3104
 * @param  config: Pointer to AIC3104 configuration structure
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef AIC3104_HardwareReset(AIC3104_Config_t *config)
{
    if (!config || !config->reset_port) {
        return HAL_ERROR;
    }
    
    /* Pull RESET pin low (active low) */
    HAL_GPIO_WritePin(config->reset_port, config->reset_pin, GPIO_PIN_RESET);
    HAL_Delay(1); /* Wait at least 10ns, 1ms is safe */
    
    /* Pull RESET pin high (release reset) */
    HAL_GPIO_WritePin(config->reset_port, config->reset_pin, GPIO_PIN_SET);
    HAL_Delay(10); /* Wait for device to stabilize */
    
    return HAL_OK;
}

/* ============================================================
 * PLL Configuration (for 48kHz @ 12.288MHz MCLK)
 * ============================================================ */

/**
 * @brief  Configure PLL for given MCLK and sample rate
 * @note   Currently optimized for 12.288MHz MCLK and 48kHz sample rate
 *         Adjust register values based on your actual MCLK frequency
 * @param  config: Pointer to AIC3104 configuration structure
 * @retval HAL_StatusTypeDef
 */
static HAL_StatusTypeDef AIC3104_ConfigurePLL(AIC3104_Config_t *config)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    /* 
     * PLL Configuration for 48kHz
     * Assuming MCLK = 12.288MHz
     * Target: PLL output = 12.288MHz (to feed codec clock)
     * 
     * PLL_OUT = MCLK * (J.D) / (P * R)
     * For simplicity: P=1, J=1, D=0, R=1 -> PLL_OUT = MCLK
     */
    
    /* Page 0, Reg 3: PLL A = 0 (MCLK as input) */
    status |= AIC3104_WriteReg(config, 0x03, 0x00);
    
    /* Page 0, Reg 4: PLL Enable (Bit 7), MCLK as source (Bits 1-0 = 00) */
    status |= AIC3104_WriteReg(config, 0x04, 0x80);
    
    /* Page 0, Reg 5: PLL P and J */
    /* P=1, J=1: 0x11 */
    status |= AIC3104_WriteReg(config, 0x05, 0x11);
    
    /* Page 0, Reg 6: PLL D (MSB), default 0x00 */
    status |= AIC3104_WriteReg(config, 0x06, 0x00);
    
    /* Page 0, Reg 7: PLL D (LSB), default 0x00 */
    status |= AIC3104_WriteReg(config, 0x07, 0x00);
    
    /* Page 0, Reg 8: PLL R */
    /* R=1: 0x01 */
    status |= AIC3104_WriteReg(config, 0x08, 0x01);
    
    return status;
}

/* ============================================================
 * Audio Interface Configuration (I2S)
 * ============================================================ */

/**
 * @brief  Configure AIC3104 audio interface (I2S)
 * @param  config: Pointer to AIC3104 configuration structure
 * @retval HAL_StatusTypeDef
 */
static HAL_StatusTypeDef AIC3104_ConfigureAudioInterface(AIC3104_Config_t *config)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    /* Page 0, Reg 9: Audio Interface Format */
    /* 
     * Bit 7-6: BCLK direction and WCLK direction
     * For STM32 Master mode:
     *   - BCLK output (Bit 6 = 1)
     *   - WCLK output (Bit 7 = 1)
     * 00: I2S mode, 00: 16-bit
     */
    if (config->i2s_mode == AIC3104_MODE_MASTER) {
        status |= AIC3104_WriteReg(config, 0x09, 0xC0); /* BCLK/WCLK output */
    } else {
        status |= AIC3104_WriteReg(config, 0x09, 0x00); /* BCLK/WCLK input */
    }
    
    /* Page 0, Reg 10: Audio Interface (continued) */
    /* 00: I2S mode, 00: 16-bit */
    status |= AIC3104_WriteReg(config, 0x0A, 0x00);
    
    /* Page 0, Reg 11: Oversampling Ratio */
    /* Assuming 48kHz, use 256x oversampling for standard quality */
    /* 11: 256x */
    status |= AIC3104_WriteReg(config, 0x0B, 0x80);
    
    return status;
}

/* ============================================================
 * Power and Path Configuration
 * ============================================================ */

/**
 * @brief  Configure AIC3104 DAC/ADC path and power
 * @param  config: Pointer to AIC3104 configuration structure
 * @retval HAL_StatusTypeDef
 */
static HAL_StatusTypeDef AIC3104_ConfigurePowerAndPath(AIC3104_Config_t *config)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    /* Delay to ensure PLL/clock stable */
    HAL_Delay(5);
    
    /* Page 0, Reg 12: High Pass Filter */
    /* Disable HPF by default */
    status |= AIC3104_WriteReg(config, 0x0C, 0x00);
    
    /* Page 0, Reg 13: DAC Power Control */
    /* Enable DAC channels when needed */
    status |= AIC3104_WriteReg(config, 0x0D, 0x00);
    
    /* Page 0, Reg 37: DAC Output Switch Control */
    /* Enable Left/Right DAC: Bits 7, 6 = 1 */
    status |= AIC3104_WriteReg(config, 0x25, 0xC0);
    
    /* Page 0, Reg 38: DAC Output Control */
    /* Route DAC to outputs */
    status |= AIC3104_WriteReg(config, 0x26, 0x00);
    
    /* Page 0, Reg 41: Left DAC to HPLOUT Routing */
    /* Route Left DAC to HPLOUT: Bit 3 = 1 */
    status |= AIC3104_WriteReg(config, 0x29, 0x08);
    
    /* Page 0, Reg 42: Right DAC to HPROUT Routing */
    /* Route Right DAC to HPROUT: Bit 5 = 1 */
    status |= AIC3104_WriteReg(config, 0x2A, 0x20);
    
    /* Page 0, Reg 43: Left DAC Digital Unmute/Mute */
    /* Unmute: Bit 3 = 0 (0x00 = unmute), default muted */
    status |= AIC3104_WriteReg(config, 0x2B, 0x00);
    
    /* Page 0, Reg 44: Right DAC Digital Unmute/Mute */
    status |= AIC3104_WriteReg(config, 0x2C, 0x00);
    
    /* Page 0, Reg 51: HPLOUT Power Up/Down and Unmute */
    /* Power up: Bit 7 = 1, Unmute: Bit 3 = 0 */
    status |= AIC3104_WriteReg(config, 0x33, 0x81);
    
    /* Page 0, Reg 65: HPROUT Power Up/Down and Unmute */
    /* Power up: Bit 7 = 1, Unmute: Bit 3 = 0 */
    status |= AIC3104_WriteReg(config, 0x41, 0x81);
    
    return status;
}

/* ============================================================
 * Main Initialization Function
 * ============================================================ */

/**
 * @brief  Initialize AIC3104 codec
 * @param  config: Pointer to AIC3104 configuration structure
 *         Must contain valid handles and pin information
 * @retval HAL_StatusTypeDef
 * 
 * @note   Sequence:
 *         1. Hardware reset via GPIO
 *         2. Configure I2C communication
 *         3. Configure PLL
 *         4. Configure audio interface (I2S)
 *         5. Configure power and signal routing
 */
HAL_StatusTypeDef AIC3104_Init(AIC3104_Config_t *config)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    if (!config) {
        return HAL_ERROR;
    }
    
    /* Step 1: Hardware Reset */
    status = AIC3104_HardwareReset(config);
    if (status != HAL_OK) {
        return status;
    }
    
    /* Step 2: Soft Reset via I2C */
    /* Page 0, Reg 1: Software Reset */
    status = AIC3104_WriteReg(config, 0x01, 0x80);
    if (status != HAL_OK) {
        return status;
    }
    
    HAL_Delay(5); /* Wait for soft reset to complete */
    
    /* Step 3: Configure PLL */
    status = AIC3104_ConfigurePLL(config);
    if (status != HAL_OK) {
        return status;
    }
    
    /* Step 4: Configure Audio Interface (I2S) */
    status = AIC3104_ConfigureAudioInterface(config);
    if (status != HAL_OK) {
        return status;
    }
    
    /* Step 5: Configure Power and Signal Routing */
    status = AIC3104_ConfigurePowerAndPath(config);
    if (status != HAL_OK) {
        return status;
    }
    
    return HAL_OK;
}

/* ============================================================
 * Utility Functions
 * ============================================================ */

/**
 * @brief  Set DAC mute control
 * @param  config: Pointer to AIC3104 configuration structure
 * @param  mute: 1 to mute, 0 to unmute
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef AIC3104_SetMute(AIC3104_Config_t *config, uint8_t mute)
{
    if (!config) {
        return HAL_ERROR;
    }
    
    uint8_t val = mute ? 0x08 : 0x00; /* Bit 3: mute control */
    HAL_StatusTypeDef status = HAL_OK;
    
    /* Left DAC mute (Reg 43) */
    status |= AIC3104_WriteReg(config, 0x2B, val);
    
    /* Right DAC mute (Reg 44) */
    status |= AIC3104_WriteReg(config, 0x2C, val);
    
    return status;
}

/**
 * @brief  Set DAC output volume
 * @param  config: Pointer to AIC3104 configuration structure
 * @param  volume_left: Left channel volume (0-0xFF, 0=mute)
 * @param  volume_right: Right channel volume (0-0xFF, 0=mute)
 * @retval HAL_StatusTypeDef
 * 
 * @note   Volume scale: 0x00 = -63dB, 0xC0 = 0dB, 0xFF = +24dB (typical)
 */
HAL_StatusTypeDef AIC3104_SetVolume(AIC3104_Config_t *config, uint8_t volume_left, uint8_t volume_right)
{
    if (!config) {
        return HAL_ERROR;
    }
    
    HAL_StatusTypeDef status = HAL_OK;
    
    /* Left DAC Volume (Reg 45) */
    status |= AIC3104_WriteReg(config, 0x2D, volume_left);
    
    /* Right DAC Volume (Reg 46) */
    status |= AIC3104_WriteReg(config, 0x2E, volume_right);
    
    return status;
}
