#ifndef SIM_HW_H
#define SIM_HW_H
#include "cytypes.h"
#include <stdint.h>
#include <string.h>

#define SIMULATOR
extern uint8_t ZCDref_Data;
extern uint8_t FB_THRSH_DAC_Data;

int16_t ADC_peak_GetResult16();
int32_t ADC_peak_CountsTo_mVolts(int32_t counts);
float ADC_peak_CountsTo_Volts(int32_t counts);


int32_t ADC_CountsTo_mVolts(int32_t counts);
float ADC_therm_CountsTo_Volts(int32_t counts);

void CT_MUX_Start(void);
void ADC_peak_Start(void);
void Sample_Hold_1_Start(void);
void Comp_1_Start(void);
void ADC_Start(void);


void CT_MUX_Select(uint8_t val);
void MUX_Only_VBus_Write(uint8_t val);
void Amux_Ctrl_Write(uint8_t control);


void Relay1_Write(uint8_t val);
void Relay2_Write(uint8_t val);
uint8_t Relay1_Read();
uint8_t Relay2_Read();

#define SG_Timer_Start()
uint32_t SG_Timer_ReadCounter();
uint32_t OnTimeCounter_ReadCounter();
void OnTimeCounter_WriteCounter(uint32_t val);

uint8_t no_fb_reg_Read();


static inline uint8 CyDmaTdAllocate(void) { return 0; }
static inline cystatus CyDmaTdSetConfiguration(uint8 tdHandle, uint16 transferCount, uint8 nextTd, uint8 configuration)
{ return CYRET_SUCCESS; }
static inline cystatus CyDmaTdSetAddress(uint8 tdHandle, uint16 source, uint16 destination) { return CYRET_SUCCESS; }
static inline cystatus CyDmaChSetInitialTd(uint8 chHandle, uint8 startTd) { return CYRET_SUCCESS; }
static inline cystatus CyDmaChEnable(uint8 chHandle, uint8 preserveTds) { return CYRET_SUCCESS; }
static inline cystatus CyDmaChDisable(uint8 chHandle) { return CYRET_SUCCESS; }

#define ADC_data_ready_StartEx(p1)
#define ADC_SAR_WRK0_PTR 0
#define Amux_Ctrl_Control_PTR 0
#define CyGlobalIntEnable

#define DDS32_1_sCTRLReg_ctrlreg__CONTROL_REG 0
#define DDS32_2_sCTRLReg_ctrlreg__CONTROL_REG 0


//#define CYDEV_PERIPH_BASE 0
//#define CYDEV_SRAM_BASE 0
//#define HI16(x) ((x>>16)&&0xFFFF)
#define CyGlobalIntDisable

#define ZCD_counter_Start()
#define FB_glitch_detect_Start()
#define ZCD_compA_Start()
#define ZCD_compB_Start()
#define CT1_comp_Start()
#define CT1_dac_Start()
#define ZCDref_Start()
#define ZCD_counter_WritePeriod(x)
#define ZCD_counter_WriteCompare(x)
//#define BCLK__BUS_CLK__MHZ 64
//#define BCLK__BUS_CLK__HZ 64000000
#define FB_glitch_detect_WritePeriod(x)
#define FB_glitch_detect_WriteCompare1(x)
#define FB_glitch_detect_WriteCompare2(x)
#define CyDelayUs(x)

#define FB_THRSH_DAC_Start()

#define temp_pwm_WriteCompare1(compare)
#define temp_pwm_WriteCompare2(compare)
#define temp_pwm_Start()

#define Disp_GREEN 0
#define Disp_RED 0
#define Disp_BLUE 0
#define Disp_WHITE 0
#define Disp_BLACK 0
#define Disp_OCEAN 0
#define Disp_ORANGE 0
#define Disp_CYAN 0
#define Disp_MAGENTA 0
#define Disp_YELLOW 0

#define Disp_MemClear(p1)
#define Disp_DrawRect(p1,p2,p3,p4,p5,p6)
#define Disp_DrawLine(p1,p2,p3,p4,p5)
#define Disp_Trigger(p1)
#define Disp_PrintString(p1,p2,p3,p4,p5)
#define Disp_Start()
#define Disp_Dim(val)

#define OnTimeCounter_Start(p1)

#define I2C_Start()

#define isr_midi_StartEx(p1)

#define IDAC_therm_Start()
#define IDAC_therm_SetValue(val)
#define ADC_therm_SetOffset(cnt)
#define ADC_therm_Start()
#define Therm_Mux_Start()
#define ADC_therm_StartConvert()
#define Bootloadable_Load()
#define EEPROM_1_UpdateTemperature()
#define CySoftwareReset()

void CyGetUniqueId(uint32_t * val);


extern uint8_t system_fault_Control;
extern uint8_t QCW_enable_Control;
extern uint8_t IVO_Control;

#define UVLO_Read() 1

void LED_com_Write(uint8_t val);
void LED_sysfault_Write(uint8_t val);
void LED3_Write(uint8_t val);
void LED4_Write(uint8_t val);

typedef uint8_t uint8;
typedef uint16_t uint16;


extern uint8_t IVO_UART_Control;


extern uint8_t DDS32_en[4];
void DDS32_1_Enable_ch(uint8_t ch);
void DDS32_2_Enable_ch(uint8_t ch);
extern uint32_t DDS32_freq[4];
void DDS32_1_Disable_ch(uint8_t ch);
void DDS32_2_Disable_ch(uint8_t ch);
uint32_t DDS32_1_SetFrequency_FP8(uint8_t ch,uint32_t freq);
uint32_t DDS32_2_SetFrequency_FP8(uint8_t ch,uint32_t freq);
extern uint32_t DDS32_noise[4];
void DDS32_1_WriteRand0(uint32_t rnd);
void DDS32_1_WriteRand1(uint32_t rnd);
void DDS32_2_WriteRand0(uint32_t rnd);
void DDS32_2_WriteRand1(uint32_t rnd);
#define DDS32_1_Start()
#define DDS32_2_Start()
#define Opamp_1_Start()

extern uint16_t ADC_therm_Offset;
void Therm_Mux_Select(uint8_t ch);
uint16_t ADC_therm_GetResult16();

void Relay3_Write(uint8_t val);
void Relay4_Write(uint8_t val);
#define dcdc_ena_Write(val)
#define digipot_clk_Write(val)
#define digipot_data_Write(val)
#define digipot_ncs_Write(val)
uint8_t Relay3_Read();
uint8_t Relay4_Read();

uint8_t VB0_Read();
uint8_t VB1_Read();
uint8_t VB2_Read();
uint8_t VB3_Read();
uint8_t VB4_Read();
uint8_t VB5_Read();


void Fan_Write(uint8_t val);
uint8_t Fan_Read();

uint8_t system_fault_Read();

//#define EEPROM_1_Start()
void EEPROM_1_Start();
uint8_t EEPROM_1_Write(const uint8 * rowData, uint8 rowNumber) ;
uint8_t EEPROM_1_ReadByte(uint16 address) ;


#define I2C_WRITE_XFER_MODE 0
#define I2C_READ_XFER_MODE 1
#define I2C_NAK_DATA 1
#define I2C_MODE_COMPLETE_XFER 1
#define I2C_MasterSendStart(p1,p2)
#define I2C_MasterClearStatus()
#define I2C_MasterWriteBuf(address,buffer,cnt,I2C_MODE_COMPLETE_XFER)
#define I2C_MasterStatus() 1
#define I2C_MSTAT_WR_CMPLT 1
#define I2C_MasterWriteByte(registerAddress)
#define I2C_MasterSendStop()
#define I2C_MasterSendRestart(address,I2C_READ_XFER_MODE)
#define I2C_MasterReadByte(I2C_NAK_DATA) 0

#define _putchar(character)

/***************************************
* API Constants
***************************************/

#define CY_DMA_INVALID_CHANNEL      0xFFu   /* Invalid Channel ID */
#define CY_DMA_INVALID_TD           0xFFu   /* Invalid TD */
#define CY_DMA_END_CHAIN_TD         0xFFu   /* End of chain TD */
#define CY_DMA_DISABLE_TD           0xFEu

#define CY_DMA_TD_SIZE              0x08u

/* "u" was removed as workaround for Keil compiler bug */
#define CY_DMA_TD_SWAP_EN           0x80
#define CY_DMA_TD_SWAP_SIZE4        0x40
#define CY_DMA_TD_AUTO_EXEC_NEXT    0x20
#define CY_DMA_TD_TERMIN_EN         0x10
#define CY_DMA_TD_TERMOUT1_EN       0x08
#define CY_DMA_TD_TERMOUT0_EN       0x04
#define CY_DMA_TD_INC_DST_ADR       0x02
#define CY_DMA_TD_INC_SRC_ADR       0x01

#define CY_DMA_NUMBEROF_TDS         128u
#define CY_DMA_NUMBEROF_CHANNELS    ((uint8)(CYDEV_DMA_CHANNELS_AVAILABLE))

/* Action register bits */
#define CY_DMA_CPU_REQ              ((uint8)(1u << 0u))
#define CY_DMA_CPU_TERM_TD          ((uint8)(1u << 1u))
#define CY_DMA_CPU_TERM_CHAIN       ((uint8)(1u << 2u))

/* Basic Status register bits */
#define CY_DMA_STATUS_CHAIN_ACTIVE  ((uint8)(1u << 0u))
#define CY_DMA_STATUS_TD_ACTIVE     ((uint8)(1u << 1u))

/* DMA controller register error bits */
#define CY_DMA_BUS_TIMEOUT          (1u << 1u)
#define CY_DMA_UNPOP_ACC            (1u << 2u)
#define CY_DMA_PERIPH_ERR           (1u << 3u)

/* Round robin bits */
#define CY_DMA_ROUND_ROBIN_ENABLE   ((uint8)(1u << 4u))

#define DMA_INVALID_CHANNEL         (CY_DMA_INVALID_CHANNEL)
#define DMA_INVALID_TD              (CY_DMA_INVALID_TD)
#define DMA_END_CHAIN_TD            (CY_DMA_END_CHAIN_TD)
#define DMAC_TD_SIZE                (CY_DMA_TD_SIZE)
#define TD_SWAP_EN                  (CY_DMA_TD_SWAP_EN)
#define TD_SWAP_SIZE4               (CY_DMA_TD_SWAP_SIZE4)
#define TD_AUTO_EXEC_NEXT           (CY_DMA_TD_AUTO_EXEC_NEXT)
#define TD_TERMIN_EN                (CY_DMA_TD_TERMIN_EN)
#define TD_TERMOUT1_EN              (CY_DMA_TD_TERMOUT1_EN)
#define TD_TERMOUT0_EN              (CY_DMA_TD_TERMOUT0_EN)
#define TD_INC_DST_ADR              (CY_DMA_TD_INC_DST_ADR)
#define TD_INC_SRC_ADR              (CY_DMA_TD_INC_SRC_ADR)
#define NUMBEROF_TDS                (CY_DMA_NUMBEROF_TDS)
#define NUMBEROF_CHANNELS           (CY_DMA_NUMBEROF_CHANNELS)
#define CPU_REQ                     (CY_DMA_CPU_REQ)
#define CPU_TERM_TD                 (CY_DMA_CPU_TERM_TD)
#define CPU_TERM_CHAIN              (CY_DMA_CPU_TERM_CHAIN)
#define STATUS_CHAIN_ACTIVE         (CY_DMA_STATUS_CHAIN_ACTIVE)
#define STATUS_TD_ACTIVE            (CY_DMA_STATUS_TD_ACTIVE)
#define DMAC_BUS_TIMEOUT            (CY_DMA_BUS_TIMEOUT)
#define DMAC_UNPOP_ACC              (CY_DMA_UNPOP_ACC)
#define DMAC_PERIPH_ERR             (CY_DMA_PERIPH_ERR)
#define ROUND_ROBIN_ENABLE          (CY_DMA_ROUND_ROBIN_ENABLE)
#define DMA_DISABLE_TD              (CY_DMA_DISABLE_TD)

#define DMAC_CFG                    (CY_DMA_CFG_PTR)
#define DMAC_ERR                    (CY_DMA_ERR_PTR)
#define DMAC_ERR_ADR                (CY_DMA_ERR_ADR_PTR)
#define DMAC_CH                     (CY_DMA_CH_STRUCT_PTR)
#define DMAC_CFGMEM                 (CY_DMA_CFGMEM_STRUCT_PTR)
#define DMAC_TDMEM                  (CY_DMA_TDMEM_STRUCT_PTR)

#define interrupterTimebase_CTRL_ENABLE 1
#define interrupterTimebase_Init()
#define interrupterIRQ_StartEx(i)
#define interrupterIRQ_Enable()
#define interrupterIRQ_Disable()
#define interrupterTimebase_WriteCounter(i)
#define interrupterTimebase_WriteCompare(i)
#define interrupterIRQ_ClearPending()
#define Opamp_2_Start()

void interrupterTimebase_WriteControlRegister(uint8_t value);
uint8_t interrupterTimebase_ReadControlRegister();
uint8_t interrupterTimebase_ReadStatusRegister();
void vTaskEnterCritical();
void vTaskExitCritical();

// DMA definition: ADC_DMA
static inline uint8 ADC_DMA_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void ADC_DMA_DmaRelease() {}
#define ADC_DMA__TD_TERMOUT_EN 0

// DMA definition: Ch1_DMA
static inline uint8 Ch1_DMA_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void Ch1_DMA_DmaRelease() {}
#define Ch1_DMA__TD_TERMOUT_EN 0

// DMA definition: Ch2_DMA
static inline uint8 Ch2_DMA_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void Ch2_DMA_DmaRelease() {}
#define Ch2_DMA__TD_TERMOUT_EN 0

// DMA definition: Ch3_DMA
static inline uint8 Ch3_DMA_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void Ch3_DMA_DmaRelease() {}
#define Ch3_DMA__TD_TERMOUT_EN 0

// DMA definition: Ch4_DMA
static inline uint8 Ch4_DMA_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void Ch4_DMA_DmaRelease() {}
#define Ch4_DMA__TD_TERMOUT_EN 0

// DMA definition: int1_dma
static inline uint8 int1_dma_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void int1_dma_DmaRelease() {}
#define int1_dma__TD_TERMOUT_EN 0

// DMA definition: ram_to_filter_DMA
static inline uint8 ram_to_filter_DMA_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void ram_to_filter_DMA_DmaRelease() {}
#define ram_to_filter_DMA__TD_TERMOUT_EN 0

// DMA definition: filter_to_fram_DMA
static inline uint8 filter_to_fram_DMA_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void filter_to_fram_DMA_DmaRelease() {}
#define filter_to_fram_DMA__TD_TERMOUT_EN 0

// DMA definition: FBC_to_ram_DMA
static inline uint8 FBC_to_ram_DMA_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void FBC_to_ram_DMA_DmaRelease() {}
#define FBC_to_ram_DMA__TD_TERMOUT_EN 0

// DMA definition: PWMA_init_DMA
static inline uint8 PWMA_init_DMA_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void PWMA_init_DMA_DmaRelease() {}
#define PWMA_init_DMA__TD_TERMOUT_EN 0

// DMA definition: PWMB_init_DMA
static inline uint8 PWMB_init_DMA_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void PWMB_init_DMA_DmaRelease() {}
#define PWMB_init_DMA__TD_TERMOUT_EN 0

// DMA definition: QCW_CL_DMA
static inline uint8 QCW_CL_DMA_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void QCW_CL_DMA_DmaRelease() {}
#define QCW_CL_DMA__TD_TERMOUT_EN 0

// DMA definition: TR1_CL_DMA
static inline uint8 TR1_CL_DMA_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void TR1_CL_DMA_DmaRelease() {}
#define TR1_CL_DMA__TD_TERMOUT_EN 0

// DMA definition: fram_to_PWMA_DMA
static inline uint8 fram_to_PWMA_DMA_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void fram_to_PWMA_DMA_DmaRelease() {}
#define fram_to_PWMA_DMA__TD_TERMOUT_EN 0

// DMA definition: PSBINIT_DMA
static inline uint8 PSBINIT_DMA_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void PSBINIT_DMA_DmaRelease() {}
#define PSBINIT_DMA__TD_TERMOUT_EN 0

// DMA definition: PWMB_PSB_DMA
static inline uint8 PWMB_PSB_DMA_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void PWMB_PSB_DMA_DmaRelease() {}
#define PWMB_PSB_DMA__TD_TERMOUT_EN 0

// DMA definition: MUX_DMA
static inline uint8 MUX_DMA_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void MUX_DMA_DmaRelease() {}
#define MUX_DMA__TD_TERMOUT_EN 0

// Filter definition: FB_Filter
static inline void FB_Filter_Start(void) {}
static inline void FB_Filter_Stop(void) {}
static inline uint8 FB_Filter_Read8(uint8 channel) { return 0; }
static inline uint16 FB_Filter_Read16(uint8 channel) { return 0; }
static inline uint32 FB_Filter_Read24(uint8 channel) { return 0; }
static inline void FB_Filter_Write8(uint8 channel, uint8 sample) {}
static inline void FB_Filter_Write16(uint8 channel, uint16 sample) {}
static inline void FB_Filter_Write24(uint8 channel, uint32 sample) {}
static inline void FB_Filter_Sleep(void) {}
static inline void FB_Filter_Wakeup(void) {}
static inline void FB_Filter_SaveConfig(void) {}
static inline void FB_Filter_RestoreConfig(void) {}
static inline void FB_Filter_Init(void) {}
static inline void FB_Filter_Enable(void) {}
static inline void FB_Filter_SetCoherency(uint8 channel, uint8 byteSelect) {}
static inline void FB_Filter_SetCoherencyEx(uint8 regSelect, uint8 key) {}
static inline void FB_Filter_SetDalign(uint8 regSelect, uint8 state) {}

#define FB_Filter_CHANNEL_A             (0u)
#define FB_Filter_CHANNEL_B             (1u)
#define FB_Filter_CHANNEL_A_INTR        (0x08u)
#define FB_Filter_CHANNEL_B_INTR        (0x10u)
#define FB_Filter_ALL_INTR              (0xf8u)
#define FB_Filter_SIGN_BIT              ((uint32)0x00800000u)
#define FB_Filter_SIGN_BYTE             ((uint32)0xFF000000u)
#define FB_Filter_ENABLED               (0x01u)
#define FB_Filter_DISABLED              (0x00u)
#define FB_Filter_KEY_LOW               (0x00u)
#define FB_Filter_KEY_MID               (0x01u)
#define FB_Filter_KEY_HIGH              (0x02u)
extern reg8 FB_Filter_DFB__HOLDA;
#define FB_Filter_HOLDA_REG (FB_Filter_DFB__HOLDA)
#define FB_Filter_HOLDA_PTR (&FB_Filter_DFB__HOLDA)
extern reg8 FB_Filter_DFB__STAGEA;
#define FB_Filter_STAGEA_REG (FB_Filter_DFB__STAGEA)
#define FB_Filter_STAGEA_PTR (&FB_Filter_DFB__STAGEA)

// PWM definition: PWMA
static inline void    PWMA_Start(void) {}
static inline void    PWMA_Stop(void) {}
static inline void    PWMA_WritePeriod(uint16 period) {}
static inline uint16 PWMA_ReadPeriod(void) { return 0; }
static inline void    PWMA_WriteCompare(uint16 compare) {}
static inline void    PWMA_WriteCompare1(uint16 compare) {}
static inline void    PWMA_WriteCompare2(uint16 compare) {}
static inline uint16 PWMA_ReadCompare(void) { return 0; }
static inline void PWMA_Init(void) {}
static inline void PWMA_Enable(void) {}
static inline void PWMA_Sleep(void) {}
static inline void PWMA_Wakeup(void) {}
static inline void PWMA_SaveConfig(void) {}
static inline void PWMA_RestoreConfig(void) {}
#define PWMA_COMPARE2_LSB            (0x00u)
#define PWMA_COMPARE2_LSB_PTR        (0x00u)
extern reg16 PWMA_PWMHW__CNT_CMP0;
#define PWMA_COMPARE1_LSB (PWMA_PWMHW__CNT_CMP0)
#define PWMA_COMPARE1_LSB_PTR (&PWMA_PWMHW__CNT_CMP0)
extern reg16 PWMA_PWMHW__PER0;
#define PWMA_PERIOD_LSB (PWMA_PWMHW__PER0)
#define PWMA_PERIOD_LSB_PTR (&PWMA_PWMHW__PER0)
extern reg16 PWMA_PWMUDB_sP16_pwmdp_u0__16BIT_A0_REG;
#define PWMA_COUNTER_LSB_PTR (PWMA_PWMUDB_sP16_pwmdp_u0__16BIT_A0_REG)
#define PWMA_COUNTER_LSB_PTR_PTR (&PWMA_PWMUDB_sP16_pwmdp_u0__16BIT_A0_REG)

// PWM definition: PWMB
static inline void    PWMB_Start(void) {}
static inline void    PWMB_Stop(void) {}
static inline void    PWMB_WritePeriod(uint16 period) {}
static inline uint16 PWMB_ReadPeriod(void) { return 0; }
static inline void    PWMB_WriteCompare(uint16 compare) {}
static inline void    PWMB_WriteCompare1(uint16 compare) {}
static inline void    PWMB_WriteCompare2(uint16 compare) {}
static inline uint16 PWMB_ReadCompare(void) { return 0; }
static inline void PWMB_Init(void) {}
static inline void PWMB_Enable(void) {}
static inline void PWMB_Sleep(void) {}
static inline void PWMB_Wakeup(void) {}
static inline void PWMB_SaveConfig(void) {}
static inline void PWMB_RestoreConfig(void) {}
#define PWMB_COMPARE2_LSB            (0x00u)
#define PWMB_COMPARE2_LSB_PTR        (0x00u)
extern reg16 PWMB_PWMHW__CNT_CMP0;
#define PWMB_COMPARE1_LSB (PWMB_PWMHW__CNT_CMP0)
#define PWMB_COMPARE1_LSB_PTR (&PWMB_PWMHW__CNT_CMP0)
extern reg16 PWMB_PWMHW__PER0;
#define PWMB_PERIOD_LSB (PWMB_PWMHW__PER0)
#define PWMB_PERIOD_LSB_PTR (&PWMB_PWMHW__PER0)
extern reg16 PWMB_PWMUDB_sP16_pwmdp_u0__16BIT_A0_REG;
#define PWMB_COUNTER_LSB_PTR (PWMB_PWMUDB_sP16_pwmdp_u0__16BIT_A0_REG)
#define PWMB_COUNTER_LSB_PTR_PTR (&PWMB_PWMUDB_sP16_pwmdp_u0__16BIT_A0_REG)

// PWM definition: interrupter1
static inline void    interrupter1_Start(void) {}
static inline void    interrupter1_Stop(void) {}
static inline void    interrupter1_WritePeriod(uint16 period) {}
static inline uint16 interrupter1_ReadPeriod(void) { return 0; }
static inline void    interrupter1_WriteCompare(uint16 compare) {}
static inline void    interrupter1_WriteCompare1(uint16 compare) {}
static inline void    interrupter1_WriteCompare2(uint16 compare) {}
static inline uint16 interrupter1_ReadCompare(void) { return 0; }
static inline void interrupter1_Init(void) {}
static inline void interrupter1_Enable(void) {}
static inline void interrupter1_Sleep(void) {}
static inline void interrupter1_Wakeup(void) {}
static inline void interrupter1_SaveConfig(void) {}
static inline void interrupter1_RestoreConfig(void) {}
#define interrupter1_COMPARE2_LSB            (0x00u)
#define interrupter1_COMPARE2_LSB_PTR        (0x00u)
extern reg16 interrupter1_PWMHW__CNT_CMP0;
#define interrupter1_COMPARE1_LSB (interrupter1_PWMHW__CNT_CMP0)
#define interrupter1_COMPARE1_LSB_PTR (&interrupter1_PWMHW__CNT_CMP0)
extern reg16 interrupter1_PWMHW__PER0;
#define interrupter1_PERIOD_LSB (interrupter1_PWMHW__PER0)
#define interrupter1_PERIOD_LSB_PTR (&interrupter1_PWMHW__PER0)
extern reg16 interrupter1_PWMUDB_sP16_pwmdp_u0__16BIT_A0_REG;
#define interrupter1_COUNTER_LSB_PTR (interrupter1_PWMUDB_sP16_pwmdp_u0__16BIT_A0_REG)
#define interrupter1_COUNTER_LSB_PTR_PTR (&interrupter1_PWMUDB_sP16_pwmdp_u0__16BIT_A0_REG)

static inline void    FB_capture_Start(void) {}
static inline void    FB_capture_Stop(void) {}
static inline void    FB_capture_SetInterruptMode(uint8 interruptMode) {}
static inline uint8   FB_capture_ReadStatusRegister(void) { return 0; }
static inline uint16  FB_capture_ReadPeriod(void) { return 0; }
static inline void    FB_capture_WritePeriod(uint16 period) {}
static inline uint16  FB_capture_ReadCounter(void) { return 0; }
static inline void    FB_capture_WriteCounter(uint16 counter) {}
extern reg16 FB_capture_TimerHW__CAP0;
#define FB_capture_CAPTURE_LSB (FB_capture_TimerHW__CAP0)
#define FB_capture_CAPTURE_LSB_PTR (&FB_capture_TimerHW__CAP0)
extern reg8 CT1_dac_viDAC8__D;
#define CT1_dac_Data_REG (CT1_dac_viDAC8__D)
#define CT1_dac_Data_PTR (&CT1_dac_viDAC8__D)
extern reg8 interrupter1_control_Sync_ctrl_reg__CONTROL_REG;
#define interrupter1_control_Control (interrupter1_control_Sync_ctrl_reg__CONTROL_REG)
#define interrupter1_control_Control_PTR (&interrupter1_control_Sync_ctrl_reg__CONTROL_REG)

void    interrupter1_control_control_write(uint8 control) ;
uint8   interrupter1_control_control_read(void) ;

#endif
