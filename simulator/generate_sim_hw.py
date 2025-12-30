dma_pattern = r'''
// DMA definition: {name}
static inline uint8 {name}_DmaInitialize(
    uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) { return 0; }
static inline void {name}_DmaRelease() {}
#define {name}__TD_TERMOUT_EN 0
'''

filter_pattern = r'''
// Filter definition: {name}
static inline void {name}_Start(void) {}
static inline void {name}_Stop(void) {}
static inline uint8 {name}_Read8(uint8 channel) { return 0; }
static inline uint16 {name}_Read16(uint8 channel) { return 0; }
static inline uint32 {name}_Read24(uint8 channel) { return 0; }
static inline void {name}_Write8(uint8 channel, uint8 sample) {}
static inline void {name}_Write16(uint8 channel, uint16 sample) {}
static inline void {name}_Write24(uint8 channel, uint32 sample) {}
static inline void {name}_Sleep(void) {}
static inline void {name}_Wakeup(void) {}
static inline void {name}_SaveConfig(void) {}
static inline void {name}_RestoreConfig(void) {}
static inline void {name}_Init(void) {}
static inline void {name}_Enable(void) {}
static inline void {name}_SetCoherency(uint8 channel, uint8 byteSelect) {}
static inline void {name}_SetCoherencyEx(uint8 regSelect, uint8 key) {}
static inline void {name}_SetDalign(uint8 regSelect, uint8 state) {}

#define {name}_CHANNEL_A             (0u)
#define {name}_CHANNEL_B             (1u)
#define {name}_CHANNEL_A_INTR        (0x08u)
#define {name}_CHANNEL_B_INTR        (0x10u)
#define {name}_ALL_INTR              (0xf8u)
#define {name}_SIGN_BIT              ((uint32)0x00800000u)
#define {name}_SIGN_BYTE             ((uint32)0xFF000000u)
#define {name}_ENABLED               (0x01u)
#define {name}_DISABLED              (0x00u)
#define {name}_KEY_LOW               (0x00u)
#define {name}_KEY_MID               (0x01u)
#define {name}_KEY_HIGH              (0x02u)
'''

pwm_pattern = r'''
// PWM definition: {name}
static inline void    {name}_Start(void) {}
static inline void    {name}_Stop(void) {}
static inline void    {name}_WritePeriod(uint16 period) {}
static inline uint16 {name}_ReadPeriod(void) { return 0; }
static inline void    {name}_WriteCompare(uint16 compare) {}
static inline void    {name}_WriteCompare1(uint16 compare) {}
static inline void    {name}_WriteCompare2(uint16 compare) {}
static inline uint16 {name}_ReadCompare(void) { return 0; }
static inline void {name}_Init(void) {}
static inline void {name}_Enable(void) {}
static inline void {name}_Sleep(void) {}
static inline void {name}_Wakeup(void) {}
static inline void {name}_SaveConfig(void) {}
static inline void {name}_RestoreConfig(void) {}
#define {name}_COMPARE2_LSB            (0x00u)
#define {name}_COMPARE2_LSB_PTR        (0x00u)
'''

timer_pattern = r'''
static inline void    {name}_Start(void) {}
static inline void    {name}_Stop(void) {}
static inline void    {name}_SetInterruptMode(uint8 interruptMode) {}
static inline uint8   {name}_ReadStatusRegister(void) { return 0; }
static inline uint16  {name}_ReadPeriod(void) { return 0; }
static inline void    {name}_WritePeriod(uint16 period) {}
static inline uint16  {name}_ReadCounter(void) { return 0; }
static inline void    {name}_WriteCounter(uint16 counter) {}
'''

control_reg_pattern_h = r'''
void    {name}_control_write(uint8 control) ;
uint8   {name}_control_read(void) ;
'''
control_reg_pattern_c = r'''
void    {name}_control_write(uint8 control) { interrupter1_control_Control = control; }
uint8   {name}_control_read(void) { return interrupter1_control_Control; }
'''

with open('sim_hw_base/sim_hw.h') as f:
    header_content = f.read()
with open('sim_hw_base/sim_hw.c') as f:
    source_content = f.read()

def define_register(name: str, macro_name: str, bits: int, content_suffix: str = ''):
    global header_content, source_content
    if content_suffix != '': content_suffix = '_' + content_suffix
    header_content += f'''\
extern reg{bits} {name};
#define {macro_name}{content_suffix} ({name})
#define {macro_name}_PTR (&{name})
'''
    source_content += f'reg{bits} {name} = 0;\n'

def define_dma(name: str):
    global header_content
    header_content += dma_pattern.replace('{name}', name)

def define_filter(name: str):
    global header_content
    header_content += filter_pattern.replace('{name}', name)
    define_register(f'{name}_DFB__HOLDA', f'{name}_HOLDA', 8, 'REG')
    define_register(f'{name}_DFB__STAGEA', f'{name}_STAGEA', 8, 'REG')

def define_timer(name: str):
    global header_content
    header_content += timer_pattern.replace('{name}', name)
    define_register(f'{name}_TimerHW__CAP0', f'{name}_CAPTURE_LSB', 16)

def define_pwm(name: str):
    global header_content
    header_content += pwm_pattern.replace('{name}', name)
    define_register(f'{name}_PWMHW__CNT_CMP0', f'{name}_COMPARE1_LSB', 16)
    define_register(f'{name}_PWMHW__PER0', f'{name}_PERIOD_LSB', 16)
    define_register(f'{name}_PWMUDB_sP16_pwmdp_u0__16BIT_A0_REG', f'{name}_COUNTER_LSB_PTR', 16)

def define_control_register(name: str):
    global header_content, source_content
    define_register(f'{name}_Sync_ctrl_reg__CONTROL_REG', f'{name}_Control', 8)
    header_content += control_reg_pattern_h.replace('{name}', name)
    source_content += control_reg_pattern_c.replace('{name}', name)

define_dma('ADC_DMA')
define_dma('Ch1_DMA')
define_dma('Ch2_DMA')
define_dma('Ch3_DMA')
define_dma('Ch4_DMA')
define_dma('int1_dma')
define_dma('ram_to_filter_DMA')
define_dma('filter_to_fram_DMA')
define_dma('FBC_to_ram_DMA')
define_dma('PWMA_init_DMA')
define_dma('PWMB_init_DMA')
define_dma('QCW_CL_DMA')
define_dma('TR1_CL_DMA')
define_dma('fram_to_PWMA_DMA')
define_dma('PSBINIT_DMA')
define_dma('PWMB_PSB_DMA')

define_filter('FB_Filter')

define_pwm('PWMA')
define_pwm('PWMB')
define_pwm('interrupter1')

define_timer('FB_capture')

define_register('CT1_dac_viDAC8__D', 'CT1_dac_Data', 8, 'REG')

define_control_register('interrupter1_control')

with open('sim_hw.h', 'w') as f:
    f.write(f'''\
#ifndef SIM_HW_H
#define SIM_HW_H
{header_content}
#endif
''')

with open('sim_hw.c', 'w') as f:
    f.write(source_content)
