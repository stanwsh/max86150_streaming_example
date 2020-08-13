
// REGISTER MAP ADDRESS

// Status Registers
static const uint8_t MAX86150_INTSTAT1 = 0x00;
static const uint8_t MAX86150_INTSTAT2 = 0x01;
static const uint8_t MAX86150_INTENABLE1 = 0x02;
static const uint8_t MAX86150_INTENABLE2 = 0x03;
// FIFO Regsiters
static const uint8_t MAX86150_FIFOWRITEPTR = 0x04;
static const uint8_t MAX86150_FIFOOVERFLOW = 0x05;
static const uint8_t MAX86150_FIFOREADPTR = 0x06;
static const uint8_t MAX86150_FIFODATA = 0x07;
static const uint8_t MAX86150_FIFOCONFIG = 0x08;
// FIFO Data Control
static const uint8_t MAX86150_FIFOCONTROL1 = 0x09;
static const uint8_t MAX86150_FIFOCONTROL2 = 0x0A;
// System Control
static const uint8_t MAX86150_SYSCONTROL = 0x0D;
// PPG Configuration
static const uint8_t MAX86150_PPGCONFIG1 = 0x0E;
static const uint8_t MAX86150_PPGCONFIG2 = 0x0F;
static const uint8_t MAX86150_LED_PROX_AMP = 0x10;
// LED Pulse Amplitude
static const uint8_t MAX86150_LED1_PULSEAMP = 0x11;
static const uint8_t MAX86150_LED2_PULSEAMP = 0x12;
static const uint8_t MAX86150_LED_RANGE = 0x14;
static const uint8_t MAX86150_LED_PILOT_PA = 0x15;
// ECG Configuration
static const uint8_t MAX86150_ECG_CONFIG1 = 0x3C;
static const uint8_t MAX86150_ECG_CONFIG3 = 0x3E;
//  Part ID
static const uint8_t MAX86150_PARTID = 0xFF;

static const uint8_t MAX86150_PROXINTTHRESH = 0x10; // ?

// REGISTER DETAILS

// Interrupt Enable 1 (0x02)
// BIT:7 A_FULL:FIFO Almost Full Flag
static const uint8_t MAX86150_INT_A_FULL_MASK = (byte)~0b10000000; // i.e. 0b01111111
static const uint8_t MAX86150_INT_A_FULL_ENABLE = 0x80;
static const uint8_t MAX86150_INT_A_FULL_DISABLE = 0x00;
// BIT:6 PPG_RDY:New PPG FIFO Data Ready
static const uint8_t MAX86150_INT_DATA_RDY_MASK = (byte)~0b01000000;
static const uint8_t MAX86150_INT_DATA_RDY_ENABLE = 0x40;
static const uint8_t MAX86150_INT_DATA_RDY_DISABLE = 0x00;
// BIT:5 ALV_OVF:Ambient Light Cancellation Overflow
static const uint8_t MAX86150_INT_ALC_OVF_MASK = (byte)~0b00100000;
static const uint8_t MAX86150_INT_ALC_OVF_ENABLE = 0x20;
static const uint8_t MAX86150_INT_ALC_OVF_DISABLE = 0x00;
// BIT:4 PROX_INT:Proximity Interrupt
static const uint8_t MAX86150_INT_PROX_INT_MASK = (byte)~0b00010000;
static const uint8_t MAX86150_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX86150_INT_PROX_INT_DISABLE = 0x00;
// BIT:0 PWR_RDY:Power Ready Flag
// ?

// Interrupt Enable 2 (0x03)
// BIT:7 VDD_OOR_EN:VDD Out-of-range Indicator Enable
static const uint8_t MAX86150_VDD_OOR_EN_MASK = (byte)~0b10000000;
static const uint8_t MAX86150_VDD_OOR_EN_ENABLE = 0x80;
static const uint8_t MAX86150_VDD_OOR_EN_DISABLE = 0x00;
// BIT:2 ECG_RDY_EN:New ECG FIFO Data Ready Interrupt Enable
static const uint8_t MAX86150_ECG_RDY_EN_MASK = (byte)~0b00000100;
static const uint8_t MAX86150_ECG_RDY_EN_ENABLE = 0x04;
static const uint8_t MAX86150_ECG_RDY_EN_DISABLE = 0x00;

// FIFO Writer Pointer (0x04)
// BIT:[4,0] FIFO_WR_PTR:FIFO Write Pointer
// This points to the location where the next sample is written.
// This pointer advances for each sample pushed on to the FIFO.
// TODO

// Overflow Counter (0x05)
// BIT:[4,0] OVF_COUNTER:FIFO Overflow Counter
// When FIFO is full, any new samples result in new or old samples getting lost depending on
// FIFO_ROLLS_ON_FULL. OVF_COUNTER counts the number of samples lost. It saturates at 0x1F.

// FIFO Read Pointer (0x06)
// BIT:[4,0] FIFO_RD_PTR:FIFO Read Pointer
// The FIFO Read Pointer points to the location from where the processor gets the next sample 
// from the FIFO tyhrough the I2C interface. This advances each time a sample is popped from the FIFO.
// The processor can also write to this pointer after reading the samples. This allows rereading (or retrying) samples from the FIFO.

// FIFO Data Register (0x07)

// FIFO Configuration (0x08)
// BIT:4 FIFO_ROLL_ON_FULL:FIFO Rolls on Full Options
// control roll over if FIFO over flows
static const uint8_t MAX86150_ROLLOVER_MASK = (byte)~0b00010000;
static const uint8_t MAX86150_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX86150_ROLLOVER_DISABLE = 0x00;

// BIT:[3,0] FIFO_A_FULL FIFO Almost Full Value
// default:17 (0xF)
static const uint8_t MAX86150_A_FULL_MASK = (byte)~0b00001111; // FIFO almost full


// default FIFO Data Time slot setting
// 4-bit value. LED1[RED] LED2[IR]
static const uint8_t SLOT_NONE = 0x00;
static const uint8_t SLOT_LED1 = 0x01;
static const uint8_t SLOT_LED2 = 0x02;
static const uint8_t SLOT_LED1_PILOT = 0x09;
static const uint8_t SLOT_LED2_PILOT = 0x0A;
static const uint8_t SLOT_ECG = 0x0D;
// FIFO Data Control Register 1 (0x09)

// FIFO Data Control Register 2 (0x0A)

// System Control (0x0D)
// BIT:2 FIFO_EN:FIFO enable
static const uint8_t MAX86150_FIFOEN_MASK = (byte)~0b00000100;
static const uint8_t MAX86150_FIFOEN_ENABLE = 0x04;
static const uint8_t MAX86150_FIFOEN_DISABLE = 0x00;
// BIT:1 SHDN: Shutdown control
static const uint8_t MAX86150_SHUTDOWN_MASK = 0xFD; //FIXED
static const uint8_t MAX86150_SHUTDOWN = 0x02;
static const uint8_t MAX86150_WAKEUP = 0x00;
// BIT:0 RESET:Reset control
static const uint8_t MAX86150_RESET_MASK = 0xFE;
static const uint8_t MAX86150_RESET = 0x01;

// PPG Configuration 1 (0x0E)
// BIT:[7,6] PPG_ADC_RGE:SpO2 ADC Range Control
static const uint8_t MAX86150_PPGADCRANGE_MASK = 0x3F;
static const uint8_t MAX86150_PPGADCRANGE_4096 = 0x00;
static const uint8_t MAX86150_PPGADCRANGE_8192 = 0x40;
static const uint8_t MAX86150_PPGADCRANGE_16384 = 0x80;
static const uint8_t MAX86150_PPGADCRANGE_32768 = 0xC0;
// BIT:[5,2] PPG_SR:SpO2 Sample Rate Control
static const uint8_t MAX86150_PPGSR_MASK = 0x3C;
static const uint8_t MAX86150_PPGSR_10 = 0x00;
static const uint8_t MAX86150_PPGSR_20 = 0x04;
static const uint8_t MAX86150_PPGSR_50 = 0x08;
static const uint8_t MAX86150_PPGSR_84 = 0x0C;
static const uint8_t MAX86150_PPGSR_100 = 0x10;
static const uint8_t MAX86150_PPGSR_200 = 0x14;
static const uint8_t MAX86150_PPGSR_400 = 0x18;
static const uint8_t MAX86150_PPGSR_800 = 0x1C;
static const uint8_t MAX86150_PPGSR_1000 = 0x20;
static const uint8_t MAX86150_PPGSR_1600 = 0x24;
static const uint8_t MAX86150_PPGSR_3200 = 0x28;
static const uint8_t MAX86150_PPGSR_10N2 = 0x2C;
static const uint8_t MAX86150_PPGSR_20N2 = 0x30;
static const uint8_t MAX86150_PPGSR_50N2 = 0x34;
static const uint8_t MAX86150_PPGSR_84N2 = 0x38;
static const uint8_t MAX86150_PPGSR_100N2 = 0x3C;

// bit[2,0]?
static const uint8_t MAX86150_MODE_MASK = 0xF8; // LED mode (questioning,how to control the use of LED?)
static const uint8_t MAX86150_MODE_REDONLY = 0x02;
static const uint8_t MAX86150_MODE_REDIRONLY = 0x03;
static const uint8_t MAX86150_MODE_MULTILED = 0x07;

// BIT[1,0] PPG_LED_PW:LED Pulse Width Control
static const uint8_t MAX86150_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX86150_PULSEWIDTH_50 = 0x00;
static const uint8_t MAX86150_PULSEWIDTH_100 = 0x01;
static const uint8_t MAX86150_PULSEWIDTH_200 = 0x02;
static const uint8_t MAX86150_PULSEWIDTH_400 = 0x03;

static const uint8_t MAX86150_SLOT1_MASK = 0xF0;
static const uint8_t MAX86150_SLOT2_MASK = 0x0F;
static const uint8_t MAX86150_SLOT3_MASK = 0xF0;
static const uint8_t MAX86150_SLOT4_MASK = 0x0F;

// PPG Configuration 2 (0x0F)
// BIT:[2,0] SMP_AVE:Sample Averaging Options
static const uint8_t MAX86150_SAMPLEAVE_MASK = (byte)~0b11100000; // wrong ? ~0b00000111
static const uint8_t MAX86150_SAMPLEAVE_1 = 0x00;				  // no averaging
static const uint8_t MAX86150_SAMPLEAVE_2 = 0x20;
static const uint8_t MAX86150_SAMPLEAVE_4 = 0x40;
static const uint8_t MAX86150_SAMPLEAVE_8 = 0x60;
static const uint8_t MAX86150_SAMPLEAVE_16 = 0x80;
static const uint8_t MAX86150_SAMPLEAVE_32 = 0xA0;

// Prox Interrupt Threshold (0x10)

// LED1 Pulse Amplitude (0x10) [IR]

// LED2 Pulse Amplitude (0x10) [RED]

// LED Range (0x14)

// LED PILOT Pulse Amplitude (0x15)

// ECG Configuration 1 (0x3C)

// ECG Configuration 3 (0x3E)



static const uint8_t MAX_30105_EXPECTEDPARTID = 0x1E;
