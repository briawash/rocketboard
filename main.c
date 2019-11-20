// PIN OUTS
//  GPS :               B0      (UART1)
//  ALTIMETER :         D4      (UART2)
//  IMU :               B1/B2   (I2C)
//  FM RECEIVER :       E4      (UART5)
//  FM TRANSMITTER :    E5      (UART5)

///pressure, temp
// cut sens.,int, fifo, i2c send1, rec1

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"

//Bit-banded addresses
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define MAX_CHARS 100
#define MAXRETRIES              5
#define USER_CTRL        0x6A
//mpu9250 definitions
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define FIFO_EN         0x23
#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define I2C_MST_CTRL     0x24
#define FIFO_EN          0x23
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define AK8963_ADDRESS   0x0C<<1
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value
#define MPU9250_ADDRESS 0x69<<1  // Device address when ADO = 1
#define ALT_READ         0xC1
#define ALT_WRITE        0xC0
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A

//DATA ARRAY (0 -> GPS start, 43 -> altimeter,
char old_data[MAX_CHARS];
char gps_data[MAX_CHARS];
char gps_data1[MAX_CHARS];
char gps_data2[MAX_CHARS];
char packet[MAX_CHARS];
uint16_t imu_data[10];
uint16_t alt_data[10];
float fTemp;
float cTemp;
int temp=0;

// only GPGGA output (time, gps coordinates)
char gps_set[] = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
char baud_set[] = "$PMTK251,115200*1F\r\n";
char freq_set[] = "$PMTK220,100*2F\r\n";
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};
enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};
enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};
uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR
float aRes= 4.0/32768.0, gRes=250.0/32768.0, mRes=10.0*4912.0/8190.0;
int accel_bias[3];
int magbias[3];
float accelBias[3];
int magCalibration[3];
float gyroBias[3];
float magOffset[3]; // x yz offsets for magnometer
float accelOffset[3]; // x yz offsets for acceleration
float gyroOffset[3]; // x yz offsets for gyro
float magdata[3]; // x yz offsets for magnometer
float acceldata[3]; // x yz offsets for acceleration
float gyrodata[3];
float altdata[5];
float altitude;
int tHeight;
uint32_t tempdata;
//Functions
void initHw();
void AltInit();
void waitMicrosecond(uint32_t);
void newLine();
void putcUart0(char c);
void putsUart0(char* str);
char getcUart0();
void getsUart0(char* s, uint8_t maxChars);
void putcUart1(char c);
void putsUart1(char* str);
char getcUart1();
void floatUart(float val);
void getsUart1(char *str);
void putcUart5(char c);
void putsUart5(char* str);
char getcUart5();
void getsUart5(char *str);
void readMagData();
void readAccelData();
void readTempData();
void readGyroData();
void imuRetrieval();
void MPUinit();
void read(uint8_t DeviceAddress, uint8_t Register, uint8_t Nbytes, uint16_t *val);
uint16_t I2C_Recv2(int8_t slave);
uint16_t I2C_Recv(int8_t slave);
uint32_t I2C_Send2(int8_t slave, uint8_t data1, uint8_t data2);
uint32_t I2C_Send1(int8_t slave, uint8_t data1);
uint32_t I2C_Send3(int8_t slave, uint8_t data1);
void AltRetrieval();
void imuEquations();
void calibrateMPU9250();
static void StartTransmit1( uint8_t RW);
static void StartTransmit( uint8_t RW);
void readByte1( uint8_t RAddress, uint8_t data);
void readByte( uint8_t RAddress, uint8_t data);
static uint8_t WriteDatas1(uint8_t data, uint8_t start, uint8_t run, uint8_t stop);
uint8_t WriteData1( uint8_t RAddress , uint8_t data);
static uint8_t WriteByte(uint8_t data, uint8_t start, uint8_t run, uint8_t stop);
uint8_t WriteData( uint8_t RAddress , uint8_t data);
void readBytes1( uint8_t RAddress, uint8_t Nbytes, uint8_t *val, int merge);
void readBytes( uint8_t RAddress, uint8_t Nbytes, uint8_t *val);
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
int main(void)
{
    initHw();

    //configure GPS module
    putsUart1(gps_set);
    putsUart1(freq_set);

    getsUart1(gps_data);
    getsUart1(gps_data);
    //clear screen
    //title
    MPUinit();
    AltInit();
    calibrateMPU9250();

    getsUart1(gps_data);
    //while(gps_data[3] != 'R')
        getsUart1(gps_data);


    TIMER1_CTL_R |= TIMER_CTL_TAEN;
    putsUart0("RocketBoard Test"); newLine();
    putsUart0("================"); newLine();
    while(1){
        GREEN_LED^=1;
          }

             //intialization();


}

//------------------------------------------------------------------------------
//  .-.-.   .-.-.   .-.-.   .-.-.   .-.-.   .-.-.   .-.-.   .-.-  .-.-.   .-.-
// / / \ \ / / \ \ / / \ \ / / \ \ / / \ \ / / \ \ / / \ \ / / \ / / \ \ / / \
// `-'   `-`-'   `-`-'   `-`-'   `-`-'   `-`-'   `-`-'   `-`-'  `-`-'   `-`-'
//------------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;
    GPIO_PORTF_DIR_R = 0x0A;  // bits 1 and 3 are outputs, other pins are inputs 1010
        GPIO_PORTF_DR2R_R = 0x0A; // set drive strength to 2mA (not needed since defaultconfiguration -- for clarity) 0011
        GPIO_PORTF_DEN_R = 0x1A;  // enable LEDs and pushbuttons 1010
        GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button

    //-----------------------------------------------------------------------------
    // USB Serial (UART0) Configuration
    //-----------------------------------------------------------------------------
    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                              // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module



    //-----------------------------------------------------------------------------
    // GPS Configuration
    //-----------------------------------------------------------------------------
    // Configure UART1 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1, leave other uarts in same status
    GPIO_PORTB_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTB_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB1_U1TX | GPIO_PCTL_PB0_U1RX;

    // Configure UART1 to 9600 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART1_CTL_R = 0;                                 // turn-off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART1_IBRD_R = 260;                              // r = 40 MHz / (Nx9.6kHz), set floor(r)=260, where N=16
    UART1_FBRD_R = 27;                               // round(fract(r)*64)=27
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module


    //-----------------------------------------------------------------------------
    // FM Transmitter/Receiver Configuration
    //-----------------------------------------------------------------------------
    // Configure UART5 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R5;    // turn-on UART0, leave other uarts in same status
    GPIO_PORTE_DEN_R |= 48;                     // default, added for clarity
    GPIO_PORTE_AFSEL_R |= 48;                   // default, added for clarity
    GPIO_PORTE_PCTL_R = GPIO_PCTL_PE5_U5TX | GPIO_PCTL_PE4_U5RX;

    // Configure UART5 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART5_CTL_R = 0;                                 // turn-off UART5 to allow safe programming
    UART5_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART5_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART5_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART5_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART5_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;    // enable TX, and module
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
            TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
            TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
            TIMER1_TAILR_R = 0x2625A0;                      // set load value to 40e6 for 1 Hz interrupt rate
            TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
            NVIC_EN0_R |= 1 << (INT_TIMER1A-16);
    // --------------------------------------------------------------------------
    // Configure the I2C for IMU (this is the simple settings for simple mode)
    //---------------------------------------------------------------------------
    // turn off I2C module
      SYSCTL_RCGCI2C_R |= 0x0001;           // activate I2C0
      SYSCTL_RCGCGPIO_R |= 0x0002;          // activate port B
      while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?

      GPIO_PORTB_AFSEL_R |= 0x0C;           // 3) enable alt funct on PB2,3
      GPIO_PORTB_ODR_R |= 0x08;             // 4) enable open drain on PB3 only
      GPIO_PORTB_DEN_R |= 0x0C;             // 5) enable digital I/O on PB2,3
                                            // 6) configure PB2,3 as I2C
      GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
      GPIO_PORTB_AMSEL_R &= ~0x0C;          // 7) disable analog functionality on PB2,3
      I2C0_MCR_R = I2C_MCR_MFE;      // 9) master function enable
      I2C0_MTPR_R = 19;              // 8) configure for 100 kbps clock

    // --------------------------------------------------------------------------
    // Configure the I2C for Altimeter (this is the simple settings for simple mode)
    //---------------------------------------------------------------------------
    // turn off I2C module
      SYSCTL_RCGCI2C_R |= 0x0002;           // activate I2C0
      SYSCTL_RCGCGPIO_R |= 0x0001;          // activate port B
      while((SYSCTL_PRGPIO_R&0x0001) == 0){};// ready?

    GPIO_PORTA_AFSEL_R |= 192;
    GPIO_PORTA_ODR_R |=0x80;
    GPIO_PORTA_DEN_R |= 192;

    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA7_I2C1SDA |GPIO_PCTL_PA6_I2C1SCL ;                         //turn on GPIO ports
    GPIO_PORTA_AMSEL_R &= ~192; // data is a 1 clock is a 0
    I2C1_MCR_R = I2C_MCR_MFE;
    I2C1_MTPR_R=19;


                // turn-on interrupt 37 (TIMER1A)

}
void TempFunc(float temp){
    int d=1,digit=1;

    while(temp<d){
        d=d*10;
        digit++;

    }

}
void TimerISR(){
    float x,y,z;
    int count=0,i=0;
    char command[8];
    putsUart0(" A");

    RED_LED^=1;
   TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring                      // set load value to 2e5 for 200 Hz interrupt rat

   gps_data1[3] = 0;
   while(gps_data1[3] != 'G')
       getsUart1(gps_data1);

   gps_data2[3] = 0;
   while(gps_data2[3] != 'R')
       getsUart1(gps_data2);


   putsUart5(gps_data2);
   putcUart5(',');
   putsUart5(gps_data1);
   putcUart5('\n');
   TIMER1_CTL_R |= TIMER_CTL_TAEN;
   TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}
void floatUart(float val){
       int temp;
       temp= val/100;
       putcUart5(temp+48);
       val=val- (temp*100);
       temp= val/10;
       putcUart5(temp+48);
       val=val- (temp*10);
       temp= val/1;
       putcUart5(temp+48);
       putcUart5('.');
       val=val*100;
      val=val -((int)(val/1000)*1000);
       temp= val/100;
       putcUart5(temp+48);
       val=val- (temp*100);
       temp= val/10;
       putcUart5(temp+48);
       val=val- (temp*10);
       temp= val/1;
       putcUart5(temp+48);
       putcUart5(',');
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");                      // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");                      // 6
    __asm("             CBZ  R1, WMS_DONE1");               // 5+1*3
    __asm("             NOP");                              // 5
    __asm("             NOP");                              // 5
    __asm("             B    WMS_LOOP1");                   // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");                      // 1
    __asm("             CBZ  R0, WMS_DONE0");               // 1
    __asm("             NOP");                              // 1
    __asm("             B    WMS_LOOP0");                   // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                                    // ---
                                                            // 40 clocks/us + error
}

void newLine() {
    putsUart0("\n\r");                              // Send newline
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}
void MPUinit(){
    uint16_t rawData[3];  // x/y/z gyro calibration data stored here
    I2C_Send2(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    waitMicrosecond(100000); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt
    // get stable time source
    I2C_Send2(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    I2C_Send2(MPU9250_ADDRESS, CONFIG, 0x03);
    I2C_Send2(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);
    I2C_Send1(MPU9250_ADDRESS,GYRO_CONFIG);
    uint8_t c = I2C_Recv2(MPU9250_ADDRESS); // get current GYRO_CONFIG register value
    waitMicrosecond(100);
    // c = c & ~0xE0; // Clear self-test bits [7:5]
      c = c & ~0x02; // Clear Fchoice bits [1:0]
      c = c & ~0x18; // Clear AFS bits [4:3]
      c = c | Gscale << 3;
      I2C_Send2(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
    // Set accelerometer full-scale range configuration
      I2C_Send1(MPU9250_ADDRESS,ACCEL_CONFIG);
      c = I2C_Recv2(MPU9250_ADDRESS); // get current ACCEL_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5]
      c = c & ~0x18;  // Clear AFS bits [4:3]
      c = c | Ascale << 3; // Set full scale range for the accelerometer
      I2C_Send2(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value
    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
      I2C_Send1(MPU9250_ADDRESS,ACCEL_CONFIG2);
      c = I2C_Recv2(MPU9250_ADDRESS);// get current ACCEL_CONFIG2 register value
      waitMicrosecond(100);
      c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
      c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
      I2C_Send2(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
      I2C_Send2(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
      I2C_Send2(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
      I2C_Send2(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    waitMicrosecond(100);
    I2C_Send2(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    waitMicrosecond(100);
    read(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values


      magOffset[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
      magOffset[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;

    waitMicrosecond(100);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    I2C_Send2(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
    waitMicrosecond(100);
}
uint32_t I2C_Send2(int8_t slave, uint8_t data1, uint8_t data2){
  while(I2C0_MCS_R&I2C_MCS_BUSY){};// wait for I2C ready
  I2C0_MSA_R = (slave<<1)&0xFE;    // MSA[7:1] is slave address
  I2C0_MSA_R &= ~0x01;             // MSA[0] is 0 for send
  I2C0_MDR_R = data1&0xFF;         // prepare first byte
  I2C0_MCS_R = (0
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                    //   & ~I2C_MCS_STOP    // no stop
                       | I2C_MCS_START    // generate start/restart
                       | I2C_MCS_RUN);    // master enable
  while(I2C0_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // check error bits
  if((I2C0_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
    I2C0_MCS_R = (0                // send stop if nonzero
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // stop
                     //  & ~I2C_MCS_START   // no start/restart
                     //  & ~I2C_MCS_RUN    // master disable
                        );
                                          // return error bits if nonzero
    return (I2C0_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
  }
  I2C0_MDR_R = data2&0xFF;         // prepare second byte
  I2C0_MCS_R = (0
                      // & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // generate stop
                      // & ~I2C_MCS_START   // no start/restart
                       | I2C_MCS_RUN);    // master enable
  while(I2C0_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // return error bits
  return (I2C0_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
}
uint32_t I2C_Send4(int8_t slave, uint8_t data1, uint8_t data2){
  while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for I2C ready
  I2C1_MSA_R = (slave<<1)&0xFE;    // MSA[7:1] is slave address
  I2C1_MSA_R &= ~0x01;             // MSA[0] is 0 for send
  I2C1_MDR_R = data1&0xFF;         // prepare first byte
  I2C1_MCS_R = (0
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                    //   & ~I2C_MCS_STOP    // no stop
                       | I2C_MCS_START    // generate start/restart
                       | I2C_MCS_RUN);    // master enable
  while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // check error bits
  if((I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
    I2C1_MCS_R = (0                // send stop if nonzero
                     //  & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // stop
                     //  & ~I2C_MCS_START   // no start/restart
                     //  & ~I2C_MCS_RUN    // master disable
                        );
                                          // return error bits if nonzero
    return (I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
  }
  I2C1_MDR_R = data2&0xFF;         // prepare second byte
  I2C1_MCS_R = (0
                      // & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // generate stop
                      // & ~I2C_MCS_START   // no start/restart
                       | I2C_MCS_RUN);    // master enable
  while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // return error bits
  return (I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
}
///
void AltInit(){

    char config[2] = {0};
        config[0] = 0x26;
        config[1] = 0xB8;
        I2C_Send4(ALT_WRITE , 0x26, 0xB8);
        // Select data configuration register(0x13)
        // Data ready event enabled for altitude, pressure, temperature(0x07)
        config[0] = 0x13;
        config[1] = 0x07;
        I2C_Send4(ALT_WRITE,0x13,0x07);
        // Select control register(0x26)
        // Active mode, OSR = 128, altimeter mode(0xB9)

}

void readAccelData()
{
    uint16_t rawData[6];
    // x/y/z accel register data stored here
    read(MPU9250_ADDRESS, 59, 6,&rawData[0]);  // Read the six raw data registers into data array
    acceldata[0]= ((rawData[0]<<8)+rawData[1])*16.0/37268;
     acceldata[1]=((rawData[2]<<8 )+rawData[3])*16.0/37268;
     acceldata[2]= ((rawData[4]<<8 )+rawData[5])*16.0/37268;
}

void readGyroData()
{
    uint16_t rawData[6];
  // x/y/z gyro register data stored here
  read(MPU9250_ADDRESS, 43, 3, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gyrodata[0]= ((rawData[0]<<8)+rawData[1])*16.0/37268;
   gyrodata[1]=((rawData[2]<<8 )+rawData[3])*16.0/37268;
   gyrodata[2]= ((rawData[4]<<8 )+rawData[5])*16.0/37268;
}

void readMagData()
{
  uint16_t rawData[6];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  // wait for magnetometer data ready bit to be set
  read(AK8963_ADDRESS, AK8963_XOUT_L,6, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
  magdata[0]= ((rawData[0]<<8)+rawData[1])*16.0/37268;
  magdata[1]=((rawData[2]<<8 )+rawData[3])*16.0/37268;
  magdata[2]= ((rawData[4]<<8 )+rawData[5])*16.0/37268;

}

void readTempData()
{
  uint16_t rawData[2];  // x/y/z gyro register data stored here
  read(MPU9250_ADDRESS, 41, 2, &rawData[2]);  // Read the two raw data registers sequentially into data array
  tempdata= ((rawData[0]<<8) + rawData[1])*16.0/37267;
 }
void getsUart0(char* s, uint8_t maxChars)
{
    uint8_t     count;                                      // Store the index current character
    char        c;                                          // Store the current character

    count = 0;                                              // Initialize the index to 0

    while(1)                                                // Loop
    {
        c = getcUart0();                                    // Wait for user pressed a character
        putcUart0(c);                                       // Put back the character to user

        if (c == 8)                                         // c is backspace
        {
            if (count > 0)                                  // Decrement count if it greater than 0
                count--;

        }
        else if (c == 13)                                   // c is return character
        {
            s[count] = '\0';                        // Assign null to last character
            putcUart0('\n');                                // Put back the character to user
            break;
        }
        else if(c >= ' ')                                   // c is displayable characters
        {
            if ((c >= 65) && (c <= 90))                     // If upper case character
                c+=32;                                      // Covert to lower case
            s[count++] = c;
        }

        if (count == maxChars)                              // If it is backspace
        {
            s[count] = '\0';                        // Assign null to last character
            break;
        }
    }
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart1(char c)
{
    while (UART1_FR_R & UART_FR_TXFF);
    UART1_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart1(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart1(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart1()
{
    while (UART1_FR_R & UART_FR_RXFE);
    return UART1_DR_R & 0xFF;
}

void getsUart1(char *str)
{
    uint8_t count;                                          // Store the index current character
    char c;                                                 // Store the current character

    count = 0;                                              // Initialize the index to 0

    while(1)                                                // Loop
    {
        c = getcUart1();                                    // Wait for user pressed a character

        if (c == '\r' || c == '\n')                         // c is return character
        {
            str[count++] = '\0';                            // Assign null to last character
            break;
        }

        else                                                // c is displayable characters
            str[count++] = c;
    }

    while (count < MAX_CHARS)                               // clear rest of string
        str[count++] = 0;
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart5(char c)
{
    while (UART5_FR_R & UART_FR_TXFF);
    UART5_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart5(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart5(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty

char getcUart5()
{
    while (UART5_FR_R & UART_FR_RXFE);
    return UART5_DR_R & 0xFF;
}

void getsUart5(char *str)
{
    uint8_t count;                                          // Store the index current character
    char c;                                                 // Store the current character

    count = 0;                                              // Initialize the index to 0

    while(1)                                                // Loop
    {
        c = getcUart5();                                    // Wait for user pressed a character

        if (c == '\r' || c == '\n')                         // c is return character
        {
            str[count++] = '\0';                            // Assign null to last character
            break;
        }

        else                                                // c is displayable characters
            str[count++] = c;
    }

    while (count < MAX_CHARS)                               // clear rest of string
        str[count++] = 0;
}

static void StartTransmit( uint8_t RW){
    I2C0_MSA_R = (MPU9250_ADDRESS << 1) +RW;
}

static uint8_t WriteByte(uint8_t data, uint8_t start, uint8_t run, uint8_t stop){
    int error;
    I2C0_MDR_R = data;
    if(start){
       while(I2C0_MCS_R & 0x40);                 // making sure line is not busy
    }

    I2C0_MCS_R = ( (run << 0) | (start << 1) | (stop << 2) );
    while(I2C0_MCS_R & 0x01);                   // bus is busy
    error = (I2C0_MCS_R & 0x02) >> 1;           // get the error flag
    return error;
}

uint8_t WriteData( uint8_t RAddress , uint8_t data){
    int error = 0;
    StartTransmit(0);        //  starting a write
    error = WriteByte(RAddress, 1, 1, 0);    //  setting the register start bit, run bit,turn off stop bit

    if(error > 0){
        error = 1;      //something went wrong
        return error;
    }

    error =WriteByte(data, 0, 1, 1);          //  set the data run it and stop
    if(error > 0){
        error = 2;                          // somtehing went wrong
        return error;
    }
    return error;
}
void read(uint8_t DeviceAddress, uint8_t Register,uint8_t Nbytes, uint16_t *val){
    int i;
    for (i=0;i<Nbytes; i++){
        I2C_Send1(DeviceAddress, Register+i);
         *val=I2C_Recv2(DeviceAddress);
         val++;

    }}

void readByte( uint8_t RAddress, uint8_t data){
    int error = 0;
    StartTransmit( 1);        //  Read
    error = WriteByte(RAddress, 1, 1, 1);    //  Set register address to read from

    if(error != 0){
        error = 1;
        return ;
    }

    data = I2C0_MDR_R;
    return;
}
uint16_t I2C_Recv2(int8_t slave){
  uint8_t data1,data2;
  int retryCounter = 1;
  do{
    while(I2C0_MCS_R&I2C_MCS_BUSY){};// wait for I2C ready
    I2C0_MSA_R = (slave<<1)&0xFE;    // MSA[7:1] is slave address
    I2C0_MSA_R |= 0x01;              // MSA[0] is 1 for receive
    I2C0_MCS_R = (0
                         | I2C_MCS_ACK      // positive data ack
                       //  & ~I2C_MCS_STOP    // no stop
                         | I2C_MCS_START    // generate start/restart
                         | I2C_MCS_RUN);    // master enable
    while(I2C0_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
    data1 = (I2C0_MDR_R&0xFF);       // MSB data sent first
    I2C0_MCS_R = (0
                       //  & ~I2C_MCS_ACK     // negative data ack (last byte)
                         | I2C_MCS_STOP     // generate stop
                       //  & ~I2C_MCS_START   // no start/restart
                         | I2C_MCS_RUN);    // master enable
    while(I2C0_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
    data2 = (I2C0_MDR_R&0xFF);       // LSB data sent last
    retryCounter = retryCounter + 1;        // increment retry counter
  }                                         // repeat if error
  while(((I2C0_MCS_R&(I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
  return (data1<<8)+data2;                  // usually returns 0xFFFF on error
}

uint16_t I2C_Recv(int8_t slave){
  uint8_t data1,data2;
  int retryCounter = 1;
  do{
    while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for I2C ready
    I2C1_MSA_R = (slave<<1)&0xFE;    // MSA[7:1] is slave address
    I2C1_MSA_R |= 0x01;              // MSA[0] is 1 for receive
    I2C1_MCS_R = (0
                         | I2C_MCS_ACK      // positive data ack
                       //  & ~I2C_MCS_STOP    // no stop
                         | I2C_MCS_START    // generate start/restart
                         | I2C_MCS_RUN);    // master enable
    while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
    data1 = (I2C1_MDR_R&0xFF);       // MSB data sent first
    I2C1_MCS_R = (0
                       //  & ~I2C_MCS_ACK     // negative data ack (last byte)
                         | I2C_MCS_STOP     // generate stop
                       //  & ~I2C_MCS_START   // no start/restart
                         | I2C_MCS_RUN);    // master enable
    while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
    data2 = (I2C1_MDR_R&0xFF);       // LSB data sent last
    retryCounter = retryCounter + 1;        // increment retry counter
  }                                         // repeat if error
  while(((I2C1_MCS_R&(I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0) && (retryCounter <= MAXRETRIES));
  return data2;                  // usually returns 0xFFFF on error
}
uint32_t I2C_Send1(int8_t slave, uint8_t data1){
  while(I2C0_MCS_R&I2C_MCS_BUSY){};// wait for I2C ready
  I2C0_MSA_R = (slave<<1)&0xFE;    // MSA[7:1] is slave address
  I2C0_MSA_R &= ~0x01;             // MSA[0] is 0 for send
  I2C0_MDR_R = data1&0xFF;         // prepare first byte
  I2C0_MCS_R = (0
                    //   & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // generate stop
                       | I2C_MCS_START    // generate start/restart
                       | I2C_MCS_RUN);    // master enable
  while(I2C0_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // return error bits
  return (I2C0_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
}
uint32_t I2C_Send3(int8_t slave, uint8_t data1){
  while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for I2C ready
  I2C1_MSA_R = (slave<<1)&0xFE;    // MSA[7:1] is slave address
  I2C1_MSA_R &= ~0x01;             // MSA[0] is 0 for send
  I2C1_MDR_R = data1&0xFF;         // prepare first byte
  I2C1_MCS_R = (0
                    //   & ~I2C_MCS_ACK     // no data ack (no data on send)
                       | I2C_MCS_STOP     // generate stop
                       | I2C_MCS_START    // generate start/restart
                       | I2C_MCS_RUN);    // master enable
  while(I2C1_MCS_R&I2C_MCS_BUSY){};// wait for transmission done
                                          // return error bits
  return (I2C1_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
}
void imuRetrieval( ){

    readAccelData();
    readGyroData();
    readMagData();
    readTempData();
}

static void StartTransmit1( uint8_t RW){
       I2C1_MSA_R = (ALT_READ << 1) +RW;
   }
   //
   // data is either going to be the register or the data
   //
   static uint8_t WriteDatas1(uint8_t data, uint8_t start, uint8_t run, uint8_t stop){
       int error;
       I2C1_MDR_R = data;
       if(start){
          while(I2C1_MCS_R & 0x40);                 // making sure line is not busy
       }
       I2C1_MCS_R = ( (run << 0) | (start << 1) | (stop << 2) );
       while(I2C1_MCS_R & 0x01);                   // bus is busy
       error = (I2C1_MCS_R & 0x02) >> 1;           // get the error flag
   return error;
   }
   uint8_t WriteData1( uint8_t RAddress , uint8_t data){
       int error = 0;
       StartTransmit1(0);        //  starting a write
       error = WriteDatas1(RAddress, 1, 1, 0);    //  setting the register start bit, run bit,turn off stop bit

       if(error > 0){
           error = 1;      //something went wrong
           return error;
       }
       error =WriteDatas1(data, 0, 1, 1);          //  set the data run it and stop
       if(error > 0){
           error = 2;                          // somtehing went wrong
           return error;
       }
       return error;
   }

   void readBytes1( uint8_t RAddress, uint8_t Nbytes, uint8_t *val, int merge){
       uint8_t data[2];
       int i;
       for( i = 0; i < Nbytes; i++) {
        readByte1(RAddress,data[i%2]);
        if( ((i%2) != 0) && merge){
            *val=(int16_t)(((int16_t)data[0] << 8) | data[1]) ;
            val++;
        }
        else if(!merge){
         readByte1(RAddress,*val);
       }
   }}
   void readByte1( uint8_t RAddress, uint8_t data){
      int error = 0;
      StartTransmit1( 1);        //  Read
       error = WriteDatas1(RAddress, 1, 1, 1);    //  Set register address to read from
       if(error != 0){
           error = 1;
           return ;
       }
       data = I2C1_MDR_R;
       return;
   }
   void AltRetrieval( ){

           uint16_t data[6] = {0};
           int i;
           int temp;

          I2C_Send3(ALT_READ,4);
          data[0]=I2C_Recv(ALT_READ);
          I2C_Send3(ALT_READ,5);
          data[1]=I2C_Recv(ALT_READ);
          I2C_Send3(ALT_READ,1);
           data[2]=I2C_Recv(ALT_READ);
           I2C_Send3(ALT_READ,2);
                      data[3]=I2C_Recv(ALT_READ);
           // Convert the data
           data[1]=(data[0]<<8)|data[1];
           data[3]=(data[2]<<8)|data[3];
           tHeight = (data[1]/ 32);
           temp =  (data[3] & 0xFF) / 16;
           altitude = tHeight / 320.0;
           cTemp = temp;
            fTemp = cTemp * 1.8 + 32;
   }
   void calibrateMPU9250()
   {
   uint16_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
   uint16_t ii, packet_count, fifo_count;
   int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
   // reset device, reset all registers, clear gyro and accelerometer bias registers
   I2C_Send2(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
   waitMicrosecond(100);
   // get stable time source
   // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

   // Configure device for bias calculation
   I2C_Send2(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
   I2C_Send2(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
   I2C_Send2(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
   I2C_Send2(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
   I2C_Send2(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
   I2C_Send2(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
   waitMicrosecond(15);
   // Configure MPU9250 gyro and accelerometer for bias calculation
   I2C_Send2(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
   I2C_Send2(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
   I2C_Send2(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
   I2C_Send2(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
   uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
   uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g
   // Configure FIFO to capture accelerometer and gyro data for bias calculation
     fifo_count = ((uint16_t)data[0] << 8) | data[1];
     packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  // for (ii = 0; ii < packet_count; ii++) {
   int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
// read data for averaging
   accel_bias[0] = data[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
         accel_bias[1] = data[1];
         accel_bias[2] = data[2];
   // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startu
   /// Push gyro biases to hardware registers
   /*  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
     writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
     writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
     writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
     writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
     writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
   */
     gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity*16.0/37267; // construct gyro bias in deg/s for later manual subtraction
     gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity*16.0/37267;
     gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity*16.0/37267;
   // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
   // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
   // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
   // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
   // the accelerometer biases calculated above must be divided by 8.
   int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
   read(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
     accel_bias_reg[0] = data[0];
   read(MPU9250_ADDRESS, YA_OFFSET_H, 1, &data[0]);
     accel_bias_reg[1] = data[0];
   read(MPU9250_ADDRESS, ZA_OFFSET_H, 1, &data[0]);
     accel_bias_reg[2] = data[0];
   uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
   uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
   for(ii = 0; ii < 3; ii++) {
   if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
     }
   // Construct total accelerometer bias, including calculated average accelerometer bias from above
     accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
     accel_bias_reg[1] -= (accel_bias[1]/8);
     accel_bias_reg[2] -= (accel_bias[2]/8);
     data[0] = (accel_bias_reg[0] )      & 0xFF;
     data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
     data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
     data[3] = (accel_bias_reg[1])      & 0xFF;
     data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
     data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
     data[5] = (accel_bias_reg[2])      & 0xFF;
     data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
   // Apparently this is not working for the acceleration biases in the MPU-9250
   // Are we handling the temperature correction bit properly?
   // Push accelerometer biases to hardware registers
   /*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
     writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
     writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
     writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
     writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
     writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
   */

   // Output scaled accelerometer biases for manual subtraction in the main program
      accelBias[0] = accel_bias[0]*16.0/37267;
      accelBias[1] = accel_bias[1]*16.0/37267;
      accelBias[2] = accel_bias[2]*16.0/37767;


   }
