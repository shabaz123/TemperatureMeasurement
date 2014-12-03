/**********************************************************************************************
 * therm.c
 * RPI <-> 430BOOST-ADS1118 Thermocouple/LCD Board
 * 
 * Revision history:
 * rev. 1 - Initial version - Nov 2013 - shabaz
 *         __                                __     ____   _____  
 *   ____ |  |   ____   _____   ____   _____/  |_  /_   | /  |  | 
 * _/ __ \|  | _/ __ \ /     \_/ __ \ /    \   __\  |   |/   |  |_
 * \  ___/|  |_\  ___/|  Y Y  \  ___/|   |  \  |    |   /    ^   /
 *  \___  >____/\___  >__|_|  /\___  >___|  /__|    |___\____   | 
 *      \/          \/      \/     \/     \/                 |__|
 *                                                                
 * Acknowledgements:
 * Based on spidev.c,
 * TI source code by Wayne Xu and
 * GPIO example by Gert van Loo and Dom http://elinux.org/RPi_Low-level_peripherals#C_2
 *
 * Description:
 * The code here is used to retrieve measurements
 * from a thermocouple.
 *
 * Syntax examples:
 * therm							// display the temperature
 * therm 10						// display the temperature every 10 seconds
 * therm 1 myfile.csv // store the temperature every 1 second into a file
 * therm lcdinit 			// Initialize and clear the LCD
 * therm msg "Hello there" // Print Hello there on line 1
 * therm 10 msg "hi" 	// display the temperature every 10 seconds
 * therm 5 outputfile.csv msg "Logging to file"		// store the temperature every 5 seconds
 *
 * Connections:
 * TI board       RPI B+
 * ------------   ------------------
 * P1_1  VCC      1     3.3V
 * P1_7  CLK      23    CLK
 * P1_8  ADS_CS   26    SPI_CE1
 * P2_8  LCD_CS   24    SPI_CE0
 * P2_9  LCD_RS   11    GPIO_17_GEN0
 * P2_1  GND      9     GND
 * P2_6  SIMO     19    MOSI
 * P2_7  SOMI     21    MISO
 ************************************************************************************************/


// include files
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <linux/spi/spidev.h>
#include <unistd.h> // sleep
#include <time.h>
#include <sys/mman.h>
#include <string.h>
#include <time.h>

// definitions
#define DBG_PRINT 0
#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

#define ADS1118_TS			   (0x0010)    
#define ADS1118_PULLUP     	   (0x0008)
#define ADS1118_NOP     	   (0x0002)  
#define ADS1118_CNVRDY     	   (0x0001)
//Set the configuration to AIN0/AIN1, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
#define ADSCON_CH0		(0x8B8A)
//Set the configuration to AIN2/AIN3, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
#define ADSCON_CH1		(0xBB8A)

#define INTERNAL_SENSOR 0
#define EXTERNAL_SIGNAL 1
#define BUFSIZE 64
#define LCD_RS_GPIO 17

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
// sets   bits which are 1 ignores bits which are 0
#define GPIO_SET *(gpio+7)
// clears bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10)
// 0 if LOW, (1<<g) if HIGH
#define GET_GPIO(g) (*(gpio+13)&(1<<g))
// Pull up/pull down
#define GPIO_PULL *(gpio+37)
// Pull up/pull down clock
#define GPIO_PULLCLK0 *(gpio+38)

// typedefs
typedef struct spi_ioc_transfer spi_t;

// global variables
int  mem_fd;
void *gpio_map;
volatile unsigned *gpio;
extern int errno;
static const char *device0 = "/dev/spidev0.0";
static const char *device1 = "/dev/spidev0.1";
int ads_fd;
int lcd_fd;
spi_t spi;
int dofile;
FILE* outfile;
int lcd_initialised=0;
uint8_t spi_bits = 8;
//uint32_t spi_speed = 2621440;
uint32_t spi_speed = 3932160;
unsigned char txbuf[BUFSIZE];
unsigned char rxbuf[BUFSIZE];
int local_comp;

// function prototypes


// functions
int
delay_ms(unsigned int msec)
{
  int ret;
  struct timespec a;
  if (msec>999)
  {
    //fprintf(stderr, "delay_ms error: delay value needs to be less than 999\n");
    msec=999;
  }
  a.tv_nsec=((long)(msec))*1E6d;
  a.tv_sec=0;
  if ((ret = nanosleep(&a, NULL)) != 0)
  {
    //fprintf(stderr, "delay_ms error: %s\n", strerror(errno));
  }
  return(0);
}

// Set up a memory regions to access GPIO
void setup_io()
{
   /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/mem \n");
      exit(-1);
   }

   /* mmap GPIO */
   gpio_map = mmap(
      NULL,             //Any adddress in our space will do
      BLOCK_SIZE,       //Map length
      PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
      MAP_SHARED,       //Shared with other processes
      mem_fd,           //File to map
      GPIO_BASE         //Offset to GPIO peripheral
   );

   close(mem_fd); //No need to keep mem_fd open after mmap

   if (gpio_map == MAP_FAILED) {
      printf("mmap error %d\n", (int)gpio_map);//errno also set!
      exit(-1);
   }

   // Always use volatile pointer!
   gpio = (volatile unsigned *)gpio_map;
}

int
spi_open(int* f_desc, int sel, uint8_t config)
{
	uint8_t spi_bits = 8;
	int ret;
	
	if (sel)
		*f_desc=open(device1, O_RDWR);
	else
		*f_desc=open(device0, O_RDWR);
	if (*f_desc<0)
	{
		fprintf(stderr, "Error opening device: %s\n", strerror(errno));
		return(-1);
  }
	ret=ioctl(*f_desc, SPI_IOC_WR_MODE, &config);
	if (ret<0)
  {
  	fprintf(stderr, "Error setting SPI write mode: %s\n", strerror(errno));
		return(-1);
  }
	ret=ioctl(*f_desc, SPI_IOC_RD_MODE, &config);
  if (ret<0)
  {
  	fprintf(stderr, "Error setting SPI read mode: %s\n", strerror(errno));
		return(-1);
  }
  ret=ioctl(*f_desc, SPI_IOC_WR_BITS_PER_WORD, &spi_bits);
  if (ret<0)
  {
  	fprintf(stderr, "Error setting SPI write bits: %s\n", strerror(errno));
		return(-1);
  }
  ret=ioctl(*f_desc, SPI_IOC_RD_BITS_PER_WORD, &spi_bits);
  if (ret<0)
  {
  	fprintf(stderr, "Error setting SPI read bits: %s\n", strerror(errno));
		return(-1);
  }
  ret=ioctl(*f_desc, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
  if (ret<0)
  {
  	fprintf(stderr, "Error setting SPI write speed: %s\n", strerror(errno));
		return(-1);
  }
  ret=ioctl(*f_desc, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
  if (ret<0)
  {
  	fprintf(stderr, "Error setting SPI read speed: %s\n", strerror(errno));
		return(-1);
  }	
	
	return(0);
}

// write command to LCD
void
lcd_writecom(unsigned char c)
{
	int ret;
	
	GPIO_CLR = 1<<LCD_RS_GPIO; //set RS low for transmitting command

	txbuf[0]=c;
	spi.len=1;
  spi.delay_usecs=0;
  spi.speed_hz=spi_speed;
  spi.bits_per_word=spi_bits;
  spi.cs_change=0;
  spi.tx_buf=(unsigned long)txbuf;
  spi.rx_buf=(unsigned long)rxbuf;

  ret=ioctl(lcd_fd, SPI_IOC_MESSAGE(1), &spi);
  if (ret<0)
  {
  	fprintf(stderr, "Error performing SPI exchange: %s\n", strerror(errno));
		exit(1);
  }
}

void
lcd_writedata(unsigned char c)	//write data
{
	int ret;
	
	GPIO_SET = 1<<LCD_RS_GPIO; //set RS high for writing data

	txbuf[0]=c;
	spi.len=1;
  spi.delay_usecs=0;
  spi.speed_hz=spi_speed;
  spi.bits_per_word=spi_bits;
  spi.cs_change=0;
  spi.tx_buf=(unsigned long)txbuf;
  spi.rx_buf=(unsigned long)rxbuf;

  ret=ioctl(lcd_fd, SPI_IOC_MESSAGE(1), &spi);
  if (ret<0)
  {
  	fprintf(stderr, "Error performing SPI exchange: %s\n", strerror(errno));
		exit(1);
  }
}

void
lcd_clear(void)
{
	lcd_writecom(0x01);
	delay_ms(2);
	lcd_writecom(0x02);
	delay_ms(2);
}

/******************************************************************************
function: void lcd_display_string(unsigned char L ,unsigned char *ptr)
introduction: display a string on singal line of LCD.
parameters:L is the display line, 0 indicates the first line, 1 indicates the second line
return value:
*******************************************************************************/
void
lcd_display_string(unsigned char line_num, char *ptr)
{
//	unsigned char i;

	if(line_num==0)		//first line
	{
		lcd_writecom(0x80);
	}
	else if(line_num==1)	//second line
	{
		lcd_writecom(0xc0);
	}

	while (*ptr)
	{
		lcd_writedata(*ptr++);
	}

}

// initialize and clear the display
void
lcd_init(void)
{
	GPIO_SET = 1<<LCD_RS_GPIO;
	lcd_writecom(0x30);	//wake up
	lcd_writecom(0x39);	//function set
	lcd_writecom(0x14);	//internal osc frequency
	lcd_writecom(0x56);	//power control
	lcd_writecom(0x6D);	//follower control
	lcd_writecom(0x70);	//contrast
	lcd_writecom(0x0C);	//display on
	lcd_writecom(0x06);	//entry mode
	lcd_writecom(0x01);	//clear
	delay_ms(20);
}

// Send four bytes (two config bytes repeated twice, and return two bytes)
int
therm_transact(void)
{
	int ret;
	
	txbuf[0]=txbuf[0] | 0x80;
	
	spi.len=4;
	txbuf[2]=txbuf[0];
	txbuf[3]=txbuf[1];

  spi.delay_usecs=0;
  spi.speed_hz=spi_speed;
  spi.bits_per_word=spi_bits;
  spi.cs_change=0;
  spi.tx_buf=(unsigned long)txbuf;
  spi.rx_buf=(unsigned long)rxbuf;

	if (DBG_PRINT)  
  	printf("sending [%02x %02x %02x %02x]. ", txbuf[0], txbuf[1], txbuf[2], txbuf[3]);

  ret=ioctl(ads_fd, SPI_IOC_MESSAGE(1), &spi);
  if (ret<0)
  {
  	fprintf(stderr, "Error performing SPI exchange: %s\n", strerror(errno));
		exit(1);
  }
  
  if (DBG_PRINT)
  	printf("received [%02x %02x]\n", rxbuf[0], rxbuf[1]);
  ret=rxbuf[0];
  ret=ret<<8;
  ret=ret | rxbuf[1];
  return(ret);
}

/******************************************************************************
 * function: local_compensation(int local_code)
 * introduction:
 * this function transform internal temperature sensor code to compensation code, which is added to thermocouple code.
 * local_data is at the first 14bits of the 16bits data register.
 * So we let the result data to be divided by 4 to replace right shifting 2 bits
 * for internal temperature sensor, 32 LSBs is equal to 1 Celsius Degree.
 * We use local_code/4 to transform local data to n* 1/32 degree.
 * the local temperature is transformed to compensation code for thermocouple directly.
 *                                                   (Tin -T[n-1])
 * comp codes = Code[n-1] + (Code[n] - Code[n-1])* {---------------}
 *													(T[n] - T[n-1])
 * for example: 5-10 degree the equation is as below
 *
 * tmp = (0x001A*(local_temp - 5))/5 + 0x0019;
 *
 * 0x0019 is the 'Code[n-1]' for 5 Degrees; 	0x001A = (Code[n] - Code[n-1])
 * (local_temp - 5) is (Tin -T[n-1]);			denominator '5' is (T[n] - T[n-1])
 *
 * the compensation range of local temperature is 0-125.
 * parameters: local_code, internal sensor result
 * return value: compensation codes
 ******************************************************************************/
int
local_compensation(int local_code)
{
	float tmp,local_temp;
	int comp;
	local_code = local_code / 4;
	local_temp = (float)local_code / 32;	//

	if (local_temp >=0 && local_temp <=5)		//0~5
	{
		tmp = (0x0019*local_temp)/5;
		comp = tmp;
	}
	else if (local_temp> 5 && local_temp <=10)	//5~10
	{
		tmp = (0x001A*(local_temp - 5))/5 + 0x0019 ;
		comp = tmp;
	}
	else if (local_temp> 10 && local_temp <=20)	//10~20
	{
		tmp = (0x0033*(local_temp - 10))/10 + 0x0033 ;
		comp = tmp;
	}
	else if (local_temp> 20 && local_temp <=30)	//20~30
	{
		tmp = (0x0034*(local_temp - 20))/10 + 0x0066 ;
		comp = tmp;
	}
	else if (local_temp> 30 && local_temp <=40)	//30~40
	{
		tmp = (0x0034*(local_temp - 30))/10 + 0x009A ;
		comp = tmp;
	}
	else if (local_temp> 40 && local_temp <=50)	//40~50
	{
		tmp = (0x0035*(local_temp - 40))/10 + 0x00CE;
		comp = tmp;
	}

	else if (local_temp> 50 && local_temp <=60)	//50~60
	{
		tmp = (0x0035*(local_temp - 50))/10 + 0x0103;
		comp = tmp;
	}
	else if (local_temp> 60 && local_temp <=80)	//60~80
	{
		tmp = (0x006A*(local_temp - 60))/20 + 0x0138;
		comp = tmp;
	}
	else if (local_temp> 80 && local_temp <=125)//80~125
	{
		tmp = (0x00EE*(local_temp - 80))/45 + 0x01A2;
		comp = tmp;
	}
	else
	{
		comp = 0;
	}
	return comp;
}

/******************************************************************************
 * function: adc_code2temp(int code)
 * introduction:
 * this function is used to convert ADC result codes to temperature.
 * converted temperature range is 0 to 500 Celsius degree
 * Omega Engineering Inc. Type K thermocouple is used, seebeck coefficient is about 40uV/Degree from 0 to 1000 degree.
 * ADC input range is +/-256mV. 16bits. so 1 LSB = 7.8125uV. the coefficient of code to temperature is 1 degree = 40/7.8125 LSBs.
 * Because of nonlinearity of thermocouple. Different coefficients are used in different temperature ranges.
 * the voltage codes is transformed to temperature as below equation
 * 							      (Codes - Code[n-1])
 * T = T[n-1] + (T[n]-T[n-1]) * {---------------------}
 * 							     (Code[n] - Code[n-1])
 *
 * parameters: code
 * return value: far-end temperature
*******************************************************************************/
int
adc_code2temp(int code)	// transform ADC code for far-end to temperature.
{
	float temp;
	int t;

	temp = (float)code;

	if (code > 0xFF6C && code <=0xFFB5)			//-30~-15
	{
		temp = (float)(15 * (temp - 0xFF6C)) / 0x0049 - 30.0f;
	}
	else if (code > 0xFFB5 && code <=0xFFFF)	//-15~0
	{
		temp = (float)(15 * (temp - 0xFFB5)) / 0x004B - 15.0f;
	}
	else if (code >=0 && code <=0x0019)			//0~5
	{
		temp = (float)(5 * (temp - 0)) / 0x0019;
	}
	else if (code >0x0019 && code <=0x0033)		//5~10
	{
		temp = (float)(5 * (temp - 0x0019)) / 0x001A + 5.0f;
	}
	else if (code >0x0033 && code <=0x0066)		//10~20
	{
		temp = (float)(10 * (temp - 0x0033)) / 0x0033 + 10.0f;
	}
	else if (code > 0x0066 && code <= 0x009A)	//20~30
	{
		temp = (float)(10 * (temp - 0x0066)) / 0x0034 + 20.0f;
	}
	else if (code > 0x009A && code <= 0x00CE)	//30~40
	{
		temp = (float)(10 * (temp - 0x009A)) / 0x0034 + 30.0f;
	}
	else if ( code > 0x00CE && code <= 0x0103)	//40~50
	{
		temp = (float)(10 * (temp - 0x00CE)) / 0x0035 + 40.0f;
	}
	else if ( code > 0x0103 && code <= 0x0138)	//50~60
	{
		temp = (float)(10 * (temp - 0x0103)) / 0x0035 + 50.0f;
	}
	else if (code > 0x0138 && code <=0x01A2)	//60~80
	{
		temp = (float)(20 * (temp - 0x0138)) / 0x006A + 60.0f;
	}
	else if (code > 0x01A2 && code <= 0x020C)	//80~100
	{
		temp = (float)((temp - 0x01A2) * 20)/ 0x06A + 80.0f;
	}
	else if (code > 0x020C && code <= 0x02DE)	//100~140
	{
		temp = (float)((temp - 0x020C) * 40)/ 0x0D2 + 100.0f;
	}
	else if (code > 0x02DE && code <= 0x03AC)	//140~180
	{
		temp = (float)((temp - 0x02DE) * 40)/ 0x00CE + 140.0f;
	}
	else if (code > 0x03AC && code <= 0x0478)	//180~220
	{
		temp = (float)((temp - 0x03AB) * 40) / 0x00CD + 180.0f;
	}
	else if (code > 0x0478 && code <= 0x0548)	//220~260
	{
		temp = (float)((temp - 0x0478) * 40) / 0x00D0 + 220.0f;
	}
	else if (code > 0x0548 && code <= 0x061B)	//260~300
	{
		temp = (float)((temp - 0x0548) * 40) / 0x00D3 + 260.0f;
	}
	else if (code > 0x061B && code <= 0x06F2)	//300~340
	{
		temp = (float)((temp - 0x061B) * 40) /  0x00D7 + 300.0f;
	}
	else if (code > 0x06F2 && code <= 0x07C7)	//340~400
	{
		temp =(float) ((temp - 0x06F2) *  40)  / 0x00D5 + 340.0f;
	}
	else if (code > 0x07C7 && code <= 0x089F)	//380~420
	{
		temp =(float) ((temp - 0x07C7) * 40)  / 0x00D8 + 380.0f;
	}

	else if (code > 0x089F && code <= 0x0978)	//420~460
	{
		temp = (float)((temp - 0x089F) * 40) / 0x00D9 + 420.0f;
	}
	else if (code > 0x0978 && code <=0x0A52)	//460~500
	{
		temp =(float)((temp - 0x0978) * 40) / 0x00DA + 460.0f;
	}
	else
	{
		temp = 0xA5A5;
	}

	t = (int)(10*temp);

	return t;
}

/******************************************************************************
 * function: ads_config (unsigned int mode) (based on TI code)
 * introduction: configure and start conversion.
 * parameters:
 * mode = 0 (INTERNAL_SENSOR), ADS1118 is set to convert the voltage of integrated temperature sensor.
 * mode = 1 (EXTERNAL_SIGNAL), ADS1118 is set to convert the voltage of thermocouple.
 * chan = 0 or 1 (ADC channel)
 * return value:
*******************************************************************************/
void
ads_config(unsigned int mode, unsigned int chan)
{

	unsigned int tmp;
	int ret;
	
	if(chan)
	{
		if (mode==EXTERNAL_SIGNAL)		// Set the configuration to AIN0/AIN1, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
			tmp = ADSCON_CH1;
		else
			tmp = ADSCON_CH1 + ADS1118_TS;// internal temperature sensor mode.DR=8sps, PULLUP on DOUT
	}
	else
	{
		if (mode==EXTERNAL_SIGNAL)		// Set the configuration to AIN0/AIN1, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
			tmp = ADSCON_CH0;
		else
			tmp = ADSCON_CH0 + ADS1118_TS;// internal temperature sensor mode.DR=8sps, PULLUP on DOUT
	}

	txbuf[0]=(unsigned char)((tmp>>8) & 0xff);
	txbuf[1]=(unsigned char)(tmp & 0xff);
	ret=therm_transact();
}

/******************************************************************************
 * function: ads_read(unsigned int mode)
 * introduction: read the ADC result and start a new conversion.
 * parameters:
 * mode = 0 (INTERNAL_SENSOR), ADS1118 is set to convert the voltage of integrated temperature sensor.
 * mode = 1 (EXTERNAL_SIGNAL), ADS1118 is set to convert the voltage of thermocouple.
 * chan = 0 or 1 (ADC channel)
 * return value:result of last conversion
 */
int
ads_read(unsigned int mode, unsigned int chan)
{
	unsigned int tmp;
	int result;

	if(chan)
	{
		if (mode==EXTERNAL_SIGNAL)		// Set the configuration to AIN0/AIN1, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
			tmp = ADSCON_CH1;
		else
			tmp = ADSCON_CH1 + ADS1118_TS;// internal temperature sensor mode.DR=8sps, PULLUP on DOUT
	}
	else
	{
		if (mode==EXTERNAL_SIGNAL)		// Set the configuration to AIN0/AIN1, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
			tmp = ADSCON_CH0;
		else
			tmp = ADSCON_CH0 + ADS1118_TS;// internal temperature sensor mode.DR=8sps, PULLUP on DOUT
	}


	txbuf[0]=(unsigned char)((tmp>>8) & 0xff);
	txbuf[1]=(unsigned char)(tmp & 0xff);
	result=therm_transact();

	return(result);
}

// returns the measured temperature
double
get_measurement(void)
{
	int result;
	int local_data;
	double result_d;
	
	ads_config(INTERNAL_SENSOR,0);  // start internal sensor measurement
	delay_ms(10);
	local_data=ads_read(EXTERNAL_SIGNAL,0); // read internal sensor measurement and start external sensor measurement
	delay_ms(10);
	result=ads_read(EXTERNAL_SIGNAL,0); // read external sensor measurement and restart external sensor measurement
	
	local_comp = local_compensation(local_data);
	result = result + local_comp;
	result=result & 0xffff;
	result = adc_code2temp(result);
	
	//printf("10x temp is %d\n", result);
	result_d=((double)result)/10;
	
	return(result_d);
}

double
get_measurement_fast(void)
{
	int result;
	double result_d;
	
	result=ads_read(EXTERNAL_SIGNAL,0); // read external sensor measurement and restart external sensor measurement
	result = result + local_comp;
	result=result & 0xffff;
	result = adc_code2temp(result);
	
	result_d=((double)result)/10;
	
	return(result_d);
}

// Convert the integer portion of unix timestamp into H:M:S
void
unixtime2string(char* int_part, char* out_time)
{
	unsigned int nutime;
	struct tm *nts;
	char buf1[100];
	char buf2[50];
	
	sscanf(int_part, "%u", &nutime);
	nts=localtime((time_t*)&nutime);
	strftime(buf1, 100, "%H:%M:%S", nts);
	
	strcpy(out_time, buf1);	
}

void sig_handler(int signo)
{
  if (signo == SIGINT)
    printf("received SIGINT\n");
  if (dofile)
  {
  	fclose(outfile);
  }
  close(ads_fd);
  lcd_clear();
	lcd_display_string(1,"Bye");
  close(lcd_fd);
  printf("Bye\n");
  exit(0);
}

int
main(int argc, char* argv[])
{
	int ret;
	int i;
	uint8_t spi_config=0;
	double tval;
	int repeat=0;
	int period=1;
	char fname[128];
	char tstring[128];
	char tstring2[128];
	time_t mytime;
	time_t desiredtime;
	struct timespec tstime;
	int not_finished=1;
	int elapsed=0;
	int showtime=0;
	
	// initialise GPIO
	setup_io();
	INP_GPIO(LCD_RS_GPIO); // must use INP_GPIO before we can use OUT_GPIO
  OUT_GPIO(LCD_RS_GPIO);
	
	// parse inputs
	dofile=0;
	if (argc>2)
	{
		if (strcmp(argv[1], "msg")==0) // print to the lcd
		{
			spi_config=0;
			ret=spi_open(&lcd_fd, 0, spi_config);
			if (ret!=0)
			{
				printf("Exiting\n");
			exit(1);
			}
			lcd_display_string(0,argv[2]);
			close(lcd_fd);
			exit(0);
		}
	}
	if (argc>1)
	{
		if (strcmp(argv[1], "-h")==0) // print help
		{
			printf("%s [sec] [filename]\n", argv[0]);
			printf("%s msg <message in quotes>\n", argv[0]);
			exit(0);
		}
		if (strcmp(argv[1], "lcdinit")==0) // initialize the LCD display
		{
			spi_config=0;
			ret=spi_open(&lcd_fd, 0, spi_config);
			if (ret!=0)
			{
				printf("Exiting\n");
				exit(1);
			}
			lcd_init();
			close(lcd_fd);
			exit(0);
		}
		if (strcmp(argv[1], "withtime")==0)
		{
			showtime=1;
		}
		else
		{
			sscanf(argv[1], "%d", &period); // period between measurements
			repeat=1;
		}
		if (argc>3)
		{
			if (strcmp(argv[2], "msg")==0) // print to the lcd
			{
				spi_config=0;
				ret=spi_open(&lcd_fd, 0, spi_config);
				if (ret!=0)
				{
					printf("Exiting\n");
					exit(1);
				}
				lcd_init();
				lcd_display_string(0,argv[3]);
				lcd_initialised=1;
			}
		}
	}
	if (argc>2)
	{
		strcpy(fname, argv[2]); // file name
		dofile=1;
		if (argc>4)
		{
			if (strcmp(argv[3], "msg")==0) // print to the lcd
			{
				spi_config=0;
				ret=spi_open(&lcd_fd, 0, spi_config);
				if (ret!=0)
				{
					printf("Exiting\n");
					exit(1);
				}
				lcd_init();
				lcd_display_string(0,argv[4]);
				lcd_initialised=1;
			}
		}
	}
	
	if (dofile)
	{
		outfile=fopen(fname, "w");
	}
	
	// open SPI for ADS1118
	spi_config |= SPI_CPHA;
	ret=spi_open(&ads_fd, 1, spi_config);
	if (ret!=0) // SPI error
	{
		printf("Exiting\n");
		exit(1);
	}

	if (repeat==0)
	{
		// display a single measurement and exit
		tval=get_measurement();
		if (showtime)
		{
			mytime = time(NULL);
			sprintf(tstring, "%d", mytime);
			unixtime2string(tstring, tstring2);
			printf("%s %#.1f\n", tstring2, tval);
		}
		else
		{
			printf("%#.1f\n", tval);
		}
		close(ads_fd);
		exit(0);
	}
	
	// open up SPI for LCD
  spi_config=0;
	ret=spi_open(&lcd_fd, 0, spi_config);
	if (ret!=0)
	{
		printf("Exiting\n");
		exit(1);
	}

	if (lcd_initialised==0)
		lcd_init();
	
	if (dofile)
	{
		// line 1 of the output file will contain the three column descriptions
		fprintf(outfile, "Time HH:MM:SS,Elapsed Sec,Temp C\n");
	}
	
	// Align on an integer number of seconds and get current time
  mytime = time(NULL);
  tstime.tv_sec=mytime+1;
  tstime.tv_nsec=0;
  clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tstime, NULL);
  mytime++;
  desiredtime=mytime;

	signal(SIGINT, sig_handler);
	
	while(not_finished)
	{
		tval=get_measurement();
		for (i=1; i<10; i++)
		{
			delay_ms(10);
			tval=tval+get_measurement_fast();
		}
		tval=tval/10;
		
		// print the time, elapsed counter and temperature
		sprintf(tstring, "%d", mytime);
		unixtime2string(tstring, tstring2);
		if (mytime==desiredtime)
		{
			printf("%s %d %#.1f\n", tstring2, elapsed, tval);
			if (dofile)
			{
				fprintf(outfile, "%s,%d,%#.1f\n", tstring2, elapsed, tval);
				fflush(outfile);
			}
			desiredtime=desiredtime+period;
		}
		sprintf(tstring, "%s %#.1f", tstring2, tval);
		lcd_display_string(1, tstring);
		// now we sleep for a certain time
		mytime++;
		tstime.tv_sec=mytime;
		elapsed++;
		clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tstime, NULL);
	}
	
  close(ads_fd);
    
  // LCD requires opposite clock polarity
  spi_config=0;
	ret=spi_open(&lcd_fd, 0, spi_config);
	if (ret!=0)
	{
		printf("Exiting\n");
		exit(1);
	}


	lcd_init();
	lcd_clear();					// LCD clear
	lcd_display_string(0,"Hello");	// display "ADS1118"
  
  close(lcd_fd);
  
  return(0);
}

