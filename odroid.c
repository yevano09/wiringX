/*
	Copyright (c) 2014 CurlyMo <curlymoo1@gmail.com>
								2012 Gordon Henderson

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

#include "wiringX.h"
#ifndef __FreeBSD__
	#include "i2c-dev.h"
#endif
#include "odroid.h"

#define	WPI_MODE_PINS		 0
#define	WPI_MODE_GPIO		 1
#define	WPI_MODE_PHYS		 3
#define	WPI_MODE_UNINITIALISED	-1

#define ODROIDC_GPIO_MASK (0xFFFFFF80)
#define ODROIDC_PERI_BASE 0xC1100000
#define GPIO_REG_OFFSET   0x8000
#define ODROID_GPIO_BASE  (ODROIDC_PERI_BASE + GPIO_REG_OFFSET)

#define GPIOY_PIN_START         80
#define GPIOY_PIN_END           96
#define GPIOX_PIN_START         97
#define GPIOX_PIN_END           118

#define GPIOY_OUTP_REG_OFFSET   0x10
#define GPIOY_INP_REG_OFFSET    0x11
#define GPIOX_FSEL_REG_OFFSET   0x0C
#define GPIOX_OUTP_REG_OFFSET   0x0D
#define GPIOX_INP_REG_OFFSET    0x0E
#define GPIOY_FSEL_REG_OFFSET   0x0F

#define NUM_PINS		32

#define	BLOCK_SIZE		(4*1024)

static volatile uint32_t *gpio;

static int pinModes[NUM_PINS];

static uint8_t gpioToShift[] = {
	0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
	0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
	0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
	0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
	0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
};

static uint8_t gpioToGPFSEL[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
	3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
	4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
	5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
};

//
// pinToGpio:
//	Take a Wiring pin (0 through X) and re-map it to the ODROID_GPIO pin
//
static int pinToGpio[64] = {
    88,  87, 116, 115, 104, 102, 103,  83, // 0..7
    -1,  -1, 117, 118, 107, 106, 105,  -1, // 8..16
    -1,  -1,  -1,  -1,  -1, 101, 100, 108, // 16..23
    97,  -1,  99,  98,  -1,  -1,  -1,  -1, // 24..31
// Padding:
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
};

//
// physToGpio:
//	Take a physical pin (1 through 40) and re-map it to the ODROID_GPIO pin
//
static int physToGpio[64] =
{
  -1,       // 0
  -1,  -1,	// 1, 2
  -1,  -1,
  -1,  -1,
  83,  -1,
  -1,  -1,
  88,  87,
 116,  -1,
 115, 104,
  -1, 102,
 107,  -1,
 106, 103,
 105, 117,
  -1, 118,	// 25, 26

  -1,  -1,
 101,  -1,
 100,  99,
 108,  -1,
  97,  98,
  -1,  -1,
  -1,  -1, // 39, 40

// Not used
  -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1,
} ;

static int sysFds[64] = {
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};

//
// offset to the GPIO Set regsiter
//
static int gpioToGPSETReg(int pin) {
	if(pin >= GPIOX_PIN_START && pin <= GPIOX_PIN_END) {
		return GPIOX_OUTP_REG_OFFSET;
	}
	if(pin >= GPIOY_PIN_START && pin <= GPIOY_PIN_END) {
		return GPIOY_OUTP_REG_OFFSET;
	}
	return -1;
}

//
// offset to the GPIO Input regsiter
//
static int  gpioToGPLEVReg (int pin) {
	if(pin >= GPIOX_PIN_START && pin <= GPIOX_PIN_END) {
		return GPIOX_INP_REG_OFFSET;
	}
	if(pin >= GPIOY_PIN_START && pin <= GPIOY_PIN_END) {
		return GPIOY_INP_REG_OFFSET;
	}
	return -1;
}

//
// offset to the GPIO bit
//
static int gpioToShiftReg(int pin) {
	if(pin >= GPIOX_PIN_START && pin <= GPIOX_PIN_END) {
		return pin - GPIOX_PIN_START;
	}
	if(pin >= GPIOY_PIN_START && pin <= GPIOY_PIN_END) {
		return pin - GPIOY_PIN_START;
	}

	return -1;
}

//
// offset to the GPIO Function register
//
static int gpioToGPFSELReg(int pin) {
	if(pin >= GPIOX_PIN_START && pin <= GPIOX_PIN_END) {
		return GPIOX_FSEL_REG_OFFSET;
	}
	if(pin >= GPIOY_PIN_START && pin <= GPIOY_PIN_END) {
		return GPIOY_FSEL_REG_OFFSET;
	}

	return -1;
}

int odroidValidGPIO(int pin) {
	if(pinToGpio[pin] != -1) {
		return 0;
	}
	return -1;
}

static int changeOwner(char *file) {
	uid_t uid = getuid();
	uid_t gid = getgid();

	if(chown(file, uid, gid) != 0) {
		if(errno == ENOENT)	{
			wiringXLog(LOG_ERR, "odroid->changeOwner: File not present: %s", file);
			return -1;
		} else {
			wiringXLog(LOG_ERR, "odroid->changeOwner: Unable to change ownership of %s: %s", file, strerror (errno));
			return -1;
		}
	}

	return 0;
}

static int odBoardRev(void) {
	FILE *cpuFd;
	char line[120];
	char *d;

	memset(line, '\0', 120);
	
	if((cpuFd = fopen("/proc/cpuinfo", "r")) == NULL) {
		wiringXLog(LOG_ERR, "odroid->identify: Unable open /proc/cpuinfo");
		return -1;
	}

	while(fgets(line, 120, cpuFd) != NULL) {
		if(strncmp(line, "Hardware", 8) == 0) {
			break;
		}
	}

	fclose(cpuFd);

	if(strlen(line) == 0) {
		return -1;
	}

	if(strncmp(line, "Hardware", 8) != 0) {
		wiringXLog(LOG_ERR, "odroid->identify: /proc/cpuinfo has no hardware line");
		return -1;
	}

	for(d = &line[strlen(line) - 1]; (*d == '\n') || (*d == '\r') ; --d)
		*d = 0 ;

	if(strstr(line, "ODROIDC") != NULL) {
		return 0;
	} else {
		return -1;
	}
}

static int setup(void)	{
	int fd;
	int boardRev;

	boardRev = odBoardRev();

	if((fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0) {
		wiringXLog(LOG_ERR, "bananapi->setup: Unable to open /dev/mem");
		return -1;
	}

	if(boardRev == 0) {
		gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, ODROID_GPIO_BASE);

		if((int32_t)gpio == -1) {
			wiringXLog(LOG_ERR, "bananapi->setup: mmap (GPIO) failed");
			return -1;
		}
	}

	return 0;
}

static int odroidDigitalRead(int pin) {
	if(pinModes[pin] != INPUT && pinModes[pin] != SYS) {
		wiringXLog(LOG_ERR, "odroid->digitalRead: Trying to write to pin %d, but it's not configured as input", pin);
		return -1;
	}

	if(odroidValidGPIO(pin) != 0) {
		wiringXLog(LOG_ERR, "odroid->digitalRead: Invalid pin number %d", pin);
		return -1;
	}	

	if((pin & ODROIDC_GPIO_MASK) == 0) {
		pin = pinToGpio[pin] ;

		 if ((*(gpio + gpioToGPLEVReg(pin)) & (1 << gpioToShiftReg(pin))) != 0) {
			return HIGH;
		} else {
			return LOW;
		}
	}
	return 0;
}

static int odroidDigitalWrite(int pin, int value) {
	if(pinModes[pin] != OUTPUT) {
		wiringXLog(LOG_ERR, "odroid->digitalWrite: Trying to write to pin %d, but it's not configured as output", pin);
		return -1;
	}

	if(odroidValidGPIO(pin) != 0) {
		wiringXLog(LOG_ERR, "odroid->digitalWrite: Invalid pin number %d", pin);
		return -1;
	}	

	if((pin & ODROIDC_GPIO_MASK) == 0) {
		pin = pinToGpio[pin] ;

		if(value == LOW)
			*(gpio + gpioToGPSETReg(pin)) &= ~(1 << gpioToShiftReg(pin));
		else
			*(gpio + gpioToGPSETReg(pin)) |=  (1 << gpioToShiftReg(pin));
	}
	return 0;
}

static int odroidPinMode(int pin, int mode) {
	int fSel, shift;

	if(odroidValidGPIO(pin) != 0) {
		wiringXLog(LOG_ERR, "odroid->pinMode: Invalid pin number %d", pin);
		return -1;
	}	

	if((pin & ODROIDC_GPIO_MASK) == 0) {
		pinModes[pin] = mode;
		pin = pinToGpio[pin];

		fSel = gpioToGPFSEL[pin];
		shift = gpioToShift[pin];

		if(mode == INPUT) {
			*(gpio + gpioToGPFSELReg(pin)) = (*(gpio + gpioToGPFSELReg(pin)) |  (1 << gpioToShiftReg(pin)));   
		} else if(mode == OUTPUT) {
			*(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift));
		}
	}
	return 0;
}

static int odroidISR(int pin, int mode) {
	int i = 0, fd = 0, match = 0, count = 0;
	const char *sMode = NULL;
	char path[35], c, line[120];
	FILE *f = NULL;

	if(odroidValidGPIO(pin) != 0) {
		wiringXLog(LOG_ERR, "odroid->isr: Invalid pin number %d", pin);
		return -1;
	}	

	pinModes[pin] = SYS;

	if(mode == INT_EDGE_FALLING) {
		sMode = "falling" ;
	} else if(mode == INT_EDGE_RISING) {
		sMode = "rising" ;
	} else if(mode == INT_EDGE_BOTH) {
		sMode = "both";
	} else {
		wiringXLog(LOG_ERR, "odroid->isr: Invalid mode. Should be INT_EDGE_BOTH, INT_EDGE_RISING, or INT_EDGE_FALLING");
		return -1;
	}

	sprintf(path, "/sys/class/gpio/gpio%d/value", pinToGpio[pin]);
	fd = open(path, O_RDWR);

	if(fd < 0) {
		if((f = fopen("/sys/class/gpio/export", "w")) == NULL) {
			wiringXLog(LOG_ERR, "odroid->isr: Unable to open GPIO export interface: %s", strerror(errno));
			return -1;
		}

		fprintf(f, "%d\n", pinToGpio[pin]);
		fclose(f);
	}

	sprintf(path, "/sys/class/gpio/gpio%d/direction", pinToGpio[pin]);
	if((f = fopen(path, "w")) == NULL) {
		wiringXLog(LOG_ERR, "odroid->isr: Unable to open GPIO direction interface for pin %d: %s", pin, strerror(errno));
		return -1;
	}

	fprintf(f, "in\n");
	fclose(f);

	sprintf(path, "/sys/class/gpio/gpio%d/edge", pinToGpio[pin]);
	if((f = fopen(path, "w")) == NULL) {
		wiringXLog(LOG_ERR, "odroid->isr: Unable to open GPIO edge interface for pin %d: %s", pin, strerror(errno));
		return -1;
	}

	if(strcasecmp(sMode, "none") == 0) {
		fprintf(f, "none\n");
	} else if(strcasecmp(sMode, "rising") == 0) {
		fprintf(f, "rising\n");
	} else if(strcasecmp(sMode, "falling") == 0) {
		fprintf(f, "falling\n");
	} else if(strcasecmp (sMode, "both") == 0) {
		fprintf(f, "both\n");
	} else {
		wiringXLog(LOG_ERR, "odroid->isr: Invalid mode: %s. Should be rising, falling or both", sMode);
		return -1;
	}
	fclose(f);

	if((f = fopen(path, "r")) == NULL) {
		wiringXLog(LOG_ERR, "odroid->isr: Unable to open GPIO edge interface for pin %d: %s", pin, strerror(errno));
		return -1;
	}

	match = 0;
	while(fgets(line, 120, f) != NULL) {
		if(strstr(line, sMode) != NULL) {
			match = 1;
			break;
		}
	}
	fclose(f);

	if(match == 0) {
		wiringXLog(LOG_ERR, "odroid->isr: Failed to set interrupt edge to %s", sMode);
		return -1;	
	}

	sprintf(path, "/sys/class/gpio/gpio%d/value", pinToGpio[pin]);
	if((sysFds[pin] = open(path, O_RDONLY)) < 0) {
		wiringXLog(LOG_ERR, "odroid->isr: Unable to open GPIO value interface: %s", strerror(errno));
		return -1;
	}
	changeOwner(path);

	sprintf(path, "/sys/class/gpio/gpio%d/edge", pinToGpio[pin]);
	changeOwner(path);

	ioctl(fd, FIONREAD, &count);
	for(i=0; i<count; ++i) {
		read(fd, &c, 1);
	}
	close(fd);

	return 0;
}

static int odroidWaitForInterrupt(int pin, int ms) {
	int x = 0;
	uint8_t c = 0;
	struct pollfd polls;

	if(odroidValidGPIO(pin) != 0) {
		wiringXLog(LOG_ERR, "odroid->waitForInterrupt: Invalid pin number %d", pin);
		return -1;
	}

	if(pinModes[pin] != SYS) {
		wiringXLog(LOG_ERR, "odroid->waitForInterrupt: Trying to read from pin %d, but it's not configured as interrupt", pin);
		return -1;
	}

	if(sysFds[pin] == -1) {
		wiringXLog(LOG_ERR, "odroid->waitForInterrupt: GPIO %d not set as interrupt", pin);
		return -1;
	}

	polls.fd = sysFds[pin];
	polls.events = POLLPRI;

	x = poll(&polls, 1, ms);

	/* Don't react to signals */
	if(x == -1 && errno == EINTR) {
		x = 0;
	}
	
	(void)read(sysFds[pin], &c, 1);
	lseek(sysFds[pin], 0, SEEK_SET);

	return x;
}

static int odroidGC(void) {
	int i = 0, fd = 0;
	char path[35];
	FILE *f = NULL;

	for(i=0;i<NUM_PINS;i++) {
		if(pinModes[i] == OUTPUT) {
			pinMode(i, INPUT);
		} else if(pinModes[i] == SYS) {
			sprintf(path, "/sys/class/gpio/gpio%d/value", pinToGpio[i]);
			if((fd = open(path, O_RDWR)) > 0) {
				if((f = fopen("/sys/class/gpio/unexport", "w")) == NULL) {
					wiringXLog(LOG_ERR, "odroid->gc: Unable to open GPIO unexport interface: %s", strerror(errno));
				}

				fprintf(f, "%d\n", pinToGpio[i]);
				fclose(f);
				close(fd);
			}
		}
		if(sysFds[i] > 0) {
			close(sysFds[i]);
		}
	}

	if(gpio) {
		munmap((void *)gpio, BLOCK_SIZE);
	}
	return 0;
}

#ifndef __FreeBSD__
static int odroidI2CRead(int fd) {
	return i2c_smbus_read_byte(fd);
}

static int odroidI2CReadReg8(int fd, int reg) {
	return i2c_smbus_read_byte_data(fd, reg);
}

static int odroidI2CReadReg16(int fd, int reg) {
	return i2c_smbus_read_word_data(fd, reg);
}

static int odroidI2CWrite(int fd, int data) {
	return i2c_smbus_write_byte(fd, data);
}

static int odroidI2CWriteReg8(int fd, int reg, int data) {
	return i2c_smbus_write_byte_data(fd, reg, data);
}

static int odroidI2CWriteReg16(int fd, int reg, int data) {
	return i2c_smbus_write_word_data(fd, reg, data);
}

static int odroidI2CSetup(int devId) {
	int rev = 0, fd = 0;
	const char *device = NULL;

	if((rev = odBoardRev()) < 0) {
		wiringXLog(LOG_ERR, "odroid->I2CSetup: Unable to determine Pi board revision");
		return -1;
	}

	device = "/dev/i2c-1";

	if((fd = open(device, O_RDWR)) < 0) {
		wiringXLog(LOG_ERR, "odroid->I2CSetup: Unable to open %s: %s", device, strerror(errno));
		return -1;
	}

	if(ioctl(fd, I2C_SLAVE, devId) < 0) {
		wiringXLog(LOG_ERR, "odroid->I2CSetup: Unable to set %s to slave mode: %s", device, strerror(errno));
		return -1;
	}

	return fd;
}
#endif

void odroidInit(void) {

	memset(pinModes, -1, NUM_PINS);

	platform_register(&odroid, "odroid");
	odroid->setup=&setup;
	odroid->pinMode=&odroidPinMode;
	odroid->digitalWrite=&odroidDigitalWrite;
	odroid->digitalRead=&odroidDigitalRead;
	odroid->identify=&odBoardRev;
	odroid->isr=&odroidISR;
	odroid->waitForInterrupt=&odroidWaitForInterrupt;
#ifndef __FreeBSD__
	odroid->I2CRead=&odroidI2CRead;
	odroid->I2CReadReg8=&odroidI2CReadReg8;
	odroid->I2CReadReg16=&odroidI2CReadReg16;
	odroid->I2CWrite=&odroidI2CWrite;
	odroid->I2CWriteReg8=&odroidI2CWriteReg8;
	odroid->I2CWriteReg16=&odroidI2CWriteReg16;
	odroid->I2CSetup=&odroidI2CSetup;
#endif
	odroid->gc=&odroidGC;
	odroid->validGPIO=&odroidValidGPIO;
}
