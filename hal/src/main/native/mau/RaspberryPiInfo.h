/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <stdint.h>
#include <list>
#include <string>

// Reference:  Wikipedia article on "Raspberry Pi"

enum class RaspberryPiModel {
	UNKNOWN,
	RPI1_B_R1_0, 				// 04/2012:  256MB:  x2, x3
	RPI1_B_R2_0, 				// 06/2012:  256MB:  x4, x5, x6
	                        	//           512MB:  xd, xe, xf
	RPI1_A_R2_0, 				// 02/2013:  256MB:  x7, x8, x9
	RPI1_BPLUS_R1_0,			// 07/2014:  512MB:  x10
	RPI_CM1_R1_0,				// 04/2/14:  512MB:  x11, x14
	RPI1_APLUS_R1_1,			// 11/2014:  256MB:  x12, 0x900032 -
	                        	//           256 or 512MB:  x15
								//           512MB:  0x900021
	RPI1_BPLUS_R1_2,			// ???????:  512MB:  x13
	RPI_ZERO_R1_2,				// 11/2015:  0x900092, 0x9200092
	RPI_ZERO_R1_3,				// 05/2016:  0x900093, 0x920093
	RPI_ZEROW_R1_3,				// 02/2017:  0x9000c1
	RPI2_B_R1_0,				// 02/2015:  0xa01040
	RPI2_B_R1_1,				// ???????:  0xa01041
	RPI3_B_R1_2,				// 02/2016:  0xa02082, 0xa22082, 0xa32082, 0xa52082
	RPI_CM3_R1_0,				// 01/2017:  0xa020a0
	RPI2_B_W_BCM2837_R1_2,		// 10/2016:  0xa22042
	RPI3_BPLUS_R1_3,			// 03/2018:  0xa020d3
	RPI3_APLUS_R1_0,			// 11/2017:  0x9020e0
	RPI4_B,					// 06/2019:  0xc03111 (4GB), 0xb03111 (2GB), 0xa03111 (1GB)
};

enum class RaspberryPiUSBPort {
	INVALID,
	RPI_USB_PORTA,				// Left, top of USB Port Block; For 1-port models, the sole USB port on the board
	RPI_USB_PORTB,				// Left, button of USB Port Block
	RPI_USB_PORTC,				// Right, top of USB Port Block
	RPI_USB_PORTD,				// Right, bottom of USB Port Block
};

class RaspberryPiDeviceInfo {
public:
	RaspberryPiModel model;
	std::string model_description;
	unsigned hwrev;
	uint32_t ram_mb;
	bool has_40pin_gpio;
	int num_usb_ports;
	RaspberryPiDeviceInfo(RaspberryPiModel model, std::string model_description, unsigned hwrev, uint32_t ram_mb, bool has_40pin_gpio, int num_usb_ports) {
		this->model = model;
		this->model_description = model_description;
		this->hwrev = hwrev;
		this->ram_mb = ram_mb;
		this->has_40pin_gpio = has_40pin_gpio;
		this->num_usb_ports = num_usb_ports;
	}
};

class USBSerialDeviceInfo {
public:
	std::string devname;
	std::string usbshortpath;
	std::string usblongpath;
	std::string deviceid;
	RaspberryPiUSBPort physical_usb_port;
	USBSerialDeviceInfo(std::string devname, std::string usbshortpath, std::string usblongpath, std::string deviceid, RaspberryPiUSBPort physical_usb_port) {
		this->devname = devname;
		this->usbshortpath = usbshortpath;
		this->usblongpath = usblongpath;
		this->deviceid = deviceid;
		this->physical_usb_port = physical_usb_port;
	}
};

std::list<USBSerialDeviceInfo> getRaspberryPiUSBSerialDeviceInfoList();
RaspberryPiDeviceInfo getRaspberryPiDeviceInfo();

