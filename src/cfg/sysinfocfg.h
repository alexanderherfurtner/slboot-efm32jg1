/*******************************************************************************
*                                                                              *
*   Description: SysInfo configuration                                         *
*                                                                              *
*   Author:  Alexander Herfurtner (mail@alexanderherfurtner.de)                *
*                                                                              *
*   Copyright (c) 2024 Alexander Herfurtner                                    *
*   All rights reserved.                                                       *
*                                                                              *
*   License: MIT License                                                       *
********************************************************************************/

#ifndef SYSINFOCFG_H
#define SYSINFOCFG_H

#include "eminc.h"

/**
 * @brief Get chip family as a string.
 *
 * @return Chip family as a string.
 */
static inline char * sysinfo_chip_family() {
	uint8_t partfamily = (uint8_t)((DEVINFO->PART & _DEVINFO_PART_DEVICE_FAMILY_MASK)
		>> _DEVINFO_PART_DEVICE_FAMILY_SHIFT);

	switch (partfamily) {
	case _DEVINFO_PART_DEVICE_FAMILY_EFM32JG1B:
		return "EFM32JG1B";
	default:
		return "unknown";
	}
}

/**
 * @brief Get chip ID as a string.
 *
 * @return Chip ID as a string.
 */
static inline char * sysinfo_chip_id() {
	uint16_t partno = (uint16_t)((DEVINFO->PART & _DEVINFO_PART_DEVICE_NUMBER_MASK)
	>> _DEVINFO_PART_DEVICE_NUMBER_SHIFT);

	switch (partno) {
	case 0xC8:
		return "EFM32JG1B200F128GM48";
	default:
		return "unknown";
	}
}

/**
 * @brief Get chip revision as a string.
 *
 * @return Chip revision as a string.
 */
static inline char * sysinfo_chip_revision() {
	static char partrev_str[8];
	char major_let;

	SYSTEM_ChipRevision_TypeDef chiprevinfo;
	SYSTEM_ChipRevisionGet(&chiprevinfo);

	major_let = (char)('A' + (chiprevinfo.major - 1));
	sprintf(partrev_str, "%c%d", major_let, chiprevinfo.minor);
	return partrev_str;
}

/**
 * @brief Get the chip temperature grade.
 *
 * @return Chip temperature grade as a string.
 */
static inline char * sysinfo_chip_tempgrade() {
	uint8_t temp_grade = (uint8_t)(DEVINFO->MEMINFO & _DEVINFO_MEMINFO_TEMPGRADE_MASK
		>> _DEVINFO_MEMINFO_TEMPGRADE_SHIFT);

	switch (temp_grade) {
	case _DEVINFO_MEMINFO_TEMPGRADE_N40TO85:
		return "-40 to 85 °C";
	case _DEVINFO_MEMINFO_TEMPGRADE_N40TO125:
		return "-40 to 125 °C";
	case _DEVINFO_MEMINFO_TEMPGRADE_N40TO105:
		return "-40 to 105 °C";
	case _DEVINFO_MEMINFO_TEMPGRADE_N0TO70:
		return "0 to 70 °C";
	default:
		return "unknown";
	}
}

/**
 * @brief Get the chip MAC address.
 *
 * @return Chip MAC address as a string.
 */
static inline char * sysinfo_chip_mac() {
	uint64_t mac64 = (uint64_t)(DEVINFO->UNIQUEH) << 32 | DEVINFO->UNIQUEL;

	static char mac_str[18];
	sprintf(mac_str, "%02X:%02X:%02X:%02X:%02x:%02X",
		(uint8_t)(mac64 >> 40),
		(uint8_t)(mac64 >> 32),
		(uint8_t)(mac64 >> 24),
		(uint8_t)(mac64 >> 16),
		(uint8_t)(mac64 >> 8),
		(uint8_t)(mac64));
	return mac_str;
}

/**
 * @brief Get the chip serial number.
 *
 * @return Chip serial number as a string.
 */
static inline char * sysinfo_chip_serialnr() {
	static char uniqueId_str[16 + 1];
	sprintf(uniqueId_str, "%08lX%08lX", DEVINFO->UNIQUEH, DEVINFO->UNIQUEL);
	return uniqueId_str;
}

/**
 * @brief Get the flash size (in KB).
 *
 * @return Flash size in KB.
 */
static inline uint32_t sysinfo_flash_size() {
	return SYSTEM_GetFlashSize();
}

/**
 * @brief Get the SRAM size (in KB).
 *
 * @ref SRAM size can be used instead.
 */
static inline uint32_t sysinfo_chip_sram_size() {
	return SYSTEM_GetSRAMSize();
}

#endif /* SYSINFOCFG_H */