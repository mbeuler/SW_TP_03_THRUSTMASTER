/*
 * usbh_hid_throttle.c
 *
 *  Created on: Nov 23, 2025
 *      Author: marcel.beuler
 */

#include "usbh_hid_throttle.h"

// Static buffer for incoming HID reports (input report with ID 1, 36 bytes)
static uint8_t throttle_rx_report_buf[THROTTLE_REPORT_SIZE];

// Static FIFO buffer for storing multiple HID reports before processing
static uint8_t fifo_buffer[128];


USBH_StatusTypeDef USBH_HID_ThrottleInit(USBH_HandleTypeDef *phost) {
	// Get pointer to the central HID class data structure
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

	// Initialize the report buffer to zero (clean start)
	for (uint32_t i = 0; i < THROTTLE_REPORT_SIZE; i++) {
	    throttle_rx_report_buf[i] = 0;
	}

	// Set expected report length according to the HID report descriptor
	HID_Handle->length = THROTTLE_REPORT_SIZE;

	// Assign the report buffer to the HID handle for incoming data
	//HID_Handle->pData = phost->device.Data;
	HID_Handle->pData = throttle_rx_report_buf;

	// Initialize the FIFO buffer for temporary storage of multiple HID reports
	// This prevents data loss if several reports arrive before the application reacts
	USBH_HID_FifoInit(&HID_Handle->fifo, fifo_buffer, sizeof(fifo_buffer));

	// Set the HID state to IDLE to enable polling and event callback processing
	HID_Handle->state = USBH_HID_IDLE;

	// Return success status after initialization
    return USBH_OK;
}



ThrottleData_t USBH_HID_GetThrottleData(uint8_t *report)
{
    ThrottleData_t data;

    // Extract 32 buttons from the first 4 bytes after the Report ID.
    // Each bit represents one button: 0 = not pressed, 1 = pressed.
    uint32_t buttonBits = report[1] | (report[2] << 8) | (report[3] << 16) | (report[4] << 24);
    for (int i = 0; i < 32; i++) {
        data.buttons[i] = (buttonBits >> i) & 0x01;
    }

    // Extract the hat switch value from the lower 4 bits of report[5].
    // Values 0–7 indicate direction, 8 means neutral (not pressed).
    data.hat_switch = report[5] & 0x0F;

    // Read X axis value (10 bits: report[6] bits 0–7, report[7] bits 0–1).
    data.x = ((uint16_t)report[6]) | (((uint16_t)report[7] & 0x03) << 8);

    // Read Y axis value (10 bits: report[8] bits 0–7, report[9] bits 0–1).
    data.y = ((uint16_t)report[8]) | (((uint16_t)report[9] & 0x03) << 8);

    // Read slider value (10 bits: report[10] bits 0–7, report[11] bits 0–1).
    data.slider = ((uint16_t)report[10]) | (((uint16_t)report[11] & 0x03) << 8);

    // Read Z axis value (14 bits: report[12] bits 0–7, report[13] bits 0–5).
    data.z = ((uint16_t)report[12]) | (((uint16_t)report[13] & 0x3F) << 8);

    // Read Rz axis value (14 bits: report[14] bits 0–7, report[15] bits 0–5).
    data.rz = ((uint16_t)report[14]) | (((uint16_t)report[15] & 0x3F) << 8);

    return data;
}


