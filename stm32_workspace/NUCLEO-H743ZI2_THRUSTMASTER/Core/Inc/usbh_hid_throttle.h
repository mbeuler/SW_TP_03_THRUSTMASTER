/*
 * usbh_hid_throttle.h
 *
 *  Created on: Nov 23, 2025
 *      Author: marcel.beuler
 */

#ifndef INC_USBH_HID_THROTTLE_H_
#define INC_USBH_HID_THROTTLE_H_

#include "usbh_core.h"
#include "usbh_hid.h"

#define THROTTLE_REPORT_SIZE 36           // Size of the HID report descriptor
                                          // (Report ID 1, including 10 vendor-specific bytes
                                          // and 10 constant bytes)

#define THROTTLE_REPORT_EFFECTIVE_SIZE 16 // Report ID 1, excluding 10 vendor-specific bytes
                                          // and 10 constant bytes

typedef struct {
    uint8_t buttons[32];   // 0 = not pressed, 1 = pressed
    uint8_t hat_switch;    // 4 bits (0–7 = direction, 8 = neutral position)
    uint16_t x;            // 10 bits (0–1023)
    uint16_t y;            // 10 bits (0–1023)
    uint16_t slider;       // 10 bits (0–1023)
    uint16_t z;            // 14 bits (0–16383)
    uint16_t rz;           // 14 bits (0–16383)
} ThrottleData_t;


USBH_StatusTypeDef USBH_HID_ThrottleInit(USBH_HandleTypeDef *phost);
ThrottleData_t USBH_HID_GetThrottleData(uint8_t *report);


#endif /* INC_USBH_HID_THROTTLE_H_ */
