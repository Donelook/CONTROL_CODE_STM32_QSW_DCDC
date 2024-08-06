#include "usbd_cdc_if.h"

// Function to send a string over USB CDC
void USB_SendString(char* message) {
    uint8_t result = CDC_Transmit_FS((uint8_t*)message, strlen(message));
    
}
