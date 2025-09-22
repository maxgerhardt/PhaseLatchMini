#include "usbd_desc.h"
#include "usbd_core.h"
#include "usbd_conf.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h" // for CDC_IN_EP

#define USBD_VID     0x0483
#define USBD_PID     0x5741 /* changed to force driver rebind for CDC */
#define USBD_LANGID_STRING     1033
// Short strings only (keep minimal)
#define USBD_MANUFACTURER_STRING  (uint8_t*)"F103"
#define USBD_PRODUCT_FS_STRING    (uint8_t*)"CDC IQ STREAM"
#define USBD_SERIALNUMBER_STRING  (uint8_t*)"12345678"
#define USBD_CONFIGURATION_FS_STRING (uint8_t*)"CFG"
#define USBD_INTERFACE_FS_STRING     (uint8_t*)"IF"

static uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef FS_Desc = {
  USBD_FS_DeviceDescriptor,
  USBD_FS_LangIDStrDescriptor,
  USBD_FS_ManufacturerStrDescriptor,
  USBD_FS_ProductStrDescriptor,
  USBD_FS_SerialStrDescriptor,
  USBD_FS_ConfigStrDescriptor,
  USBD_FS_InterfaceStrDescriptor
};

__ALIGN_BEGIN static uint8_t hUSBDDeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
  0x12, USB_DESC_TYPE_DEVICE,
  0x00, 0x02,
  0x00, 0x00, 0x00,
  0x40,
  LOBYTE(USBD_VID), HIBYTE(USBD_VID),
  LOBYTE(USBD_PID), HIBYTE(USBD_PID),
  0x00, 0x02,
  1, 2, 3,
  1
};

__ALIGN_BEGIN static uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
  USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING,
  LOBYTE(USBD_LANGID_STRING), HIBYTE(USBD_LANGID_STRING)
};

static uint8_t* USBD_StringDescriptor(const uint8_t* str, uint16_t* length) {
  static uint8_t desc[64];
  uint8_t idx = 0;
  if (str) {
    uint8_t len = 0;
    while (str[len] && len < 31) len++;
    desc[idx++] = (len * 2) + 2;
    desc[idx++] = USB_DESC_TYPE_STRING;
    for (uint8_t i = 0; i < len; i++) {
      desc[idx++] = str[i];
      desc[idx++] = 0;
    }
  } else {
    desc[0] = 2;
    desc[1] = USB_DESC_TYPE_STRING;
  }
  *length = desc[0];
  return desc;
}

static uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed; *length = sizeof(hUSBDDeviceDesc);
  // No descriptor debug injection in minimal trimmed build
  return hUSBDDeviceDesc;
}
static uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed; *length =  sizeof(USBD_LangIDDesc); return USBD_LangIDDesc;
}
static uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  return USBD_StringDescriptor(USBD_MANUFACTURER_STRING, length);
}
static uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  return USBD_StringDescriptor(USBD_PRODUCT_FS_STRING, length);
}
static uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  return USBD_StringDescriptor(USBD_CONFIGURATION_FS_STRING, length);
}
static uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  return USBD_StringDescriptor(USBD_INTERFACE_FS_STRING, length);
}
static uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  return USBD_StringDescriptor(USBD_SERIALNUMBER_STRING, length);
}
