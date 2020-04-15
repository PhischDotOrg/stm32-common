/*-
 * $Copyright$
-*/

#ifndef _USBTYPES_HPP_6F681E9D_BBA0_4F3F_A86C_C5464EF17DB6
#define _USBTYPES_HPP_6F681E9D_BBA0_4F3F_A86C_C5464EF17DB6

#include <stdint.h>

namespace usb {

typedef struct UsbDeviceDescriptor_s UsbDeviceDescriptor_t;
typedef struct UsbDeviceQualifierDescriptor_s UsbDeviceQualifierDescriptor_t;
typedef struct UsbConfigurationDescriptor_s UsbConfigurationDescriptor_t;
typedef struct UsbInterfaceDescriptor_s UsbInterfaceDescriptor_t;
typedef struct UsbEndpointDescriptor_s UsbEndpointDescriptor_t;

/*******************************************************************************
 *
 ******************************************************************************/
typedef enum UsbDescriptorTypeId_e {
    e_Device            = 0x01,
    e_Configuration     = 0x02,
    e_String            = 0x03,
    e_Interface         = 0x04,
    e_Endpoint          = 0x05,
    e_DeviceQualifier   = 0x06,
    e_OtherSpeedConfig  = 0x07
} UsbDescriptorTypeId_t;

/*******************************************************************************
 *
 ******************************************************************************/
struct UsbEndpointDescriptor_s {
    uint8_t     m_bLength;
    uint8_t     m_bDescriptorType;
    uint8_t     m_bEndpointAddress;
    uint8_t     m_bmAttributes;
    struct {
      uint8_t   m_loByte;
      uint8_t   m_hiByte;
    }           m_wMaxPacketSize;
    uint8_t     m_bInterval;
} __attribute__((packed));

/*******************************************************************************
 *
 ******************************************************************************/
typedef enum UsbInterfaceClass_e {
    e_UsbInterface_BaseClass                = 0x00,
    e_UsbInterface_AudioDevice              = 0x01,
    e_UsbInterface_CommunicationDeviceClass = 0x02,
    e_UsbInterface_HumanInterfaceDevice     = 0x03,
    e_UsbInterface_PhysicalDevice           = 0x05,
    e_UsbInterface_StillImagingDevice       = 0x06,
    e_UsbInterface_Printer                  = 0x07,
    e_UsbInterface_MassStorageDevice        = 0x08,
    e_UsbInterface_Hub                      = 0x09,
    e_UsbInterface_CdcData                  = 0x0A,
    e_UsbInterface_SmartCard                = 0x0B,
    e_UsbInterface_ContentSecurityDevice    = 0x0D,
    e_UsbInterface_VideoDevice              = 0x0E,
    e_UsbInterface_PersonalHealthcareDevice = 0x0F,
    e_UsbInterface_AudioVideoDevice         = 0x10,
    e_UsbInterface_TypeC_Bridge             = 0x12,
    e_UsbInterface_DiagnosticDevice         = 0xDC,
    e_UsbInterface_WirelessController       = 0xE0,
    e_UsbInterface_Misc                     = 0xE0,
    e_UsbInterface_ApplicationSpecific      = 0xFE,
    e_UsbInterface_VendorSpecific           = 0xFF,
} UsbInterfaceClass_t;

/*******************************************************************************
 *
 ******************************************************************************/
struct UsbInterfaceDescriptor_s {
    uint8_t                         m_bLength;
    uint8_t                         m_bDescriptorType;
    uint8_t                         m_bInterfaceNumber;
    uint8_t                         m_bAlternateSetting;
    uint8_t                         m_bNumEndpoints;
    UsbInterfaceClass_t             m_bInterfaceClass;
    uint8_t                         m_bInterfaceSubClass;
    uint8_t                         m_bInterfaceProtocol;
    uint8_t                         m_iInterface;
} __attribute__((packed));

/*******************************************************************************
 *
 ******************************************************************************/
struct UsbConfigurationDescriptor_s {
    uint8_t                         m_bLength;
    uint8_t                         m_bDescriptorType;
    struct {
      uint8_t                       m_loByte;
      uint8_t                       m_hiByte;
    }  m_wTotalLength;
    uint8_t                         m_bNumInterfaces;
    uint8_t                         m_bConfigurationValue;
    uint8_t                         m_iConfiguration;
    uint8_t                         m_bmAttributes;
    uint8_t                         m_bMaxPower;
} __attribute__((packed));

/*******************************************************************************
 *
 ******************************************************************************/
struct UsbDeviceDescriptor_s {
    uint8_t     m_bLength;
    uint8_t     m_bDescriptorType;
    uint8_t     m_bcdUsb[2];
    uint8_t     m_bDeviceClass;
    uint8_t     m_bDeviceSubClass;
    uint8_t     m_bDeviceProtocol;
    uint8_t     m_bMaxPacketSize0;
    uint8_t     m_idVendor[2];
    uint8_t     m_idProduct[2];
    uint8_t     m_bcdDevice[2];
    uint8_t     m_iManufacturer;
    uint8_t     m_iProduct;
    uint8_t     m_iSerialNumber;
    uint8_t     m_bNumConfigurations;
} __attribute__((packed));

/*******************************************************************************
 *
 ******************************************************************************/
struct UsbDeviceQualifierDescriptor_s {
    uint8_t     m_bLength;
    uint8_t     m_bDescriptorType;
    uint8_t     m_bcdUsb[2];
    uint8_t     m_bDeviceClass;
    uint8_t     m_bDeviceSubClass;
    uint8_t     m_bDeviceProtocol;
    uint8_t     m_bMaxPacketSize0;
    uint8_t     m_bNumConfigurations;
    uint8_t     m_bReserved;
    
    UsbDeviceQualifierDescriptor_s(const UsbDeviceDescriptor_t &p_deviceDescriptor)
      : m_bLength(p_deviceDescriptor.m_bLength),
      m_bDescriptorType(e_DeviceQualifier),
      m_bDeviceClass(p_deviceDescriptor.m_bDeviceClass),
      m_bDeviceSubClass(p_deviceDescriptor.m_bDeviceSubClass),
      m_bDeviceProtocol(p_deviceDescriptor.m_bDeviceProtocol),
      m_bMaxPacketSize0(p_deviceDescriptor.m_bMaxPacketSize0),
      m_bNumConfigurations(p_deviceDescriptor.m_bNumConfigurations),
      m_bReserved(0) {
        m_bcdUsb[0] = p_deviceDescriptor.m_bcdUsb[0];
        m_bcdUsb[1] = p_deviceDescriptor.m_bcdUsb[1];
    };

private:
    UsbDeviceQualifierDescriptor_s();
} __attribute__((packed));

/*******************************************************************************
 *
 ******************************************************************************/
typedef enum UsbStringDescriptorId_e {
    e_StrDesc_LanguageId    = 0x00,
    e_StrDesc_Manufacturer  = 0x01,
    e_StrDesc_Product       = 0x02,
    e_StrDesc_SerialNumber  = 0x03,
    e_StrDesc_Configuration = 0x04,
    e_StrDesc_Interface     = 0x05,
    e_StrDesc_Max           = 0x06
} UsbStringDescriptorId_t;

/*******************************************************************************
 *
 ******************************************************************************/
template<typename CharT>
struct UsbStringDescriptorT {
private:
    static unsigned constexpr len(const CharT * const p_string) {
        return *p_string == '\0' ? 0 : 1 + UsbStringDescriptorT<CharT>::len(p_string + 1);
    }

    UsbStringDescriptorT(void);

public:
    const CharT * const m_string;
    const uint8_t       m_length;

    UsbStringDescriptorT(const CharT * const p_string) : m_string(p_string), m_length(len(m_string)) { };
} __attribute__((packed));

typedef UsbStringDescriptorT<char> UsbStringDescriptor;
typedef UsbStringDescriptor UsbStringDescriptor_t;

/*******************************************************************************
 *
 ******************************************************************************/
struct UsbLangId_s {
    uint8_t     m_loByte;
    uint8_t     m_hiByte;

    UsbLangId_s(const uint16_t p_langId) : m_loByte(p_langId & 0xFF),
      m_hiByte((p_langId >> 8) & 0xFF) {

    }
    
    ~UsbLangId_s() {
        
    }

private:
    UsbLangId_s(void);
} __attribute__((packed));

typedef struct UsbLangId_s UsbLangId_t;

/*******************************************************************************
 *
 ******************************************************************************/
struct UsbLangIdStringDescriptor_s {
private:
    static unsigned constexpr len(const UsbLangId_t * const p_langIds) {
        return ((p_langIds->m_hiByte == 0) && (p_langIds->m_loByte == 0)) ? 0 : 1 + len(p_langIds + 1);
    }
    UsbLangIdStringDescriptor_s(void);

public:
    const uint8_t               m_numLanguages;
    const UsbLangId_t * const   m_langIds;

    UsbLangIdStringDescriptor_s(const UsbLangId_t * p_langIds) : m_numLanguages(len(p_langIds)), m_langIds(p_langIds) {
        
    }
} __attribute__((packed));

typedef struct UsbLangIdStringDescriptor_s UsbLangIdStringDescriptor_t;

/*******************************************************************************
 *
 ******************************************************************************/
struct UsbStringDescriptorTable_s {
    const UsbLangIdStringDescriptor_t   m_languageIds;
    const UsbStringDescriptor_t         m_manufacturer;
    const UsbStringDescriptor_t         m_product;
    const UsbStringDescriptor_t         m_serialNumber;
    const UsbStringDescriptor_t         m_configuration;
    const UsbStringDescriptor_t         m_interface;
} __attribute__((packed));

typedef struct UsbStringDescriptorTable_s UsbStringDescriptorTable_t;

/*******************************************************************************
 *
 ******************************************************************************/
union UsbStringDescriptors_u {
    UsbStringDescriptorTable_t      m_stringDescriptorTable;
    UsbStringDescriptor_t           m_stringDescriptors[e_StrDesc_Max];
} __attribute__((packed));

typedef union UsbStringDescriptors_u UsbStringDescriptors_t;
/*******************************************************************************
 *
 ******************************************************************************/
} /* namespace usb */

#endif /* _USBTYPES_HPP_6F681E9D_BBA0_4F3F_A86C_C5464EF17DB6 */
