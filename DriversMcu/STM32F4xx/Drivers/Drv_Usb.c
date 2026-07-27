#include "drv_usb.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "AnoPTv8.h"

/*!< endpoint address */
#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x01
#define CDC_INT_EP 0x82

#define USBD_VID           0x0483
#define USBD_PID           0xA060
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

/*!< config descriptor size */
#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)

/*!< global descriptor */
static const uint8_t cdc_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, 0x02),
    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    12,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'A', 0x00,                  /* wcChar0 */
    'N', 0x00,                  /* wcChar1 */
    'O', 0x00,                  /* wcChar2 */
    'T', 0x00,                  /* wcChar3 */
    'C', 0x00,                  /* wcChar4 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    28,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'A', 0x00,                  /* wcChar0 */
    'n', 0x00,                  /* wcChar1 */
    'o', 0x00,                  /* wcChar2 */
    ' ', 0x00,                  /* wcChar3 */
    'C', 0x00,                  /* wcChar4 */
    'o', 0x00,                  /* wcChar5 */
    'r', 0x00,                  /* wcChar6 */
    'e', 0x00,                  /* wcChar7 */
    ' ', 0x00,                  /* wcChar8 */
    '4', 0x00,                  /* wcChar9 */
    '2', 0x00,                  /* wcChar10 */
    '9', 0x00,                  /* wcChar11 */
    ' ', 0x00,                  /* wcChar12 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    10,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    '2', 0x00,                  /* wcChar0 */
    '0', 0x00,                  /* wcChar1 */
    '2', 0x00,                  /* wcChar2 */
    '4', 0x00,                  /* wcChar3 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x02,
    0x02,
    0x01,
    0x40,
    0x01,
    0x00,
#endif
    0x00
};

#define HWCDCBUFLEN		128

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdcReadBuf[HWCDCBUFLEN];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdcSendBuf[HWCDCBUFLEN];

volatile bool ep_tx_busy_flag = false;

#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#endif

void usbd_configure_done_callback(void)
{
    ep_tx_busy_flag = false;
    /* setup first out ep read transfer */
    usbd_ep_start_read(CDC_OUT_EP, cdcReadBuf, HWCDCBUFLEN);
}

void usbd_cdc_acm_bulk_out(uint8_t ep, uint32_t nbytes)
{
    //USB_LOG_RAW("cdc%d out len:%d\r\n", ep, nbytes);
    for(int i=0; i<nbytes; i++)
    {
        AnoPTv8HwRecvByte(LT_USBCDC, cdcReadBuf[i]);
    }
    /* setup next out ep read transfer */
    usbd_ep_start_read(ep, cdcReadBuf, HWCDCBUFLEN);
}

void usbd_cdc_acm_bulk_in(uint8_t ep, uint32_t nbytes)
{
    //USB_LOG_RAW("cdc%d in len:%d\r\n", ep, nbytes);

    if ((nbytes % CDC_MAX_MPS) == 0 && nbytes) {
        /* send zlp */
        usbd_ep_start_write(ep, NULL, 0);
    } else {
        ep_tx_busy_flag = false;
    }
}

struct usbd_endpoint cdc_out_ep1 = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_bulk_out
};

struct usbd_endpoint cdc_in_ep1 = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_bulk_in
};

struct usbd_interface intf0;
struct usbd_interface intf1;

#include "stm32f4xx.h"
#include "Drv_Sys.h"

void usb_bsp_inti(void)
{
    /* Configure  PA10, PA11, PA12 as alternate OTG_FS                          */
    RCC->AHB1ENR    |=   RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER     =  (GPIOA->MODER  & ~(3  << 22)) | (2  << 22);
    GPIOA->OTYPER   &= ~(1 << 11);
    GPIOA->AFR[1]    =  (GPIOA->AFR[1] & ~(15 << 12)) | (10 << 12);
    GPIOA->OSPEEDR  |=  (3 << 22);
    GPIOA->PUPDR    &= ~(3 << 22);

    GPIOA->MODER     =  (GPIOA->MODER  & ~(3  << 24)) | (2  << 24);
    GPIOA->OTYPER   &= ~(1 << 12);
    GPIOA->AFR[1]    =  (GPIOA->AFR[1] & ~(15 << 16)) | (10 << 16);
    GPIOA->OSPEEDR  |=  (3 << 24);
    GPIOA->PUPDR    &= ~(3 << 24);

    RCC->AHB2ENR    |=  (1 <<  7);        /* Enable clock for OTG FS            */
    MyDelayMs    (10);            /* Wait ~10 ms                        */
    RCC->AHB2RSTR   |=  (1 <<  7);        /* Reset OTG FS clock                 */
    MyDelayMs    (10);            /* Wait ~10 ms                        */
    RCC->AHB2RSTR   &= ~(1 <<  7);
    MyDelayMs    (50);            /* Wait ~40 ms                        */


    NVIC_EnableIRQ   (OTG_FS_IRQn);        /* Enable OTG interrupt               */
    ((OTG_FS_TypeDef *) USB_BASE)->GAHBCFG    |=  1 | (1 << 7);     /* Enable interrupts                  */
}

void DrvUsbInit(void)
{
    usb_bsp_inti();

    usbd_desc_register(cdc_descriptor);

    usbd_add_interface(usbd_cdc_acm_init_intf(&intf0));
    usbd_add_interface(usbd_cdc_acm_init_intf(&intf1));
    usbd_add_endpoint(&cdc_out_ep1);
    usbd_add_endpoint(&cdc_in_ep1);

    usbd_initialize();
}

#define CDCTXBUFLEN		1024

uint8_t CdcTxDataBuf[CDCTXBUFLEN];
uint16_t CdnTxDataBufInIndex = 0;
uint16_t CdnTxDataBufOutIndex = 0;
void DrvUsbCdcAddTxData(const uint8_t * buf, uint16_t len)
{
    for(int i=0; i<len; i++)
    {
        CdcTxDataBuf[CdnTxDataBufInIndex++] = *(buf+i);
        if(CdnTxDataBufInIndex >= CDCTXBUFLEN)
            CdnTxDataBufInIndex = 0;
    }
}

void DrvUsbRunTask1MS(void)
{
    static uint8_t _cdcTxBustCnt = 0;

    if(CdnTxDataBufInIndex != CdnTxDataBufOutIndex)
    {
        uint16_t _dlen = 0;

        if(ep_tx_busy_flag == false)
        {
            _cdcTxBustCnt = 0;
            if(CdnTxDataBufInIndex > CdnTxDataBufOutIndex)
                _dlen = CdnTxDataBufInIndex - CdnTxDataBufOutIndex;
            else
                _dlen = CDCTXBUFLEN - CdnTxDataBufOutIndex + CdnTxDataBufInIndex;
            if(HWCDCBUFLEN < _dlen)
                _dlen = HWCDCBUFLEN;
            ep_tx_busy_flag = true;
            for(int i=0; i<_dlen; i++)
            {
                cdcSendBuf[i] = CdcTxDataBuf[CdnTxDataBufOutIndex++];
                if(CdnTxDataBufOutIndex >= CDCTXBUFLEN)
                    CdnTxDataBufOutIndex = 0;
            }
            usbd_ep_start_write(CDC_IN_EP, cdcSendBuf, _dlen);
        }
        else
        {
            _cdcTxBustCnt++;
            if(_cdcTxBustCnt >= 5)
            {
                _cdcTxBustCnt = 0;
                ep_tx_busy_flag = false;
            }
        }
    }
}



