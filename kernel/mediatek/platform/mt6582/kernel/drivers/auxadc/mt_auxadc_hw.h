#ifndef _MTK_ADC_HW_H
#define _MTK_ADC_HW_H

#include <mach/mt_reg_base.h>

#define AUXADC_CON0                     (AUXADC_BASE + 0x000)
#define AUXADC_CON1                     (AUXADC_BASE + 0x004)
#define AUXADC_CON2                     (AUXADC_BASE + 0x010)
#define AUXADC_DAT0                     (AUXADC_BASE + 0x014)
#define AUXADC_TP_CMD            (AUXADC_BASE + 0x05c)
#define AUXADC_TP_ADDR           (AUXADC_BASE + 0x060)
#define AUXADC_TP_CON0           (AUXADC_BASE + 0x064)
#define AUXADC_TP_DATA0          (AUXADC_BASE + 0x074)

#define PAD_AUX_XP				13
#define TP_CMD_ADDR_X			0x005

#define AUXADC_CON_RTP		(APMIXEDSYS_BASE + 0x404)

#endif   /*_MTK_ADC_HW_H*/

