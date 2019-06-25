#ifndef _MT_GPIO_H_
#define _MT_GPIO_H_

//mark for delete, need remove other driver build error
#include <linux/ioctl.h>
#include <linux/fs.h>

//FIX-ME: 
//#if (defined(CONFIG_MT6589_FPGA))
#if (defined(CONFIG_FPGA))
//#define CONFIG_MT_GPIO_FPGA_ENABLE
#include <mach/mt_gpio_fpga.h>
#else
// FIX-ME: marked for early porting
#include <cust_gpio_usage.h>
#include <mach/mt_gpio_base.h>
#include <mach/mt_gpio_affix.h>
#endif
#include <mach/mt_gpio_ext.h>

//#define MT_GPIO_BASE_START 0
//typedef enum GPIO_PIN
//{    
//    GPIO_UNSUPPORTED = -1,    
//	GPIO0 = MT_GPIO_BASE_START,
//    GPIO1  , GPIO2  , GPIO3  , GPIO4  , GPIO5  , GPIO6  , GPIO7  , 
//    GPIO8  , GPIO9  , GPIO10 , GPIO11 , GPIO12 , GPIO13 , GPIO14 , GPIO15 , 
//    GPIO16 , GPIO17 , GPIO18 , GPIO19 , GPIO20 , GPIO21 , GPIO22 , GPIO23 , 
//    GPIO24 , GPIO25 , GPIO26 , GPIO27 , GPIO28 , GPIO29 , GPIO30 , GPIO31 , 
//    GPIO32 , GPIO33 , GPIO34 , GPIO35 , GPIO36 , GPIO37 , GPIO38 , GPIO39 , 
//    GPIO40 , GPIO41 , GPIO42 , GPIO43 , GPIO44 , GPIO45 , GPIO46 , GPIO47 , 
//    GPIO48 , GPIO49 , GPIO50 , GPIO51 , GPIO52 , GPIO53 , GPIO54 , GPIO55 , 
//    GPIO56 , GPIO57 , GPIO58 , GPIO59 , GPIO60 , GPIO61 , GPIO62 , GPIO63 , 
//    GPIO64 , GPIO65 , GPIO66 , GPIO67 , GPIO68 , GPIO69 , GPIO70 , GPIO71 , 
//    GPIO72 , GPIO73 , GPIO74 , GPIO75 , GPIO76 , GPIO77 , GPIO78 , GPIO79 , 
//    GPIO80 , GPIO81 , GPIO82 , GPIO83 , GPIO84 , GPIO85 , GPIO86 , GPIO87 , 
//    GPIO88 , GPIO89 , GPIO90 , GPIO91 , GPIO92 , GPIO93 , GPIO94 , GPIO95 , 
//    GPIO96 , GPIO97 , GPIO98 , GPIO99 , GPIO100, GPIO101, GPIO102, GPIO103, 
//    GPIO104, GPIO105, GPIO106, GPIO107, GPIO108, GPIO109, GPIO110, GPIO111, 
//    GPIO112, GPIO113, GPIO114, GPIO115, GPIO116, GPIO117, GPIO118, GPIO119, 
//    GPIO120, GPIO121, GPIO122, GPIO123, GPIO124, GPIO125, GPIO126, GPIO127, 
//    GPIO128, GPIO129, GPIO130, GPIO131, GPIO132, GPIO133, GPIO134, GPIO135, 
//    GPIO136, GPIO137, GPIO138, GPIO139, GPIO140, GPIO141, GPIO142, GPIO143, 
//    GPIO144, GPIO145, GPIO146, GPIO147, GPIO148, GPIO149, GPIO150, GPIO151, 
//    GPIO152, GPIO153, GPIO154, GPIO155, GPIO156, GPIO157, GPIO158, GPIO159, 
//    GPIO160, GPIO161, GPIO162, GPIO163, GPIO164, GPIO165, GPIO166, GPIO167, 
//    GPIO168, GPIO169, GPIO170, GPIO171, GPIO172, GPIO173, GPIO174, GPIO175, 
//    GPIO176, GPIO177, GPIO178, GPIO179, GPIO180, GPIO181, GPIO182, GPIO183, 
//    GPIO184, GPIO185, GPIO186, GPIO187, GPIO188, GPIO189, GPIO190, GPIO191, 
//    GPIO192, GPIO193, GPIO194, GPIO195, GPIO196, GPIO197, GPIO198, GPIO199, 
//    GPIO200, GPIO201, GPIO202, GPIO203, GPIO204, GPIO205, GPIO206, GPIO207, 
//    GPIO208, GPIO209, GPIO210, GPIO211, GPIO212, GPIO213, GPIO214, GPIO215, 
//    GPIO216, GPIO217, GPIO218, GPIO219, GPIO220, GPIO221, GPIO222, GPIO223, 
//    GPIO224, GPIO225, GPIO226, GPIO227, GPIO228, GPIO229, GPIO230, GPIO231,
//    MT_GPIO_BASE_MAX
//}GPIO_PIN;    
//#define MT_GPIO_EXT_START 232
//typedef enum GPIO_PIN_EXT
//{    
//    GPIO232 = MT_GPIO_EXT_START,    
//    GPIO233, GPIO234, GPIO235, GPIO236, GPIO237, GPIO238, GPIO239,	
//    GPIO240, GPIO241, GPIO242, GPIO243, GPIO244, GPIO245, GPIO246, GPIO247,	
//    GPIO248, GPIO249, GPIO250, GPIO251, GPIO252, GPIO253, GPIO254, GPIO255,	
//    GPIO256, GPIO257, GPIO258, GPIO259, GPIO260, GPIO261, GPIO262, GPIO263,	
//    GPIO264, GPIO265, GPIO266, GPIO267, GPIO268, GPIO269, GPIO270, GPIO271,	
//    GPIO272, GPIO273, GPIO274, GPIO275, GPIO276, GPIO277, GPIO278, GPIO279,	
//    GPIO280
//}GPIO_PIN_EXT;    
//
//typedef enum GPIO_PIN_EXT1
//{    
//    GPIOEXT0 = MT_GPIO_EXT_START,    
//    GPIOEXT1, GPIOEXT2, GPIOEXT3, GPIOEXT4, GPIOEXT5, GPIOEXT6, GPIOEXT7,	
//    GPIOEXT8 , GPIOEXT9 , GPIOEXT10, GPIOEXT11, GPIOEXT12, GPIOEXT13, GPIOEXT14, GPIOEXT15,	
//    GPIOEXT16, GPIOEXT17, GPIOEXT18, GPIOEXT19, GPIOEXT20, GPIOEXT21, GPIOEXT22, GPIOEXT23,	
//    GPIOEXT24, GPIOEXT25, GPIOEXT26, GPIOEXT27, GPIOEXT28, GPIOEXT29, GPIOEXT30, GPIOEXT31,	
//    GPIOEXT32, GPIOEXT33, GPIOEXT34, GPIOEXT35, GPIOEXT36, GPIOEXT37, GPIOEXT38, GPIOEXT39,	
//    GPIOEXT40, GPIOEXT41, GPIOEXT42, GPIOEXT43, GPIOEXT44, GPIOEXT45, GPIOEXT46, GPIOEXT47,	
//    GPIOEXT48, MT_GPIO_EXT_MAX
//}GPIO_PIN_EXT1;    
//#define MT_GPIO_MAX_PIN MT_GPIO_EXT_MAX
/******************************************************************************
* Enumeration for GPIO pin
******************************************************************************/
/* GPIO MODE CONTROL VALUE*/
typedef enum {
    GPIO_MODE_UNSUPPORTED = -1,
    GPIO_MODE_GPIO  = 0,
    GPIO_MODE_00    = 0,
    GPIO_MODE_01    = 1,
    GPIO_MODE_02    = 2,
    GPIO_MODE_03    = 3,
    GPIO_MODE_04    = 4,
    GPIO_MODE_05    = 5,
    GPIO_MODE_06    = 6,
    GPIO_MODE_07    = 7,

    GPIO_MODE_MAX,
    GPIO_MODE_DEFAULT = GPIO_MODE_01,
} GPIO_MODE;
/*----------------------------------------------------------------------------*/
/* GPIO DIRECTION */
typedef enum {
    GPIO_DIR_UNSUPPORTED = -1,
    GPIO_DIR_IN     = 0,
    GPIO_DIR_OUT    = 1,

    GPIO_DIR_MAX,
    GPIO_DIR_DEFAULT = GPIO_DIR_IN,
} GPIO_DIR;
/*----------------------------------------------------------------------------*/
/* GPIO PULL ENABLE*/
typedef enum {
    GPIO_PULL_EN_UNSUPPORTED = -1,
    GPIO_PULL_DISABLE = 0,
    GPIO_PULL_ENABLE  = 1,

    GPIO_PULL_EN_MAX,
    GPIO_PULL_EN_DEFAULT = GPIO_PULL_ENABLE,
} GPIO_PULL_EN;
/*----------------------------------------------------------------------------*/
/* GPIO IES*/
typedef enum {
    GPIO_IES_UNSUPPORTED = -1,
    GPIO_IES_DISABLE = 0,
    GPIO_IES_ENABLE  = 1,

    GPIO_IES_MAX,
    GPIO_IES_DEFAULT = GPIO_IES_ENABLE,
} GPIO_IES;
/*----------------------------------------------------------------------------*/
/* GPIO PULL-UP/PULL-DOWN*/
typedef enum {
    GPIO_PULL_UNSUPPORTED = -1,
    GPIO_PULL_DOWN  = 0,
    GPIO_PULL_UP    = 1,

    GPIO_PULL_MAX,
    GPIO_PULL_DEFAULT = GPIO_PULL_DOWN
} GPIO_PULL;
/*----------------------------------------------------------------------------*/
/* GPIO INVERSION */
typedef enum {
    GPIO_DATA_INV_UNSUPPORTED = -1,
    GPIO_DATA_UNINV = 0,
    GPIO_DATA_INV   = 1,

    GPIO_DATA_INV_MAX,
    GPIO_DATA_INV_DEFAULT = GPIO_DATA_UNINV
} GPIO_INVERSION;
/*----------------------------------------------------------------------------*/
/* GPIO OUTPUT */
typedef enum {
    GPIO_OUT_UNSUPPORTED = -1,
    GPIO_OUT_ZERO = 0,
    GPIO_OUT_ONE  = 1,

    GPIO_OUT_MAX,
    GPIO_OUT_DEFAULT = GPIO_OUT_ZERO,
    GPIO_DATA_OUT_DEFAULT = GPIO_OUT_ZERO,  /*compatible with DCT*/
} GPIO_OUT;
/*----------------------------------------------------------------------------*/
/* GPIO INPUT */
typedef enum {
    GPIO_IN_UNSUPPORTED = -1,
    GPIO_IN_ZERO = 0,
    GPIO_IN_ONE  = 1,

    GPIO_IN_MAX,
} GPIO_IN;

/******************************************************************************
* GPIO Driver interface 
******************************************************************************/
/*direction*/
int mt_set_gpio_dir(unsigned long pin, unsigned long dir);
int mt_get_gpio_dir(unsigned long pin);

/*pull enable*/
int mt_set_gpio_pull_enable(unsigned long pin, unsigned long enable);
int mt_get_gpio_pull_enable(unsigned long pin);
/*IES*/
int mt_set_gpio_ies(unsigned long pin, unsigned long enable);
int mt_get_gpio_ies(unsigned long pin);
/*pull select*/
int mt_set_gpio_pull_select(unsigned long pin, unsigned long select);    
int mt_get_gpio_pull_select(unsigned long pin);

/*data inversion*/
int mt_set_gpio_inversion(unsigned long pin, unsigned long enable);
int mt_get_gpio_inversion(unsigned long pin);

/*input/output*/
int mt_set_gpio_out(unsigned long pin, unsigned long output);
int mt_get_gpio_out(unsigned long pin);
int mt_get_gpio_in(unsigned long pin);

/*mode control*/
int mt_set_gpio_mode(unsigned long pin, unsigned long mode);
int mt_get_gpio_mode(unsigned long pin);

/*misc functions for protect GPIO*/
//void mt_gpio_dump(GPIO_REGS *regs,GPIOEXT_REGS *regs_ext);
void gpio_dump_regs(void);

#endif //_MT_GPIO_H_
