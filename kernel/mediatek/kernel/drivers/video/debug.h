/*
 * File: drivers/video/omap_new/debug.c
 *
 * Debug support for the omapfb driver
 *
 * Copyright (C) 2004 Nokia Corporation
 * Author: Imre Deak <imre.deak@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef __MTKFB_DEBUG_H
#define __MTKFB_DEBUG_H

void DBG_Init(void);
void DBG_Deinit(void);

void DBG_OnTriggerLcd(void);
void DBG_OnTeDelayDone(void);
void DBG_OnLcdDone(void);
void ConfigPara_Init(void);
void ConfigPara_Deinit(void);
int fb_config_execute_cmd(void);
int fbconfig_get_esd_check_exec(void);


#include <linux/mmprofile.h>
extern struct MTKFB_MMP_Events_t
{
    MMP_Event MTKFB;
    MMP_Event PanDisplay;
    MMP_Event SetOverlayLayer;
    MMP_Event SetOverlayLayers;
    MMP_Event TrigOverlayOut;
    MMP_Event UpdateScreenImpl;
    MMP_Event VSync;
    MMP_Event UpdateConfig;
    MMP_Event ConfigOVL;
    MMP_Event ConfigAAL;
    MMP_Event ConfigMemOut;
    MMP_Event ScreenUpdate;
    MMP_Event CaptureFramebuffer;
    MMP_Event RegUpdate;
    MMP_Event EarlySuspend;
    MMP_Event DispDone;
    MMP_Event DSICmd;
    MMP_Event DSIIRQ;
    MMP_Event EsdCheck;
    MMP_Event WaitVSync;
    MMP_Event LayerDump;
    MMP_Event Layer[4];
    MMP_Event OvlDump;
    MMP_Event FBDump;
    MMP_Event DSIRead;
    MMP_Event GetLayerInfo;
    MMP_Event LayerInfo[4];
    MMP_Event IOCtrl;
    MMP_Event Debug;
} MTKFB_MMP_Events;

#ifdef MTKFB_DBG
#include "disp_drv_log.h"

#define DBG_BUF_SIZE		    2048
#define MAX_DBG_INDENT_LEVEL	5
#define DBG_INDENT_SIZE		    3
#define MAX_DBG_MESSAGES	    0

static int dbg_indent;
static int dbg_cnt;
static char dbg_buf[DBG_BUF_SIZE];
static spinlock_t dbg_spinlock = SPIN_LOCK_UNLOCKED;

static inline void dbg_print(int level, const char *fmt, ...)
{
	if (level <= MTKFB_DBG) {
		if (!MAX_DBG_MESSAGES || dbg_cnt < MAX_DBG_MESSAGES) {
			va_list args;
			int	ind = dbg_indent;
			unsigned long flags;

			spin_lock_irqsave(&dbg_spinlock, flags);
			dbg_cnt++;
			if (ind > MAX_DBG_INDENT_LEVEL)
				ind = MAX_DBG_INDENT_LEVEL;

			DISP_LOG_PRINT(ANDROID_LOG_INFO, "DBG", "%*s", ind * DBG_INDENT_SIZE, "");
			va_start(args, fmt);
			vsnprintf(dbg_buf, sizeof(dbg_buf), fmt, args);
			DISP_LOG_PRINT(ANDROID_LOG_INFO, "DBG", dbg_buf);
			va_end(args);
			spin_unlock_irqrestore(&dbg_spinlock, flags);
		}
	}
}

#define DBGPRINT	dbg_print

#define DBGENTER(level)	do { \
		dbg_print(level, "%s: Enter\n", __FUNCTION__); \
		dbg_indent++; \
	} while (0)

#define DBGLEAVE(level)	do { \
		dbg_indent--; \
		dbg_print(level, "%s: Leave\n", __FUNCTION__); \
	} while (0)

// Debug Macros

#define MTKFB_DBG_EVT_NONE    0x00000000
#define MTKFB_DBG_EVT_FUNC    0x00000001  /* Function Entry     */
#define MTKFB_DBG_EVT_ARGU    0x00000002  /* Function Arguments */
#define MTKFB_DBG_EVT_INFO    0x00000003  /* Information        */

#define MTKFB_DBG_EVT_MASK    (MTKFB_DBG_EVT_NONE)

#define MSG(evt, fmt, args...)                              \
    do {                                                    \
        if ((MTKFB_DBG_EVT_##evt) & MTKFB_DBG_EVT_MASK) {   \
            DISP_LOG_PRINT(ANDROID_LOG_INFO, "DBG", fmt, ##args);                            \
        }                                                   \
    } while (0)

#define MSG_FUNC_ENTER(f)   MSG(FUNC, "<FB_ENTER>: %s\n", __FUNCTION__)
#define MSG_FUNC_LEAVE(f)   MSG(FUNC, "<FB_LEAVE>: %s\n", __FUNCTION__)


#else	/* MTKFB_DBG */

#define DBGPRINT(level, format, ...)
#define DBGENTER(level)
#define DBGLEAVE(level)

// Debug Macros

#define MSG(evt, fmt, args...)
#define MSG_FUNC_ENTER()
#define MSG_FUNC_LEAVE()

#endif	/* MTKFB_DBG */
//*****************debug for fbconfig tool in kernel part*************//
#define MAX_INSTRUCTION 32

typedef enum
{
	RECORD_CMD = 0,
	RECORD_MS = 1,
	RECORD_PIN_SET	= 2,	
} RECORD_TYPE;

typedef struct CONFIG_RECORD{
    struct CONFIG_RECORD * next;
    RECORD_TYPE type;//msleep;cmd;setpin;resetpin.
    int ins_num;
    int ins_array[MAX_INSTRUCTION];
}CONFIG_RECORD;


typedef enum
{
	HS_PRPR = 0,
	HS_ZERO = 1,
	HS_TRAIL= 2,
	TA_GO= 3,
	TA_SURE= 4,
	TA_GET= 5,
	DA_HS_EXIT= 6,
	CLK_ZERO= 7,
	CLK_TRAIL= 8,
	CONT_DET= 9,
	CLK_HS_PRPR= 10,
	CLK_HS_POST= 11,
	CLK_HS_EXIT= 12,
	HPW= 13,
	HFP= 14,
	HBP= 15,
	VPW= 16,
	VFP= 17,
	VBP= 18,
	MAX= 0XFF,	
}MIPI_SETTING_TYPE;	

typedef struct MIPI_TIMING{     
    MIPI_SETTING_TYPE type;
	unsigned int value;
}MIPI_TIMING;

typedef struct FBCONFIG_LAYER_INFO{     
    int layer_enable[3]; //layer id :0 1 2 3
	unsigned int layer_size[3] ;
}FBCONFIG_LAYER_INFO;

typedef struct ESD_PARA{     
    int addr;
	int para_num;
	char * esd_ret_buffer;
}ESD_PARA;


#endif /* __MTKFB_DEBUG_H */
