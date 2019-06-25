#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/autoconf.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/param.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/xlog.h>
#include <linux/aee.h>

#include <asm/io.h>


#include <mach/irqs.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_irq.h>
#include <mach/irqs.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_irq.h>
#include <mach/sync_write.h>
#include <mach/mt_smi.h>


#include "ddp_cmdq.h"
#include "ddp_drv.h"
#include "ddp_reg.h"
#include "ddp_path.h"

extern unsigned long * cmdq_pBuffer;
extern wait_queue_head_t cmq_wait_queue[CMDQ_THREAD_NUM];
//extern unsigned char cmq_status[CMDQ_THREAD_NUM];
extern unsigned int dbg_log;

cmdq_buff_t cmdqBufTbl[CMDQ_BUFFER_NUM];

int taskIDStatusTbl[MAX_CMDQ_TASK_ID];

task_resource_t taskResOccuTbl[MAX_CMDQ_TASK_ID]; //0~255

unsigned int hwResTbl = 0; //0~20 bit 0-> usable HW, 1-> in use
unsigned int cmdqThreadResTbl[CMDQ_THREAD_NUM]; 

unsigned int cmdqThreadTaskList_R[CMDQ_THREAD_NUM]; //Read pointer for each thread
unsigned int cmdqThreadTaskList_W[CMDQ_THREAD_NUM];//Write pointer for each thread
int cmdqThreadTaskList[CMDQ_THREAD_NUM][CMDQ_THREAD_LIST_LENGTH]; //each Thread's Current Job 

//Add for ISR loss problem
int cmdqThreadTaskNumber[CMDQ_THREAD_NUM]; //Record Job Sequence
int cmdqThreadFirstJobID[CMDQ_THREAD_NUM]; //Record First Task ID
long cmdqThreadEofCnt[MAX_CMDQ_TASK_ID]; //Record / Predict CMDQ EOF cnt


int rdma_status0, rdma_status1, rdma_status2, rdma_status4, rdma_status6, rdma_status26;
int mutex_fail_cnt = 0;
int rdma_fail_cnt = 0;
int wrot_fail_cnt = 0;
int mdp_reset_cnt = 0;
int timeout_cnt = 0;
int total_exec_cnt = 0;
int wait_queue_timeout_cnt = 0;
int cmdq_done_mdp_busy_cnt = 0;
int cmdq_isr_fail_cnt = 0;
int mdp_busy_long_cnt = 0;
int mmsys_cg_con0, disp_mutex_insta, disp_reg_commit, cmdq_inst0, cmdq_inst1, cmdq_inst2, cmdq_inst3, cmdq_pc, cmdq_end, cmdq_wait_event;
int rdma_reg[15];
int rsz0_reg[6];
int rsz1_reg[6];
int wdma_reg[8];
int wrot_reg[15];
struct timeval frame_interval;


spinlock_t gCmdqMgrLock;

#define CMDQ_WRN(string, args...) if(dbg_log) printk("[CMDQ]"string,##args)
//#define CMDQ_MSG(string, args...) printk(string,##args)
#define CMDQ_ERR(string, args...) if(1) printk("[CMDQ State]"string,##args)
#define CMDQ_MSG(string, args...) if(dbg_log) printk(string,##args)
//#define CMDQ_ERR(string, args...) if(dbg_log) printk(string,##args)
#define CMDQ_IRQ(string, args...) if(dbg_log) printk("[CMDQ]"string,##args)

#define CMDQ_MDP_AEE(string, args...) do{\
	xlog_printk(ANDROID_LOG_ERROR,  "MDP", "error: "string, ##args); \
	aee_kernel_warning("MDP", "error: "string, ##args);  \
}while(0)

#define CMDQ_ISP_AEE(string, args...) do{\
	xlog_printk(ANDROID_LOG_ERROR,  "ISP", "error: "string, ##args); \
	aee_kernel_warning("ISP", "error: "string, ##args);  \
}while(0)

typedef struct {
    int moduleType[cbMAX];
    CMDQ_TIMEOUT_PTR cmdqTimeout_cb[cbMAX];
    CMDQ_RESET_PTR cmdqReset_cb[cbMAX];
} CMDQ_CONFIG_CB_ARRAY;


CMDQ_CONFIG_CB_ARRAY g_CMDQ_CB_Array = { {cbMDP, cbISP}, {NULL, NULL}, {NULL, NULL}};//if cbMAX == 2


void cmdqForceFreeAll(int cmdqThread);
void cmdqForceFree_SW(int taskID);

extern void smi_dumpDebugMsg(void);

int MDPTimeOutDump(int params)
{
    printk("\n\n\n MDP cmdqTimeout_cb Test %d\n\n\n", params);
    return 0;
}

int MDPResetProcess(int params)
{
    printk("\n\n\n MDP cmdqReset_cb Test %d\n\n\n", params);
    return 0;
}



void dumpDebugInfo(void)
{

    int i = 0;
    CMDQ_ERR("\n\n\ncmdqTaskAssigned R: %d W: %d \n\n\n",cmdqThreadTaskList_R[0], cmdqThreadTaskList_W[0]);

    for(i=0;i<CMDQ_BUFFER_NUM;i++)
    {
        CMDQ_ERR("cmdqBufTbl %x : [%x %lx %lx] \n",i ,cmdqBufTbl[i].Owner, cmdqBufTbl[i].VA, cmdqBufTbl[i].MVA);
    }
}

bool checkMdpEngineStatus(unsigned int engineFlag)
{
//    unsigned int value;

    if (engineFlag & (0x1 << tIMGI))
    {
        if(clock_is_on(MT_CG_IMAGE_CAM_CAM))
        {
//            value = DISP_REG_GET(0xF5004160);
            DISP_REG_SET(0xF5004160, 0x06000);
        
            if (0x11 != (DISP_REG_GET(0xF5004164) & 0x11))
            {
//                pr_emerg("ISP engine is busy 0x%8x, 0x%8x\n", DISP_REG_GET(0xF5004160), DISP_REG_GET(0xF5004164));
                return true;
            }
        }
    }

    if (engineFlag & (0x1 << tRDMA0))
    {
        if(clock_is_on(MT_CG_DISP0_MDP_RDMA))
        {
            if (0x100 != (DISP_REG_GET(0xF4001408) & 0x7FF00))
            {
                //pr_emerg("RDMA engine is busy: %d\n", DISP_REG_GET(0xF4001408));
                return true;
            }
        }
    }

    if (engineFlag & (0x1 << tWROT))
    {
        if(clock_is_on(MT_CG_DISP0_MDP_WROT))
        {
            DISP_REG_SET(0xF4005018, 0xB00);
            if (0x0 != (DISP_REG_GET(0xF40050D0) & 0x1F))
            {
                //pr_emerg("WROT engine is busy %d\n", DISP_REG_GET(0xF40050D0));
                return true;
            }
        }
    }

    if (engineFlag & (0x1 << tWDMA1))
    {
        if(clock_is_on(MT_CG_DISP0_MDP_WDMA))
        {
            if (0x1 != (DISP_REG_GET(0xF40040A0) & 0x3FF))
            {
                //pr_emerg("WDMA1 engine is busy %d\n", DISP_REG_GET(0xF40040A0));
                return true;
            }
        }
    }

    return false;
}


void resetMdpEngine(unsigned int engineFlag)
{
    int loop_count;
//    int reg_val;
#if 1
    if (engineFlag & (0x01 << tIMGI))
    {
        printk("Reset ISP Pass2 start\n");
#if 0
        if(clock_is_on(MT_CG_IMAGE_CAM_CAM))
        {
            // Disable MDP Crop
            reg_val = DISP_REG_GET(0xF5004110);
            DISP_REG_SET(0xF5004110, (reg_val & ~0x08000));

            // Clear UV resampler
            DISP_REG_SET(0xF500408C, 0x00800000);

            DISP_REG_SET(0xF5004160, 0x06000);
        
            loop_count = 0;
            while(loop_count <= 50000)
            {
                if (0x11 == (DISP_REG_GET(0xF5004164) & 0x11))
                    break;

                loop_count++;
            }

            if (loop_count > 50000)
            {
                printk(KERN_DEBUG "Reset ISP failed\n");
            }

            // Set UV resampler
            DISP_REG_SET(0xF500408C, 0x00000000);
        }        
#endif        
        if(g_CMDQ_CB_Array.cmdqReset_cb[cbISP]!=NULL)
            g_CMDQ_CB_Array.cmdqReset_cb[cbISP](0);

        printk("Reset ISP Pass2 end\n");
    }
#endif // 0

    if (engineFlag & (0x1 << tRDMA0))
    {
        if(clock_is_on(MT_CG_DISP0_MDP_RDMA))
        {
            DISP_REG_SET(0xF4001008, 0x1);
        
            loop_count = 0;
            while(loop_count <= 50000)
            {
                if (0x100 == (DISP_REG_GET(0xF4001408) & 0x7FF00))
                    break;

                loop_count++;
            }

            if (loop_count > 50000)
            {
                printk(KERN_DEBUG "Reset RDMA failed\n");
            }

            DISP_REG_SET(0xF4001008, 0x0);
        }
    }

    if (engineFlag & (0x1 << tSCL0))
    {
        if(clock_is_on(MT_CG_DISP0_MDP_RSZ0))
        {
            DISP_REG_SET(0xF4002000, 0x0);
            DISP_REG_SET(0xF4002000, 0x10000);
            DISP_REG_SET(0xF4002000, 0x0);
        }
    }

    if (engineFlag & (0x1 << tSCL1))
    {
        if(clock_is_on(MT_CG_DISP0_MDP_RSZ1))
        {
            DISP_REG_SET(0xF4003000, 0x0);
            DISP_REG_SET(0xF4003000, 0x10000);
            DISP_REG_SET(0xF4003000, 0x0);
        }
    }

    if (engineFlag & (0x1 << tTDSHP))
    {
        if(clock_is_on(MT_CG_DISP0_MDP_TDSHP))
        {
            DISP_REG_SET(0xF4006100, 0x0);
            DISP_REG_SET(0xF4006100, 0x2);
            DISP_REG_SET(0xF4006100, 0x0);
        }
    }

    if (engineFlag & (0x1 << tWROT))
    {
        if(clock_is_on(MT_CG_DISP0_MDP_WROT))
        {
            DISP_REG_SET(0xF4005010, 0x1);

            loop_count = 0;
            while(loop_count <= 50000)
            {
                if (0x0 == (DISP_REG_GET(0xF4005014) & 0x1))
                    break;

                loop_count++;
            }

            if (loop_count > 50000)
            {
                printk(KERN_DEBUG "Reset ROT failed\n");
            }

            DISP_REG_SET(0xF4005010, 0x0);
        }
    }

    if (engineFlag & (0x1 << tWDMA1))
    {
        if(clock_is_on(MT_CG_DISP0_MDP_WDMA))
        {
            DISP_REG_SET(0xF400400C, 0x1);

            loop_count = 0;
            while(loop_count <= 50000)
            {
                if (0x1 == (DISP_REG_GET(0xF40040A0) & 0x3FF))
                    break;

                loop_count++;
            }
        
            if (loop_count > 50000)
            {
                printk(KERN_DEBUG "Reset WDMA failed\n");
            }
        
            DISP_REG_SET(0xF400400C, 0x0);
        }
    }

    mdp_reset_cnt++;
}



void dumpMDPRegInfo(void)
{
    int reg_temp1, reg_temp2, reg_temp3;

    printk(KERN_DEBUG "[CMDQ]RDMA_SRC_CON: 0x%08x, RDMA_SRC_BASE_0: 0x%08x, RDMA_MF_BKGD_SIZE_IN_BYTE: 0x%08x\n", 
           DISP_REG_GET(0xF4001030), 
           DISP_REG_GET(0xF4001040),
           DISP_REG_GET(0xF4001060));
    printk(KERN_DEBUG "[CMDQ]RDMA_MF_SRC_SIZE: 0x%08x, RDMA_MF_CLIP_SIZE: 0x%08x, RDMA_MF_OFFSET_1: 0x%08x\n", 
           DISP_REG_GET(0xF4001070), 
           DISP_REG_GET(0xF4001078), 
           DISP_REG_GET(0xF4001080));
    printk(KERN_DEBUG "[CMDQ]RDMA_SRC_END_0: 0x%08x, RDMA_SRC_OFFSET_0: 0x%08x, RDMA_SRC_OFFSET_W_0: 0x%08x\n", 
           DISP_REG_GET(0xF4001100), 
           DISP_REG_GET(0xF4001118), 
           DISP_REG_GET(0xF4001130));
    printk(KERN_DEBUG "[CMDQ]RDMA_MON_STA_0: 0x%08x, RDMA_MON_STA_1: 0x%08x, RDMA_MON_STA_2: 0x%08x\n", 
           DISP_REG_GET(0xF4001400), 
           DISP_REG_GET(0xF4001408),
           DISP_REG_GET(0xF4001410));
    printk(KERN_DEBUG "[CMDQ]RDMA_MON_STA_4: 0x%08x, RDMA_MON_STA_6: 0x%08x, RDMA_MON_STA_26: 0x%08x\n", 
           DISP_REG_GET(0xF4001420), 
           DISP_REG_GET(0xF4001430),
           DISP_REG_GET(0xF40014D0));

    printk(KERN_DEBUG "[CMDQ]WDMA_CFG: 0x%08x, WDMA_SRC_SIZE: 0x%08x, WDMA_DST_W_IN_BYTE = 0x%08x\n", 
           DISP_REG_GET(0xF4004014), 
           DISP_REG_GET(0xF4004018),
           DISP_REG_GET(0xF4004028));
    printk(KERN_DEBUG "[CMDQ]WDMA_DST_ADDR0: 0x%08x, WDMA_DST_UV_PITCH: 0x%08x, WDMA_DST_ADDR_OFFSET0 = 0x%08x\n", 
           DISP_REG_GET(0xF4004024), 
           DISP_REG_GET(0xF4004078),
           DISP_REG_GET(0xF4004080));
    printk(KERN_DEBUG "[CMDQ]WDMA_STATUS: 0x%08x, WDMA_INPUT_CNT: 0x%08x\n", 
           DISP_REG_GET(0xF40040A0), 
           DISP_REG_GET(0xF40040A8));

    printk(KERN_DEBUG "[CMDQ]VIDO_CTRL: 0x%08x, VIDO_MAIN_BUF_SIZE: 0x%08x, VIDO_SUB_BUF_SIZE: 0x%08x\n", 
           DISP_REG_GET(0xF4005000), 
           DISP_REG_GET(0xF4005008),
           DISP_REG_GET(0xF400500C));
    
    printk(KERN_DEBUG "[CMDQ]VIDO_TAR_SIZE: 0x%08x, VIDO_BASE_ADDR: 0x%08x, VIDO_OFST_ADDR: 0x%08x\n", 
           DISP_REG_GET(0xF4005024), 
           DISP_REG_GET(0xF4005028),
           DISP_REG_GET(0xF400502C));
    
    printk(KERN_DEBUG "[CMDQ]VIDO_DMA_PERF: 0x%08x, VIDO_STRIDE: 0x%08x, VIDO_IN_SIZE: 0x%08x\n", 
           DISP_REG_GET(0xF4005004), 
           DISP_REG_GET(0xF4005030),
           DISP_REG_GET(0xF4005078));
    
    DISP_REG_SET(0xF4005018, 0x00000100);
    reg_temp1 = DISP_REG_GET(0xF40050D0);
    DISP_REG_SET(0xF4005018, 0x00000200);
    reg_temp2 = DISP_REG_GET(0xF40050D0);
    DISP_REG_SET(0xF4005018, 0x00000300);
    reg_temp3 = DISP_REG_GET(0xF40050D0);
    printk(KERN_DEBUG "[CMDQ]VIDO_DBG1: 0x%08x, VIDO_DBG2: 0x%08x, VIDO_DBG3: 0x%08x\n", reg_temp1, reg_temp2, reg_temp3);
    
    DISP_REG_SET(0xF4005018, 0x00000500);
    reg_temp1 = DISP_REG_GET(0xF40050D0);
    DISP_REG_SET(0xF4005018, 0x00000800);
    reg_temp2 = DISP_REG_GET(0xF40050D0);
    DISP_REG_SET(0xF4005018, 0x00000B00);
    reg_temp3 = DISP_REG_GET(0xF40050D0);
    printk(KERN_DEBUG "[CMDQ]VIDO_DBG5: 0x%08x, VIDO_DBG8: 0x%08x, VIDO_DBGB: 0x%08x\n", reg_temp1, reg_temp2, reg_temp3);
}

void dumpRegDebugInfo(unsigned int engineFlag, int cmdqIndex, cmdq_buff_t bufferAddr)
{
    int reg_temp1;
    int loop_count;
    int index;

    timeout_cnt++;

    //if (0 == rdma_fail_cnt)
    {
        mmsys_cg_con0 = DISP_REG_GET(0xF4000100);
        disp_mutex_insta = DISP_REG_GET(0xF400E004);
        disp_reg_commit = DISP_REG_GET(0xF400E00C);
        cmdq_pc = DISP_REG_GET(DISP_REG_CMDQ_THRx_PC(cmdqIndex));
        reg_temp1 = bufferAddr.VA + (cmdq_pc - bufferAddr.MVA);
        if((bufferAddr.VA <= reg_temp1) && ((bufferAddr.VA + bufferAddr.blocksize) >= reg_temp1))
        {
            if (reg_temp1 != (bufferAddr.VA + bufferAddr.blocksize))
            {
                cmdq_inst0 = DISP_REG_GET(reg_temp1-8);
                cmdq_inst1 = DISP_REG_GET(reg_temp1-4);
                cmdq_inst2 = DISP_REG_GET(reg_temp1);
                cmdq_inst3 = DISP_REG_GET(reg_temp1+4);
            }
            else
            {            
                cmdq_inst0 = DISP_REG_GET(reg_temp1-16);
                cmdq_inst1 = DISP_REG_GET(reg_temp1-12);
                cmdq_inst2 = DISP_REG_GET(reg_temp1-8);
                cmdq_inst3 = DISP_REG_GET(reg_temp1-4);
            }
        }
        cmdq_end = DISP_REG_GET(DISP_REG_CMDQ_THRx_END_ADDR(cmdqIndex));
        cmdq_wait_event = DISP_REG_GET(DISP_REG_CMDQ_THRx_WAIT_EVENTS0(cmdqIndex));\
        
        rdma_reg[0] = DISP_REG_GET(0xF4001030);
        rdma_reg[1] = DISP_REG_GET(0xF4001040);
        rdma_reg[2] = DISP_REG_GET(0xF4001060);
        rdma_reg[3] = DISP_REG_GET(0xF4001070);
        rdma_reg[4] = DISP_REG_GET(0xF4001078);
        rdma_reg[5] = DISP_REG_GET(0xF4001080);
        rdma_reg[6] = DISP_REG_GET(0xF4001100);
        rdma_reg[7] = DISP_REG_GET(0xF4001118);
        rdma_reg[8] = DISP_REG_GET(0xF4001130);
        rdma_reg[9] = DISP_REG_GET(0xF4001400);
        rdma_reg[10] = DISP_REG_GET(0xF4001408);
        rdma_reg[11] = DISP_REG_GET(0xF4001410);
        rdma_reg[12] = DISP_REG_GET(0xF4001420);
        rdma_reg[13] = DISP_REG_GET(0xF4001430);
        rdma_reg[14] = DISP_REG_GET(0xF40014D0);
        
        rsz0_reg[0] = DISP_REG_GET(0xF4002004);
        DISP_REG_SET(0xF4002040, 0x00000003);
        rsz0_reg[1] = DISP_REG_GET(0xF4002044);
        rsz0_reg[2] = DISP_REG_GET(0xF4002014);
        rsz0_reg[3] = DISP_REG_GET(0xF400200C);
        rsz0_reg[4] = DISP_REG_GET(0xF4002010);
        rsz0_reg[5] = DISP_REG_GET(0xF4002018);

        rsz1_reg[0] = DISP_REG_GET(0xF4003004);
        DISP_REG_SET(0xF4003040, 0x00000003);
        rsz1_reg[1] = DISP_REG_GET(0xF4003044);
        rsz1_reg[2] = DISP_REG_GET(0xF4003014);
        rsz1_reg[3] = DISP_REG_GET(0xF400300C);
        rsz1_reg[4] = DISP_REG_GET(0xF4003010);
        rsz1_reg[5] = DISP_REG_GET(0xF4003018);

        wdma_reg[0] = DISP_REG_GET(0xF4004014);
        wdma_reg[1] = DISP_REG_GET(0xF4004018);
        wdma_reg[2] = DISP_REG_GET(0xF4004028);
        wdma_reg[3] = DISP_REG_GET(0xF4004024);
        wdma_reg[4] = DISP_REG_GET(0xF4004078);
        wdma_reg[5] = DISP_REG_GET(0xF4004080);
        wdma_reg[6] = DISP_REG_GET(0xF40040A0);
        wdma_reg[7] = DISP_REG_GET(0xF40040A8);

        wrot_reg[0] = DISP_REG_GET(0xF4005000);
        wrot_reg[1] = DISP_REG_GET(0xF4005008);
        wrot_reg[2] = DISP_REG_GET(0xF400500C);
        wrot_reg[3] = DISP_REG_GET(0xF4005024);
        wrot_reg[4] = DISP_REG_GET(0xF4005028);
        wrot_reg[5] = DISP_REG_GET(0xF400502C);
        wrot_reg[6] = DISP_REG_GET(0xF4005004);
        wrot_reg[7] = DISP_REG_GET(0xF4005030);
        wrot_reg[8] = DISP_REG_GET(0xF4005078);        
        DISP_REG_SET(0xF4005018, 0x00000100);
        wrot_reg[9] = DISP_REG_GET(0xF40050D0);
        DISP_REG_SET(0xF4005018, 0x00000200);
        wrot_reg[10] = DISP_REG_GET(0xF40050D0);
        DISP_REG_SET(0xF4005018, 0x00000300);
        wrot_reg[11] = DISP_REG_GET(0xF40050D0);
        DISP_REG_SET(0xF4005018, 0x00000500);
        wrot_reg[12] = DISP_REG_GET(0xF40050D0);
        DISP_REG_SET(0xF4005018, 0x00000800);
        wrot_reg[13] = DISP_REG_GET(0xF40050D0);
        DISP_REG_SET(0xF4005018, 0x00000B00);
        wrot_reg[14] = DISP_REG_GET(0xF40050D0);
    }

    printk(KERN_DEBUG "DisSys clock state %d\n", subsys_is_on(SYS_DIS));
    printk(KERN_DEBUG "ISPSys clock state %d\n", subsys_is_on(SYS_ISP));

    // dump MMSYS clock setting
    printk(KERN_DEBUG "[CMDQ]engineFlag = 0x%08x, MMSYS_CG_CON0 = 0x%08x\n", engineFlag, mmsys_cg_con0);

    // dump mutex status
    printk(KERN_DEBUG "[CMDQ]DISP_MUTEX_INTSTA = 0x%08x, DISP_REG_COMMIT = 0x%08x\n", 
           disp_mutex_insta,
           disp_reg_commit);

    if (0 != DISP_REG_GET(0xF400E00C))
    {
        mutex_fail_cnt++;

        for (loop_count = 0; loop_count < 8; loop_count++)
        {        
            printk(KERN_DEBUG "[CMDQ] Mutex reset  = 0x%08x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX_RST(loop_count)));
            printk(KERN_DEBUG "[CMDQ] Mutex module = 0x%08x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(loop_count)));
        }
    }

    // dump current instruction
    printk(KERN_DEBUG "[CMDQ]CMDQ current inst0 = 0x%08x0x%08x, inst1 = 0x%08x0x%08x\n", cmdq_inst0, cmdq_inst1, cmdq_inst2, cmdq_inst3);

    // dump CMDQ status
    printk(KERN_DEBUG "[CMDQ]CMDQ_THR%d_PC = 0x%08x, CMDQ_THR%d_END_ADDR = 0x%08x, CMDQ_THR%d_WAIT_TOKEN = 0x%08x\n", 
           cmdqIndex, cmdq_pc,
           cmdqIndex, cmdq_end,
           cmdqIndex, cmdq_wait_event);

    if (engineFlag & (0x1 << tIMGI))
    {
        printk(KERN_DEBUG "CLK_CFG_0  = 0x%08x\n", DISP_REG_GET(CLK_CFG_0));
        printk(KERN_DEBUG "CLK_CFG_3  = 0x%08x\n", DISP_REG_GET(CLK_CFG_3));
        printk(KERN_DEBUG "ISP_CLK_CG = 0x%08x\n", DISP_REG_GET(0xF5000000));

        if(clock_is_on(MT_CG_IMAGE_CAM_CAM))
        {
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] start MT6582\n");
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] tpipe_id = 0x00000000\n");
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004000 = 0x%08x\n", DISP_REG_GET(0xF5004000));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004004 = 0x%08x\n", DISP_REG_GET(0xF5004004));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004008 = 0x%08x\n", DISP_REG_GET(0xF5004008));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1500400C = 0x%08x\n", DISP_REG_GET(0xF500400C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004010 = 0x%08x\n", DISP_REG_GET(0xF5004010));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004018 = 0x%08x\n", DISP_REG_GET(0xF5004018));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1500401C = 0x%08x\n", DISP_REG_GET(0xF500401C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004050 = 0x%08x\n", DISP_REG_GET(0xF5004050));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004054 = 0x%08x\n", DISP_REG_GET(0xF5004054));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004078 = 0x%08x\n", DISP_REG_GET(0xF5004078));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004110 = 0x%08x\n", DISP_REG_GET(0xF5004110));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1500422C = 0x%08x\n", DISP_REG_GET(0xF500422C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004240 = 0x%08x\n", DISP_REG_GET(0xF5004240));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1500427C = 0x%08x\n", DISP_REG_GET(0xF500427C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x150042B4 = 0x%08x\n", DISP_REG_GET(0xF50042B4));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004308 = 0x%08x\n", DISP_REG_GET(0xF5004308));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1500430C = 0x%08x\n", DISP_REG_GET(0xF500430C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004310 = 0x%08x\n", DISP_REG_GET(0xF5004310));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1500431C = 0x%08x\n", DISP_REG_GET(0xF500431C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004328 = 0x%08x\n", DISP_REG_GET(0xF5004328));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1500432C = 0x%08x\n", DISP_REG_GET(0xF500432C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004330 = 0x%08x\n", DISP_REG_GET(0xF5004330));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1500433C = 0x%08x\n", DISP_REG_GET(0xF500433C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004534 = 0x%08x\n", DISP_REG_GET(0xF5004534));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004538 = 0x%08x\n", DISP_REG_GET(0xF5004538));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1500453C = 0x%08x\n", DISP_REG_GET(0xF500453C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004800 = 0x%08x\n", DISP_REG_GET(0xF5004800));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x150048A0 = 0x%08x\n", DISP_REG_GET(0xF50048A0));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x150049C4 = 0x%08x\n", DISP_REG_GET(0xF50049C4));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x150049E4 = 0x%08x\n", DISP_REG_GET(0xF50049E4));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x150049E8 = 0x%08x\n", DISP_REG_GET(0xF50049E8));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x150049EC = 0x%08x\n", DISP_REG_GET(0xF50049EC));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004A20 = 0x%08x\n", DISP_REG_GET(0xF5004A20));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004ACC = 0x%08x\n", DISP_REG_GET(0xF5004ACC));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004B00 = 0x%08x\n", DISP_REG_GET(0xF5004B00));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004B04 = 0x%08x\n", DISP_REG_GET(0xF5004B04));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004B08 = 0x%08x\n", DISP_REG_GET(0xF5004B08));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004B0C = 0x%08x\n", DISP_REG_GET(0xF5004B0C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004B10 = 0x%08x\n", DISP_REG_GET(0xF5004B10));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004B14 = 0x%08x\n", DISP_REG_GET(0xF5004B14));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004B18 = 0x%08x\n", DISP_REG_GET(0xF5004B18));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004B1C = 0x%08x\n", DISP_REG_GET(0xF5004B1C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004B20 = 0x%08x\n", DISP_REG_GET(0xF5004B20));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x15004F50 = 0x%08x\n", DISP_REG_GET(0xF5004F50));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1400001C = 0x%08x\n", DISP_REG_GET(0xF400001C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14000020 = 0x%08x\n", DISP_REG_GET(0xF4000020));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14000024 = 0x%08x\n", DISP_REG_GET(0xF4000024));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14000028 = 0x%08x\n", DISP_REG_GET(0xF4000028));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1400002C = 0x%08x\n", DISP_REG_GET(0xF400002C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14000038 = 0x%08x\n", DISP_REG_GET(0xF4000038));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1400003C = 0x%08x\n", DISP_REG_GET(0xF400003C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14000040 = 0x%08x\n", DISP_REG_GET(0xF4000040));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14000044 = 0x%08x\n", DISP_REG_GET(0xF4000044));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14000048 = 0x%08x\n", DISP_REG_GET(0xF4000048));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14001000 = 0x%08x\n", DISP_REG_GET(0xF4001000));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14001020 = 0x%08x\n", DISP_REG_GET(0xF4001020));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14001030 = 0x%08x\n", DISP_REG_GET(0xF4001030));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14001060 = 0x%08x\n", DISP_REG_GET(0xF4001060));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14001090 = 0x%08x\n", DISP_REG_GET(0xF4001090));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14002000 = 0x%08x\n", DISP_REG_GET(0xF4002000));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14002004 = 0x%08x\n", DISP_REG_GET(0xF4002004));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1400200C = 0x%08x\n", DISP_REG_GET(0xF400200C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14002010 = 0x%08x\n", DISP_REG_GET(0xF4002010));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14002014 = 0x%08x\n", DISP_REG_GET(0xF4002014));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14002018 = 0x%08x\n", DISP_REG_GET(0xF4002018));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1400201C = 0x%08x\n", DISP_REG_GET(0xF400201C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14002020 = 0x%08x\n", DISP_REG_GET(0xF4002020));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14002024 = 0x%08x\n", DISP_REG_GET(0xF4002024));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14002028 = 0x%08x\n", DISP_REG_GET(0xF4002028));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1400202C = 0x%08x\n", DISP_REG_GET(0xF400202C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14002030 = 0x%08x\n", DISP_REG_GET(0xF4002030));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14002034 = 0x%08x\n", DISP_REG_GET(0xF4002034));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14002038 = 0x%08x\n", DISP_REG_GET(0xF4002038));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14003000 = 0x%08x\n", DISP_REG_GET(0xF4003000));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14003004 = 0x%08x\n", DISP_REG_GET(0xF4003004));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1400300C = 0x%08x\n", DISP_REG_GET(0xF400300C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14003010 = 0x%08x\n", DISP_REG_GET(0xF4003010));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14003014 = 0x%08x\n", DISP_REG_GET(0xF4003014));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14003018 = 0x%08x\n", DISP_REG_GET(0xF4003018));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1400301C = 0x%08x\n", DISP_REG_GET(0xF400301C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14003020 = 0x%08x\n", DISP_REG_GET(0xF4003020));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14003024 = 0x%08x\n", DISP_REG_GET(0xF4003024));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14003028 = 0x%08x\n", DISP_REG_GET(0xF4003028));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1400302C = 0x%08x\n", DISP_REG_GET(0xF400302C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14003030 = 0x%08x\n", DISP_REG_GET(0xF4003030));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14003034 = 0x%08x\n", DISP_REG_GET(0xF4003034));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14003038 = 0x%08x\n", DISP_REG_GET(0xF4003038));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14004008 = 0x%08x\n", DISP_REG_GET(0xF4004008));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14004014 = 0x%08x\n", DISP_REG_GET(0xF4004014));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14004018 = 0x%08x\n", DISP_REG_GET(0xF4004018));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14004028 = 0x%08x\n", DISP_REG_GET(0xF4004028));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14004078 = 0x%08x\n", DISP_REG_GET(0xF4004078));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14005000 = 0x%08x\n", DISP_REG_GET(0xF4005000));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14005030 = 0x%08x\n", DISP_REG_GET(0xF4005030));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1400503C = 0x%08x\n", DISP_REG_GET(0xF400503C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1400506C = 0x%08x\n", DISP_REG_GET(0xF400506C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x1400507C = 0x%08x\n", DISP_REG_GET(0xF400507C));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14005084 = 0x%08x\n", DISP_REG_GET(0xF4005084));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14006000 = 0x%08x\n", DISP_REG_GET(0xF4006000));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] 0x14006110 = 0x%08x\n", DISP_REG_GET(0xF4006110));
            printk(KERN_DEBUG "[ISP/MDP][TPIPE_DumpReg] end MT6582\n");
        }
        else
        {
            printk(KERN_DEBUG "[CMDQ] Incorrectly, ISP clock is in off state\n");
        }
    }

    // dump RDMA debug registers
    if (engineFlag & (0x1 << tRDMA0))
    {
        printk(KERN_DEBUG "[CMDQ]RDMA_SRC_CON: 0x%08x, RDMA_SRC_BASE_0: 0x%08x, RDMA_MF_BKGD_SIZE_IN_BYTE: 0x%08x\n", 
               rdma_reg[0], 
               rdma_reg[1],
               rdma_reg[2]);
        printk(KERN_DEBUG "[CMDQ]RDMA_MF_SRC_SIZE: 0x%08x, RDMA_MF_CLIP_SIZE: 0x%08x, RDMA_MF_OFFSET_1: 0x%08x\n", 
               rdma_reg[3], 
               rdma_reg[4], 
               rdma_reg[5]);
        printk(KERN_DEBUG "[CMDQ]RDMA_SRC_END_0: 0x%08x, RDMA_SRC_OFFSET_0: 0x%08x, RDMA_SRC_OFFSET_W_0: 0x%08x\n", 
               rdma_reg[6], 
               rdma_reg[7], 
               rdma_reg[8]);
        printk(KERN_DEBUG "[CMDQ](R)RDMA_MON_STA_0: 0x%08x, RDMA_MON_STA_1: 0x%08x, RDMA_MON_STA_2: 0x%08x\n", 
               rdma_status0, 
               rdma_status1,
               rdma_status2);
        printk(KERN_DEBUG "[CMDQ](R)RDMA_MON_STA_4: 0x%08x, RDMA_MON_STA_6: 0x%08x, RDMA_MON_STA_26: 0x%08x\n", 
               rdma_status4, 
               rdma_status6,
               rdma_status26);
        printk(KERN_DEBUG "[CMDQ]RDMA_MON_STA_0: 0x%08x, RDMA_MON_STA_1: 0x%08x, RDMA_MON_STA_2: 0x%08x\n", 
               rdma_reg[9], 
               rdma_reg[10],
               rdma_reg[11]);
        printk(KERN_DEBUG "[CMDQ]RDMA_MON_STA_4: 0x%08x, RDMA_MON_STA_6: 0x%08x, RDMA_MON_STA_26: 0x%08x\n", 
               rdma_reg[12], 
               rdma_reg[13],
               rdma_reg[14]);
    
        if (0x100 != (DISP_REG_GET(0xF4001408) & 0x100))
        {
            rdma_fail_cnt++;
        }
    }
    
    // dump RSZ debug registers
    if (engineFlag & (0x1 << tSCL0))
    {
        printk(KERN_DEBUG "[CMDQ]RSZ0_CFG: 0x%08x, RSZ0_INPUT_CNT: 0x%08x, RSZ0_HORIZONTAL_COEFF_STEP = 0x%08x\n", 
               rsz0_reg[0], 
               rsz0_reg[1],
               rsz0_reg[2]);

        printk(KERN_DEBUG "[CMDQ]RSZ0_IN_SIZE: 0x%08x, RSZ0_OUT_SIZE: 0x%08x, RSZ0_VERTICAL_COEFF_STEP = 0x%08x\n", 
               rsz0_reg[3], 
               rsz0_reg[4],
               rsz0_reg[5]);
    }

    if (engineFlag & (0x1 << tSCL1))
    {
        printk(KERN_DEBUG "[CMDQ]RSZ1_CFG: 0x%08x, RSZ1_INPUT_CNT: 0x%08x, RSZ1_HORIZONTAL_COEFF_STEP = 0x%08x\n", 
               rsz1_reg[0], 
               rsz1_reg[1],
               rsz1_reg[2]);

        printk(KERN_DEBUG "[CMDQ]RSZ1_IN_SIZE: 0x%08x, RSZ1_OUT_SIZE: 0x%08x, RSZ1_VERTICAL_COEFF_STEP = 0x%08x\n", 
               rsz1_reg[3], 
               rsz1_reg[4],
               rsz1_reg[5]);
    }

    // dump WDMA debug registers
    if (engineFlag & (0x1 << tWDMA1))
    {
        printk(KERN_DEBUG "[CMDQ]WDMA_CFG: 0x%08x, WDMA_SRC_SIZE: 0x%08x, WDMA_DST_W_IN_BYTE = 0x%08x\n", 
               wdma_reg[0],  //0x14
               wdma_reg[1],
               wdma_reg[2]);
        printk(KERN_DEBUG "[CMDQ]WDMA_DST_ADDR0: 0x%08x, WDMA_DST_UV_PITCH: 0x%08x, WDMA_DST_ADDR_OFFSET0 = 0x%08x\n", 
               wdma_reg[3], 
               wdma_reg[4],
               wdma_reg[5]);
        printk(KERN_DEBUG "[CMDQ]WDMA_STATUS: 0x%08x, WDMA_INPUT_CNT: 0x%08x\n", 
               wdma_reg[6], 
               wdma_reg[7]);

    //Dump Addtional WDMA debug info
        DISP_REG_SET(0xF4004014, (wdma_reg[0] & (0x0FFFFFFF)));
        printk(KERN_DEBUG "WDMA Debug Info 0xF4004014: 0x%08x , 0xF40040ac: 0x%08x \n", DISP_REG_GET(0xF4004014),DISP_REG_GET(0xF40040AC));
        DISP_REG_SET(0xF4004014, 0x10000000 |(wdma_reg[0] & (0x0FFFFFFF)));
        printk(KERN_DEBUG "WDMA Debug Info 0xF4004014: 0x%08x , 0xF40040ac: 0x%08x \n", DISP_REG_GET(0xF4004014),DISP_REG_GET(0xF40040AC));
        DISP_REG_SET(0xF4004014, 0x20000000 |(wdma_reg[0] & (0x0FFFFFFF)));
        printk(KERN_DEBUG "WDMA Debug Info 0xF4004014: 0x%08x , 0xF40040ac: 0x%08x \n", DISP_REG_GET(0xF4004014),DISP_REG_GET(0xF40040AC));
        DISP_REG_SET(0xF4004014, 0x30000000 |(wdma_reg[0] & (0x0FFFFFFF)));
        printk(KERN_DEBUG "WDMA Debug Info 0xF4004014: 0x%08x , 0xF40040ac: 0x%08x \n", DISP_REG_GET(0xF4004014),DISP_REG_GET(0xF40040AC));
        DISP_REG_SET(0xF4004014, 0x40000000 |(wdma_reg[0] & (0x0FFFFFFF)));
        printk(KERN_DEBUG "WDMA Debug Info 0xF4004014: 0x%08x , 0xF40040ac: 0x%08x \n", DISP_REG_GET(0xF4004014),DISP_REG_GET(0xF40040AC));
        DISP_REG_SET(0xF4004014, 0x50000000 |(wdma_reg[0] & (0x0FFFFFFF)));
        printk(KERN_DEBUG "WDMA Debug Info 0xF4004014: 0x%08x , 0xF40040ac: 0x%08x \n", DISP_REG_GET(0xF4004014),DISP_REG_GET(0xF40040AC));
        DISP_REG_SET(0xF4004014, 0x60000000 |(wdma_reg[0] & (0x0FFFFFFF)));
        printk(KERN_DEBUG "WDMA Debug Info 0xF4004014: 0x%08x , 0xF40040ac: 0x%08x \n", DISP_REG_GET(0xF4004014),DISP_REG_GET(0xF40040AC));
        DISP_REG_SET(0xF4004014, 0x70000000 |(wdma_reg[0] & (0x0FFFFFFF)));
        printk(KERN_DEBUG "WDMA Debug Info 0xF4004014: 0x%08x , 0xF40040ac: 0x%08x \n", DISP_REG_GET(0xF4004014),DISP_REG_GET(0xF40040AC));
        DISP_REG_SET(0xF4004014, 0x80000000 |(wdma_reg[0] & (0x0FFFFFFF)));
        printk(KERN_DEBUG "WDMA Debug Info 0xF4004014: 0x%08x , 0xF40040ac: 0x%08x \n", DISP_REG_GET(0xF4004014),DISP_REG_GET(0xF40040AC));
        DISP_REG_SET(0xF4004014, 0x90000000 |(wdma_reg[0] & (0x0FFFFFFF)));
        printk(KERN_DEBUG "WDMA Debug Info 0xF4004014: 0x%08x , 0xF40040ac: 0x%08x \n", DISP_REG_GET(0xF4004014),DISP_REG_GET(0xF40040AC));
        DISP_REG_SET(0xF4004014, 0xA0000000 |(wdma_reg[0] & (0x0FFFFFFF)));
        printk(KERN_DEBUG "WDMA Debug Info 0xF4004014: 0x%08x , 0xF40040ac: 0x%08x \n", DISP_REG_GET(0xF4004014),DISP_REG_GET(0xF40040AC));
        DISP_REG_SET(0xF4004014, 0xB0000000 |(wdma_reg[0] & (0x0FFFFFFF)));
        printk(KERN_DEBUG "WDMA Debug Info 0xF4004014: 0x%08x , 0xF40040ac: 0x%08x \n", DISP_REG_GET(0xF4004014),DISP_REG_GET(0xF40040AC));
        DISP_REG_SET(0xF4004014, 0xC0000000 |(wdma_reg[0] & (0x0FFFFFFF)));
        printk(KERN_DEBUG "WDMA Debug Info 0xF4004014: 0x%08x , 0xF40040ac: 0x%08x \n", DISP_REG_GET(0xF4004014),DISP_REG_GET(0xF40040AC));
        DISP_REG_SET(0xF4004014, 0xD0000000 |(wdma_reg[0] & (0x0FFFFFFF)));
        printk(KERN_DEBUG "WDMA Debug Info 0xF4004014: 0x%08x , 0xF40040ac: 0x%08x \n", DISP_REG_GET(0xF4004014),DISP_REG_GET(0xF40040AC));
        DISP_REG_SET(0xF4004014, 0xE0000000 |(wdma_reg[0] & (0x0FFFFFFF)));
        printk(KERN_DEBUG "WDMA Debug Info 0xF4004014: 0x%08x , 0xF40040ac: 0x%08x \n", DISP_REG_GET(0xF4004014),DISP_REG_GET(0xF40040AC));
        DISP_REG_SET(0xF4004014, 0xF0000000 |(wdma_reg[0] & (0x0FFFFFFF)));
        printk(KERN_DEBUG "WDMA Debug Info 0xF4004014: 0x%08x , 0xF40040ac: 0x%08x \n", DISP_REG_GET(0xF4004014),DISP_REG_GET(0xF40040AC));

    }

    // dump WROT debug registers
    if (engineFlag & (0x1 << tWROT))
    {
        printk(KERN_DEBUG "[CMDQ]VIDO_CTRL: 0x%08x, VIDO_MAIN_BUF_SIZE: 0x%08x, VIDO_SUB_BUF_SIZE: 0x%08x\n", 
               wrot_reg[0], 
               wrot_reg[1],
               wrot_reg[2]);

        printk(KERN_DEBUG "[CMDQ]VIDO_TAR_SIZE: 0x%08x, VIDO_BASE_ADDR: 0x%08x, VIDO_OFST_ADDR: 0x%08x\n", 
               wrot_reg[3], 
               wrot_reg[4],
               wrot_reg[5]);

        printk(KERN_DEBUG "[CMDQ]VIDO_DMA_PERF: 0x%08x, VIDO_STRIDE: 0x%08x, VIDO_IN_SIZE: 0x%08x\n", 
               wrot_reg[6], 
               wrot_reg[7],
               wrot_reg[8]);

        printk(KERN_DEBUG "[CMDQ]VIDO_DBG1: 0x%08x, VIDO_DBG2: 0x%08x, VIDO_DBG3: 0x%08x\n", 
               wrot_reg[9], 
               wrot_reg[10], 
               wrot_reg[11]);
/*
        if (0x80 == (reg_temp3 & 0x80))
        {
            wrot_fail_cnt++;
        }
*/
        printk(KERN_DEBUG "[CMDQ]VIDO_DBG5: 0x%08x, VIDO_DBG8: 0x%08x, VIDO_DBGB: 0x%08x\n", 
               wrot_reg[12], 
               wrot_reg[13], 
               wrot_reg[14]);

    }

    //SMI Dump
    smi_dumpDebugMsg();

    //CMDQ Timeout Callbacks
    
    for(index = 0 ; index < cbMAX; index++)
    {
        if(g_CMDQ_CB_Array.cmdqTimeout_cb[index]!=NULL)
            g_CMDQ_CB_Array.cmdqTimeout_cb[index](0);
    }
    

    
}

void cmdqBufferTbl_init(unsigned long va_base, unsigned long mva_base)
{
    int i = 0;
    unsigned long flags;
    
    spin_lock_irqsave(&gCmdqMgrLock, flags);

    for(i=0;i<CMDQ_BUFFER_NUM;i++)
    {
        cmdqBufTbl[i].Owner = -1; //free buffer
        cmdqBufTbl[i].VA = va_base + (i*CMDQ_BUFFER_SIZE);
        cmdqBufTbl[i].MVA =  mva_base + (i*CMDQ_BUFFER_SIZE);

        CMDQ_MSG("cmdqBufferTbl_init %x : [%x %lx %lx] \n",i ,cmdqBufTbl[i].Owner, cmdqBufTbl[i].VA, cmdqBufTbl[i].MVA);
    }

    for(i=0;i<MAX_CMDQ_TASK_ID;i++)
    {
        taskIDStatusTbl[i] = -1; //mark as free ID
        taskResOccuTbl[i].cmdBufID = -1;
        taskResOccuTbl[i].cmdqThread= -1;
        cmdqThreadEofCnt[i] = -1;
    }

    for(i=0;i<CMDQ_THREAD_NUM;i++)
    {
        cmdqThreadResTbl[i] = 0; 
        cmdqThreadTaskList_R[i] = 0;
        cmdqThreadTaskList_W[i] = 0;
        cmdqThreadTaskNumber[i] = 0;
        cmdqThreadFirstJobID[i] = -1;
    }
    
    spin_unlock_irqrestore(&gCmdqMgrLock, flags);

    //cmdqRegisterCallback(cbMDP, MDPTimeOutDump ,MDPResetProcess);

}

int cmdqResource_required(void)
{
    int i = 0;
    int assignedTaskID = -1;

//    spin_lock(&gCmdqMgrLock);

    //Find free ID
    for(i=0;i<MAX_CMDQ_TASK_ID;i++)
    {
        if(taskIDStatusTbl[i]==-1) 
        {
            assignedTaskID = i;
            taskIDStatusTbl[assignedTaskID] = 1; //mark as occupied
            break;
        }
    }

    if(assignedTaskID == -1)
    {
        CMDQ_ERR("No useable ID !!!\n");
        dumpDebugInfo();
        return -1;
    }
    //Find free Buffer
    for(i=0;i<CMDQ_BUFFER_NUM;i++)
    {
        if(cmdqBufTbl[i].Owner == -1)
        {
            cmdqBufTbl[i].Owner = assignedTaskID;
            taskResOccuTbl[assignedTaskID].cmdBufID = i;

            //printk(KERN_DEBUG "\n=========CMDQ Buffer %x is owned by %x==========\n", taskResOccuTbl[assignedTaskID].cmdBufID, cmdqBufTbl[i].Owner);
            break;
        }
    }

    //DEBUG
    //dumpDebugInfo();

    if(taskResOccuTbl[assignedTaskID].cmdBufID == -1)
    {
        CMDQ_ERR("No Free Buffer !!! Total reset CMDQ Driver\n");
        dumpDebugInfo();
        //taskIDStatusTbl[assignedTaskID] = -1; //return ID, resource allocation fail
        cmdqForceFreeAll(0);

        for(i=0;i<MAX_CMDQ_TASK_ID;i++)
        {
            taskIDStatusTbl[i] = -1; //mark as CANCEL
            taskResOccuTbl[i].cmdBufID = -1;
            taskResOccuTbl[i].cmdqThread= -1;
        }
        return -1;
    } 
    
//    spin_unlock(&gCmdqMgrLock);
    
    return assignedTaskID;
}


void cmdqResource_free(int taskID)
{
    int bufID = -1;

    //spin_lock(&gCmdqMgrLock);

    if(taskID == -1 ||taskID>=MAX_CMDQ_TASK_ID)
    {
        CMDQ_ERR("\n=================Free Invalid Task ID================\n");
        dumpDebugInfo();
        return;
    }

    bufID = taskResOccuTbl[taskID].cmdBufID;

    printk(KERN_DEBUG "=============Free Buf %x own by [%x=%x]===============\n",bufID,taskID,cmdqBufTbl[bufID].Owner);

    if(bufID != -1) //Free All resource and return ID
    {
        taskIDStatusTbl[taskID] = 3; //mark for complete
        taskResOccuTbl[taskID].cmdBufID = -1;
        taskResOccuTbl[taskID].cmdqThread= -1;
        cmdqBufTbl[bufID].Owner = -1;
    }
    else
    {
        CMDQ_ERR("\n=================Free Invalid Buffer ID================\n");
        dumpDebugInfo();
    }

    //spin_unlock(&gCmdqMgrLock);

}


cmdq_buff_t * cmdqBufAddr(int taskID)
{
    int bufID = -1;

    if((-1 == taskID) || (taskID >= MAX_CMDQ_TASK_ID))
    {
        CMDQ_ERR("cmdqBufAddr Invalid ID %d\n", taskID);
        return NULL;
    }

    bufID = taskResOccuTbl[taskID].cmdBufID;

    if((CMDQ_BUFFER_NUM < bufID) || (bufID < 0))
    {
        return NULL;
    }

    return &cmdqBufTbl[bufID];

}


void cmdqHwClockOn(unsigned int engineFlag, bool firstTask)
{
    //Start! Power on clock!        
    //M4U
    //larb_clock_on(0, "MDP");
    //MDP
#if 0
    //enable_clock(MT_CG_INFRA_SMI, "M4U");
    //enable_clock(MT_CG_INFRA_M4U, "M4U");
#endif // 0

    if(firstTask)
    {
        //if(!clock_is_on(MT_CG_DISP0_SMI_COMMON))
        {
            enable_clock(MT_CG_DISP0_SMI_COMMON, "SMI_COMMON");
        }

        //if(!clock_is_on(MT_CG_DISP0_SMI_LARB0))
        {
            enable_clock(MT_CG_DISP0_SMI_LARB0, "SMI_LARB0");
        }

        //if(!clock_is_on(MT_CG_DISP0_MM_CMDQ))
        {
            enable_clock(MT_CG_DISP0_MM_CMDQ, "MM_CMDQ");
        }

        //if(!clock_is_on(MT_CG_DISP0_MUTEX))
        {
            enable_clock(MT_CG_DISP0_MUTEX, "MUTEX");
        }

        //TODO! Should reset All
        DISP_REG_SET(DISP_REG_CMDQ_THRx_RESET(0), 1);
        DISP_REG_SET(DISP_REG_CMDQ_THRx_RESET(1), 1);
    }

    if (engineFlag & (0x1 << tIMGI))
    {
        if(!clock_is_on(MT_CG_DISP0_CAM_MDP))
        {
            enable_clock(MT_CG_IMAGE_CAM_SMI, "CAMERA");
            enable_clock(MT_CG_IMAGE_CAM_CAM, "CAMERA");
            enable_clock(MT_CG_IMAGE_SEN_TG,  "CAMERA");
            enable_clock(MT_CG_IMAGE_SEN_CAM, "CAMERA");
            enable_clock(MT_CG_IMAGE_LARB2_SMI, "CAMERA");

            enable_clock(MT_CG_DISP0_CAM_MDP, "CAM_MDP"); 
        }
    }
    
    if (engineFlag & (0x1 << tRDMA0))
    {
        if(!clock_is_on(MT_CG_DISP0_MDP_RDMA))
        {
            enable_clock(MT_CG_DISP0_MDP_RDMA, "MDP_RDMA");
        }
    }

    if (engineFlag & (0x1 << tSCL0))
    {
        if(!clock_is_on(MT_CG_DISP0_MDP_RSZ0))
        {
            enable_clock(MT_CG_DISP0_MDP_RSZ0, "MDP_RSZ0");
        }
    }

    if (engineFlag & (0x1 << tSCL1))
    {
        if(!clock_is_on(MT_CG_DISP0_MDP_RSZ1))
        {
            enable_clock(MT_CG_DISP0_MDP_RSZ1, "MDP_RSZ1");
        }
    }

    if (engineFlag & (0x1 << tTDSHP))
    {
        if(!clock_is_on(MT_CG_DISP0_MDP_TDSHP))
        {
            enable_clock(MT_CG_DISP0_MDP_TDSHP, "MDP_TDSHP");
        }
    }

    if (engineFlag & (0x1 << tWROT))
    {
        if(!clock_is_on(MT_CG_DISP0_MDP_WROT))
        {
            enable_clock(MT_CG_DISP0_MDP_WROT, "MDP_WROT");
        }
    }

    if (engineFlag & (0x1 << tWDMA1))
    {
        if(!clock_is_on(MT_CG_DISP0_MDP_WDMA))
        {
            enable_clock(MT_CG_DISP0_MDP_WDMA, "MDP_WDMA");
        }
    }

    CMDQ_MSG("\n\n\n=========== Power On %x ==============\n\n\n",engineFlag);
    
}

void cmdqHwClockOff(unsigned int engineFlag)
{
    //Finished! Power off clock!   
    //M4U
    //larb_clock_off(0, "MDP");
    //MDP

    if (engineFlag & (0x1 << tWDMA1))
    {
        if(clock_is_on(MT_CG_DISP0_MDP_WDMA))
        {
            disable_clock(MT_CG_DISP0_MDP_WDMA, "MDP_WDMA");
        }
    }

    if (engineFlag & (0x1 << tWROT))
    {
        if(clock_is_on(MT_CG_DISP0_MDP_WROT))
        {
            disable_clock(MT_CG_DISP0_MDP_WROT, "MDP_WROT");
        }
    }

    if (engineFlag & (0x1 << tTDSHP))
    {
        if(clock_is_on(MT_CG_DISP0_MDP_TDSHP))
        {
            disable_clock(MT_CG_DISP0_MDP_TDSHP, "MDP_TDSHP");
        }
    }

    if (engineFlag & (0x1 << tSCL1))
    {
        if(clock_is_on(MT_CG_DISP0_MDP_RSZ1))
        {
            disable_clock(MT_CG_DISP0_MDP_RSZ1, "MDP_RSZ1");
        }
    }

    if (engineFlag & (0x1 << tSCL0))
    {
        if(clock_is_on(MT_CG_DISP0_MDP_RSZ0))
        {
            disable_clock(MT_CG_DISP0_MDP_RSZ0, "MDP_RSZ0");
        }
    }

    if (engineFlag & (0x1 << tRDMA0))
    {
        if(clock_is_on(MT_CG_DISP0_MDP_RDMA))
        {
            disable_clock(MT_CG_DISP0_MDP_RDMA, "MDP_RDMA");
        }
    }

    if (engineFlag & (0x1 << tIMGI))
    {
        if(clock_is_on(MT_CG_DISP0_CAM_MDP))
        {
            disable_clock(MT_CG_DISP0_CAM_MDP, "CAM_MDP");
            disable_clock(MT_CG_IMAGE_CAM_SMI, "CAMERA");
            disable_clock(MT_CG_IMAGE_CAM_CAM, "CAMERA");
            disable_clock(MT_CG_IMAGE_SEN_TG,  "CAMERA");
            disable_clock(MT_CG_IMAGE_SEN_CAM, "CAMERA");
            disable_clock(MT_CG_IMAGE_LARB2_SMI, "CAMERA");
        }
    }

    //if(clock_is_on(MT_CG_DISP0_MUTEX))
    {
        disable_clock(MT_CG_DISP0_MUTEX, "MUTEX");
    }

    //if(clock_is_on(MT_CG_DISP0_MM_CMDQ))
    {
        disable_clock(MT_CG_DISP0_MM_CMDQ, "MM_CMDQ");
    }

    //if(clock_is_on(MT_CG_DISP0_SMI_LARB0))
    {
        disable_clock(MT_CG_DISP0_SMI_LARB0, "SMI_LARB0");
    }

    //if(clock_is_on(MT_CG_DISP0_SMI_COMMON))
    {
        disable_clock(MT_CG_DISP0_SMI_COMMON, "SMI_COMMON");
    }

#if 0
    //disable_clock(MT_CG_INFRA_SMI, "M4U");
    //disable_clock(MT_CG_INFRA_M4U, "M4U");   
#endif // 0

    CMDQ_MSG("\n\n\n===========Power Off %x ==============\n\n\n",engineFlag);
}


bool cmdqTaskAssigned(int taskID, unsigned int priority, unsigned int engineFlag, unsigned int blocksize)
{
    int i = 0;
    int cmdqThread = -1;
    cmdq_buff_t * pCmdqAddr = NULL;
    unsigned long flags;
    unsigned long ins_leng = 0;
    unsigned long *cmdq_pc_head = 0;
    int buf_id;
    int pre_task_id;
    int pre_w_ptr;
//    int nxt_task_id;    
//    int nxt_r_ptr;
    cmdq_buff_t * pPre_cmdqAddr = NULL;
//   cmdq_buff_t * pNxt_cmdqAddr = NULL;
//    unsigned long reg_val; 
    bool ret = true;
    int cmdq_polling_timeout = 0;
    long wq_ret = 0;
//    int rdma_token, wdma_token, wrot_token;
    struct timeval start_t, end_t;
//    bool engine_busy = false;

//    int restore_pc = 0;
//    int restore_endaddr = 0;

    int ring = 0; 
//    long ringed_cnt = 0;
        
    ins_leng = blocksize>>2; //CMDQ instruction: 4 byte

    total_exec_cnt++;
//    printk(KERN_DEBUG "CMDQ_INFO: total exec: %d, timeout: %d, cmdq done MDP busy: %d, wait queue: %d, cmdq isr: %d, mdp busy: %d\n", 
//           total_exec_cnt, timeout_cnt, cmdq_done_mdp_busy_cnt, wait_queue_timeout_cnt, cmdq_isr_fail_cnt, mdp_busy_long_cnt);
//    printk(KERN_DEBUG "CMDQ_INFO: MDP engine fail: mutex: %d, rdma: %d, wrot: %d, MDP_reset: %d\n", 
//           mutex_fail_cnt, rdma_fail_cnt, wrot_fail_cnt, mdp_reset_cnt);

    //CS++++++++++++++++++++++++++++++++++++++++++++++++++
    spin_lock_irqsave(&gCmdqMgrLock,flags); 

//    printk(KERN_DEBUG "Previous frame period: %d sec, %d usec\n", (int)frame_interval.tv_sec, (int)frame_interval.tv_usec);

    do_gettimeofday(&start_t);

    CMDQ_MSG("\n\n\n==============cmdqTaskAssigned  %d %d %x %d =============\n\n\n", taskID, priority, engineFlag, blocksize);

    if((engineFlag & hwResTbl) == 0) //Free HW available
    {
    
        for(i=0;i<CMDQ_THREAD_NUM;i++) //Find new free thread
        {
            if(cmdqThreadResTbl[i] == 0)
            {
                cmdqThread = i;
                break;
            }
        }
        
        if(cmdqThread != -1)
        {
            cmdqThreadResTbl[cmdqThread] = engineFlag;
            taskResOccuTbl[taskID].cmdqThread = cmdqThread;
        }
        else
        {
            CMDQ_ERR("Cannot find CMDQ thread\n");
            cmdqForceFree_SW(taskID);
            spin_unlock_irqrestore(&gCmdqMgrLock,flags); 
            return false;
        }

       //Update HE resource TBL
       hwResTbl |= engineFlag;


        //Get Buffer info
        pCmdqAddr = cmdqBufAddr(taskID); //new Thread, current taskID must be first
        if(NULL == pCmdqAddr)
        {
            CMDQ_ERR("CmdQ buf address is NULL\n");
            cmdqForceFree_SW(taskID);
            spin_unlock_irqrestore(&gCmdqMgrLock,flags); 
            return false;
        }

        //Start! Power on(TODO)!
       
        cmdqHwClockOn(cmdqThreadResTbl[cmdqThread],true);


        //Update EOC Cnt
        cmdqThreadFirstJobID[cmdqThread] = taskID;
        cmdqThreadTaskNumber[cmdqThread] = 1; //first Job

        cmdqThreadEofCnt[taskID] = DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread)) + 1; //should be "1" always!!

        if(cmdqThreadEofCnt[taskID] > 65535)
        {
            cmdqThreadEofCnt[taskID] = cmdqThreadEofCnt[taskID] - 65536; //overflow
            ring = 1;
        }
        
        printk(KERN_DEBUG "First Task %d done until %d equals %ld\n", taskID, DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread)), cmdqThreadEofCnt[taskID] );
        
       //Insert job to CMDQ thread
        cmdqThreadTaskList[cmdqThread][cmdqThreadTaskList_W[cmdqThread]] = taskID; //assign task to T' write pointer
        cmdqThreadTaskList_W[cmdqThread] = (cmdqThreadTaskList_W[cmdqThread] + 1) % CMDQ_THREAD_LIST_LENGTH; //increase write pointer

        
        CMDQ_MSG("\n\n\ncmdqTaskAssigned R: %d W: %d \n\n\n",cmdqThreadTaskList_R[cmdqThread], cmdqThreadTaskList_W[cmdqThread]);

       // CMDQ_MSG("======cmdqTaskAssigned====\n");
       // dumpDebugInfo();
       // CMDQ_MSG("======cmdqTaskAssigned====\n");
                
        //Update CMDQ buffer parameter (CMDQ size / tail pointer)
        buf_id = taskResOccuTbl[taskID].cmdBufID;
        cmdq_pc_head = (unsigned long*)pCmdqAddr->VA;
        cmdqBufTbl[buf_id].blocksize = blocksize;
        cmdqBufTbl[buf_id].blockTailAddr = (cmdq_pc_head+ins_leng-1);

        //DBG message
        printk(KERN_DEBUG "==========DISP_IOCTL_EXEC_COMMAND Task: %d ,Thread: %d, PC[0x%lx], EOC[0x%lx] =========\n",taskID, cmdqThread,(unsigned long)pCmdqAddr->MVA ,(unsigned long)(pCmdqAddr->MVA + cmdqBufTbl[buf_id].blocksize));

        // get RDMA status before start to run
        if (0 == rdma_fail_cnt)
        {
            rdma_status0 = DISP_REG_GET(0xF4001400);
            rdma_status1 = DISP_REG_GET(0xF4001408);
            rdma_status2 = DISP_REG_GET(0xF4001410);
            rdma_status4 = DISP_REG_GET(0xF4001420);
            rdma_status6 = DISP_REG_GET(0xF4001430);
            rdma_status26 = DISP_REG_GET(0xF40014D0);
        }


        // enable CMDQ interrupt and set timeout cycles
        DISP_REG_SET(DISP_REG_CMDQ_THRx_EN(cmdqThread), 1);
        
        DISP_REG_SET(DISP_REG_CMDQ_THRx_IRQ_FLAG_EN(cmdqThread),0x1);  //Enable Each IRQ        

        DISP_REG_SET(DISP_REG_CMDQ_THRx_INSTN_TIMEOUT_CYCLES(cmdqThread), CMDQ_TIMEOUT);  //Set time out IRQ: 2^20 cycle

        DISP_REG_SET(DISP_REG_CMDQ_THRx_SUSPEND(cmdqThread),1);
        cmdq_polling_timeout = 0;
        while((DISP_REG_GET(DISP_REG_CMDQ_THRx_STATUS(cmdqThread))&0x2) == 0)
        {
            cmdq_polling_timeout++;
            if(cmdq_polling_timeout>1000)
            {
                break;
            }
        }        

        //Execuction
        DISP_REG_SET(DISP_REG_CMDQ_THRx_PC(cmdqThread), pCmdqAddr->MVA);
        //printk("1 Set PC to 0x%x",pCmdqAddr->MVA);
        DISP_REG_SET(DISP_REG_CMDQ_THRx_END_ADDR(cmdqThread), pCmdqAddr->MVA + cmdqBufTbl[buf_id].blocksize);           
            
        DISP_REG_SET(DISP_REG_CMDQ_THRx_SUSPEND(cmdqThread),0);
            


        spin_unlock_irqrestore(&gCmdqMgrLock,flags);     
        //CS--------------------------------------------------

//Schedule out
        //wait_event_interruptible(cmq_wait_queue[cmdqThread], (taskIDStatusTbl[taskID] == -1));        

        //wq_ret = wait_event_interruptible_timeout(cmq_wait_queue[cmdqThread], (taskIDStatusTbl[taskID] == 3), HZ/10); 
        //wq_ret = wait_event_interruptible_timeout(cmq_wait_queue[cmdqThread], (DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread)) >= cmdqThreadEofCnt[taskID]) , HZ/10);
        if(ring == 0)
            wq_ret = wait_event_interruptible_timeout(cmq_wait_queue[cmdqThread], (DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread))>=cmdqThreadEofCnt[taskID]), HZ);
        else
            wq_ret = wait_event_interruptible_timeout(cmq_wait_queue[cmdqThread],((DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread))<65500)&&(DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread))>=cmdqThreadEofCnt[taskID])), HZ);
        
        smp_rmb();

        do_gettimeofday(&end_t);        
        
//Clear Status
        spin_lock_irqsave(&gCmdqMgrLock,flags); 

        if(wq_ret != 0)
        {
            cmdqThreadComplete(cmdqThread, taskID);             
        }    
        if(wq_ret == 0)//if(taskIDStatusTbl[taskID] != -1)
        {            
            CMDQ_ERR("A Task %d [%d] CMDQ Status, PC: 0x%x 0x%x\n",taskID, taskIDStatusTbl[taskID], DISP_REG_GET(DISP_REG_CMDQ_THRx_PC(cmdqThread)), DISP_REG_GET(DISP_REG_CMDQ_THRx_END_ADDR(cmdqThread)));
            printk("Task %d : %d %ld", taskID, DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread)), cmdqThreadEofCnt[taskID] );

            if(pCmdqAddr!=NULL)
                dumpRegDebugInfo(cmdqThreadResTbl[cmdqThread], cmdqThread, *pCmdqAddr);            
            
            //Warm reset CMDQ!

            // CMDQ FD interrupt received, but wait queue timeout
            if((ring == 0)&&(DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread))>=cmdqThreadEofCnt[taskID]))
            {
                wait_queue_timeout_cnt++;
                cmdqThreadComplete(cmdqThread, taskID); 
                CMDQ_ERR("Status A\n");
            }
            else if ((ring == 1)&&((DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread))<65500)&&(DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread))>=cmdqThreadEofCnt[taskID])))
            {
                wait_queue_timeout_cnt++;
                cmdqThreadComplete(cmdqThread, taskID); 
                CMDQ_ERR("Status B\n");
            }
            else // True Timeout
            {
                CMDQ_ERR("Status C\n");
                ret = false;
                
                if(taskIDStatusTbl[taskID]==2)
                {
                    CMDQ_ERR("Status D\n");
                }
                else
                {
                    //assert aee
                    if((cmdq_inst3&0x08000000) !=0) //Polling some register
                    {
                        if((cmdq_inst3&0x00400000)!=0) //22 bit == 1 -> ISP
                        {
                            CMDQ_ISP_AEE("Polling ISP timout in CMDQ, current inst = 0x%08x  0x%08x\n", cmdq_inst2, cmdq_inst3);
                        }
                        else //22 bit == 0 -> MDP
                        {
                            CMDQ_MDP_AEE("Polling MDP timout in CMDQ, current inst = 0x%08x  0x%08x\n", cmdq_inst2, cmdq_inst3);
                        }
                    }
                    else if((cmdq_inst3&0x20000000) !=0) //waiting some event
                    {
                        if((cmdq_inst3&0xFFFFFF)==0x17 || (cmdq_inst3&0xFFFFFF)==0x18)
                        {
                            CMDQ_ISP_AEE("Wait ISP event timout in CMDQ, current inst = 0x%08x  0x%08x\n", cmdq_inst2, cmdq_inst3);
                        }
                        else
                        {
                            CMDQ_MDP_AEE("Wait MDP event timout in CMDQ, current inst = 0x%08x  0x%08x\n", cmdq_inst2, cmdq_inst3);
                        }
                    }
                    else
                    {
                        CMDQ_MDP_AEE("Wait MM command queue timeout, current inst = 0x%08x  0x%08x\n", cmdq_inst2, cmdq_inst3);
                    }
                        
                    cmdqForceFreeAll(cmdqThread);
                }

            }
           
        }

        taskIDStatusTbl[taskID] = -1; //free at last

        spin_unlock_irqrestore(&gCmdqMgrLock,flags);     

    }
    else // no free HW
    {

 //       //CS++++++++++++++++++++++++++++++++++++++++++++++++++
 //       spin_lock_irqsave(&gCmdqMgrLock,flags); 

    
        CMDQ_MSG("======CMDQ: No Free HW resource====\n");

         // enable CMDQ interrupt and set timeout cycles
        DISP_REG_SET(DISP_REG_CMDQ_THRx_IRQ_FLAG_EN(cmdqThread),0x1);  //Enable Each IRQ        
        DISP_REG_SET(DISP_REG_CMDQ_THRx_INSTN_TIMEOUT_CYCLES(cmdqThread), CMDQ_TIMEOUT);  //Set time out IRQ: 2^20 cycle

        //Find Match HW in CMDQ
        for(i=0;i<CMDQ_THREAD_NUM;i++) //Find new free thread
        {
            //CMDQ_ERR("Findind....ThreadRes %x  engineFlag %x=========\n",cmdqThreadResTbl[i] , engineFlag);
            if(cmdqThreadResTbl[i] == engineFlag) //Use Same HW
            {
                cmdqThread = i;

                
                if(cmdqThreadTaskList_W[cmdqThread]==0)
                    pre_w_ptr = (CMDQ_THREAD_LIST_LENGTH - 1); //round
                else
                    pre_w_ptr = cmdqThreadTaskList_W[cmdqThread]-1;
                
                 pre_task_id = cmdqThreadTaskList[cmdqThread][pre_w_ptr];
                
                //Get Buffer info
                pCmdqAddr = cmdqBufAddr(taskID); 
                pPre_cmdqAddr = cmdqBufAddr(pre_task_id); 
                if((NULL == pCmdqAddr))
                {
                    CMDQ_ERR("CmdQ buf address is NULL ,taskID %d, CMDQ 0x%x\n" , taskID, (unsigned int)pCmdqAddr);
                    dumpDebugInfo();
                    cmdqForceFree_SW(taskID);           
                    spin_unlock_irqrestore(&gCmdqMgrLock,flags); 
                    return false;
                }

                if(DISP_REG_GET(DISP_REG_CMDQ_THRx_EN(cmdqThread)) == 0) 
                {
                        //Warm reset
                        DISP_REG_SET(DISP_REG_CMDQ_THRx_RESET(cmdqThread), 1);
                        //PC reset
                        DISP_REG_SET(DISP_REG_CMDQ_THRx_PC(cmdqThread), pCmdqAddr->MVA);
                        //printk("2 Set PC to 0x%x",pCmdqAddr->MVA);
                        //EN
                        DISP_REG_SET(DISP_REG_CMDQ_THRx_EN(cmdqThread), 1);
                }
                else
                {
                    DISP_REG_SET(DISP_REG_CMDQ_THRx_SUSPEND(cmdqThread),1);
                    cmdq_polling_timeout = 0;
                    while((DISP_REG_GET(DISP_REG_CMDQ_THRx_STATUS(cmdqThread))&0x2) == 0)
                    {
                        cmdq_polling_timeout++;
                        if(cmdq_polling_timeout>1000)
                        {
                            CMDQ_ERR("CMDQ SUSPEND fail!%x %x %x %x\n", taskID ,DISP_REG_GET(DISP_REG_CMDQ_THRx_EN(cmdqThread)), DISP_REG_GET(DISPSYS_CMDQ_BASE+0x78), DISP_REG_GET(DISPSYS_CMDQ_BASE+0x7c));
                            //Warm reset
                            DISP_REG_SET(DISP_REG_CMDQ_THRx_RESET(cmdqThread), 1);
                            //PC reset
                            DISP_REG_SET(DISP_REG_CMDQ_THRx_PC(cmdqThread), pCmdqAddr->MVA);
                            //printk("3 Set PC to 0x%x",pCmdqAddr->MVA);
                            //EN
                            DISP_REG_SET(DISP_REG_CMDQ_THRx_EN(cmdqThread), 1);
                            break;
                        }
                    }
                }
                CMDQ_MSG("PC: 0x%x -> 0x%x\n",(unsigned int)DISP_REG_GET(DISP_REG_CMDQ_THRx_PC(cmdqThread)),(unsigned int)DISP_REG_GET(DISP_REG_CMDQ_THRx_END_ADDR(cmdqThread)));
                CMDQ_MSG("======CMDQ: Task %d [%x] Find Matched Thread: %d [%x] and suspend====\n",taskID,engineFlag,cmdqThread, cmdqThreadResTbl[cmdqThread]);
                break;
            }
            else if((cmdqThreadResTbl[i] & engineFlag) != 0) //Overlaped HW
            {
                cmdqThread = i;

                if(cmdqThreadTaskList_W[cmdqThread]==0)
                    pre_w_ptr = (CMDQ_THREAD_LIST_LENGTH - 1); //round
                else
                    pre_w_ptr = cmdqThreadTaskList_W[cmdqThread]-1;
                
                 pre_task_id = cmdqThreadTaskList[cmdqThread][pre_w_ptr];
                
                //Get Buffer info
                pCmdqAddr = cmdqBufAddr(taskID); 
                pPre_cmdqAddr = cmdqBufAddr(pre_task_id); 

               
                if((NULL == pCmdqAddr))
                {
                    CMDQ_ERR("CmdQ buf address is NULL ,taskID %d, CMDQ 0x%x\n" , taskID, (unsigned int)pCmdqAddr);
                    dumpDebugInfo();
                    cmdqForceFree_SW(taskID);           
                    spin_unlock_irqrestore(&gCmdqMgrLock,flags); 
                    return false;
                }
               
                if(DISP_REG_GET(DISP_REG_CMDQ_THRx_EN(cmdqThread)) == 0) 
                {
                        //Warm reset
                        DISP_REG_SET(DISP_REG_CMDQ_THRx_RESET(cmdqThread), 1);
                        //PC reset
                        DISP_REG_SET(DISP_REG_CMDQ_THRx_PC(cmdqThread), pCmdqAddr->MVA);
                        //printk("4 Set PC to 0x%x",pCmdqAddr->MVA);
                        //EN
                        DISP_REG_SET(DISP_REG_CMDQ_THRx_EN(cmdqThread), 1);
                }
                else
                {
                    DISP_REG_SET(DISP_REG_CMDQ_THRx_SUSPEND(cmdqThread),1);
                    cmdq_polling_timeout = 0;
                    while((DISP_REG_GET(DISP_REG_CMDQ_THRx_STATUS(cmdqThread))&0x2) == 0)
                    {
                        cmdq_polling_timeout++;
                        if(cmdq_polling_timeout>1000)
                        {
                            CMDQ_ERR("CMDQ SUSPEND fail!%x %x %x %x\n", taskID ,DISP_REG_GET(DISP_REG_CMDQ_THRx_EN(cmdqThread)), DISP_REG_GET(DISPSYS_CMDQ_BASE+0x78), DISP_REG_GET(DISPSYS_CMDQ_BASE+0x7c));
                            //Warm reset
                            DISP_REG_SET(DISP_REG_CMDQ_THRx_RESET(cmdqThread), 1);
                            //PC reset
                            DISP_REG_SET(DISP_REG_CMDQ_THRx_PC(cmdqThread), pCmdqAddr->MVA);
                            //printk("5 Set PC to 0x%x",pCmdqAddr->MVA);
                            //EN
                            DISP_REG_SET(DISP_REG_CMDQ_THRx_EN(cmdqThread), 1);
                            break;
                        }
                    }
                }


                CMDQ_MSG("PC: 0x%x -> 0x%x\n",(unsigned int)DISP_REG_GET(DISP_REG_CMDQ_THRx_PC(cmdqThread)),(unsigned int)DISP_REG_GET(DISP_REG_CMDQ_THRx_END_ADDR(cmdqThread)));
                
                CMDQ_MSG("======CMDQ: Task %d [%x] Find Corresponding Thread: %d [%x] and suspend====\n",taskID,engineFlag,cmdqThread, cmdqThreadResTbl[cmdqThread]);
                if((engineFlag&~(cmdqThreadResTbl[i]))!=0) //More Engine then Current CMDQ T
                {
                   //POWER ON! (TODO)
                    cmdqHwClockOn((engineFlag&~(cmdqThreadResTbl[i])),false);
                   //Update CMDQ Thread Table
                   cmdqThreadResTbl[i] |= engineFlag;
                   //Update HE resource TBL
                   hwResTbl |= engineFlag;
                   
                   CMDQ_MSG("========update CMDQ T %d resource %x=======\n",cmdqThread, cmdqThreadResTbl[cmdqThread]);
                }
                break;

            }
        }

        if(cmdqThread == -1) //cannot find any thread
        {
            CMDQ_ERR("=========CMDQ Job append Error happen!! %x %x=================\n",engineFlag, hwResTbl);

            if((engineFlag & hwResTbl) == 0) //Free HW available
            {
                CMDQ_ERR("=========DAMN!!!!!!!!=================\n");
            }
            for(i=0;i<CMDQ_THREAD_NUM;i++) //Find new free thread
            {
                CMDQ_ERR("ThreadRes %x  engineFlag %x\n",cmdqThreadResTbl[i] , engineFlag);
            }
            cmdqForceFree_SW(taskID);
            spin_unlock_irqrestore(&gCmdqMgrLock,flags);            
            return false;
        }

        //Update EOC Cnt
        cmdqThreadTaskNumber[cmdqThread] ++; //n-th jobs

        if(cmdqThreadFirstJobID[cmdqThread] == -1)
        {
            CMDQ_ERR("cmdqThreadFirstJobID is NULL\n");
            cmdqForceFree_SW(taskID);           
            spin_unlock_irqrestore(&gCmdqMgrLock,flags); 
            return false;
        }
            

        //cmdqThreadEofCnt[taskID] = cmdqThreadEofCnt[cmdqThreadFirstJobID[cmdqThread]] + cmdqThreadTaskNumber[cmdqThread] - 1;

        //Because cmdqThreadEofCnt[cmdqThreadFirstJobID[cmdqThread]] == 1 always, cmdqThreadEofCnt[taskID] == cmdqThreadTaskNumber[cmdqThread] 
        cmdqThreadEofCnt[taskID] = cmdqThreadTaskNumber[cmdqThread] ;

        if(cmdqThreadEofCnt[taskID] > 65535)
        {
            cmdqThreadEofCnt[taskID] = cmdqThreadEofCnt[taskID] - 65536; //overflow
            ring = 1;
        }
        
        printk(KERN_DEBUG "%d-th Task %d done until %d equals %ld\n", cmdqThreadTaskNumber[cmdqThread] , taskID, DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread)), cmdqThreadEofCnt[taskID] );

        //Insert job to CMDQ thread
    
        cmdqThreadTaskList[cmdqThread][cmdqThreadTaskList_W[cmdqThread]] = taskID; //assign task to T' write pointer
        cmdqThreadTaskList_W[cmdqThread] = (cmdqThreadTaskList_W[cmdqThread] + 1) % CMDQ_THREAD_LIST_LENGTH; //increase write pointer
        
        CMDQ_MSG("\n\n\ncmdqTaskAssigned(Insert) R: %d W: %d \n\n\n",cmdqThreadTaskList_R[cmdqThread], cmdqThreadTaskList_W[cmdqThread]);


        //Update CMDQ buffer parameter (CMDQ size / tail pointer)
        buf_id = taskResOccuTbl[taskID].cmdBufID;
        cmdq_pc_head = (unsigned long *)pCmdqAddr->VA;
        cmdqBufTbl[buf_id].blocksize = blocksize;
        cmdqBufTbl[buf_id].blockTailAddr = (cmdq_pc_head+ins_leng-1);
       
        printk(KERN_DEBUG "==========DISP_IOCTL_EXEC_COMMAND+ Task: %d ,Thread: %d, PC[0x%lx], EOC[0x%lx] =========\n",taskID, cmdqThread,(unsigned long)pCmdqAddr->MVA ,(unsigned long)(pCmdqAddr->MVA + cmdqBufTbl[buf_id].blocksize));

        //Thread is already complete, but ISR have not coming yet
        if(DISP_REG_GET(DISP_REG_CMDQ_THRx_PC(cmdqThread))==DISP_REG_GET(DISP_REG_CMDQ_THRx_END_ADDR(cmdqThread)))
        {
            DISP_REG_SET(DISP_REG_CMDQ_THRx_PC(cmdqThread), pCmdqAddr->MVA);
            //printk("6 Set PC to 0x%x",pCmdqAddr->MVA);
            printk(KERN_DEBUG "\n==============Reset %d's PC  to ADDR[0x%lx]===================\n",cmdqThread, pCmdqAddr->MVA);
        }
        else
        {
            
            if((NULL == pPre_cmdqAddr)) 
            {
                CMDQ_ERR("CmdQ pre-buf address is NULL ,pre_task_id %d, pPre_cmdqAddr 0x%x\n" , pre_task_id, (unsigned int)pPre_cmdqAddr);
                dumpDebugInfo();
                cmdqForceFree_SW(taskID);   
                spin_unlock_irqrestore(&gCmdqMgrLock,flags); 
                return false;

            }
        
            *(pPre_cmdqAddr->blockTailAddr) = 0x10000001;//Jump: Absolute
            *(pPre_cmdqAddr->blockTailAddr-1) = pCmdqAddr->MVA; //Jump to here

            CMDQ_MSG("\n==============Modify %d's Pre-ID %d Jump ADDR[0x%lx] :0x%lx , 0x%lx ===================\n",cmdqThread, pre_task_id ,(unsigned long)(pPre_cmdqAddr->blockTailAddr) ,*(pPre_cmdqAddr->blockTailAddr) ,*(pPre_cmdqAddr->blockTailAddr-1) );
        }
        //Modify Thread END addr
        DISP_REG_SET(DISP_REG_CMDQ_THRx_END_ADDR(cmdqThread), pCmdqAddr->MVA + cmdqBufTbl[buf_id].blocksize);         

        DISP_REG_SET(DISP_REG_CMDQ_THRx_SUSPEND(cmdqThread),0);
        spin_unlock_irqrestore(&gCmdqMgrLock,flags);  
        //CS--------------------------------------------------

        //Schedule out
        //wait_event_interruptible(cmq_wait_queue[cmdqThread], (taskIDStatusTbl[taskID] == -1));        
//    wq_ret = wait_event_interruptible_timeout(cmq_wait_queue[cmdqThread], (taskIDStatusTbl[taskID] == 3), HZ/10);
        if(ring == 0)
            wq_ret = wait_event_interruptible_timeout(cmq_wait_queue[cmdqThread], (DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread))>=cmdqThreadEofCnt[taskID]), HZ);
        else
            wq_ret = wait_event_interruptible_timeout(cmq_wait_queue[cmdqThread],((DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread))<65500)&&(DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread))>=cmdqThreadEofCnt[taskID])), HZ);

        
        smp_rmb();

        do_gettimeofday(&end_t);
        
        //Clear Status
        spin_lock_irqsave(&gCmdqMgrLock,flags); 

        if(wq_ret != 0)
        {
            cmdqThreadComplete(cmdqThread, taskID); 
        }   
        
        if(wq_ret == 0)//if(taskIDStatusTbl[taskID] != -1)
        {
            CMDQ_ERR("B Task %d [%d] CMDQ Status, PC: 0x%x 0x%x\n",taskID, taskIDStatusTbl[taskID], DISP_REG_GET(DISP_REG_CMDQ_THRx_PC(cmdqThread)), DISP_REG_GET(DISP_REG_CMDQ_THRx_END_ADDR(cmdqThread)));
            printk("Task %d : %d %ld", taskID, DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread)), cmdqThreadEofCnt[taskID] );

            if(pCmdqAddr!=NULL)
                dumpRegDebugInfo(cmdqThreadResTbl[cmdqThread], cmdqThread, *pCmdqAddr);            

            
            //Warm reset CMDQ!
            
            // CMDQ FD interrupt received, but wait queue timeout
            if((ring == 0)&&(DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread))>=cmdqThreadEofCnt[taskID]))
            {
                wait_queue_timeout_cnt++;
                cmdqThreadComplete(cmdqThread, taskID); 
                CMDQ_ERR("Status E\n");
            }
            else if ((ring == 1)&&((DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread))<65500)&&(DISP_REG_GET(DISP_REG_CMDQ_THRx_EXEC_CMDS_CNT(cmdqThread))>=cmdqThreadEofCnt[taskID])))
            {
                wait_queue_timeout_cnt++;
                cmdqThreadComplete(cmdqThread, taskID); 
                CMDQ_ERR("Status F\n");
            }
            else // True Timeout
            {
                if (DISP_REG_GET(DISP_REG_CMDQ_THRx_PC(cmdqThread))==DISP_REG_GET(DISP_REG_CMDQ_THRx_END_ADDR(cmdqThread)))
                {
                    cmdq_isr_fail_cnt++;
                }
                CMDQ_ERR("Status G\n");
                ret = false;

                if(taskIDStatusTbl[taskID]==2)
                {
                    CMDQ_ERR("Status H\n");
                }
                else
                {
                    //assert aee
                    if((cmdq_inst3&0x08000000) !=0) //Polling some register
                    {
                        if((cmdq_inst3&0x00400000)!=0) //22 bit == 1 -> ISP
                        {
                            CMDQ_ISP_AEE("Polling ISP timout in CMDQ, current inst = 0x%08x  0x%08x\n", cmdq_inst2, cmdq_inst3);
                        }
                        else //22 bit == 0 -> MDP
                        {
                            CMDQ_MDP_AEE("Polling MDP timout in CMDQ, current inst = 0x%08x  0x%08x\n", cmdq_inst2, cmdq_inst3);
                        }
                    }
                    else if((cmdq_inst3&0x20000000) !=0) //waiting some event
                    {
                        if((cmdq_inst3&0xFFFFFF)==0x17 || (cmdq_inst3&0xFFFFFF)==0x18)
                        {
                            CMDQ_ISP_AEE("Wait ISP event timout in CMDQ, current inst = 0x%08x  0x%08x\n", cmdq_inst2, cmdq_inst3);
                        }
                        else
                        {
                            CMDQ_MDP_AEE("Wait MDP event timout in CMDQ, current inst = 0x%08x  0x%08x\n", cmdq_inst2, cmdq_inst3);
                        }
                    }
                    else
                    {
                        CMDQ_MDP_AEE("Wait MM command queue timeout, current inst = 0x%08x  0x%08x\n", cmdq_inst2, cmdq_inst3);
                    }
                    
                    cmdqForceFreeAll(cmdqThread);
                }           
            }
           
        }

        taskIDStatusTbl[taskID] = -1; //free at last
        spin_unlock_irqrestore(&gCmdqMgrLock,flags);  

        
    }

    if (0 == rdma_fail_cnt)
    {
        frame_interval.tv_sec = end_t.tv_sec - start_t.tv_sec;
        frame_interval.tv_usec = end_t.tv_usec - start_t.tv_usec;
    }

//    spin_unlock(&gCmdqMgrLock);
    return ret;
}

void cmdqThreadPowerOff(int cmdqThread)
{
    // check MDP engine busy
    if(true == checkMdpEngineStatus(cmdqThreadResTbl[cmdqThread]))
    {
        printk("cmdqThreadPowerOff when MDP Busy!\n");
        mdp_busy_long_cnt++;
        resetMdpEngine(cmdqThreadResTbl[cmdqThread]);
    }

    
    cmdqHwClockOff(cmdqThreadResTbl[cmdqThread]);
    
    //Return HW resource
    hwResTbl &= ~(cmdqThreadResTbl[cmdqThread]);
    
    //clear T' res table
    cmdqThreadResTbl[cmdqThread] = 0;         
    CMDQ_MSG("\n======All job complete in cmdqThread: %d ====\n", cmdqThread);


}


void cmdqThreadComplete(int cmdqThread, int taskID)
{

    cmdqResource_free(taskID); //task complete

    //CMDQ read pointer ++
    cmdqThreadTaskList[cmdqThread][cmdqThreadTaskList_R[cmdqThread]] = 0; //Mark for complete at T' read pointer
    cmdqThreadTaskList_R[cmdqThread] = (cmdqThreadTaskList_R[cmdqThread] + 1) % CMDQ_THREAD_LIST_LENGTH; //increase Read pointer


//0322
    if(cmdqThreadTaskList_R[cmdqThread] == cmdqThreadTaskList_W[cmdqThread]) //no task needed
    {
        //power off!
        CMDQ_MSG("============cmdqThreadComplete Task! %d R: %d W: %d \n",taskID,cmdqThreadTaskList_R[cmdqThread], cmdqThreadTaskList_W[cmdqThread]);
        cmdqThreadPowerOff(cmdqThread);
    }

}

void cmdqForceFreeAll(int cmdqThread)
{
    //SW force init
    int i = 0;
    unsigned int cmdq_polling_timeout;

    printk("Status X\n");

    for(i=0;i<CMDQ_BUFFER_NUM;i++)
    {
        cmdqBufTbl[i].Owner = -1; //free buffer
    }

    for(i=0;i<MAX_CMDQ_TASK_ID;i++)
    {
        if(taskIDStatusTbl[i]!=-1)
            taskIDStatusTbl[i] = 2; //mark as CANCEL
        taskResOccuTbl[i].cmdBufID = -1;
        taskResOccuTbl[i].cmdqThread= -1;
    }



    //Warm reset CMDQ
    DISP_REG_SET(DISP_REG_CMDQ_THRx_RESET(cmdqThread), 1);
    cmdq_polling_timeout = 0;
    while(DISP_REG_GET(DISP_REG_CMDQ_THRx_RESET(cmdqThread)) == 1)
    {
        cmdq_polling_timeout++;
        if(cmdq_polling_timeout>1000)
        {
            CMDQ_ERR("CMDQ warm reset status%x %x %x\n",DISP_REG_GET(DISP_REG_CMDQ_THRx_EN(cmdqThread)), DISP_REG_GET(DISPSYS_CMDQ_BASE+0x78), DISP_REG_GET(DISPSYS_CMDQ_BASE+0x7c));
            break;
        }
    }     

    //Warm reset MDP
    resetMdpEngine(cmdqThreadResTbl[cmdqThread]);

    //Resource Free
    for(i=0;i<CMDQ_THREAD_NUM;i++)
    {
        cmdqThreadResTbl[i] = 0; 
        cmdqThreadTaskList_R[i] = 0;
        cmdqThreadTaskList_W[i] = 0;
        cmdqThreadTaskNumber[i] = 0;
        cmdqThreadFirstJobID[i] = -1;
    }
    
    hwResTbl = 0;

    if (clock_is_on(MT_CG_DISP0_MDP_RDMA))
        disable_clock(MT_CG_DISP0_MDP_RDMA, "MDP_RDMA");
    if (clock_is_on(MT_CG_DISP0_MDP_RSZ0))
        disable_clock(MT_CG_DISP0_MDP_RSZ0, "MDP_RSZ0");
    if (clock_is_on(MT_CG_DISP0_MDP_RSZ1))
        disable_clock(MT_CG_DISP0_MDP_RSZ1, "MDP_RSZ1");
    if (clock_is_on(MT_CG_DISP0_MDP_TDSHP))
        disable_clock(MT_CG_DISP0_MDP_TDSHP, "MDP_TDSHP");
    if (clock_is_on(MT_CG_DISP0_MDP_WROT))
        disable_clock(MT_CG_DISP0_MDP_WROT, "MDP_WROT");
    if (clock_is_on(MT_CG_DISP0_MDP_WDMA))
        disable_clock(MT_CG_DISP0_MDP_WDMA, "MDP_WDMA");
//    if (clock_is_on(MT_CG_MM_CMDQ_SW_CG))
//        disable_clock(MT_CG_MM_CMDQ_SW_CG, "MM_CMDQ");
//    if (clock_is_on(MT_CG_MM_CMDQ_SMI_IF_SW_CG))
//        disable_clock(MT_CG_MM_CMDQ_SMI_IF_SW_CG, "MM_CMDQ_SMI_IF");

}


void cmdqForceFree_SW(int taskID)
{
    //SW force init
//    int i = 0;
    int bufID = -1;

    if(taskID == -1 ||taskID>=MAX_CMDQ_TASK_ID)
    {
        printk("\n cmdqForceFree_SW Free Invalid Task ID \n");
        dumpDebugInfo();
        return;
    }

    taskIDStatusTbl[taskID] = -1; //mark for free
    taskResOccuTbl[taskID].cmdBufID = -1;
    taskResOccuTbl[taskID].cmdqThread= -1;
    cmdqThreadEofCnt[taskID]  = -1;

    bufID = taskResOccuTbl[taskID].cmdBufID;

    printk("\n cmdqForceFree_SW Free Buf %x own by [%x=%x]\n",bufID,taskID,cmdqBufTbl[bufID].Owner);

    if(bufID != -1) //Free All resource and return ID
    {
        cmdqBufTbl[bufID].Owner = -1;
    }
    else
    {
        CMDQ_ERR("\n cmdqForceFree_SW Safely Free Invalid Buffer ID\n");
        dumpDebugInfo();
    }

    //FIXME! work around in 6572 (only one thread in 6572)
    hwResTbl = 0;

}


void cmdqTerminated(void)
{
    unsigned long flags;

    spin_lock_irqsave(&gCmdqMgrLock,flags); 

    if((cmdqThreadTaskList_R[0] == cmdqThreadTaskList_W[0]) &&  (cmdqThreadResTbl[0] !=0)) //no task needed, but resource leaked!
    {
        CMDQ_ERR("\n======CMDQ Process terminated handling : %d ====\n", cmdqThreadResTbl[0]);

        cmdqForceFreeAll(0);
    }

    spin_unlock_irqrestore(&gCmdqMgrLock,flags);  
}

int cmdqRegisterCallback(int index, CMDQ_TIMEOUT_PTR pTimeoutFunc , CMDQ_RESET_PTR pResetFunc)
{
    if((index>=2)||(NULL == pTimeoutFunc) || (NULL == pResetFunc))
    {
        printk("Warning! [Func]%s register NULL function : %p,%p\n", __func__ , pTimeoutFunc , pResetFunc);
        return -1;
    }

    g_CMDQ_CB_Array.cmdqTimeout_cb[index] = pTimeoutFunc;
    g_CMDQ_CB_Array.cmdqReset_cb[index] = pResetFunc;


    return 0;
}




