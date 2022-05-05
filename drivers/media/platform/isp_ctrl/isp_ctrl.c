/*
 * Interface for the Renesas RZ/V2L CRU and ISP Library
 *
 * Copyright (C) 2021 Renesas Electronics Corporation
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <media/v4l2-event.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/isp_ctrl.h>
#include <linux/rzg2l_isp_ctrl.h>
#include <linux/kthread.h>
#include <linux/simple_isp.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/simple_isp_default.h>
#include <uapi/linux/renesas-v4l2-controls.h>
#include <media/videobuf2-v4l2.h>

//#define DBG_QUEUE // for queue debug
#ifdef DBG_QUEUE
#define qprintk(fmt, arg...)		printk(fmt, ##arg)
#else
#define qprintk(fmt, arg...)
#endif

//#define DBG_DQBUF
#ifdef DBG_DQBUF
#define dqbufprintk(fmt, arg...)		printk(fmt, ##arg)
#else
#define dqbufprintk(fmt, arg...)
#endif

#define ISP_CTRL_MAX_CAM_ADR_IDX       (4)
#define ISP_CTRL_DRP_LEVEL_DEF_VAL		(-1)	/* May change in the future */
#define ISP_CTRL_OUT_FMT_YUY2           (0x00)
#define ISP_CTRL_OUT_FMT_UYVY           (0x01)
#define ISP_CTRL_OUT_FMT_RGB24          (0x10)
#define ISP_CTRL_OUT_FMT_ARGB32         (0x11)
#define ISP_CTRL_OUT_FMT_XRGB32         (0x12)
// for change format
#define ISP_CTRL_SET_IN_FMT				(0x01)
#define ISP_CTRL_CHGTBL_MAX_NUM			(0x03)  // for table index & check infmt value
#define ISP_CTRL_G_EXT_CTRLS_NUM_ID		(9)

typedef struct isp_open_info {
//	enum v4l2_rz_isp_in_fmt_range     inFMT;
	u32   inFMT;
	u32   outFMT;
	u32   width;
	u32   height;
	u32   frameInterval;
	u8    opened;
/* ISP MOD-K START */
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *expose;
/* ISP MOD-K END   */
	struct task_struct *td;
	struct task_struct *td2;
}t_isp_info;

typedef struct v4l2_rz_isp_set_AE{
	int ae_on;
	int expose_level;
	int target_y;
	int threshold;
}t_v4l2_rz_isp_set_AE;

typedef struct isp_wakeup_entry{
	int					errCode; 
	unsigned int		index;
	struct list_head	list;
}t_isp_wakeup_entry;

typedef struct isplib_FuncIdTbl{
	__u32	cid;
	int		fid;
}t_isplib_FuncIdTbl;

static t_isp_dma_addr_entry dma_addr_info_head = {0,0};
static t_isp_wakeup_entry wakeup_entry_head;
static int drpLevel = ISP_CTRL_DRP_LEVEL_DEF_VAL;

static t_isp_info ispOpenInfo;

static u8 cam_image_physadr_got = 0;
unsigned long cam_image_physadr_list[ISP_CTRL_MAX_CAM_ADR_IDX];    // RAW Image buffer
unsigned char waitCallback=0;
static DECLARE_WAIT_QUEUE_HEAD(cru_wait_queue);
static DECLARE_WAIT_QUEUE_HEAD(isp_go_queue);

static u8 get_camera_physadr_idx = 0;
static u8 num_camera_images = 0;
t_v4l2_rz_isp_set_AE setAE;

// for change format
static u8 setINFMTFlag = 0;
static int changeInFmtToBusFmt[ISP_CTRL_CHGTBL_MAX_NUM] = {MEDIA_BUS_FMT_SRGGB8_1X8, MEDIA_BUS_FMT_SRGGB10_1X10, MEDIA_BUS_FMT_SRGGB12_1X12};

struct v4l2_subdev_format changeFmt = { 0 };

wait_queue_head_t* q_head;

spinlock_t wakeup_list_lock;
unsigned int err_dqbuf_index = 0;

static u8 ispRunState = ISP_CTRL_STOP; // ISP RUN FLAG

#define MAX_VB_BUFFER   (8) //MAX CRU SLOT NUM
struct vb2_buffer vb_data[MAX_VB_BUFFER];
static int vb_data_rp;
static int vb_data_wp;

static t_isplib_FuncIdTbl isplibFuncIdTbl[ISP_CTRL_G_EXT_CTRLS_NUM_ID] = {
	{V4L2_CID_RZ_ISP_ALL, ISP_FUNC_ALL},
	{V4L2_CID_RZ_ISP_BL, ISP_FUNC_BL},
	{V4L2_CID_RZ_ISP_WB, ISP_FUNC_WB},
	{V4L2_CID_RZ_ISP_GAMMA, ISP_FUNC_GAMMA},
	{V4L2_CID_RZ_ISP_CMX, ISP_FUNC_CMX},
	{V4L2_CID_RZ_ISP_2DNR, ISP_FUNC_2DNR},
	{V4L2_CID_RZ_ISP_3DNR, ISP_FUNC_3DNR},
	{V4L2_CID_RZ_ISP_EMP, ISP_FUNC_EMP},
	{V4L2_CID_RZ_ISP_AWB, ISP_FUNC_AWB},
};

/*
 * prototype
 */
static int isp_ctrl_feedback_thread(void *arg);
static int isp_ctrl_go_thread(void *arg);
static void isp_ctrl_isp_callback(int result,unsigned long out_adr);

/*
 * public
 */
void isp_ctrl_init(struct file *file)
{
	setINFMTFlag = 0;

	ispOpenInfo.inFMT = ISP_LIMIT_IN_FMT_DEFAULT;
	ispOpenInfo.outFMT = 0;
	ispOpenInfo.width = 0;
	ispOpenInfo.height = 0;
	ispOpenInfo.frameInterval = ISP_LIMIT_FRM_INTERVAL_DEFAULT;
	ispOpenInfo.opened = 0;

	ispOpenInfo.gain = NULL;
	ispOpenInfo.expose = NULL;
	ispOpenInfo.td = NULL;
	ispOpenInfo.td2 = NULL;

	// for AE
	setAE.ae_on = 0;
	setAE.expose_level = 0;
	setAE.target_y = 0;
	setAE.threshold = 0;
	waitCallback = 0;

	wakeup_list_lock = __SPIN_LOCK_UNLOCKED(wakeup_list_lock);
    INIT_LIST_HEAD(&wakeup_entry_head.list);
	err_dqbuf_index = 0;
	ispRunState = ISP_CTRL_RUN;
	cam_image_physadr_got = 0;
	get_camera_physadr_idx = 0;
	num_camera_images = 0;
}

void isp_ctrl_deinit(void)
{
	ispRunState = ISP_CTRL_STOP;
}

u8 isp_ctrl_get_run_state(void)
{
	return ispRunState;
}

unsigned long isp_ctrl_get_camera_phys_addr(void)
{
	unsigned long value = 0;

	if( cam_image_physadr_got ){
		value = cam_image_physadr_list[get_camera_physadr_idx];
		get_camera_physadr_idx++;
		if( get_camera_physadr_idx >= num_camera_images ){
			get_camera_physadr_idx = 0;
		}
	}
	return value;
}

/*
 * wakeup list
 */
#ifdef DBG_QUEUE
static void isp_ctrl_wakeup_show_list(void)
{
	struct list_head *listptr;
	t_isp_wakeup_entry *entry;
	
	dprintk("%s: start\n", __func__);
	qprintk("%s: errCode = %d, index = %d (list %p, prev = %p, next = %p)\n", 
			__func__, wakeup_entry_head.errCode, wakeup_entry_head.index, &wakeup_entry_head.list, wakeup_entry_head.list.prev, wakeup_entry_head.list.next);
	if (list_empty(&wakeup_entry_head.list)) {
		qprintk("%s: list_empty\n", __func__);
	}else{
		list_for_each(listptr, &wakeup_entry_head.list) {
			entry = list_entry(listptr, t_isp_wakeup_entry, list);
			qprintk("%s: errCode = %d, index = %d (list %p, prev = %p, next = %p)\n", 
					__func__, entry->errCode, entry->index, &entry->list, entry->list.prev, entry->list.next);
		}
	}
	dprintk("%s: finish\n", __func__);
}
#endif

void isp_ctrl_wakeup_add_list_tail(int error, unsigned int index)
{
    t_isp_wakeup_entry* set_entry;

    set_entry = kmalloc(sizeof(t_isp_wakeup_entry), GFP_KERNEL);
	dprintk("%s: start\n", __func__);
	set_entry->errCode = error;
	set_entry->index = index;
	list_add_tail(&set_entry->list, &wakeup_entry_head.list);
#ifdef DBG_QUEUE
    qprintk("%s: call isp_ctrl_show_list\n", __func__);
	isp_ctrl_wakeup_show_list();
    qprintk("%s: ret  isp_ctrl_show_list\n", __func__);
#endif
    
	dprintk("%s: finish\n", __func__);
	return;
}

t_isp_wakeup_entry* isp_ctrl_wakeup_get_list(void)
{
	t_isp_wakeup_entry *entry = NULL;

	dprintk("%s: start\n", __func__);
	if (list_empty(&wakeup_entry_head.list)) {
		qprintk("%s: list_empty\n", __func__);
	}else{
		qprintk("%s: NOT list_empty\n", __func__);
		entry = list_entry(wakeup_entry_head.list.next, t_isp_wakeup_entry, list);
		qprintk("errCode = %d, index = %d (list %p, prev = %p, next = %p)\n", 
				entry->errCode, entry->index, &entry->list, entry->list.prev, entry->list.next);
	}
	dprintk("%s: finish\n", __func__);

	return entry;
}

void isp_ctrl_wakeup_del_list(void)
{
	t_isp_wakeup_entry* del_ptr;

	dprintk("%s: start\n", __func__);
	del_ptr = list_entry(wakeup_entry_head.list.next, t_isp_wakeup_entry, list);
    qprintk("%s: list_del: list.next = %p, del_prt = %p\n", __func__, wakeup_entry_head.list.next, del_ptr);
    qprintk("%s: call isp_ctrl_show_list\n", __func__);
#ifdef DBG_QUEUE
	isp_ctrl_wakeup_show_list();
#endif
    qprintk("%s: ret  isp_ctrl_show_list\n", __func__);
    list_del(wakeup_entry_head.list.next);
    kfree(del_ptr);
    qprintk("%s: kfree = %p\n", __func__, del_ptr);
    qprintk("%s: call isp_ctrl_show_list\n", __func__);
#ifdef DBG_QUEUE
	isp_ctrl_wakeup_show_list();
#endif
    qprintk("%s: ret  isp_ctrl_show_list\n", __func__);
	dprintk("%s: finish\n", __func__);
	return;
}

static void isp_ctrl_wakeup_clear_list(void)
{
	t_isp_wakeup_entry *entry;
	t_isp_wakeup_entry *entryNull = NULL;

	dprintk("%s: start\n", __func__);

	if (list_empty(&wakeup_entry_head.list)) {
		qprintk("%s: list_empty\n", __func__);
	}else{
		qprintk("%s: NOT list_empty\n", __func__);
		list_for_each_entry_safe(entry,entryNull,&wakeup_entry_head.list,list){
#ifdef DBG_QUEUE
			isp_ctrl_wakeup_show_list();
#endif
			qprintk("%s: entry = %p, entry->list = %p\n", __func__, entry, &entry->list);
			list_del(&entry->list);
			qprintk("%s: kfree(%p)\n", __func__, entry);
			kfree(entry);
		}
	}
	dprintk("%s: finish\n", __func__);
}

/*
 * dma addr list
 */
#ifdef DBG_QUEUE
static void isp_ctrl_show_list(void)
{
	struct list_head *listptr;
	t_isp_dma_addr_entry *entry;
	
	dprintk("%s: start\n", __func__);
	qprintk("%s: in_addr = 0x%lx, out_addr = 0x%lx (list %p, prev = %p, next = %p)\n", 
			__func__, dma_addr_info_head.in_addr, dma_addr_info_head.out_addr, &dma_addr_info_head.list, dma_addr_info_head.list.prev, dma_addr_info_head.list.next);
	if (list_empty(&dma_addr_info_head.list)) {
		qprintk("%s: list_empty\n", __func__);
	}else{
		list_for_each(listptr, &dma_addr_info_head.list) {
			entry = list_entry(listptr, t_isp_dma_addr_entry, list);
			qprintk("%s: in_addr = 0x%lx, out_addr = 0x%lx (list %p, prev = %p, next = %p)\n", 
					__func__, entry->in_addr, entry->out_addr, &entry->list, entry->list.prev, entry->list.next);
		}
	}
	dprintk("%s: finish\n", __func__);
}
#endif

void isp_ctrl_dma_add_list_tail(unsigned long in_addr, unsigned long out_addr)
{
    t_isp_dma_addr_entry* set_entry;

    set_entry = kmalloc(sizeof(t_isp_dma_addr_entry), GFP_KERNEL);
	dprintk("%s: start\n", __func__);
    qprintk("%s: kmalloc = %p\n", __func__, set_entry);
	set_entry->in_addr = in_addr;
	set_entry->out_addr = out_addr;
	list_add_tail(&set_entry->list, &dma_addr_info_head.list);
    qprintk("%s: call isp_ctrl_show_list\n", __func__);
#ifdef DBG_QUEUE
	isp_ctrl_show_list();
#endif
    qprintk("%s: ret  isp_ctrl_show_list\n", __func__);
    
	dprintk("%s: finish\n", __func__);
	return;
}
//void isp_ctrl_dma_del_list(unsigned long in_addr, unsigned long out_addr)
void isp_ctrl_dma_del_list(void)
{
	t_isp_dma_addr_entry* del_ptr;

	dprintk("%s: start\n", __func__);
	del_ptr = list_entry(dma_addr_info_head.list.next, t_isp_dma_addr_entry, list);
    qprintk("%s: list_del: list.next = %p, del_prt = %p\n", __func__, dma_addr_info_head.list.next, del_ptr);
    qprintk("%s: call isp_ctrl_show_list\n", __func__);
#ifdef DBG_QUEUE
	isp_ctrl_show_list();
#endif
    qprintk("%s: ret  isp_ctrl_show_list\n", __func__);
    list_del(dma_addr_info_head.list.next);
    kfree(del_ptr);
    qprintk("%s: kfree = %p\n", __func__, del_ptr);
    qprintk("%s: call isp_ctrl_show_list\n", __func__);
#ifdef DBG_QUEUE
	isp_ctrl_show_list();
#endif
    qprintk("%s: ret  isp_ctrl_show_list\n", __func__);
	dprintk("%s: finish\n", __func__);
	return;
}
t_isp_dma_addr_entry* isp_ctrl_dma_get_list(void)
{
	t_isp_dma_addr_entry *entry = NULL;

	dprintk("%s: start\n", __func__);
	if (list_empty(&dma_addr_info_head.list)) {
		qprintk("%s: list_empty\n", __func__);
	}else{
		qprintk("%s: NOT list_empty\n", __func__);
		entry = list_entry(dma_addr_info_head.list.next, t_isp_dma_addr_entry, list);
		qprintk("in_addr = 0x%lx, out_addr = 0x%lx (list %p, prev = %p, next = %p)\n", 
				entry->in_addr, entry->out_addr, &entry->list, entry->list.prev, entry->list.next);
	}
	dprintk("%s: finish\n", __func__);

	return entry;
}
static void isp_ctrl_dma_clear_list(void)
{
	t_isp_dma_addr_entry *entry;
	t_isp_dma_addr_entry *entryNull = NULL;

	dprintk("%s: start\n", __func__);

	if (list_empty(&dma_addr_info_head.list)) {
		qprintk("%s: list_empty\n", __func__);
	}else{
		qprintk("%s: NOT list_empty\n", __func__);
		list_for_each_entry_safe(entry,entryNull,&dma_addr_info_head.list,list){
#ifdef DBG_QUEUE
			isp_ctrl_show_list();
#endif
			qprintk("%s: entry = %p, entry->list = %p\n", __func__, entry, &entry->list);
			list_del(&entry->list);
			qprintk("%s: kfree(%p)\n", __func__, entry);
			kfree(entry);
		}
	}
	dprintk("%s: finish\n", __func__);
}

/*
 * static functions
 */
static int isp_ctrl_isp_start(struct file *file)
{
	int ret;
	struct v4l2_pix_format pix;
	unsigned short stride;
	unsigned char *cam_param_p = NULL;
	dprintk("[info] %s: start\n", __func__);

	// set in FMT
	switch( ispOpenInfo.inFMT )
	{
		case V4L2_RZ_ISP_IN_FMT_RAW8:
			pix.pixelformat = V4L2_PIX_FMT_SRGGB8;
			break;
		case V4L2_RZ_ISP_IN_FMT_RAW10:
			pix.pixelformat = V4L2_PIX_FMT_SRGGB10;
			break;
		case V4L2_RZ_ISP_IN_FMT_RAW12:
			pix.pixelformat = V4L2_PIX_FMT_SRGGB12;
			break;
		default:
			// do not pass
			break;
	}

	pix.width = ispOpenInfo.width;
	// set format
	rzg2l_cru_set_pixelformat(file, pix.pixelformat);

	// get stride
	stride = rzg2l_cru_isp_format_bytesperline(&pix);

	ret = ISP_open(ispOpenInfo.width, ispOpenInfo.height, stride, ispOpenInfo.frameInterval, ispOpenInfo.inFMT, ispOpenInfo.outFMT, cam_param_p);
	if( ret < 0 ){
		dprintk("[error] %s: return ISP_open, ret=%d\n", __func__, ret);
		return ret;
	}
	dprintk("[info] %s: return ISP_open, ret=%d\n", __func__, ret);

	if( drpLevel == ISP_CTRL_DRP_LEVEL_DEF_VAL ){
		ISP_get_param(NULL, NULL, &drpLevel, &setAE.ae_on, &setAE.expose_level, &setAE.target_y, &setAE.threshold);
	}else{
		ISP_get_param(NULL, NULL, NULL, &setAE.ae_on, &setAE.expose_level, &setAE.target_y, &setAE.threshold);
	}

	ispOpenInfo.td = kthread_create(isp_ctrl_feedback_thread, NULL, "isp_ctrl_feedback_thread");
	wake_up_process(ispOpenInfo.td);
	//for CRU
	vb_data_rp = 0;
	vb_data_wp = 0;
	ispOpenInfo.td2 = kthread_create(isp_ctrl_go_thread, NULL, "isp_ctrl_go_thread");
	wake_up_process(ispOpenInfo.td2);

	return ret;
}

static int isp_ctrl_isp_finish(void)
{
	int ret;
	dprintk("[info] %s: start\n", __func__);

	kthread_stop(ispOpenInfo.td);
	wake_up_interruptible(&cru_wait_queue);
	kthread_stop(ispOpenInfo.td2);
	wake_up_interruptible(&isp_go_queue);

#ifdef DBG_QUEUE
	isp_ctrl_show_list();
#endif
	isp_ctrl_dma_clear_list();
#ifdef DBG_QUEUE
	isp_ctrl_show_list();
#endif

	ret = ISP_close();

#ifdef DBG_QUEUE
	isp_ctrl_wakeup_show_list();
#endif
	isp_ctrl_wakeup_clear_list();
#ifdef DBG_QUEUE
	isp_ctrl_wakeup_show_list();
#endif
	dprintk("[info] %s: call ISP_close ret = %d\n", __func__, ret);
	return ret;
}

static void isp_ctrl_init_open_info(void)
{
	// for format
	// inFMT and frameInterval may already be set, so do not initialize
	ispOpenInfo.outFMT = 0;
	ispOpenInfo.width  = 0;
	ispOpenInfo.height = 0;
	ispOpenInfo.opened = 0;

	// for AE
	setAE.ae_on = 0;
	setAE.expose_level = 0;
	setAE.target_y = 0;
	setAE.threshold = 0;
}

void isp_ctrl_buffer_done(struct vb2_buffer *vb)
{
    memcpy( &vb_data[vb_data_wp], vb, sizeof(struct vb2_buffer) );
    vb_data_wp = vb_data_wp >= (MAX_VB_BUFFER-1) ? 0 : (vb_data_wp+1);
    wake_up_interruptible(&isp_go_queue);
}

void isp_ctrl_buffer_done_main(void)
{
    struct vb2_buffer *vb = &vb_data[vb_data_rp];
	int ret=0;
	t_isp_dma_addr_entry *getData = isp_ctrl_dma_get_list();
	unsigned long flags;

    vb_data_rp = vb_data_rp >= (MAX_VB_BUFFER-1) ? 0 : (vb_data_rp+1);

	dprintk("[info]%s in\n",__func__);

	if( getData == NULL ){
		return;
	}
	if( (getData->out_addr == 0) || (getData->in_addr == 0))
	{
		return;
	}
	if(waitCallback){
		if(q_head){
			spin_lock_irqsave(&wakeup_list_lock, flags);
			isp_ctrl_wakeup_add_list_tail(-EBUSY, vb->index);
			spin_unlock_irqrestore(&wakeup_list_lock, flags);
			dqbufprintk("b1 skip r=%d, i=%d\n", ret, vb->index);
			wake_up(q_head); 
		}
		goto skip;
	}
	waitCallback = 1;
	dqbufprintk("b0 isp_go i=%d out_addr=0x%08x\n",vb->index , getData->out_addr);
	ret = ISP_go(getData->in_addr,getData->out_addr,drpLevel,isp_ctrl_isp_callback);
	if(ret<0)
	{
		dprintk("[err]%s ISP_go err %d\n",__func__,ret);
		waitCallback = 0;

		if(q_head){
			spin_lock_irqsave(&wakeup_list_lock, flags);
			isp_ctrl_wakeup_add_list_tail(ret, vb->index);
			spin_unlock_irqrestore(&wakeup_list_lock, flags);
			dqbufprintk("b1 err r=%d, i=%d\n", ret, vb->index);
			wake_up(q_head); 
		}
	}else{
		// ISP_go OK
		err_dqbuf_index = vb->index;
		dqbufprintk("b1 OK r=%d, i=%d\n", ret, vb->index);
	}

skip:
	isp_ctrl_dma_del_list();
	return;
}

static int isp_ctrl_feedback_thread(void *arg)
{
	wait_queue_entry_t  wait;
	/* ISP MOD-K START */
//	unsigned char gain_p; 
//	unsigned char expose_p;
	int gain_p; 
	int expose_p;
	/* ISP MOD-K END   */
	
    dprintk("[info]%s start\n",__func__);
    init_waitqueue_entry(&wait, current);
	while(!kthread_should_stop())
	{
		DECLARE_WAITQUEUE(wait, current);
		set_current_state(TASK_INTERRUPTIBLE);
        dprintk("[info]%s isp_callback waiting...\n",__func__);
        add_wait_queue(&cru_wait_queue,&wait);
		schedule();
		remove_wait_queue(&cru_wait_queue,&wait);
	    dprintk("[info]%s isp_callback called\n",__func__);
	    if(kthread_should_stop()){
	    	break;
		}
		/* ISP MOD-K START */
    	if(setAE.ae_on)
    	{
	    	//ISP_AE_Get
    		ISP_AE_Get(&gain_p,&expose_p);
	    	dprintk("[info]%s ISP_AE_Get gain:%d  exposure=%d\n",__func__,gain_p,expose_p);
	    	//[I2C] set gain
			if(ispOpenInfo.gain)
	    	{
				dprintk("[info]%s ov5645_set gain=%d\n",__func__,gain_p);
				v4l2_ctrl_s_ctrl(ispOpenInfo.gain,gain_p);
			}
			//[I2C] set shutter speed
			if(ispOpenInfo.expose)
			{
				dprintk("[info]%s ov5645_set expose=%d\n",__func__,expose_p);
				v4l2_ctrl_s_ctrl(ispOpenInfo.expose,expose_p);	
			}
		}
	}
	/* ISP MOD-K END   */
	ispOpenInfo.td = 0;

	return 0; /* end of thread */
}

static int isp_ctrl_go_thread(void *arg)
{
	wait_queue_entry_t  wait;
	
    dprintk("[info]%s start\n",__func__);
    init_waitqueue_entry(&wait, current);
	while(!kthread_should_stop())
	{
		DECLARE_WAITQUEUE(wait, current);
		set_current_state(TASK_INTERRUPTIBLE);
        dprintk("[info]%s isp_callback waiting...\n",__func__);
        add_wait_queue(&isp_go_queue,&wait);
		schedule();
		remove_wait_queue(&isp_go_queue,&wait);
	    dprintk("[info]%s isp_callback called\n",__func__);
	    if(kthread_should_stop()){
	    	break;
		}
        isp_ctrl_buffer_done_main();
	}
	ispOpenInfo.td2 = 0;

	return 0; /* end of thread */
}

static void isp_ctrl_isp_callback(int result,unsigned long out_adr)
{
	unsigned long flags;

	spin_lock_irqsave(&wakeup_list_lock, flags);
	if( result<0 ){
		isp_ctrl_wakeup_add_list_tail(result, err_dqbuf_index);
		dqbufprintk("c1 err r=%d, i=%d\n", result, err_dqbuf_index);
	}else{
		dqbufprintk("c1-OK idx = %d", err_dqbuf_index);
	}
	spin_unlock_irqrestore(&wakeup_list_lock, flags);

	if(ispOpenInfo.td)
	{
        wake_up_interruptible(&cru_wait_queue);
	}

	if(q_head){
		wake_up(q_head); 
	}
	waitCallback = 0;
	return;
}

static int isp_ctrl_set_format(struct file *file)
{
	struct v4l2_subdev *csi_subdev = rzg2l_cru_get_csi_subdev(file);
	struct v4l2_subdev *ov_subdev = rgz2l_csi2_get_remote(csi_subdev);
	int ret;
	struct v4l2_subdev_format workChangeFmt = { 0 };


	changeFmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	changeFmt.format.width = ispOpenInfo.width;
	changeFmt.format.height = ispOpenInfo.height;
	changeFmt.format.field = V4L2_FIELD_NONE;
	changeFmt.format.colorspace = 0;
	changeFmt.format.ycbcr_enc = 0;
	changeFmt.format.quantization = 0;
	changeFmt.format.xfer_func = 0;
	// code is already set in IN_FMT

	workChangeFmt = changeFmt;	// save original data

	// csi
	ret = v4l2_subdev_call(csi_subdev, pad, set_fmt, NULL, &changeFmt);
	if ( ret ){
		ret = -EPIPE;
		return ret;
	}else{
		dprintk("[info] %s: success setting csi2 format.\n", __func__);
		dprintk("[info] %s: changeFmt.format.width = %d, changeFmt.format.height = %d, changeFmt.format.field = %d\n", __func__, changeFmt.format.width, changeFmt.format.height, changeFmt.format.field);
	}

	changeFmt = workChangeFmt;	// reverse original data

	// ov5645
	ret = v4l2_subdev_call(ov_subdev, pad, set_fmt, NULL, &changeFmt);
	if ( ret ){
		ret = -EPIPE;
		return ret;
	}else{
		dprintk("[info] %s: success setting ov5645 format.\n", __func__);
		dprintk("[info] %s: changeFmt.format.width = %d, changeFmt.format.height = %d, changeFmt.format.field = %d\n", __func__, changeFmt.format.width, changeFmt.format.height, changeFmt.format.field);
	}
	return ret;
}

/*
 * ISP Control Module API for V4L2
 */
int isp_ctrl_querycap(ioctl_querycap func, struct file *file, void *fh, struct v4l2_capability *cap)
{
	int ret;
	dprintk("[info] %s: start\n", __func__);
    ret = func(file, fh, cap);
	strlcpy(cap->driver, "rzv2l_isp", sizeof(cap->driver));
	strlcpy(cap->card, "RZV2L_CRU", sizeof(cap->card));
    return ret;
}

int isp_ctrl_try_fmt(ioctl_try_fmt func, struct file *file, void *fh, struct v4l2_format *fmt)
{
	dprintk("[info] %s: start\n", __func__);
    return func(file, fh, fmt);
}

int isp_ctrl_g_fmt(ioctl_g_fmt func, struct file *file, void *fh, struct v4l2_format *fmt)
{
	dprintk("[info] %s: start\n", __func__);
    return func(file, fh, fmt);
}

int isp_ctrl_s_fmt(ioctl_s_fmt func, struct file *file, void *fh, struct v4l2_format *fmt)
{
    struct v4l2_pix_format *pix = &(fmt->fmt.pix);

	dprintk("[info] %s: start\n", __func__);
	// check format
	if ( (pix->pixelformat != V4L2_PIX_FMT_YUYV) &&
		(pix->pixelformat != V4L2_PIX_FMT_UYVY) &&
		(pix->pixelformat != V4L2_PIX_FMT_RGB24) &&
		(pix->pixelformat != V4L2_PIX_FMT_ARGB32) &&
		(pix->pixelformat != V4L2_PIX_FMT_XRGB32)){
		return -EINVAL;
	}
	// init open info
	isp_ctrl_init_open_info();

	switch( pix->pixelformat )
	{
		case V4L2_PIX_FMT_YUYV:
			ispOpenInfo.outFMT = ISP_CTRL_OUT_FMT_YUY2;
			break;
		case V4L2_PIX_FMT_UYVY:
			ispOpenInfo.outFMT = ISP_CTRL_OUT_FMT_UYVY;
			break;
		case V4L2_PIX_FMT_RGB24:
			ispOpenInfo.outFMT = ISP_CTRL_OUT_FMT_RGB24;
			break;
		case V4L2_PIX_FMT_ARGB32:
			ispOpenInfo.outFMT = ISP_CTRL_OUT_FMT_ARGB32;
			break;
		case V4L2_PIX_FMT_XRGB32:
			ispOpenInfo.outFMT = ISP_CTRL_OUT_FMT_XRGB32;
			break;
		default:
			// do not pass
			break;
	}

	ispOpenInfo.width = pix->width;
	ispOpenInfo.height = pix->height;

    return func(file, fh, fmt);
}

int isp_ctrl_enum_fmt(ioctl_enum_fmt func, struct file *file, void *fh, struct v4l2_fmtdesc *fmtdesc)
{
	dprintk("[info] %s: start\n", __func__);
	return func(file, fh, fmtdesc);
}

int isp_ctrl_g_selection(ioctl_g_selection func, struct file *file, void *fh, struct v4l2_selection *selection)
{
	dprintk("[info] %s: start\n", __func__);
    return func(file, fh, selection);
}

int isp_ctrl_s_selection(ioctl_s_selection func, struct file *file, void *fh, struct v4l2_selection *selection)
{
	dprintk("[info] %s: start\n", __func__);
    return func(file, fh, selection);
}

int isp_ctrl_enuminput(ioctl_enuminput func, struct file *file, void *fh, struct v4l2_input *input)
{
	dprintk("[info] %s: start\n", __func__);
    return func(file, fh, input);
}

int isp_ctrl_g_input(ioctl_g_input func, struct file *file, void *fh, unsigned int *input)
{
	dprintk("[info] %s: start. value = %d\n", __func__, *(input));
    return func(file, fh, input);
}

int isp_ctrl_s_input(ioctl_s_input func, struct file *file, void *fh, unsigned int input)
{
	dprintk("[info] %s: start. value = %d\n", __func__, input);
    return func(file, fh, input);
}

int isp_ctrl_reqbufs(ioctl_reqbufs func, struct file *file, void *fh, struct v4l2_requestbuffers *buf)
{
	dprintk("[info] %s: start\n", __func__);
    return func(file, fh, buf);
}

int isp_ctrl_create_bufs(ioctl_create_bufs func, struct file *file, void *fh, struct v4l2_create_buffers *buf)
{
	dprintk("[info] %s: start\n", __func__);
    return func(file, fh, buf);
}

int isp_ctrl_querybuf(ioctl_querybuf func, struct file *file, void *fh, struct v4l2_buffer *buf)
{
	int ret;
	dprintk("[info] %s: start, opend=%d\n", __func__, ispOpenInfo.opened);
	if( !(ispOpenInfo.opened) ){
		ret = isp_ctrl_isp_start(file);
		if( ret < 0 ){
			return ret;
		}
		ispOpenInfo.opened = 1;
	}
	return func(file, fh, buf);
}

int isp_ctrl_qbuf(ioctl_qbuf func, struct file *file, void *fh, struct v4l2_buffer *buf)
{
    int ret = 0;

	dqbufprintk("[info] %s: start index=%d\n", __func__, buf->index);
	if( !cam_image_physadr_got ){
		ret = ISP_get_in_buffer( cam_image_physadr_list );
		if( ret == 0 ){
			ret = -ENOSPC;    // -ENOSPC = -28
			return ret;
		}else if( ret < 0 ){
			return ret;
		}
		num_camera_images = ret;
		/* dma list initialize */
        INIT_LIST_HEAD(&dma_addr_info_head.list);
		cam_image_physadr_got = 1;
	}
	ret = func(file, fh, buf);
	dqbufprintk("[info] %s: finish ret = %d, buf->index = %d\n", __func__, ret, buf->index);
	return ret;
}

int isp_ctrl_dqbuf(ioctl_dqbuf func, struct file *file, void *fh, struct v4l2_buffer *buf)
{
	int ret = 0;
	t_isp_wakeup_entry* getEntry;
	unsigned long flags;

	ret = func(file, fh, buf);
	dqbufprintk("d0 r %d i %d\n", ret, buf->index);

	if( ret < 0 ){
		dqbufprintk("d0 dqbuf error finish");
		return ret;
	}

	spin_lock_irqsave(&wakeup_list_lock, flags);
	getEntry = isp_ctrl_wakeup_get_list();
	spin_unlock_irqrestore(&wakeup_list_lock, flags);

	if( getEntry != NULL ){
		dqbufprintk("d0 eC=%d, ei=%d", getEntry->errCode, getEntry->index);
		if( getEntry->index == buf->index ){
			int retErrCode = getEntry->errCode;

			dqbufprintk("d1 dqbuf i %d",buf->index);
			spin_lock_irqsave(&wakeup_list_lock, flags);
			isp_ctrl_wakeup_del_list();
			spin_unlock_irqrestore(&wakeup_list_lock, flags);

			dqbufprintk("d1 qbuf i %d",buf->index);
			ret = vb2_ioctl_qbuf(file, fh, buf);
			if(ret==0){
				ret = retErrCode;
			}
		}
	}

	dprintk("[info] %s: finish ret = %d \n", __func__,ret);
	dqbufprintk("d0 finish r=%d",ret);
	return ret;
}

int isp_ctrl_expbuf(ioctl_expbuf func, struct file *file, void *fh, struct v4l2_exportbuffer *buf)
{
	dprintk("[info] %s: start\n", __func__);
    return func(file, fh, buf);
}

int isp_ctrl_prepare_buf(ioctl_prepare_buf func, struct file *file, void *fh, struct v4l2_buffer *buf)
{
	dprintk("[info] %s: start\n", __func__);
    return func(file, fh, buf);
}

int isp_ctrl_streamon(ioctl_streamon func, struct file *file, void *fh, enum v4l2_buf_type type)
{
	/* ISP MOD-K START */
//	unsigned char gain_p; 
//	unsigned char expose_p;
	int gain_p; 
	int expose_p;
	int ret = 0;

	struct v4l2_subdev *csi_subdev = rzg2l_cru_get_csi_subdev(file);
	struct v4l2_subdev *ov_subdev = rgz2l_csi2_get_remote(csi_subdev);	

	dprintk("[info] %s: start\n", __func__);
	// set input format
	if( setINFMTFlag != ISP_CTRL_SET_IN_FMT)
	{
		unsigned short getInFmt=0;
		ISP_get_param(&getInFmt, NULL, NULL, NULL, NULL, NULL, NULL);
		if( (getInFmt < 0) || (getInFmt >= ISP_CTRL_CHGTBL_MAX_NUM) ){
			return -EINVAL;
		}
		changeFmt.format.code = changeInFmtToBusFmt[getInFmt];
	}else{
		changeFmt.format.code = changeInFmtToBusFmt[ispOpenInfo.inFMT];
	}
	setINFMTFlag = 0;
	ret = isp_ctrl_set_format(file);
	if( ret < 0 )
		return ret;

	ispOpenInfo.gain = v4l2_ctrl_find(ov_subdev->ctrl_handler,V4L2_CID_GAIN);
	ispOpenInfo.expose = v4l2_ctrl_find(ov_subdev->ctrl_handler,V4L2_CID_EXPOSURE);

	// AE ON check
   	if(setAE.ae_on)
   	{
    	//ISP_AE_Get
   		ISP_AE_Get(&gain_p,&expose_p);
    	dprintk("[info]%s ISP_AE_Get gain:%d  exposure=%d\n",__func__,gain_p,expose_p);
    	//[I2C] set gain
    	if(ispOpenInfo.gain)
    	{
			dprintk("[info]%s ov5645_set gain=%d\n",__func__,gain_p);
			v4l2_ctrl_s_ctrl(ispOpenInfo.gain,gain_p);
		}
		//[I2C] set shutter speed
		if(ispOpenInfo.expose)
		{
			dprintk("[info]%s ov5645_set expose=%d\n",__func__,expose_p);
			v4l2_ctrl_s_ctrl(ispOpenInfo.expose,expose_p);
		}
	}
	/* ISP MOD-K END   */

	return func(file, fh, type);
}

int isp_ctrl_streamoff(ioctl_streamoff func, struct file *file, void *fh, enum v4l2_buf_type type)
{
    int ret = 0;

	dprintk("[info] %s: start\n", __func__);
	ret = func(file, fh, type);
	if(ispOpenInfo.opened)
	{
		ret = isp_ctrl_isp_finish();
		ispOpenInfo.opened = 0;
	}
    return ret;
}

int isp_ctrl_log_status(ioctl_log_status func, struct file *file, void *fh)
{
	dprintk("[info] %s: start\n", __func__);
    return func(file, fh);
}

int isp_ctrl_subscribe_event(ioctl_subscribe_event func, struct v4l2_fh *fh, const struct v4l2_event_subscription *sub)
{
	dprintk("[info] %s: start\n", __func__);
    return func(fh, sub);
}

int isp_ctrl_unsubscribe_event(ioctl_unsubscribe_event func, struct v4l2_fh *fh, const struct v4l2_event_subscription *sub)
{
	dprintk("[info] %s: start\n", __func__);
    return func(fh, sub);
}

// IN_FMT
static int isp_ctrl_set_IN_FMT(int value)
{
	int ret = 0;
	dprintk("[info] %s: V4L2_CID_RZ_ISP_IN_FMT val=%d\n", __func__, value);
	// check param
	if( (value <  V4L2_RZ_ISP_IN_FMT_RAW8) || (value >  V4L2_RZ_ISP_IN_FMT_RAW12) ){
		ret = -EINVAL;
	}else{
		ispOpenInfo.inFMT = value;
		switch(value){
			case V4L2_RZ_ISP_IN_FMT_RAW8:
				changeFmt.format.code = MEDIA_BUS_FMT_SRGGB8_1X8;
				break;
			case V4L2_RZ_ISP_IN_FMT_RAW10:
				changeFmt.format.code = MEDIA_BUS_FMT_SRGGB10_1X10;
				break;
			case V4L2_RZ_ISP_IN_FMT_RAW12:
				changeFmt.format.code = MEDIA_BUS_FMT_SRGGB12_1X12;
				break;
			default:
				// do not pass
				break;
		}
		// set flag
		setINFMTFlag = ISP_CTRL_SET_IN_FMT;
	}
	dprintk("[info] %s: ret=%d\n", __func__, ret);
	return ret;
}

// V4L2_CID_RZ_ISP_FRM_INTERVAL
static int isp_ctrl_set_FRM_INTERVAL(int value)
{
	int ret = 0;
	dprintk("[info] %s: V4L2_CID_RZ_ISP_FRM_INTERVAL val=%d\n", __func__, value);
	// check param
	if( value <  0 ){
		ret = -EINVAL;
	}else{
		ispOpenInfo.frameInterval = value;
	}
	dprintk("[info] %s: ret=%d\n", __func__, ret);
	return ret;
}

// V4L2_CID_RZ_ISP_BL, V4L2_CID_RZ_ISP_WB, V4L2_CID_RZ_ISP_GAMMA, V4L2_CID_RZ_ISP_CMX
// V4L2_CID_RZ_ISP_2DNR, V4L2_CID_RZ_ISP_3DNR, V4L2_CID_RZ_ISP_EMP
static int isp_ctrl_set_ISP_set(int func, int value)
{
	int ret = 0;
	dprintk("[info] %s: val=%d\n", __func__, value);
	ret = ISP_set(func, &value);
	dprintk("[info] %s: ret=%d\n", __func__, ret);
	return ret;
}

// V4L2_CID_RZ_ISP_DRP_LV
static int isp_ctrl_set_DRP_LV(int value)
{
	int ret = 0;
	dprintk("[info] %s: V4L2_CID_RZ_ISP_DRP_LV val=%d\n", __func__, value);
	if( value < 0 ){
		ret = -EINVAL;
	}else{
		drpLevel = value;
	}
	dprintk("[info] %s: ret=%d\n", __func__, ret);
	return ret;
}

// V4L2_CID_RZ_ISP_AE, V4L2_CID_RZ_ISP_EXPOSE_LV, V4L2_CID_RZ_ISP_T_BL, V4L2_CID_RZ_ISP_THRESHOLD
static int isp_ctrl_set_ISP_AE_Set(int value, int* dstValue)
{
	int oldVal;
	int ret = 0;

	dprintk("[info] %s: org  ae_on = %d, expose_level = %d, target_y = %d, threshold = %d\n",
			__func__, setAE.ae_on, setAE.expose_level, setAE.target_y, setAE.threshold);
	oldVal = *dstValue;
	*dstValue = value;
	dprintk("[info] %s: set  ae_on = %d, expose_level = %d, target_y = %d, threshold = %d\n",
			__func__, setAE.ae_on, setAE.expose_level, setAE.target_y, setAE.threshold);
	ret = ISP_AE_Set(setAE.ae_on, setAE.expose_level, setAE.target_y, setAE.threshold);
	if( ret < 0 ){
		// undo
		*dstValue = oldVal;
		dprintk("[info] %s: undo ae_on = %d, expose_level = %d, target_y = %d, threshold = %d\n",
				__func__, setAE.ae_on, setAE.expose_level, setAE.target_y, setAE.threshold);
	}
	dprintk("[info] %s: fix ae_on = %d, expose_level = %d, target_y = %d, threshold = %d\n",
			__func__, setAE.ae_on, setAE.expose_level, setAE.target_y, setAE.threshold);
	dprintk("[info] %s: ret=%d\n", __func__, ret);
	return ret;
}

// V4L2_CID_RZ_ISP_ALL
static int isp_ctrl_set_s_ctrl_ALL(unsigned char* ptr)
{
	int ret;
	ret = ISP_set(ISP_FUNC_ALL, ptr);
	dprintk("[info] %s: ret=%d\n", __func__, ret);
	return ret;
}

int isp_ctrl_s_ctrl(ioctl_s_ctrl func, struct v4l2_ctrl *ctrl)
{
	int ret = 0;

	dprintk("[info] %s: start\n", __func__);
	switch(ctrl->id)
	{
		case V4L2_CID_RZ_ISP_IN_FMT:
			ret = isp_ctrl_set_IN_FMT(ctrl->val);
			break;
		case V4L2_CID_RZ_ISP_FRM_INTERVAL:
			ret = isp_ctrl_set_FRM_INTERVAL(ctrl->val);
			break;
		case V4L2_CID_RZ_ISP_BL:
			ret = isp_ctrl_set_ISP_set(ISP_FUNC_BL, ctrl->val);
			break;
		case V4L2_CID_RZ_ISP_WB:
			ret = isp_ctrl_set_ISP_set(ISP_FUNC_WB, ctrl->val);
			break;
		case V4L2_CID_RZ_ISP_GAMMA:
			ret = isp_ctrl_set_ISP_set(ISP_FUNC_GAMMA, ctrl->val);
			break;
		case V4L2_CID_RZ_ISP_CMX:
			ret = isp_ctrl_set_ISP_set(ISP_FUNC_CMX, ctrl->val);
			break;
		case V4L2_CID_RZ_ISP_2DNR:
			ret = isp_ctrl_set_ISP_set(ISP_FUNC_2DNR, ctrl->val);
			break;
		case V4L2_CID_RZ_ISP_3DNR:
			ret = isp_ctrl_set_ISP_set(ISP_FUNC_3DNR, ctrl->val);
			break;
		case V4L2_CID_RZ_ISP_EMP:
			ret = isp_ctrl_set_ISP_set(ISP_FUNC_EMP, ctrl->val);
			break;
		case V4L2_CID_RZ_ISP_DRP_LV:
			ret = isp_ctrl_set_DRP_LV(ctrl->val);
			break;
		case V4L2_CID_RZ_ISP_AE:
			ret = isp_ctrl_set_ISP_AE_Set(ctrl->val, &setAE.ae_on);
			break;
		case V4L2_CID_RZ_ISP_EXPOSE_LV:
			ret = isp_ctrl_set_ISP_AE_Set(ctrl->val, &setAE.expose_level);
			break;
		case V4L2_CID_RZ_ISP_T_BL:
			ret = isp_ctrl_set_ISP_AE_Set(ctrl->val, &setAE.target_y);
			break;
		case V4L2_CID_RZ_ISP_THRESHOLD:
			ret = isp_ctrl_set_ISP_AE_Set(ctrl->val, &setAE.threshold);
			break;
		case V4L2_CID_RZ_ISP_ALL:
			ret = isp_ctrl_set_s_ctrl_ALL(ctrl->p_new.p_u8);
			break;
		case V4L2_CID_RZ_ISP_AWB:
			dprintk("[info] %s: V4L2_CID_RZ_ISP_AWB\n", __func__);
			ret = isp_ctrl_set_ISP_set(ISP_FUNC_AWB, ctrl->val);
			dprintk("[info] %s: ret=%d\n", __func__, ret);
			break;
		default:
			ret = func(ctrl);
			break;
	}
	dprintk("[info] %s: finish\n", __func__);
	return ret;
}

int isp_ctrl_g_ext_ctrl(struct v4l2_ext_control *ctrl)
{
	int ret=0;
	int i;
	int fid=0;
	char allData[V4L2_RZ_ISP_ALL_DIMS];

	dprintk("[info] %s: start\n", __func__);
	dprintk("[info] %s: ctrl = 0x%p, id = %d\n", __func__, ctrl, ctrl->id);

	// check id
	for(i=0; i<ISP_CTRL_G_EXT_CTRLS_NUM_ID; i++){
		if( ctrl->id == isplibFuncIdTbl[i].cid ){
			fid = isplibFuncIdTbl[i].fid;
			break;
		}
	}

	if( i >= ISP_CTRL_G_EXT_CTRLS_NUM_ID ){
		// not find a function id
		// ISP_get is not use
		switch(ctrl->id)
		{
			case V4L2_CID_RZ_ISP_IN_FMT:
				dprintk("%s: V4L2_CID_RZ_ISP_IN_FMT route\n", __func__);
				ctrl->value = ispOpenInfo.inFMT;
				break;
			case V4L2_CID_RZ_ISP_FRM_INTERVAL:
				dprintk("%s: V4L2_CID_RZ_ISP_FRM_INTERVAL route\n", __func__);
				ctrl->value = ispOpenInfo.frameInterval;
				break;
			case V4L2_CID_RZ_ISP_DRP_LV:
				dprintk("%s: V4L2_CID_RZ_ISP_DRP_LV route\n", __func__);
				ctrl->value = drpLevel;
				break;
			case V4L2_CID_RZ_ISP_AE:
				dprintk("%s: V4L2_CID_RZ_ISP_AE route\n", __func__);
				ISP_get_param(NULL, NULL, NULL, &ctrl->value, NULL, NULL, NULL);
				break;
			case V4L2_CID_RZ_ISP_EXPOSE_LV:
				dprintk("%s: V4L2_CID_RZ_ISP_EXPOSE_LV route\n", __func__);
				ISP_get_param(NULL, NULL, NULL, NULL, &ctrl->value, NULL, NULL);
				break;
			case V4L2_CID_RZ_ISP_T_BL:
				dprintk("%s: V4L2_CID_RZ_ISP_T_BL route\n", __func__);
				ISP_get_param(NULL, NULL, NULL, NULL, NULL, &ctrl->value, NULL);
				break;
			case V4L2_CID_RZ_ISP_THRESHOLD:
				dprintk("%s: V4L2_CID_RZ_ISP_THRESHOLD route\n", __func__);
				ISP_get_param(NULL, NULL, NULL, NULL, NULL, NULL, &ctrl->value);
				break;
			default:
				// do not pass
				dprintk("[info] %s: default route\n", __func__);
				ret = -EINVAL;
				break;
		}
	}else{
		switch(ctrl->id)
		{
			case V4L2_CID_RZ_ISP_ALL:
				ret = ISP_get(fid, allData);
				if( ret == 0 ){
					// copy
					if( 0 != copy_to_user(ctrl->ptr, allData, V4L2_RZ_ISP_ALL_DIMS) ){
						ret = -EFAULT;
					}
				}
				break;
			case V4L2_CID_RZ_ISP_BL:
			case V4L2_CID_RZ_ISP_WB:
			case V4L2_CID_RZ_ISP_GAMMA:
			case V4L2_CID_RZ_ISP_CMX:
			case V4L2_CID_RZ_ISP_2DNR:
			case V4L2_CID_RZ_ISP_3DNR:
			case V4L2_CID_RZ_ISP_EMP:
			case V4L2_CID_RZ_ISP_AWB:
				ret = ISP_get(fid, &ctrl->value);
				break;
			default:
				// do not pass
				dprintk("[info] %s: ISP_get default route\n", __func__);
				ret = -EINVAL;
				break;
		}
	}

	dprintk("[info] %s: finish, ret = %d, value = %d\n", __func__, ret, ctrl->value);
	return ret;
}
