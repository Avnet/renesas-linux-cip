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
#ifndef __ISP_CTRL_H__
#define __ISP_CTRL_H__

#include <media/v4l2-ctrls.h>
#include <media/videobuf2-core.h>

//#define ISP_CTRL_DEBUG
#ifdef ISP_CTRL_DEBUG
#define isp_ctrl_dprintk(fmt, arg...)		printk(fmt, ##arg)
#else
#define isp_ctrl_dprintk(fmt, arg...)
#endif

/* 
 * struct
 */

typedef struct isp_ctrl_dma_addr_entry{
    unsigned long in_addr;
    unsigned long out_addr;
    struct list_head list;
}t_isp_dma_addr_entry;


/*
 * ISP Control Module API for V4L2
 */
typedef int (*ioctl_querycap)(struct file *file, void *fh, struct v4l2_capability *cap);
int isp_ctrl_querycap(ioctl_querycap func, struct file *file, void *fh, struct v4l2_capability *cap);

typedef int (*ioctl_try_fmt)(struct file *file, void *fh, struct v4l2_format *fmt);
int isp_ctrl_try_fmt(ioctl_try_fmt func, struct file *file, void *fh, struct v4l2_format *fmt);

typedef int (*ioctl_g_fmt)(struct file *file, void *fh, struct v4l2_format *fmt);
int isp_ctrl_g_fmt(ioctl_g_fmt func, struct file *file, void *fh, struct v4l2_format *fmt);

typedef int (*ioctl_s_fmt)(struct file *file, void *fh, struct v4l2_format *fmt);
int isp_ctrl_s_fmt(ioctl_s_fmt func, struct file *file, void *fh, struct v4l2_format *fmt);

typedef int (*ioctl_enum_fmt)(struct file *file, void *fh, struct v4l2_fmtdesc *fmtdesc);
int isp_ctrl_enum_fmt(ioctl_enum_fmt func, struct file *file, void *fh, struct v4l2_fmtdesc *fmtdesc);

typedef int (*ioctl_g_selection)(struct file *file, void *fh, struct v4l2_selection *selection);
int isp_ctrl_g_selection(ioctl_g_selection func, struct file *file, void *fh, struct v4l2_selection *selection);

typedef int (*ioctl_s_selection)(struct file *file, void *fh, struct v4l2_selection *selection);
int isp_ctrl_s_selection(ioctl_s_selection func, struct file *file, void *fh, struct v4l2_selection *selection);

typedef int (*ioctl_enuminput)(struct file *file, void *fh, struct v4l2_input *input);
int isp_ctrl_enuminput(ioctl_enuminput func, struct file *file, void *fh, struct v4l2_input *input);

typedef int (*ioctl_g_input)(struct file *file, void *fh, unsigned int *input);
int isp_ctrl_g_input(ioctl_g_input func, struct file *file, void *fh, unsigned int *input);

typedef int (*ioctl_s_input)(struct file *file, void *fh, unsigned int input);
int isp_ctrl_s_input(ioctl_s_input func, struct file *file, void *fh, unsigned int input);

typedef int (*ioctl_reqbufs)(struct file *file, void *fh, struct v4l2_requestbuffers *buf);
int isp_ctrl_reqbufs(ioctl_reqbufs func, struct file *file, void *fh, struct v4l2_requestbuffers *buf);

typedef int (*ioctl_create_bufs)(struct file *file, void *fh, struct v4l2_create_buffers *buf);
int isp_ctrl_create_bufs(ioctl_create_bufs func, struct file *file, void *fh, struct v4l2_create_buffers *buf);

typedef int (*ioctl_querybuf)(struct file *file, void *fh, struct v4l2_buffer *buf);
int isp_ctrl_querybuf(ioctl_querybuf func, struct file *file, void *fh, struct v4l2_buffer *buf);

typedef int (*ioctl_qbuf)(struct file *file, void *fh, struct v4l2_buffer *buf);
int isp_ctrl_qbuf(ioctl_qbuf func, struct file *file, void *fh, struct v4l2_buffer *buf);

typedef int (*ioctl_dqbuf)(struct file *file, void *fh, struct v4l2_buffer *buf);
int isp_ctrl_dqbuf(ioctl_dqbuf func, struct file *file, void *fh, struct v4l2_buffer *buf);

typedef int (*ioctl_expbuf)(struct file *file, void *fh, struct v4l2_exportbuffer *buf);
int isp_ctrl_expbuf(ioctl_expbuf func, struct file *file, void *fh, struct v4l2_exportbuffer *buf);

typedef int (*ioctl_prepare_buf)(struct file *file, void *fh, struct v4l2_buffer *buf);
int isp_ctrl_prepare_buf(ioctl_prepare_buf func, struct file *file, void *fh, struct v4l2_buffer *buf);

typedef int (*ioctl_streamon)(struct file *file, void *fh, enum v4l2_buf_type type);
int isp_ctrl_streamon(ioctl_streamon func, struct file *file, void *fh, enum v4l2_buf_type type);

typedef int (*ioctl_streamoff)(struct file *file, void *fh, enum v4l2_buf_type type);
int isp_ctrl_streamoff(ioctl_streamoff func, struct file *file, void *fh, enum v4l2_buf_type type);

typedef int (*ioctl_log_status)(struct file *file, void *fh);
int isp_ctrl_log_status(ioctl_log_status func, struct file *file, void *fh);

typedef int (*ioctl_subscribe_event)(struct v4l2_fh *fh, const struct v4l2_event_subscription *sub);
int isp_ctrl_subscribe_event(ioctl_subscribe_event func, struct v4l2_fh *fh, const struct v4l2_event_subscription *sub);

typedef int (*ioctl_unsubscribe_event)(struct v4l2_fh *fh, const struct v4l2_event_subscription *sub);
int isp_ctrl_unsubscribe_event(ioctl_unsubscribe_event func, struct v4l2_fh *fh, const struct v4l2_event_subscription *sub);

typedef int (*ioctl_s_ctrl)(struct v4l2_ctrl *ctrl);
int isp_ctrl_s_ctrl(ioctl_s_ctrl func, struct v4l2_ctrl *ctrl);

extern unsigned long isp_ctrl_get_camera_phys_addr(void);
extern void isp_ctrl_dma_add_list_tail(unsigned long in_addr, unsigned long out_addr);
extern t_isp_dma_addr_entry* isp_ctrl_dma_get_list(void);
extern void isp_ctrl_dma_del_list(void);
extern void isp_ctrl_init(struct file *file);
extern void isp_ctrl_buffer_done(struct vb2_buffer *vb);
extern u8 isp_ctrl_get_run_state(void);
extern void isp_ctrl_deinit(void);
extern int isp_ctrl_g_ext_ctrl(struct v4l2_ext_control *ctrl);
extern int isp_ctrl_chk_index_complete(int index);

#define ISP_CTRL_RUN				(1)
#define ISP_CTRL_STOP				(0)

extern wait_queue_head_t* q_head;

#endif // __ISP_CTRL_H__
