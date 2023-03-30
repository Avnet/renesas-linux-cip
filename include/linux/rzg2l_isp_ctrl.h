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
#ifndef __RZG2L_ISP_CTRL_H__
#define __RZG2L_ISP_CTRL_H__

extern u32 rzg2l_cru_isp_format_bytesperline(struct v4l2_pix_format *pix);
extern struct v4l2_subdev *rzg2l_cru_get_csi_subdev(struct file *file);
extern struct v4l2_subdev *rgz2l_csi2_get_remote(struct v4l2_subdev *sd);
extern void rzg2l_cru_set_pixelformat(struct file *file, __u32 format);

#endif // __RZG2L_ISP_CTRL_H__
