/* SPDX-License-Identifier: ((GPL-2.0+ WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 *  Video for Linux Two controls header file
 *
 *  Copyright (C) 1999-2012 the contributors
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  Alternatively you can redistribute this file under the terms of the
 *  BSD license as stated below:
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *  3. The names of its contributors may not be used to endorse or promote
 *     products derived from this software without specific prior written
 *     permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  The contents of this header was split off from videodev2.h. All control
 *  definitions should be added to this header, which is included by
 *  videodev2.h.
 */

#ifndef __LINUX_RENESAS_V4L2_CONTROLS_H
#define __LINUX_RENESAS_V4L2_CONTROLS_H

#include <linux/v4l2-controls.h>

#define V4L2_CID_RZ_ISP_IN_FMT		(V4L2_CID_CAMERA_CLASS_BASE+34)
enum v4l2_rz_isp_in_fmt_range {
	V4L2_RZ_ISP_IN_FMT_RAW8 		= 0,
	V4L2_RZ_ISP_IN_FMT_RAW10		= 1,
	V4L2_RZ_ISP_IN_FMT_RAW12		= 2,
};
#define V4L2_CID_RZ_ISP_FRM_INTERVAL		(V4L2_CID_CAMERA_CLASS_BASE+35)
#define V4L2_CID_RZ_ISP_BL			(V4L2_CID_CAMERA_CLASS_BASE+36)
#define V4L2_CID_RZ_ISP_WB			(V4L2_CID_CAMERA_CLASS_BASE+37)
enum v4l2_rz_isp_wb_range {
	V4L2_RZ_ISP_WB_DAYLIGHT			= 0,
	V4L2_RZ_ISP_WB_HORIZON			= 1,
	V4L2_RZ_ISP_WB_WHITE			= 2,
	V4L2_RZ_ISP_WB_STUDIO_LAMP		= 3,
};
#define V4L2_CID_RZ_ISP_GAMMA			(V4L2_CID_CAMERA_CLASS_BASE+38)
#define V4L2_CID_RZ_ISP_CMX			(V4L2_CID_CAMERA_CLASS_BASE+39)
enum v4l2_rz_isp_cmx_range {
	V4L2_RZ_ISP_CMX_NONE			= 0,
	V4L2_RZ_ISP_CMX_NORMAL			= 1,
	V4L2_RZ_ISP_CMX_VIVID			= 2,
	V4L2_RZ_ISP_CMX_SEPIA			= 3,
};
#define V4L2_CID_RZ_ISP_2DNR			(V4L2_CID_CAMERA_CLASS_BASE+40)
#define V4L2_CID_RZ_ISP_3DNR			(V4L2_CID_CAMERA_CLASS_BASE+41)
#define V4L2_CID_RZ_ISP_EMP			(V4L2_CID_CAMERA_CLASS_BASE+42)
enum v4l2_rz_isp_emp_range {
	V4L2_RZ_ISP_EMP_NONE			= 0,
	V4L2_RZ_ISP_EMP_WEAK			= 1,
	V4L2_RZ_ISP_EMP_NORMAL			= 2,
	V4L2_RZ_ISP_EMP_STRONG			= 3,
};
#define V4L2_CID_RZ_ISP_DRP_LV		(V4L2_CID_CAMERA_CLASS_BASE+43)
#define V4L2_CID_RZ_ISP_AE			(V4L2_CID_CAMERA_CLASS_BASE+44)
#define V4L2_CID_RZ_ISP_EXPOSE_LV	(V4L2_CID_CAMERA_CLASS_BASE+45)
#define V4L2_CID_RZ_ISP_T_BL		(V4L2_CID_CAMERA_CLASS_BASE+46)
#define V4L2_CID_RZ_ISP_THRESHOLD	(V4L2_CID_CAMERA_CLASS_BASE+47)
#define V4L2_CID_RZ_ISP_ALL			(V4L2_CID_CAMERA_CLASS_BASE+48)
#define V4L2_RZ_ISP_ALL_DIMS		(512)
#define V4L2_CID_RZ_ISP_AWB			(V4L2_CID_CAMERA_CLASS_BASE+49)

#define V4L2_CID_RZ_ISP_DETAIL		(V4L2_CID_RZ_ISP_ALL)
#endif
