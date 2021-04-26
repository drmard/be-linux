// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* $XFree86$ */
/* $XdotOrg$ */
/*
 * Data and prototypes for init.c
 *
 * Copyright (C) 2001-2005 by Thomas Winischhofer, Vienna, Austria
 *
 * If distributed as part of the Linux kernel, the following license terms
 * apply:
 *
 * * This program is free software; you can redistribute it and/or modify
 * * it under the terms of the GNU General Public License as published by
 * * the Free Software Foundation; either version 2 of the named License,
 * * or any later version.
 * *
 * * This program is distributed in the hope that it will be useful,
 * * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * * GNU General Public License for more details.
 * *
 * * You should have received a copy of the GNU General Public License
 * * along with this program; if not, write to the Free Software
 * * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Otherwise, the following license terms apply:
 *
 * * Redistribution and use in source and binary forms, with or without
 * * modification, are permitted provided that the following conditions
 * * are met:
 * * 1) Redistributions of source code must retain the above copyright
 * *    notice, this list of conditions and the following disclaimer.
 * * 2) Redistributions in binary form must reproduce the above copyright
 * *    notice, this list of conditions and the following disclaimer in the
 * *    documentation and/or other materials provided with the distribution.
 * * 3) The name of the author may not be used to endorse or promote products
 * *    derived from this software without specific prior written permission.
 * *
 * * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author:	Thomas Winischhofer <thomas@winischhofer.net>
 *
 */

#ifndef _SISUSB_INIT_H_
#define _SISUSB_INIT_H_

/* SiS_ModeType */
#define ModeText		0x00
#define ModeCGA			0x01
#define ModeEGA			0x02
#define ModeVGA			0x03
#define Mode15Bpp		0x04
#define Mode16Bpp		0x05
#define Mode24Bpp		0x06
#define Mode32Bpp		0x07

#define ModeTypeMask		0x07
#define IsTextMode		0x07

#define DACInfoFlag		0x0018
#define MemoryInfoFlag		0x01E0
#define MemorySizeShift		5

/* modeflag */
#define Charx8Dot		0x0200
#define LineCompareOff		0x0400
#define CRT2Mode		0x0800
#define HalfDCLK		0x1000
#define NoSupportSimuTV		0x2000
#define NoSupportLCDScale	0x4000	/* SiS bridge: No scaling possible (no matter what panel) */
#define DoubleScanMode		0x8000

/* Infoflag */
#define SupportTV		0x0008
#define SupportTV1024		0x0800
#define SupportCHTV		0x0800
#define Support64048060Hz	0x0800	/* Special for 640x480 LCD */
#define SupportHiVision		0x0010
#define SupportYPbPr750p	0x1000
#define SupportLCD		0x0020
#define SupportRAMDAC2		0x0040	/* All           (<= 100Mhz) */
#define SupportRAMDAC2_135	0x0100	/* All except DH (<= 135Mhz) */
#define SupportRAMDAC2_162	0x0200	/* B, C          (<= 162Mhz) */
#define SupportRAMDAC2_202	0x0400	/* C             (<= 202Mhz) */
#define InterlaceMode		0x0080
#define SyncPP			0x0000
#define SyncPN			0x4000
#define SyncNP			0x8000
#define SyncNN			0xc000

/* SetFlag */
#define ProgrammingCRT2		0x0001
#define LowModeTests		0x0002
#define LCDVESATiming		0x0008
#define EnableLVDSDDA		0x0010
#define SetDispDevSwitchFlag	0x0020
#define CheckWinDos		0x0040
#define SetDOSMode		0x0080

/* Index in ModeResInfo table */
#define SIS_RI_320x200		0
#define SIS_RI_320x240		1
#define SIS_RI_320x400		2
#define SIS_RI_400x300		3
#define SIS_RI_512x384		4
#define SIS_RI_640x400		5
#define SIS_RI_640x480		6
#define SIS_RI_800x600		7
#define SIS_RI_1024x768		8
#define SIS_RI_1280x1024	9
#define SIS_RI_1600x1200	10
#define SIS_RI_1920x1440	11
#define SIS_RI_2048x1536	12
#define SIS_RI_720x480		13
#define SIS_RI_720x576		14
#define SIS_RI_1280x960		15
#define SIS_RI_800x480		16
#define SIS_RI_1024x576		17
#define SIS_RI_1280x720		18
#define SIS_RI_856x480		19
#define SIS_RI_1280x768		20
#define SIS_RI_1400x1050	21
#define SIS_RI_1152x864		22	/* Up to here SiS conforming */
#define SIS_RI_848x480		23
#define SIS_RI_1360x768		24
#define SIS_RI_1024x600		25
#define SIS_RI_1152x768		26
#define SIS_RI_768x576		27
#define SIS_RI_1360x1024	28
#define SIS_RI_1680x1050	29
#define SIS_RI_1280x800		30
#define SIS_RI_1920x1080	31
#define SIS_RI_960x540		32
#define SIS_RI_960x600		33

#define SIS_VIDEO_CAPTURE	0x00 - 0x30
#define SIS_VIDEO_PLAYBACK	0x02 - 0x30
#define SIS_CRT2_PORT_04	0x04 - 0x30

/* Mode numbers */
static const unsigned short ModeIndex_320x200[] = { 0x59, 0x41, 0x00, 0x4f };
static const unsigned short ModeIndex_320x240[] = { 0x50, 0x56, 0x00, 0x53 };
static const unsigned short ModeIndex_400x300[] = { 0x51, 0x57, 0x00, 0x54 };
static const unsigned short ModeIndex_512x384[] = { 0x52, 0x58, 0x00, 0x5c };
static const unsigned short ModeIndex_640x400[] = { 0x2f, 0x5d, 0x00, 0x5e };
static const unsigned short ModeIndex_640x480[] = { 0x2e, 0x44, 0x00, 0x62 };
static const unsigned short ModeIndex_720x480[] = { 0x31, 0x33, 0x00, 0x35 };
static const unsigned short ModeIndex_720x576[] = { 0x32, 0x34, 0x00, 0x36 };
static const unsigned short ModeIndex_768x576[] = { 0x5f, 0x60, 0x00, 0x61 };
static const unsigned short ModeIndex_800x480[] = { 0x70, 0x7a, 0x00, 0x76 };
static const unsigned short ModeIndex_800x600[] = { 0x30, 0x47, 0x00, 0x63 };
static const unsigned short ModeIndex_848x480[] = { 0x39, 0x3b, 0x00, 0x3e };
static const unsigned short ModeIndex_856x480[] = { 0x3f, 0x42, 0x00, 0x45 };
static const unsigned short ModeIndex_960x540[] = { 0x1d, 0x1e, 0x00, 0x1f };
static const unsigned short ModeIndex_960x600[] = { 0x20, 0x21, 0x00, 0x22 };
static const unsigned short ModeIndex_1024x768[] = { 0x38, 0x4a, 0x00, 0x64 };
static const unsigned short ModeIndex_1024x576[] = { 0x71, 0x74, 0x00, 0x77 };
static const unsigned short ModeIndex_1152x864[] = { 0x29, 0x2a, 0x00, 0x2b };
static const unsigned short ModeIndex_1280x720[] = { 0x79, 0x75, 0x00, 0x78 };
static const unsigned short ModeIndex_1280x768[] = { 0x23, 0x24, 0x00, 0x25 };
static const unsigned short ModeIndex_1280x1024[] = { 0x3a, 0x4d, 0x00, 0x65 };

static const unsigned char SiS_MDA_DAC[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15,
	0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15,
	0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15,
	0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15,
	0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F
};

static const unsigned char SiS_CGA_DAC[] = {
	0x00, 0x10, 0x04, 0x14, 0x01, 0x11, 0x09, 0x15,
	0x00, 0x10, 0x04, 0x14, 0x01, 0x11, 0x09, 0x15,
	0x2A, 0x3A, 0x2E, 0x3E, 0x2B, 0x3B, 0x2F, 0x3F,
	0x2A, 0x3A, 0x2E, 0x3E, 0x2B, 0x3B, 0x2F, 0x3F,
	0x00, 0x10, 0x04, 0x14, 0x01, 0x11, 0x09, 0x15,
	0x00, 0x10, 0x04, 0x14, 0x01, 0x11, 0x09, 0x15,
	0x2A, 0x3A, 0x2E, 0x3E, 0x2B, 0x3B, 0x2F, 0x3F,
	0x2A, 0x3A, 0x2E, 0x3E, 0x2B, 0x3B, 0x2F, 0x3F
};

static const unsigned char SiS_EGA_DAC[] = {
	0x00, 0x10, 0x04, 0x14, 0x01, 0x11, 0x05, 0x15,
	0x20, 0x30, 0x24, 0x34, 0x21, 0x31, 0x25, 0x35,
	0x08, 0x18, 0x0C, 0x1C, 0x09, 0x19, 0x0D, 0x1D,
	0x28, 0x38, 0x2C, 0x3C, 0x29, 0x39, 0x2D, 0x3D,
	0x02, 0x12, 0x06, 0x16, 0x03, 0x13, 0x07, 0x17,
	0x22, 0x32, 0x26, 0x36, 0x23, 0x33, 0x27, 0x37,
	0x0A, 0x1A, 0x0E, 0x1E, 0x0B, 0x1B, 0x0F, 0x1F,
	0x2A, 0x3A, 0x2E, 0x3E, 0x2B, 0x3B, 0x2F, 0x3F
};

static const unsigned char SiS_VGA_DAC[] = {
	0x00, 0x10, 0x04, 0x14, 0x01, 0x11, 0x09, 0x15,
	0x2A, 0x3A, 0x2E, 0x3E, 0x2B, 0x3B, 0x2F, 0x3F,
	0x00, 0x05, 0x08, 0x0B, 0x0E, 0x11, 0x14, 0x18,
	0x1C, 0x20, 0x24, 0x28, 0x2D, 0x32, 0x38, 0x3F,
	0x00, 0x10, 0x1F, 0x2F, 0x3F, 0x1F, 0x27, 0x2F,
	0x37, 0x3F, 0x2D, 0x31, 0x36, 0x3A, 0x3F, 0x00,
	0x07, 0x0E, 0x15, 0x1C, 0x0E, 0x11, 0x15, 0x18,
	0x1C, 0x14, 0x16, 0x18, 0x1A, 0x1C, 0x00, 0x04,
	0x08, 0x0C, 0x10, 0x08, 0x0A, 0x0C, 0x0E, 0x10,
	0x0B, 0x0C, 0x0D, 0x0F, 0x10
};

static const struct SiS_St SiSUSB_SModeIDTable[] = {
	{0x03, 0x0010, 0x18, 0x02, 0x02, 0x00, 0x01, 0x03, 0x40},
	{0xff, 0x0000, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};

static const struct SiS_StResInfo_S SiSUSB_StResInfo[] = {
	{640, 400},
	{640, 350},
	{720, 400},
	{720, 350},
	{640, 480}
};

static const struct SiS_ModeResInfo SiSUSB_ModeResInfo[] = {
	{320, 200, 8, 8},	/* 0x00 */
	{320, 240, 8, 8},	/* 0x01 */
	{320, 400, 8, 8},	/* 0x02 */
	{400, 300, 8, 8},	/* 0x03 */
	{512, 384, 8, 8},	/* 0x04 */
	{640, 400, 8, 16},	/* 0x05 */
	{640, 480, 8, 16},	/* 0x06 */
	{800, 600, 8, 16},	/* 0x07 */
	{1024, 768, 8, 16},	/* 0x08 */
	{1280, 1024, 8, 16},	/* 0x09 */
	{1600, 1200, 8, 16},	/* 0x0a */
	{1920, 1440, 8, 16},	/* 0x0b */
	{2048, 1536, 8, 16},	/* 0x0c */
	{720, 480, 8, 16},	/* 0x0d */
	{720, 576, 8, 16},	/* 0x0e */
	{1280, 960, 8, 16},	/* 0x0f */
	{800, 480, 8, 16},	/* 0x10 */
	{1024, 576, 8, 16},	/* 0x11 */
	{1280, 720, 8, 16},	/* 0x12 */
	{856, 480, 8, 16},	/* 0x13 */
	{1280, 768, 8, 16},	/* 0x14 */
	{1400, 1050, 8, 16},	/* 0x15 */
	{1152, 864, 8, 16},	/* 0x16 */
	{848, 480, 8, 16},	/* 0x17 */
	{1360, 768, 8, 16},	/* 0x18 */
	{1024, 600, 8, 16},	/* 0x19 */
	{1152, 768, 8, 16},	/* 0x1a */
	{768, 576, 8, 16},	/* 0x1b */
	{1360, 1024, 8, 16},	/* 0x1c */
	{1680, 1050, 8, 16},	/* 0x1d */
	{1280, 800, 8, 16},	/* 0x1e */
	{1920, 1080, 8, 16},	/* 0x1f */
	{960, 540, 8, 16},	/* 0x20 */
	{960, 600, 8, 16}	/* 0x21 */
};

static const struct SiS_StandTable SiSUSB_StandTable[] = {
	/* MD_3_400 - mode 0x03 - 400 */
	{
	 0x50, 0x18, 0x10, 0x1000,
	 {0x00, 0x03, 0x00, 0x02},
	 0x67,
	 {0x5f, 0x4f, 0x50, 0x82, 0x55, 0x81, 0xbf, 0x1f,
	  0x00, 0x4f, 0x0d, 0x0e, 0x00, 0x00, 0x00, 0x00,
	  0x9c, 0x8e, 0x8f, 0x28, 0x1f, 0x96, 0xb9, 0xa3,
	  0xff},
	 {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x14, 0x07,
	  0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
	  0x0c, 0x00, 0x0f, 0x08},
	 {0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x0e, 0x00, 0xff}
	 },
	/* Generic for VGA and higher */
	{
	 0x00, 0x00, 0x00, 0x0000,
	 {0x01, 0x0f, 0x00, 0x0e},
	 0x23,
	 {0x5f, 0x4f, 0x50, 0x82, 0x54, 0x80, 0x0b, 0x3e,
	  0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  0xea, 0x8c, 0xdf, 0x28, 0x40, 0xe7, 0x04, 0xa3,
	  0xff},
	 {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	  0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	  0x01, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x05, 0x0f, 0xff}
	 }
};

static const struct SiS_Ext SiSUSB_EModeIDTable[] = {
	{0x2e, 0x0a1b, 0x0101, SIS_RI_640x480, 0x00, 0x00, 0x05, 0x05, 0x08, 2},	/* 640x480x8 */
	{0x2f, 0x0a1b, 0x0100, SIS_RI_640x400, 0x00, 0x00, 0x05, 0x05, 0x10, 0},	/* 640x400x8 */
	{0x30, 0x2a1b, 0x0103, SIS_RI_800x600, 0x00, 0x00, 0x07, 0x06, 0x00, 3},	/* 800x600x8 */
	{0x31, 0x4a1b, 0x0000, SIS_RI_720x480, 0x00, 0x00, 0x06, 0x06, 0x11, -1},	/* 720x480x8 */
	{0x32, 0x4a1b, 0x0000, SIS_RI_720x576, 0x00, 0x00, 0x06, 0x06, 0x12, -1},	/* 720x576x8 */
	{0x33, 0x4a1d, 0x0000, SIS_RI_720x480, 0x00, 0x00, 0x06, 0x06, 0x11, -1},	/* 720x480x16 */
	{0x34, 0x6a1d, 0x0000, SIS_RI_720x576, 0x00, 0x00, 0x06, 0x06, 0x12, -1},	/* 720x576x16 */
	{0x35, 0x4a1f, 0x0000, SIS_RI_720x480, 0x00, 0x00, 0x06, 0x06, 0x11, -1},	/* 720x480x32 */
	{0x36, 0x6a1f, 0x0000, SIS_RI_720x576, 0x00, 0x00, 0x06, 0x06, 0x12, -1},	/* 720x576x32 */
	{0x38, 0x0a1b, 0x0105, SIS_RI_1024x768, 0x00, 0x00, 0x08, 0x07, 0x13, 4},	/* 1024x768x8 */
	{0x3a, 0x0e3b, 0x0107, SIS_RI_1280x1024, 0x00, 0x00, 0x00, 0x00, 0x2f, 8},	/* 1280x1024x8 */
	{0x41, 0x9a1d, 0x010e, SIS_RI_320x200, 0x00, 0x00, 0x04, 0x04, 0x1a, 0},	/* 320x200x16 */
	{0x44, 0x0a1d, 0x0111, SIS_RI_640x480, 0x00, 0x00, 0x05, 0x05, 0x08, 2},	/* 640x480x16 */
	{0x47, 0x2a1d, 0x0114, SIS_RI_800x600, 0x00, 0x00, 0x07, 0x06, 0x00, 3},	/* 800x600x16 */
	{0x4a, 0x0a3d, 0x0117, SIS_RI_1024x768, 0x00, 0x00, 0x08, 0x07, 0x13, 4},	/* 1024x768x16 */
	{0x4d, 0x0e7d, 0x011a, SIS_RI_1280x1024, 0x00, 0x00, 0x00, 0x00, 0x2f, 8},	/* 1280x1024x16 */
	{0x50, 0x9a1b, 0x0132, SIS_RI_320x240, 0x00, 0x00, 0x04, 0x04, 0x1b, 2},	/* 320x240x8  */
	{0x51, 0xba1b, 0x0133, SIS_RI_400x300, 0x00, 0x00, 0x07, 0x07, 0x1c, 3},	/* 400x300x8  */
	{0x52, 0xba1b, 0x0134, SIS_RI_512x384, 0x00, 0x00, 0x00, 0x00, 0x1d, 4},	/* 512x384x8  */
	{0x56, 0x9a1d, 0x0135, SIS_RI_320x240, 0x00, 0x00, 0x04, 0x04, 0x1b, 2},	/* 320x240x16 */
	{0x57, 0xba1d, 0x0136, SIS_RI_400x300, 0x00, 0x00, 0x07, 0x07, 0x1c, 3},	/* 400x300x16 */
	{0x58, 0xba1d, 0x0137, SIS_RI_512x384, 0x00, 0x00, 0x00, 0x00, 0x1d, 4},	/* 512x384x16 */
	{0x59, 0x9a1b, 0x0138, SIS_RI_320x200, 0x00, 0x00, 0x04, 0x04, 0x1a, 0},	/* 320x200x8  */
	{0x5c, 0xba1f, 0x0000, SIS_RI_512x384, 0x00, 0x00, 0x00, 0x00, 0x1d, 4},	/* 512x384x32 */
	{0x5d, 0x0a1d, 0x0139, SIS_RI_640x400, 0x00, 0x00, 0x05, 0x07, 0x10, 0},	/* 640x400x16 */
	{0x5e, 0x0a1f, 0x0000, SIS_RI_640x400, 0x00, 0x00, 0x05, 0x07, 0x10, 0},	/* 640x400x32 */
	{0x62, 0x0a3f, 0x013a, SIS_RI_640x480, 0x00, 0x00, 0x05, 0x05, 0x08, 2},	/* 640x480x32 */
	{0x63, 0x2a3f, 0x013b, SIS_RI_800x600, 0x00, 0x00, 0x07, 0x06, 0x00, 3},	/* 800x600x32 */
	{0x64, 0x0a7f, 0x013c, SIS_RI_1024x768, 0x00, 0x00, 0x08, 0x07, 0x13, 4},	/* 1024x768x32 */
	{0x65, 0x0eff, 0x013d, SIS_RI_1280x1024, 0x00, 0x00, 0x00, 0x00, 0x2f, 8},	/* 1280x1024x32 */
	{0x70, 0x6a1b, 0x0000, SIS_RI_800x480, 0x00, 0x00, 0x07, 0x07, 0x1e, -1},	/* 800x480x8 */
	{0x71, 0x4a1b, 0x0000, SIS_RI_1024x576, 0x00, 0x00, 0x00, 0x00, 0x21, -1},	/* 1024x576x8 */
	{0x74, 0x4a1d, 0x0000, SIS_RI_1024x576, 0x00, 0x00, 0x00, 0x00, 0x21, -1},	/* 1024x576x16 */
	{0x75, 0x0a3d, 0x0000, SIS_RI_1280x720, 0x00, 0x00, 0x00, 0x00, 0x24, 5},	/* 1280x720x16 */
	{0x76, 0x6a1f, 0x0000, SIS_RI_800x480, 0x00, 0x00, 0x07, 0x07, 0x1e, -1},	/* 800x480x32 */
	{0x77, 0x4a1f, 0x0000, SIS_RI_1024x576, 0x00, 0x00, 0x00, 0x00, 0x21, -1},	/* 1024x576x32 */
	{0x78, 0x0a3f, 0x0000, SIS_RI_1280x720, 0x00, 0x00, 0x00, 0x00, 0x24, 5},	/* 1280x720x32 */
	{0x79, 0x0a3b, 0x0000, SIS_RI_1280x720, 0x00, 0x00, 0x00, 0x00, 0x24, 5},	/* 1280x720x8 */
	{0x7a, 0x6a1d, 0x0000, SIS_RI_800x480, 0x00, 0x00, 0x07, 0x07, 0x1e, -1},	/* 800x480x16 */
	{0x23, 0x0e3b, 0x0000, SIS_RI_1280x768, 0x00, 0x00, 0x00, 0x00, 0x27, 6},	/* 1280x768x8 */
	{0x24, 0x0e7d, 0x0000, SIS_RI_1280x768, 0x00, 0x00, 0x00, 0x00, 0x27, 6},	/* 1280x768x16 */
	{0x25, 0x0eff, 0x0000, SIS_RI_1280x768, 0x00, 0x00, 0x00, 0x00, 0x27, 6},	/* 1280x768x32 */
	{0x39, 0x6a1b, 0x0000, SIS_RI_848x480, 0x00, 0x00, 0x00, 0x00, 0x28, -1},	/* 848x480 */
	{0x3b, 0x6a3d, 0x0000, SIS_RI_848x480, 0x00, 0x00, 0x00, 0x00, 0x28,
	 -1},
	{0x3e, 0x6a7f, 0x0000, SIS_RI_848x480, 0x00, 0x00, 0x00, 0x00, 0x28,
	 -1},
	{0x3f, 0x6a1b, 0x0000, SIS_RI_856x480, 0x00, 0x00, 0x00, 0x00, 0x2a, -1},	/* 856x480 */
	{0x42, 0x6a3d, 0x0000, SIS_RI_856x480, 0x00, 0x00, 0x00, 0x00, 0x2a,
	 -1},
	{0x45, 0x6a7f, 0x0000, SIS_RI_856x480, 0x00, 0x00, 0x00, 0x00, 0x2a,
	 -1},
	{0x4f, 0x9a1f, 0x0000, SIS_RI_320x200, 0x00, 0x00, 0x04, 0x04, 0x1a, 0},	/* 320x200x32 */
	{0x53, 0x9a1f, 0x0000, SIS_RI_320x240, 0x00, 0x00, 0x04, 0x04, 0x1b, 2},	/* 320x240x32 */
	{0x54, 0xba1f, 0x0000, SIS_RI_400x300, 0x00, 0x00, 0x07, 0x07, 0x1c, 3},	/* 400x300x32 */
	{0x5f, 0x6a1b, 0x0000, SIS_RI_768x576, 0x00, 0x00, 0x06, 0x06, 0x2c, -1},	/* 768x576 */
	{0x60, 0x6a1d, 0x0000, SIS_RI_768x576, 0x00, 0x00, 0x06, 0x06, 0x2c,
	 -1},
	{0x61, 0x6a3f, 0x0000, SIS_RI_768x576, 0x00, 0x00, 0x06, 0x06, 0x2c,
	 -1},
	{0x1d, 0x6a1b, 0x0000, SIS_RI_960x540, 0x00, 0x00, 0x00, 0x00, 0x2d, -1},	/* 960x540 */
	{0x1e, 0x6a3d, 0x0000, SIS_RI_960x540, 0x00, 0x00, 0x00, 0x00, 0x2d,
	 -1},
	{0x1f, 0x6a7f, 0x0000, SIS_RI_960x540, 0x00, 0x00, 0x00, 0x00, 0x2d,
	 -1},
	{0x20, 0x6a1b, 0x0000, SIS_RI_960x600, 0x00, 0x00, 0x00, 0x00, 0x2e, -1},	/* 960x600 */
	{0x21, 0x6a3d, 0x0000, SIS_RI_960x600, 0x00, 0x00, 0x00, 0x00, 0x2e,
	 -1},
	{0x22, 0x6a7f, 0x0000, SIS_RI_960x600, 0x00, 0x00, 0x00, 0x00, 0x2e,
	 -1},
	{0x29, 0x4e1b, 0x0000, SIS_RI_1152x864, 0x00, 0x00, 0x00, 0x00, 0x33, -1},	/* 1152x864 */
	{0x2a, 0x4e3d, 0x0000, SIS_RI_1152x864, 0x00, 0x00, 0x00, 0x00, 0x33,
	 -1},
	{0x2b, 0x4e7f, 0x0000, SIS_RI_1152x864, 0x00, 0x00, 0x00, 0x00, 0x33,
	 -1},
	{0xff, 0x0000, 0x0000, 0, 0x00, 0x00, 0x00, 0x00, 0x00, -1}
};

static const struct SiS_Ext2 SiSUSB_RefIndex[] = {
	{0x085f, 0x0d, 0x03, 0x05, 0x05, 0x30, 800, 600, 0x40, 0x00, 0x00},	/* 0x0 */
	{0x0067, 0x0e, 0x04, 0x05, 0x05, 0x30, 800, 600, 0x40, 0x00, 0x00},	/* 0x1 */
	{0x0067, 0x0f, 0x08, 0x48, 0x05, 0x30, 800, 600, 0x40, 0x00, 0x00},	/* 0x2 */
	{0x0067, 0x10, 0x07, 0x8b, 0x05, 0x30, 800, 600, 0x40, 0x00, 0x00},	/* 0x3 */
	{0x0047, 0x11, 0x0a, 0x00, 0x05, 0x30, 800, 600, 0x40, 0x00, 0x00},	/* 0x4 */
	{0x0047, 0x12, 0x0d, 0x00, 0x05, 0x30, 800, 600, 0x40, 0x00, 0x00},	/* 0x5 */
	{0x0047, 0x13, 0x13, 0x00, 0x05, 0x30, 800, 600, 0x20, 0x00, 0x00},	/* 0x6 */
	{0x0107, 0x14, 0x1c, 0x00, 0x05, 0x30, 800, 600, 0x20, 0x00, 0x00},	/* 0x7 */
	{0xc85f, 0x05, 0x00, 0x04, 0x04, 0x2e, 640, 480, 0x40, 0x00, 0x00},	/* 0x8 */
	{0xc067, 0x06, 0x02, 0x04, 0x04, 0x2e, 640, 480, 0x40, 0x00, 0x00},	/* 0x9 */
	{0xc067, 0x07, 0x02, 0x47, 0x04, 0x2e, 640, 480, 0x40, 0x00, 0x00},	/* 0xa */
	{0xc067, 0x08, 0x03, 0x8a, 0x04, 0x2e, 640, 480, 0x40, 0x00, 0x00},	/* 0xb */
	{0xc047, 0x09, 0x05, 0x00, 0x04, 0x2e, 640, 480, 0x40, 0x00, 0x00},	/* 0xc */
	{0xc047, 0x0a, 0x09, 0x00, 0x04, 0x2e, 640, 480, 0x40, 0x00, 0x00},	/* 0xd */
	{0xc047, 0x0b, 0x0e, 0x00, 0x04, 0x2e, 640, 480, 0x40, 0x00, 0x00},	/* 0xe */
	{0xc047, 0x0c, 0x15, 0x00, 0x04, 0x2e, 640, 480, 0x40, 0x00, 0x00},	/* 0xf */
	{0x487f, 0x04, 0x00, 0x00, 0x00, 0x2f, 640, 400, 0x30, 0x55, 0x6e},	/* 0x10 */
	{0xc06f, 0x3c, 0x01, 0x06, 0x13, 0x31, 720, 480, 0x30, 0x00, 0x00},	/* 0x11 */
	{0x006f, 0x3d, 0x6f, 0x06, 0x14, 0x32, 720, 576, 0x30, 0x00, 0x00},	/* 0x12 (6f was 03) */
	{0x0087, 0x15, 0x06, 0x00, 0x06, 0x38, 1024, 768, 0x30, 0x00, 0x00},	/* 0x13 */
	{0xc877, 0x16, 0x0b, 0x06, 0x06, 0x38, 1024, 768, 0x20, 0x00, 0x00},	/* 0x14 */
	{0xc067, 0x17, 0x0f, 0x49, 0x06, 0x38, 1024, 768, 0x20, 0x00, 0x00},	/* 0x15 */
	{0x0067, 0x18, 0x11, 0x00, 0x06, 0x38, 1024, 768, 0x20, 0x00, 0x00},	/* 0x16 */
	{0x0047, 0x19, 0x16, 0x8c, 0x06, 0x38, 1024, 768, 0x20, 0x00, 0x00},	/* 0x17 */
	{0x0107, 0x1a, 0x1b, 0x00, 0x06, 0x38, 1024, 768, 0x10, 0x00, 0x00},	/* 0x18 */
	{0x0107, 0x1b, 0x1f, 0x00, 0x06, 0x38, 1024, 768, 0x10, 0x00, 0x00},	/* 0x19 */
	{0x407f, 0x00, 0x00, 0x00, 0x00, 0x41, 320, 200, 0x30, 0x56, 0x4e},	/* 0x1a */
	{0xc07f, 0x01, 0x00, 0x04, 0x04, 0x50, 320, 240, 0x30, 0x00, 0x00},	/* 0x1b */
	{0x007f, 0x02, 0x04, 0x05, 0x05, 0x51, 400, 300, 0x30, 0x00, 0x00},	/* 0x1c */
	{0xc077, 0x03, 0x0b, 0x06, 0x06, 0x52, 512, 384, 0x30, 0x00, 0x00},	/* 0x1d */
	{0x0077, 0x32, 0x40, 0x08, 0x18, 0x70, 800, 480, 0x30, 0x00, 0x00},	/* 0x1e */
	{0x0047, 0x33, 0x07, 0x08, 0x18, 0x70, 800, 480, 0x30, 0x00, 0x00},	/* 0x1f */
	{0x0047, 0x34, 0x0a, 0x08, 0x18, 0x70, 800, 480, 0x30, 0x00, 0x00},	/* 0x20 */
	{0x0077, 0x35, 0x0b, 0x09, 0x19, 0x71, 1024, 576, 0x30, 0x00, 0x00},	/* 0x21 */
	{0x0047, 0x36, 0x11, 0x09, 0x19, 0x71, 1024, 576, 0x30, 0x00, 0x00},	/* 0x22 */
	{0x0047, 0x37, 0x16, 0x09, 0x19, 0x71, 1024, 576, 0x30, 0x00, 0x00},	/* 0x23 */
	{0x1137, 0x38, 0x19, 0x0a, 0x0c, 0x75, 1280, 720, 0x30, 0x00, 0x00},	/* 0x24 */
	{0x1107, 0x39, 0x1e, 0x0a, 0x0c, 0x75, 1280, 720, 0x30, 0x00, 0x00},	/* 0x25 */
	{0x1307, 0x3a, 0x20, 0x0a, 0x0c, 0x75, 1280, 720, 0x30, 0x00, 0x00},	/* 0x26 */
	{0x0077, 0x42, 0x5b, 0x08, 0x11, 0x23, 1280, 768, 0x30, 0x00, 0x00},	/* 0x27 */
	{0x0087, 0x45, 0x57, 0x00, 0x16, 0x39, 848, 480, 0x30, 0x00, 0x00},	/* 0x28 38Hzi  */
	{0xc067, 0x46, 0x55, 0x0b, 0x16, 0x39, 848, 480, 0x30, 0x00, 0x00},	/* 0x29 848x480-60Hz   */
	{0x0087, 0x47, 0x57, 0x00, 0x17, 0x3f, 856, 480, 0x30, 0x00, 0x00},	/* 0x2a 856x480-38Hzi  */
	{0xc067, 0x48, 0x57, 0x00, 0x17, 0x3f, 856, 480, 0x30, 0x00, 0x00},	/* 0x2b 856x480-60Hz   */
	{0x006f, 0x4d, 0x71, 0x06, 0x15, 0x5f, 768, 576, 0x30, 0x00, 0x00},	/* 0x2c 768x576-56Hz   */
	{0x0067, 0x52, 0x6a, 0x00, 0x1c, 0x1d, 960, 540, 0x30, 0x00, 0x00},	/* 0x2d 960x540 60Hz */
	{0x0077, 0x53, 0x6b, 0x0b, 0x1d, 0x20, 960, 600, 0x30, 0x00, 0x00},	/* 0x2e 960x600 60Hz */
	{0x0087, 0x1c, 0x11, 0x00, 0x07, 0x3a, 1280, 1024, 0x30, 0x00, 0x00},	/* 0x2f */
	{0x0137, 0x1d, 0x19, 0x07, 0x07, 0x3a, 1280, 1024, 0x00, 0x00, 0x00},	/* 0x30 */
	{0x0107, 0x1e, 0x1e, 0x00, 0x07, 0x3a, 1280, 1024, 0x00, 0x00, 0x00},	/* 0x31 */
	{0x0207, 0x1f, 0x20, 0x00, 0x07, 0x3a, 1280, 1024, 0x00, 0x00, 0x00},	/* 0x32 */
	{0x0127, 0x54, 0x6d, 0x00, 0x1a, 0x29, 1152, 864, 0x30, 0x00, 0x00},	/* 0x33 1152x864-60Hz  */
	{0x0127, 0x44, 0x19, 0x00, 0x1a, 0x29, 1152, 864, 0x30, 0x00, 0x00},	/* 0x34 1152x864-75Hz  */
	{0x0127, 0x4a, 0x1e, 0x00, 0x1a, 0x29, 1152, 864, 0x30, 0x00, 0x00},	/* 0x35 1152x864-85Hz  */
	{0xffff, 0x00, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0x00, 0x00}
};

static const struct SiS_CRT1Table SiSUSB_CRT1Table[] = {
	{{0x2d, 0x27, 0x28, 0x90, 0x2c, 0x80, 0xbf, 0x1f,
	  0x9c, 0x8e, 0x8f, 0x96, 0xb9, 0x30, 0x00, 0x00,
	  0x00}},		/* 0x0 */
	{{0x2d, 0x27, 0x28, 0x90, 0x2c, 0x80, 0x0b, 0x3e,
	  0xe9, 0x8b, 0xdf, 0xe7, 0x04, 0x00, 0x00, 0x00,
	  0x00}},		/* 0x1 */
	{{0x3d, 0x31, 0x31, 0x81, 0x37, 0x1f, 0x72, 0xf0,
	  0x58, 0x8c, 0x57, 0x57, 0x73, 0x20, 0x00, 0x05,
	  0x01}},		/* 0x2 */
	{{0x4f, 0x3f, 0x3f, 0x93, 0x45, 0x0d, 0x24, 0xf5,
	  0x02, 0x88, 0xff, 0xff, 0x25, 0x10, 0x00, 0x01,
	  0x01}},		/* 0x3 */
	{{0x5f, 0x4f, 0x50, 0x82, 0x55, 0x81, 0xbf, 0x1f,
	  0x9c, 0x8e, 0x8f, 0x96, 0xb9, 0x30, 0x00, 0x05,
	  0x00}},		/* 0x4 */
	{{0x5f, 0x4f, 0x4f, 0x83, 0x55, 0x81, 0x0b, 0x3e,
	  0xe9, 0x8b, 0xdf, 0xe8, 0x0c, 0x00, 0x00, 0x05,
	  0x00}},		/* 0x5 */
	{{0x63, 0x4f, 0x4f, 0x87, 0x56, 0x9b, 0x06, 0x3e,
	  0xe8, 0x8a, 0xdf, 0xe7, 0x07, 0x00, 0x00, 0x01,
	  0x00}},		/* 0x6 */
	{{0x64, 0x4f, 0x4f, 0x88, 0x55, 0x9d, 0xf2, 0x1f,
	  0xe0, 0x83, 0xdf, 0xdf, 0xf3, 0x10, 0x00, 0x01,
	  0x00}},		/* 0x7 */
	{{0x63, 0x4f, 0x4f, 0x87, 0x5a, 0x81, 0xfb, 0x1f,
	  0xe0, 0x83, 0xdf, 0xdf, 0xfc, 0x10, 0x00, 0x05,
	  0x00}},		/* 0x8 */
	{{0x65, 0x4f, 0x4f, 0x89, 0x58, 0x80, 0xfb, 0x1f,
	  0xe0, 0x83, 0xdf, 0xdf, 0xfc, 0x10, 0x00, 0x05,
	  0x61}},		/* 0x9 */
	{{0x65, 0x4f, 0x4f, 0x89, 0x58, 0x80, 0x01, 0x3e,
	  0xe0, 0x83, 0xdf, 0xdf, 0x02, 0x00, 0x00, 0x05,
	  0x61}},		/* 0xa */
	{{0x67, 0x4f, 0x4f, 0x8b, 0x58, 0x81, 0x0d, 0x3e,
	  0xe0, 0x83, 0xdf, 0xdf, 0x0e, 0x00, 0x00, 0x05,
	  0x61}},		/* 0xb */
	{{0x65, 0x4f, 0x4f, 0x89, 0x57, 0x9f, 0xfb, 0x1f,
	  0xe6, 0x8a, 0xdf, 0xdf, 0xfc, 0x10, 0x00, 0x01,
	  0x00}},		/* 0xc */
	{{0x7b, 0x63, 0x63, 0x9f, 0x6a, 0x93, 0x6f, 0xf0,
	  0x58, 0x8a, 0x57, 0x57, 0x70, 0x20, 0x00, 0x05,
	  0x01}},		/* 0xd */
	{{0x7f, 0x63, 0x63, 0x83, 0x6c, 0x1c, 0x72, 0xf0,
	  0x58, 0x8c, 0x57, 0x57, 0x73, 0x20, 0x00, 0x06,
	  0x01}},		/* 0xe */
	{{0x7d, 0x63, 0x63, 0x81, 0x6e, 0x1d, 0x98, 0xf0,
	  0x7c, 0x82, 0x57, 0x57, 0x99, 0x00, 0x00, 0x06,
	  0x01}},		/* 0xf */
	{{0x7f, 0x63, 0x63, 0x83, 0x69, 0x13, 0x6f, 0xf0,
	  0x58, 0x8b, 0x57, 0x57, 0x70, 0x20, 0x00, 0x06,
	  0x01}},		/* 0x10 */
	{{0x7e, 0x63, 0x63, 0x82, 0x6b, 0x13, 0x75, 0xf0,
	  0x58, 0x8b, 0x57, 0x57, 0x76, 0x20, 0x00, 0x06,
	  0x01}},		/* 0x11 */
	{{0x81, 0x63, 0x63, 0x85, 0x6d, 0x18, 0x7a, 0xf0,
	  0x58, 0x8b, 0x57, 0x57, 0x7b, 0x20, 0x00, 0x06,
	  0x61}},		/* 0x12 */
	{{0x83, 0x63, 0x63, 0x87, 0x6e, 0x19, 0x81, 0xf0,
	  0x58, 0x8b, 0x57, 0x57, 0x82, 0x20, 0x00, 0x06,
	  0x61}},		/* 0x13 */
	{{0x85, 0x63, 0x63, 0x89, 0x6f, 0x1a, 0x91, 0xf0,
	  0x58, 0x8b, 0x57, 0x57, 0x92, 0x20, 0x00, 0x06,
	  0x61}},		/* 0x14 */
	{{0x99, 0x7f, 0x7f, 0x9d, 0x84, 0x1a, 0x96, 0x1f,
	  0x7f, 0x83, 0x7f, 0x7f, 0x97, 0x10, 0x00, 0x02,
	  0x00}},		/* 0x15 */
	{{0xa3, 0x7f, 0x7f, 0x87, 0x86, 0x97, 0x24, 0xf5,
	  0x02, 0x88, 0xff, 0xff, 0x25, 0x10, 0x00, 0x02,
	  0x01}},		/* 0x16 */
	{{0xa1, 0x7f, 0x7f, 0x85, 0x86, 0x97, 0x24, 0xf5,
	  0x02, 0x88, 0xff, 0xff, 0x25, 0x10, 0x00, 0x02,
	  0x01}},		/* 0x17 */
	{{0x9f, 0x7f, 0x7f, 0x83, 0x85, 0x91, 0x1e, 0xf5,
	  0x00, 0x83, 0xff, 0xff, 0x1f, 0x10, 0x00, 0x02,
	  0x01}},		/* 0x18 */
	{{0xa7, 0x7f, 0x7f, 0x8b, 0x89, 0x95, 0x26, 0xf5,
	  0x00, 0x83, 0xff, 0xff, 0x27, 0x10, 0x00, 0x02,
	  0x01}},		/* 0x19 */
	{{0xa9, 0x7f, 0x7f, 0x8d, 0x8c, 0x9a, 0x2c, 0xf5,
	  0x00, 0x83, 0xff, 0xff, 0x2d, 0x14, 0x00, 0x02,
	  0x62}},		/* 0x1a */
	{{0xab, 0x7f, 0x7f, 0x8f, 0x8d, 0x9b, 0x35, 0xf5,
	  0x00, 0x83, 0xff, 0xff, 0x36, 0x14, 0x00, 0x02,
	  0x62}},		/* 0x1b */
	{{0xcf, 0x9f, 0x9f, 0x93, 0xb2, 0x01, 0x14, 0xba,
	  0x00, 0x83, 0xff, 0xff, 0x15, 0x00, 0x00, 0x03,
	  0x00}},		/* 0x1c */
	{{0xce, 0x9f, 0x9f, 0x92, 0xa9, 0x17, 0x28, 0x5a,
	  0x00, 0x83, 0xff, 0xff, 0x29, 0x09, 0x00, 0x07,
	  0x01}},		/* 0x1d */
	{{0xce, 0x9f, 0x9f, 0x92, 0xa5, 0x17, 0x28, 0x5a,
	  0x00, 0x83, 0xff, 0xff, 0x29, 0x09, 0x00, 0x07,
	  0x01}},		/* 0x1e */
	{{0xd3, 0x9f, 0x9f, 0x97, 0xab, 0x1f, 0x2e, 0x5a,
	  0x00, 0x83, 0xff, 0xff, 0x2f, 0x09, 0x00, 0x07,
	  0x01}},		/* 0x1f */
	{{0x09, 0xc7, 0xc7, 0x8d, 0xd3, 0x0b, 0xe0, 0x10,
	  0xb0, 0x83, 0xaf, 0xaf, 0xe1, 0x2f, 0x01, 0x04,
	  0x00}},		/* 0x20 */
	{{0x09, 0xc7, 0xc7, 0x8d, 0xd3, 0x0b, 0xe0, 0x10,
	  0xb0, 0x83, 0xaf, 0xaf, 0xe1, 0x2f, 0x01, 0x04,
	  0x00}},		/* 0x21 */
	{{0x09, 0xc7, 0xc7, 0x8d, 0xd3, 0x0b, 0xe0, 0x10,
	  0xb0, 0x83, 0xaf, 0xaf, 0xe1, 0x2f, 0x01, 0x04,
	  0x00}},		/* 0x22 */
	{{0x09, 0xc7, 0xc7, 0x8d, 0xd3, 0x0b, 0xe0, 0x10,
	  0xb0, 0x83, 0xaf, 0xaf, 0xe1, 0x2f, 0x01, 0x04,
	  0x00}},		/* 0x23 */
	{{0x09, 0xc7, 0xc7, 0x8d, 0xd3, 0x0b, 0xe0, 0x10,
	  0xb0, 0x83, 0xaf, 0xaf, 0xe1, 0x2f, 0x01, 0x04,
	  0x00}},		/* 0x24 */
	{{0x09, 0xc7, 0xc7, 0x8d, 0xd3, 0x0b, 0xe0, 0x10,
	  0xb0, 0x83, 0xaf, 0xaf, 0xe1, 0x2f, 0x01, 0x04,
	  0x00}},		/* 0x25 */
	{{0x09, 0xc7, 0xc7, 0x8d, 0xd3, 0x0b, 0xe0, 0x10,
	  0xb0, 0x83, 0xaf, 0xaf, 0xe1, 0x2f, 0x01, 0x04,
	  0x00}},		/* 0x26 */
	{{0x40, 0xef, 0xef, 0x84, 0x03, 0x1d, 0xda, 0x1f,
	  0xa0, 0x83, 0x9f, 0x9f, 0xdb, 0x1f, 0x41, 0x01,
	  0x00}},		/* 0x27 */
	{{0x43, 0xef, 0xef, 0x87, 0x06, 0x00, 0xd4, 0x1f,
	  0xa0, 0x83, 0x9f, 0x9f, 0xd5, 0x1f, 0x41, 0x05,
	  0x63}},		/* 0x28 */
	{{0x45, 0xef, 0xef, 0x89, 0x07, 0x01, 0xd9, 0x1f,
	  0xa0, 0x83, 0x9f, 0x9f, 0xda, 0x1f, 0x41, 0x05,
	  0x63}},		/* 0x29 */
	{{0x40, 0xef, 0xef, 0x84, 0x03, 0x1d, 0xda, 0x1f,
	  0xa0, 0x83, 0x9f, 0x9f, 0xdb, 0x1f, 0x41, 0x01,
	  0x00}},		/* 0x2a */
	{{0x40, 0xef, 0xef, 0x84, 0x03, 0x1d, 0xda, 0x1f,
	  0xa0, 0x83, 0x9f, 0x9f, 0xdb, 0x1f, 0x41, 0x01,
	  0x00}},		/* 0x2b */
	{{0x40, 0xef, 0xef, 0x84, 0x03, 0x1d, 0xda, 0x1f,
	  0xa0, 0x83, 0x9f, 0x9f, 0xdb, 0x1f, 0x41, 0x01,
	  0x00}},		/* 0x2c */
	{{0x59, 0xff, 0xff, 0x9d, 0x17, 0x13, 0x33, 0xba,
	  0x00, 0x83, 0xff, 0xff, 0x34, 0x0f, 0x41, 0x05,
	  0x44}},		/* 0x2d */
	{{0x5b, 0xff, 0xff, 0x9f, 0x18, 0x14, 0x38, 0xba,
	  0x00, 0x83, 0xff, 0xff, 0x39, 0x0f, 0x41, 0x05,
	  0x44}},		/* 0x2e */
	{{0x5b, 0xff, 0xff, 0x9f, 0x18, 0x14, 0x3d, 0xba,
	  0x00, 0x83, 0xff, 0xff, 0x3e, 0x0f, 0x41, 0x05,
	  0x44}},		/* 0x2f */
	{{0x5d, 0xff, 0xff, 0x81, 0x19, 0x95, 0x41, 0xba,
	  0x00, 0x84, 0xff, 0xff, 0x42, 0x0f, 0x41, 0x05,
	  0x44}},		/* 0x30 */
	{{0x55, 0xff, 0xff, 0x99, 0x0d, 0x0c, 0x3e, 0xba,
	  0x00, 0x84, 0xff, 0xff, 0x3f, 0x0f, 0x41, 0x05,
	  0x00}},		/* 0x31 */
	{{0x7f, 0x63, 0x63, 0x83, 0x6c, 0x1c, 0x72, 0xba,
	  0x27, 0x8b, 0xdf, 0xdf, 0x73, 0x00, 0x00, 0x06,
	  0x01}},		/* 0x32 */
	{{0x7f, 0x63, 0x63, 0x83, 0x69, 0x13, 0x6f, 0xba,
	  0x26, 0x89, 0xdf, 0xdf, 0x6f, 0x00, 0x00, 0x06,
	  0x01}},		/* 0x33 */
	{{0x7f, 0x63, 0x63, 0x82, 0x6b, 0x13, 0x75, 0xba,
	  0x29, 0x8c, 0xdf, 0xdf, 0x75, 0x00, 0x00, 0x06,
	  0x01}},		/* 0x34 */
	{{0xa3, 0x7f, 0x7f, 0x87, 0x86, 0x97, 0x24, 0xf1,
	  0xaf, 0x85, 0x3f, 0x3f, 0x25, 0x30, 0x00, 0x02,
	  0x01}},		/* 0x35 */
	{{0x9f, 0x7f, 0x7f, 0x83, 0x85, 0x91, 0x1e, 0xf1,
	  0xad, 0x81, 0x3f, 0x3f, 0x1f, 0x30, 0x00, 0x02,
	  0x01}},		/* 0x36 */
	{{0xa7, 0x7f, 0x7f, 0x88, 0x89, 0x95, 0x26, 0xf1,
	  0xb1, 0x85, 0x3f, 0x3f, 0x27, 0x30, 0x00, 0x02,
	  0x01}},		/* 0x37 */
	{{0xce, 0x9f, 0x9f, 0x92, 0xa9, 0x17, 0x28, 0xc4,
	  0x7a, 0x8e, 0xcf, 0xcf, 0x29, 0x21, 0x00, 0x07,
	  0x01}},		/* 0x38 */
	{{0xce, 0x9f, 0x9f, 0x92, 0xa5, 0x17, 0x28, 0xd4,
	  0x7a, 0x8e, 0xcf, 0xcf, 0x29, 0x21, 0x00, 0x07,
	  0x01}},		/* 0x39 */
	{{0xd3, 0x9f, 0x9f, 0x97, 0xab, 0x1f, 0x2e, 0xd4,
	  0x7d, 0x81, 0xcf, 0xcf, 0x2f, 0x21, 0x00, 0x07,
	  0x01}},		/* 0x3a */
	{{0xdc, 0x9f, 0x9f, 0x80, 0xaf, 0x9d, 0xe6, 0xff,
	  0xc0, 0x83, 0xbf, 0xbf, 0xe7, 0x10, 0x00, 0x07,
	  0x01}},		/* 0x3b */
	{{0x6b, 0x59, 0x59, 0x8f, 0x5e, 0x8c, 0x0b, 0x3e,
	  0xe9, 0x8b, 0xdf, 0xe7, 0x04, 0x00, 0x00, 0x05,
	  0x00}},		/* 0x3c */
	{{0x6d, 0x59, 0x59, 0x91, 0x60, 0x89, 0x53, 0xf0,
	  0x41, 0x84, 0x3f, 0x3f, 0x54, 0x00, 0x00, 0x05,
	  0x41}},		/* 0x3d */
	{{0x86, 0x6a, 0x6a, 0x8a, 0x74, 0x06, 0x8c, 0x15,
	  0x4f, 0x83, 0xef, 0xef, 0x8d, 0x30, 0x00, 0x02,
	  0x00}},		/* 0x3e */
	{{0x81, 0x6a, 0x6a, 0x85, 0x70, 0x00, 0x0f, 0x3e,
	  0xeb, 0x8e, 0xdf, 0xdf, 0x10, 0x00, 0x00, 0x02,
	  0x00}},		/* 0x3f */
	{{0xa3, 0x7f, 0x7f, 0x87, 0x86, 0x97, 0x1e, 0xf1,
	  0xae, 0x85, 0x57, 0x57, 0x1f, 0x30, 0x00, 0x02,
	  0x01}},		/* 0x40 */
	{{0xa3, 0x7f, 0x7f, 0x87, 0x86, 0x97, 0x24, 0xf5,
	  0x02, 0x88, 0xff, 0xff, 0x25, 0x10, 0x00, 0x02,
	  0x01}},		/* 0x41 */
	{{0xce, 0x9f, 0x9f, 0x92, 0xa9, 0x17, 0x20, 0xf5,
	  0x03, 0x88, 0xff, 0xff, 0x21, 0x10, 0x00, 0x07,
	  0x01}},		/* 0x42 */
	{{0xe6, 0xae, 0xae, 0x8a, 0xbd, 0x90, 0x3d, 0x10,
	  0x1a, 0x8d, 0x19, 0x19, 0x3e, 0x2f, 0x00, 0x03,
	  0x00}},		/* 0x43 */
	{{0xc3, 0x8f, 0x8f, 0x87, 0x9b, 0x0b, 0x82, 0xef,
	  0x60, 0x83, 0x5f, 0x5f, 0x83, 0x10, 0x00, 0x07,
	  0x01}},		/* 0x44 */
	{{0x86, 0x69, 0x69, 0x8A, 0x74, 0x06, 0x8C, 0x15,
	  0x4F, 0x83, 0xEF, 0xEF, 0x8D, 0x30, 0x00, 0x02,
	  0x00}},		/* 0x45 */
	{{0x83, 0x69, 0x69, 0x87, 0x6f, 0x1d, 0x03, 0x3E,
	  0xE5, 0x8d, 0xDF, 0xe4, 0x04, 0x00, 0x00, 0x06,
	  0x00}},		/* 0x46 */
	{{0x86, 0x6A, 0x6A, 0x8A, 0x74, 0x06, 0x8C, 0x15,
	  0x4F, 0x83, 0xEF, 0xEF, 0x8D, 0x30, 0x00, 0x02,
	  0x00}},		/* 0x47 */
	{{0x81, 0x6A, 0x6A, 0x85, 0x70, 0x00, 0x0F, 0x3E,
	  0xEB, 0x8E, 0xDF, 0xDF, 0x10, 0x00, 0x00, 0x02,
	  0x00}},		/* 0x48 */
	{{0xdd, 0xa9, 0xa9, 0x81, 0xb4, 0x97, 0x26, 0xfd,
	  0x01, 0x8d, 0xff, 0x00, 0x27, 0x10, 0x00, 0x03,
	  0x01}},		/* 0x49 */
	{{0xd9, 0x8f, 0x8f, 0x9d, 0xba, 0x0a, 0x8a, 0xff,
	  0x60, 0x8b, 0x5f, 0x5f, 0x8b, 0x10, 0x00, 0x03,
	  0x01}},		/* 0x4a */
	{{0xea, 0xae, 0xae, 0x8e, 0xba, 0x82, 0x40, 0x10,
	  0x1b, 0x87, 0x19, 0x1a, 0x41, 0x0f, 0x00, 0x03,
	  0x00}},		/* 0x4b */
	{{0xd3, 0x9f, 0x9f, 0x97, 0xab, 0x1f, 0xf1, 0xff,
	  0xc0, 0x83, 0xbf, 0xbf, 0xf2, 0x10, 0x00, 0x07,
	  0x01}},		/* 0x4c */
	{{0x75, 0x5f, 0x5f, 0x99, 0x66, 0x90, 0x53, 0xf0,
	  0x41, 0x84, 0x3f, 0x3f, 0x54, 0x00, 0x00, 0x05,
	  0x41}},
	{{0x2d, 0x27, 0x28, 0x90, 0x2c, 0x80, 0x0b, 0x3e,
	  0xe9, 0x8b, 0xdf, 0xe7, 0x04, 0x00, 0x00, 0x00,
	  0x00}},		/* 0x4e */
	{{0xcd, 0x9f, 0x9f, 0x91, 0xab, 0x1c, 0x3a, 0xff,
	  0x20, 0x83, 0x1f, 0x1f, 0x3b, 0x10, 0x00, 0x07,
	  0x21}},		/* 0x4f */
	{{0x15, 0xd1, 0xd1, 0x99, 0xe2, 0x19, 0x3d, 0x10,
	  0x1a, 0x8d, 0x19, 0x19, 0x3e, 0x2f, 0x01, 0x0c,
	  0x20}},		/* 0x50 */
	{{0x0e, 0xef, 0xef, 0x92, 0xfe, 0x03, 0x30, 0xf0,
	  0x1e, 0x83, 0x1b, 0x1c, 0x31, 0x00, 0x01, 0x00,
	  0x61}},		/* 0x51 */
	{{0x85, 0x77, 0x77, 0x89, 0x7d, 0x01, 0x31, 0xf0,
	  0x1e, 0x84, 0x1b, 0x1c, 0x32, 0x00, 0x00, 0x02,
	  0x41}},		/* 0x52 */
	{{0x87, 0x77, 0x77, 0x8b, 0x81, 0x0b, 0x68, 0xf0,
	  0x5a, 0x80, 0x57, 0x57, 0x69, 0x00, 0x00, 0x02,
	  0x01}},		/* 0x53 */
	{{0xcd, 0x8f, 0x8f, 0x91, 0x9b, 0x1b, 0x7a, 0xff,
	  0x64, 0x8c, 0x5f, 0x62, 0x7b, 0x10, 0x00, 0x07,
	  0x41}}		/* 0x54 */
};

static const struct SiS_VCLKData SiSUSB_VCLKData[] = {
	{0x1b, 0xe1, 25},	/* 0x00 */
	{0x4e, 0xe4, 28},	/* 0x01 */
	{0x57, 0xe4, 31},	/* 0x02 */
	{0xc3, 0xc8, 36},	/* 0x03 */
	{0x42, 0xe2, 40},	/* 0x04 */
	{0xfe, 0xcd, 43},	/* 0x05 */
	{0x5d, 0xc4, 44},	/* 0x06 */
	{0x52, 0xe2, 49},	/* 0x07 */
	{0x53, 0xe2, 50},	/* 0x08 */
	{0x74, 0x67, 52},	/* 0x09 */
	{0x6d, 0x66, 56},	/* 0x0a */
	{0x5a, 0x64, 65},	/* 0x0b */
	{0x46, 0x44, 67},	/* 0x0c */
	{0xb1, 0x46, 68},	/* 0x0d */
	{0xd3, 0x4a, 72},	/* 0x0e */
	{0x29, 0x61, 75},	/* 0x0f */
	{0x6e, 0x46, 76},	/* 0x10 */
	{0x2b, 0x61, 78},	/* 0x11 */
	{0x31, 0x42, 79},	/* 0x12 */
	{0xab, 0x44, 83},	/* 0x13 */
	{0x46, 0x25, 84},	/* 0x14 */
	{0x78, 0x29, 86},	/* 0x15 */
	{0x62, 0x44, 94},	/* 0x16 */
	{0x2b, 0x41, 104},	/* 0x17 */
	{0x3a, 0x23, 105},	/* 0x18 */
	{0x70, 0x44, 108},	/* 0x19 */
	{0x3c, 0x23, 109},	/* 0x1a */
	{0x5e, 0x43, 113},	/* 0x1b */
	{0xbc, 0x44, 116},	/* 0x1c */
	{0xe0, 0x46, 132},	/* 0x1d */
	{0x54, 0x42, 135},	/* 0x1e */
	{0xea, 0x2a, 139},	/* 0x1f */
	{0x41, 0x22, 157},	/* 0x20 */
	{0x70, 0x24, 162},	/* 0x21 */
	{0x30, 0x21, 175},	/* 0x22 */
	{0x4e, 0x22, 189},	/* 0x23 */
	{0xde, 0x26, 194},	/* 0x24 */
	{0x62, 0x06, 202},	/* 0x25 */
	{0x3f, 0x03, 229},	/* 0x26 */
	{0xb8, 0x06, 234},	/* 0x27 */
	{0x34, 0x02, 253},	/* 0x28 */
	{0x58, 0x04, 255},	/* 0x29 */
	{0x24, 0x01, 265},	/* 0x2a */
	{0x9b, 0x02, 267},	/* 0x2b */
	{0x70, 0x05, 270},	/* 0x2c */
	{0x25, 0x01, 272},	/* 0x2d */
	{0x9c, 0x02, 277},	/* 0x2e */
	{0x27, 0x01, 286},	/* 0x2f */
	{0x3c, 0x02, 291},	/* 0x30 */
	{0xef, 0x0a, 292},	/* 0x31 */
	{0xf6, 0x0a, 310},	/* 0x32 */
	{0x95, 0x01, 315},	/* 0x33 */
	{0xf0, 0x09, 324},	/* 0x34 */
	{0xfe, 0x0a, 331},	/* 0x35 */
	{0xf3, 0x09, 332},	/* 0x36 */
	{0xea, 0x08, 340},	/* 0x37 */
	{0xe8, 0x07, 376},	/* 0x38 */
	{0xde, 0x06, 389},	/* 0x39 */
	{0x52, 0x2a, 54},	/* 0x3a 301 TV */
	{0x52, 0x6a, 27},	/* 0x3b 301 TV */
	{0x62, 0x24, 70},	/* 0x3c 301 TV */
	{0x62, 0x64, 70},	/* 0x3d 301 TV */
	{0xa8, 0x4c, 30},	/* 0x3e 301 TV */
	{0x20, 0x26, 33},	/* 0x3f 301 TV */
	{0x31, 0xc2, 39},	/* 0x40 */
	{0x60, 0x36, 30},	/* 0x41 Chrontel */
	{0x40, 0x4a, 28},	/* 0x42 Chrontel */
	{0x9f, 0x46, 44},	/* 0x43 Chrontel */
	{0x97, 0x2c, 26},	/* 0x44 */
	{0x44, 0xe4, 25},	/* 0x45 Chrontel */
	{0x7e, 0x32, 47},	/* 0x46 Chrontel */
	{0x8a, 0x24, 31},	/* 0x47 Chrontel */
	{0x97, 0x2c, 26},	/* 0x48 Chrontel */
	{0xce, 0x3c, 39},	/* 0x49 */
	{0x52, 0x4a, 36},	/* 0x4a Chrontel */
	{0x34, 0x61, 95},	/* 0x4b */
	{0x78, 0x27, 108},	/* 0x4c - was 102 */
	{0x66, 0x43, 123},	/* 0x4d Modes 0x26-0x28 (1400x1050) */
	{0x41, 0x4e, 21},	/* 0x4e */
	{0xa1, 0x4a, 29},	/* 0x4f Chrontel */
	{0x19, 0x42, 42},	/* 0x50 */
	{0x54, 0x46, 58},	/* 0x51 Chrontel */
	{0x25, 0x42, 61},	/* 0x52 */
	{0x44, 0x44, 66},	/* 0x53 Chrontel */
	{0x3a, 0x62, 70},	/* 0x54 Chrontel */
	{0x62, 0xc6, 34},	/* 0x55 848x480-60 */
	{0x6a, 0xc6, 37},	/* 0x56 848x480-75 - TEMP */
	{0xbf, 0xc8, 35},	/* 0x57 856x480-38i,60 */
	{0x30, 0x23, 88},	/* 0x58 1360x768-62 (is 60Hz!) */
	{0x52, 0x07, 149},	/* 0x59 1280x960-85 */
	{0x56, 0x07, 156},	/* 0x5a 1400x1050-75 */
	{0x70, 0x29, 81},	/* 0x5b 1280x768 LCD */
	{0x45, 0x25, 83},	/* 0x5c 1280x800  */
	{0x70, 0x0a, 147},	/* 0x5d 1680x1050 */
	{0x70, 0x24, 162},	/* 0x5e 1600x1200 */
	{0x5a, 0x64, 65},	/* 0x5f 1280x720 - temp */
	{0x63, 0x46, 68},	/* 0x60 1280x768_2 */
	{0x31, 0x42, 79},	/* 0x61 1280x768_3 - temp */
	{0, 0, 0},		/* 0x62 - custom (will be filled out at run-time) */
	{0x5a, 0x64, 65},	/* 0x63 1280x720 (LCD LVDS) */
	{0x70, 0x28, 90},	/* 0x64 1152x864@60 */
	{0x41, 0xc4, 32},	/* 0x65 848x480@60 */
	{0x5c, 0xc6, 32},	/* 0x66 856x480@60 */
	{0x76, 0xe7, 27},	/* 0x67 720x480@60 */
	{0x5f, 0xc6, 33},	/* 0x68 720/768x576@60 */
	{0x52, 0x27, 75},	/* 0x69 1920x1080i 60Hz interlaced */
	{0x7c, 0x6b, 38},	/* 0x6a 960x540@60 */
	{0xe3, 0x56, 41},	/* 0x6b 960x600@60 */
	{0x45, 0x25, 83},	/* 0x6c 1280x800 */
	{0x70, 0x28, 90},	/* 0x6d 1152x864@60 */
	{0x15, 0xe1, 20},	/* 0x6e 640x400@60 (fake, not actually used) */
	{0x5f, 0xc6, 33},	/* 0x6f 720x576@60 */
	{0x37, 0x5a, 10},	/* 0x70 320x200@60 (fake, not actually used) */
	{0x2b, 0xc2, 35}	/* 0x71 768@576@60 */
};

int SiSUSBSetMode(struct SiS_Private *SiS_Pr, unsigned short ModeNo);
int SiSUSBSetVESAMode(struct SiS_Private *SiS_Pr, unsigned short VModeNo);

extern int sisusb_setreg(struct sisusb_usb_data *sisusb, u32 port, u8 data);
extern int sisusb_getreg(struct sisusb_usb_data *sisusb, u32 port, u8 * data);
extern int sisusb_setidxreg(struct sisusb_usb_data *sisusb, u32 port,
			    u8 index, u8 data);
extern int sisusb_getidxreg(struct sisusb_usb_data *sisusb, u32 port,
			    u8 index, u8 * data);
extern int sisusb_setidxregandor(struct sisusb_usb_data *sisusb, u32 port,
				 u8 idx, u8 myand, u8 myor);
extern int sisusb_setidxregor(struct sisusb_usb_data *sisusb, u32 port,
			      u8 index, u8 myor);
extern int sisusb_setidxregand(struct sisusb_usb_data *sisusb, u32 port,
			       u8 idx, u8 myand);

void sisusb_delete(struct kref *kref);
int sisusb_writeb(struct sisusb_usb_data *sisusb, u32 adr, u8 data);
int sisusb_readb(struct sisusb_usb_data *sisusb, u32 adr, u8 * data);
int sisusb_copy_memory(struct sisusb_usb_data *sisusb, char *src,
		       u32 dest, int length);
int sisusb_reset_text_mode(struct sisusb_usb_data *sisusb, int init);
int sisusbcon_do_font_op(struct sisusb_usb_data *sisusb, int set, int slot,
			 u8 * arg, int cmapsz, int ch512, int dorecalc,
			 struct vc_data *c, int fh, int uplock);
void sisusb_set_cursor(struct sisusb_usb_data *sisusb, unsigned int location);
int sisusb_console_init(struct sisusb_usb_data *sisusb, int first, int last);
void sisusb_console_exit(struct sisusb_usb_data *sisusb);
void sisusb_init_concode(void);

#endif
