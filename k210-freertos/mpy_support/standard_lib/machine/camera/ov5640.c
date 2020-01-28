/*
 * This file is part of the MicroPython K210 project, https://github.com/loboris/MicroPython_K210_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 LoBo (https://github.com/loboris)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
 * Based on OV5640 driver from OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * OV5640 driver.
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <devices.h>
#include "syslog.h"
#include "dvp_camera.h"
#include "ov5640_regs.h"
#include "modmachine.h"

#define OV5640_NUM_ALLOWED_SIZES    8


static const char *TAG = "[OV5640]";

static const uint8_t allowed_sizes[OV5640_NUM_ALLOWED_SIZES] = {
    FRAMESIZE_QVGA,     // 320x240
    FRAMESIZE_HVGA,     // 480x320
    FRAMESIZE_VGA,      // 640x480
    FRAMESIZE_SVGA,     // 800x600
    FRAMESIZE_720P,     // 1280x720
    FRAMESIZE_1080P,    // 1920x1080
    FRAMESIZE_960P,     // 1280x960
    FRAMESIZE_5MPP,     // 2592x1944 *
};

static sensor_t *ov5640_sensor = NULL;
static handle_t file_ov5640 = 0;
static handle_t file_sccb = 0;
#define REGLENGTH 16

static const uint8_t OV5640_EFFECTS_TBL[9][4]=
{
    //5580 5583  5584  5003
    {0x06, 0x40, 0x10, 0x08}, // Normal
    {0x1E, 0xA0, 0x40, 0x08}, // Blueish (cool light)
    {0x1E, 0x80, 0xC0, 0x08}, // Redish (warm)
    {0x1E, 0x80, 0x80, 0x08}, // Black and white
    {0x1E, 0x40, 0xA0, 0x08}, // Sepia
    {0x40, 0x40, 0x10, 0x08}, // Negative
    {0x1E, 0x60, 0x60, 0x08}, // Greenish
    {0x1E, 0xf0, 0xf0, 0x08}, // Overexposure
    {0x06, 0x40, 0x10, 0x09}, // Solarize
};

static const uint8_t OV5640_SATURATION_TBL[7][6]=
{
    {0x0C, 0x30, 0x3D, 0x3E, 0x3D, 0x01}, //-3
    {0x10, 0x3D, 0x4D, 0x4E, 0x4D, 0x01}, //-2
    {0x15, 0x52, 0x66, 0x68, 0x66, 0x02}, //-1
    {0x1A, 0x66, 0x80, 0x82, 0x80, 0x02}, //+0
    {0x1F, 0x7A, 0x9A, 0x9C, 0x9A, 0x02}, //+1
    {0x24, 0x8F, 0xB3, 0xB6, 0xB3, 0x03}, //+2
    {0x2B, 0xAB, 0xD6, 0xDA, 0xD6, 0x04}, //+3
};

static const uint8_t OV5640_LIGHTMODE_TBL[5][7]=
{
    {0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x00}, //Auto
    {0x06, 0x1C, 0x04, 0x00, 0x04, 0xF3, 0x01}, //Sunny
    {0x05, 0x48, 0x04, 0x00, 0x07, 0xCF, 0x01}, //Office
    {0x06, 0x48, 0x04, 0x00, 0x04, 0xD3, 0x01}, //Cloudy
    {0x04, 0x10, 0x04, 0x00, 0x08, 0x40, 0x01}, //Home
};

const static uint8_t OV5640_EXPOSURE_TBL[7][6]=
{
    {0x10,0x08,0x10,0x08,0x20,0x10}, //-3
    {0x20,0x18,0x41,0x20,0x18,0x10}, //-2
    {0x30,0x28,0x61,0x30,0x28,0x10}, //-1
    {0x38,0x30,0x61,0x38,0x30,0x10}, //0
    {0x40,0x38,0x71,0x40,0x38,0x10}, //+1
    {0x50,0x48,0x90,0x50,0x48,0x20}, //+2
    {0x60,0x58,0xa0,0x60,0x58,0x20}, //+3
};


//------------------------------------------------------
static uint8_t OV5640_WR_Reg(uint16_t reg, uint8_t data)
{
    sccb_dev_write_byte(file_ov5640, reg, data);
    return 0;
}

//---------------------------------------------------------
static uint8_t OV5640_WR_Reg16(uint16_t reg, uint16_t data)
{
    sccb_dev_write_byte(file_ov5640, reg, (uint8_t)(data >> 8));
    sccb_dev_write_byte(file_ov5640, reg+1, (uint8_t)(data & 0xff));
    return 0;
}

//-----------------------------------------------------------
static uint8_t OV5640_RD_Reg(uint16_t reg, uint8_t *reg_data)
{
    *reg_data = sccb_dev_read_byte(file_ov5640, reg);
    return 0;
}

//-------------------------------------------------
static void wrSensorRegs(const uint16_t (*regs)[2])
{
    for (int i = 0; regs[i][0]; i++) {
        sccb_dev_write_byte(file_ov5640, regs[i][0], regs[i][1]);
    }
}

//--------------------------------
static int reset(sensor_t *sensor)
{
    // Reset all registers
    OV5640_WR_Reg(0x3008, 0x42);
    // Delay 10 ms
    mp_hal_delay_ms(10);

    // === Write default registers ===
    wrSensorRegs(default_regs);

    OV5640_WR_Reg(0x3008, 0x02);
    mp_hal_delay_ms(30);

    #ifdef OV5640_USE_AUTO_FOCUS
    // === Load and start auto focus firmware ====
    uint8_t reg = 0;
    int address = 0x8000;
    int tmo;

    OV5640_WR_Reg(0x3000,0x20); // reset MCU
    for (i=0; i< sizeof(sensor_af_fw_regs); i++) {
        OV5640_WR_Reg(address, sensor_af_fw_regs[i]);
        address++;
    }
    OV5640_WR_Reg(0x3022, 0x00);
    OV5640_WR_Reg(0x3023, 0x00);
    OV5640_WR_Reg(0x3024, 0x00);
    OV5640_WR_Reg(0x3025, 0x00);
    OV5640_WR_Reg(0x3026, 0x00);
    OV5640_WR_Reg(0x3027, 0x00);
    OV5640_WR_Reg(0x3028, 0x00);
    OV5640_WR_Reg(0x3029, 0x7f);
    OV5640_WR_Reg(0x3000, 0x00); // start firimware

    // Delay
    mp_hal_delay_ms(10);
    // Wait for AF firmware start
    tmo = 20;
    while (tmo) {
        OV5640_RD_Reg(0x3029, &reg);
        if (reg == 0x70) break;
        mp_hal_delay_ms(3);
        tmo--;
    }
    if (tmo == 0) {
        LOGW(TAG, "Auto focus firmware not started (%02X)", reg);
    }

    // Enable auto focus
    OV5640_WR_Reg(0x3023, 0x01);
    OV5640_WR_Reg(0x3022, 0x04); // Continuous Focus

    mp_hal_delay_ms(30);
    #endif

    return 0;
}

//--------------------------
static int sleep(int enable)
{
    uint8_t reg;
    if (enable) reg = 0x42;
    else reg = 0x02;

    // Write back register
    return OV5640_WR_Reg(0x3008, reg);
}

//------------------------------------
static int read_reg(uint16_t reg_addr)
{
    uint8_t reg_data;
    if (OV5640_RD_Reg(reg_addr, &reg_data) != 0) {
        return -1;
    }
    return reg_data;
}

//--------------------------------------------------------
static int write_reg(uint16_t reg_addr, uint16_t reg_data)
{
    return OV5640_WR_Reg(reg_addr, reg_data);
}

//---------------------------------------------------------
static int mod_reg(uint16_t reg, uint8_t mask, uint8_t val)
{
    uint8_t readval;
    int ret;

    ret = OV5640_RD_Reg(reg, &readval);
    if (ret) return ret;

    readval &= ~mask;
    val &= mask;
    val |= readval;

    return OV5640_WR_Reg(reg, val);
}

// set the output size
// -----------------------------------------------------------------------------------------
static int OV5640_OutSize_Set(uint16_t offx, uint16_t offy, uint16_t width, uint16_t height)
{
    OV5640_WR_Reg(0X3212, 0X03);        // group write register

    OV5640_WR_Reg(0x3808, width>>8);
    OV5640_WR_Reg(0x3809, width&0xff);
    OV5640_WR_Reg(0x380a, height>>8);
    OV5640_WR_Reg(0x380b, height&0xff);

    OV5640_WR_Reg(0x3810, offx>>8);
    OV5640_WR_Reg(0x3811, offx&0xff);
    OV5640_WR_Reg(0x3812, offy>>8);
    OV5640_WR_Reg(0x3813, offy&0xff);

    OV5640_WR_Reg(0X3212, 0X13);        // Group hold end
    OV5640_WR_Reg(0X3212, 0Xa3);        // Group launch enable & Group launch

    return 0;
}

//-------------------------------------------
int ov5640_check_framesize(uint8_t framesize)
{
    int i = OV5640_NUM_ALLOWED_SIZES;
    for (i=0; i<OV5640_NUM_ALLOWED_SIZES; i++) {
        if (allowed_sizes[i] == framesize) break;
    }
    if (i >= OV5640_NUM_ALLOWED_SIZES) return -1;
    return 0;
}

//-------------------------------------------
static int set_framesize(uint8_t framesize)
{
    if (ov5640_check_framesize(framesize) < 0) {
        LOGW(TAG, "Frame size %d not allowed", framesize);
        return -1;
    }
    ov5640_sensor->framesize = framesize;
    return OV5640_OutSize_Set(0, 0, dvp_cam_resolution[framesize][0], dvp_cam_resolution[framesize][1]);
}

//-------------------------------------------
static void OV5640_Exposure(uint8_t exposure)
{
        OV5640_WR_Reg(0x3212,0x03); //start group 3
        OV5640_WR_Reg(0x3a0f,OV5640_EXPOSURE_TBL[exposure][0]);
        OV5640_WR_Reg(0x3a10,OV5640_EXPOSURE_TBL[exposure][1]);
        OV5640_WR_Reg(0x3a1b,OV5640_EXPOSURE_TBL[exposure][2]);
        OV5640_WR_Reg(0x3a1e,OV5640_EXPOSURE_TBL[exposure][3]);
        OV5640_WR_Reg(0x3a11,OV5640_EXPOSURE_TBL[exposure][4]);
        OV5640_WR_Reg(0x3a1f,OV5640_EXPOSURE_TBL[exposure][5]);
        OV5640_WR_Reg(0x3212,0x13); //end group 3
        OV5640_WR_Reg(0x3212,0xa3); //launch group 3
}

//---------------------------------------------
static int set_pixformat(pixformat_t pixformat)
{
    uint16_t width = dvp_cam_resolution[ov5640_sensor->framesize][0];
    uint16_t height = dvp_cam_resolution[ov5640_sensor->framesize][1];
    uint8_t nm = 0;

    switch (pixformat) {
        case PIXFORMAT_RGB565:
            wrSensorRegs(OV5640_vga_preview_regs);

            OV5640_RD_Reg(0x3a00, &nm);
            if (width > 800) nm |= 0x04;
            else nm &= 0xfb;

            OV5640_WR_Reg(0X3212, 0X03); // group write register
            //OV5640_WR_Reg16(0x3a00, nm);
            // set resolution
            OV5640_WR_Reg16(0x3808, width);
            OV5640_WR_Reg16(0x380a, height);

            OV5640_WR_Reg(0x4300, 0x61); // RGB565
            OV5640_WR_Reg(0x501f, 0x01); // ISP RGB
            OV5640_WR_Reg(0x3503, 0x00); // AE on
            OV5640_WR_Reg(0x3821, 0x00); // jpeg disable
            OV5640_WR_Reg(0X3212, 0X13); // Group hold end
            OV5640_WR_Reg(0X3212, 0Xa3); // Group launch enable & Group launch
            break;
        case PIXFORMAT_JPEG:
            OV5640_RD_Reg(0x3a00, &nm);
            if (width > 800) nm |= 0x04;
            else nm &= 0xfb;

            OV5640_WR_Reg(0X3212, 0X03); // group write register
            //OV5640_WR_Reg16(0x3a00, nm);
            // Set JPEG fixed width and height
            OV5640_WR_Reg16(0x4602, width);
            OV5640_WR_Reg16(0x4604, height/2);
            // set resolution
            OV5640_WR_Reg16(0x3808, width);
            OV5640_WR_Reg16(0x380a, height);
            // subsample increment
            OV5640_WR_Reg16(0x3814, 0x1111);
            OV5640_WR_Reg(0X3212, 0X13); // Group hold end
            OV5640_WR_Reg(0X3212, 0Xa3); // Group launch enable & Group launch

            wrSensorRegs(OV5640_JPEG_CAPTURE_regs);
            break;
        default:
            return -1;
    }
    return 0;
}

//----------------------------
static int set_quality(int qs)
{
    if ((qs < 2) || (qs > 60)) return -1;

    OV5640_WR_Reg(0x4407, qs & 0xbf);
    return 0;
}

//Contrast:
//     contrast:  -3 ~ +3
//--------------------------------
static int set_contrast(int level)
{
    level += 3;
    uint8_t reg0val=0X00;
    uint8_t reg1val=0X20;
    switch(level)
    {
        case 0://-3
            reg1val=reg0val=0X14;
            break;
        case 1://-2
            reg1val=reg0val=0X18;
            break;
        case 2://-1
            reg1val=reg0val=0X1C;
            break;
        case 4://1
            reg0val=0X10;
            reg1val=0X24;
            break;
        case 5://2
            reg0val=0X18;
            reg1val=0X28;
            break;
        case 6://3
            reg0val=0X1C;
            reg1val=0X2C;
            break;
        default:
            return -1;
    }
    OV5640_WR_Reg(0x3212,0x03); //start group 3
    OV5640_WR_Reg(0x5585,reg0val);
    OV5640_WR_Reg(0x5586,reg1val);
    OV5640_WR_Reg(0x3212,0x13); //end group 3
    OV5640_WR_Reg(0x3212,0xa3); //launch group 3
    return 0;
}

//Brightness
//     bright:  -4 ~ +4
//----------------------------------
static int set_brightness(int level)
{
    level += 4;
    if ((level < 0) || (level > 8)) return -1;
    uint8_t brtval;
    if (level<4) brtval = 4-level;
    else brtval = level - 4;

    OV5640_WR_Reg(0x3212, 0x03); //start group 3
    OV5640_WR_Reg(0x5587, brtval<<4);
    if (level<4) OV5640_WR_Reg(0x5588, 0x09);
    else OV5640_WR_Reg(0x5588, 0x01);
    OV5640_WR_Reg(0x3212, 0x13); //end group 3
    OV5640_WR_Reg(0x3212, 0xa3); //launch group 3

    return 0;
}

// Color Saturation:
//   sat:  -3 ~ +3
//----------------------------------
static int set_saturation(int level)
{
    level += 3;
    if ((level < 0) || (level > 6)) return -1;
    uint8_t i;
    OV5640_WR_Reg(0x3212, 0x03); //start group 3
    OV5640_WR_Reg(0x5381, 0x1c);
    OV5640_WR_Reg(0x5382, 0x5a);
    OV5640_WR_Reg(0x5383, 0x06);
    for(i=0; i<6; i++) {
        OV5640_WR_Reg(0x5384+i,OV5640_SATURATION_TBL[level][i]);
    }
    OV5640_WR_Reg(0x538b, 0x98);
    OV5640_WR_Reg(0x538a, 0x01);
    OV5640_WR_Reg(0x3212, 0x13); //end group 3
    OV5640_WR_Reg(0x3212, 0xa3); //launch group 3
    return 0;
}

//----------------------------------------
static int set_special_effect(uint8_t sde)
{
    if (sde > 8) return -1;
    OV5640_WR_Reg(0x3212, 0x03); //start group 3
    OV5640_WR_Reg(0x5580, OV5640_EFFECTS_TBL[sde][0]);
    OV5640_WR_Reg(0x5583, OV5640_EFFECTS_TBL[sde][1]);// sat U
    OV5640_WR_Reg(0x5584, OV5640_EFFECTS_TBL[sde][2]);// sat V
    OV5640_WR_Reg(0x5003, OV5640_EFFECTS_TBL[sde][3]);
    OV5640_WR_Reg(0x3212, 0x13); //end group 3
    OV5640_WR_Reg(0x3212, 0xa3); //launch group 3
    return 0;
}

//---------------------------------
static int set_colorbar(int enable)
{
    /*
        PRE ISP TEST SETTING 1 register
        -------------------------------
        Bit[7]: Pre ISP test enable
          0: Test disable
          1: Color bar enable
        Bit[6]: Rolling
        Bit[5]: Transparent
        Bit[4]: Square BW
        Bit[3:2]: Pre ISP bar style
          00: Standard 8 color bar
          01: Gradual change at vertical mode 1
          10: Gradual change at horizontal
          11: Gradual change at vertical mode 2
        Bit[1:0]: Test select
          00: Color bar
          01: Random data
          10: Square data
          11: Black image
     */
    OV5640_WR_Reg(0x503D, enable & 0xFF);
    return 0;
}

//--------------------------------
static int set_hmirror(int enable)
{
    uint8_t reg;
    int ret = OV5640_RD_Reg(0x3821, &reg);
    if (enable){
        ret |= OV5640_WR_Reg(0x3821, reg&0x06);
    } else {
        ret |= OV5640_WR_Reg(0x3821, reg|0xF9);
    }
    return ret;
}

//------------------------------
static int set_vflip(int enable)
{
    uint8_t reg;
    int ret = OV5640_RD_Reg(0x3820, &reg);
    if (enable){
        ret |= OV5640_WR_Reg(0x3820, reg&0xF9);
    } else {
        ret |= OV5640_WR_Reg(0x3820, reg|0x06);
    }
    return ret;
}

// light mode:
//      0: auto
//      1: sunny
//      2: office
//      3: cloudy
//      4: home
//-------------------------------------
static int set_light_mode(uint8_t mode)
{
    if (mode > 4) return -1;
    uint8_t i;
    OV5640_WR_Reg(0x3212, 0x03); //start group 3
    for (i=0; i<7; i++) {
        OV5640_WR_Reg(0x3400+i, OV5640_LIGHTMODE_TBL[mode][i]);
    }
    OV5640_WR_Reg(0x3212,0x13); //end group 3
    OV5640_WR_Reg(0x3212,0xa3); //launch group 3
    return 0;
}

//-----------------------------------
static int set_night_mode(int enable)
{
    uint8_t reg;
    OV5640_RD_Reg(0x3a00, &reg);
    if (enable) reg |= 0x04;
    else reg &= 0xfb;
    OV5640_WR_Reg(0x3a00, reg);
    return 0;
}


//---------------------------------------------------------
static int read_id(uint16_t *manuf_id, uint16_t *device_id)
{
    uint8_t reg;
    OV5640_RD_Reg(OV5640_REG_CHIP_ID, &reg);
    *device_id = (reg << 8);
    OV5640_RD_Reg(OV5640_REG_CHIP_ID+1, &reg);
    *device_id |= reg;
    *manuf_id = 0;
    return 0;
}

//-----------------
static int deinit()
{
    if (ov5640_sensor) {
        LOGD(TAG, "Sensor deinit");
        if (file_ov5640) io_close(file_ov5640);
        if (file_sccb) io_close(file_sccb);
        file_sccb = 0;
        file_ov5640 = 0;
        ov5640_sensor->scpp_handle = 0;
        ov5640_sensor->scpp_dev_handle = 0;
        ov5640_sensor = NULL;
    }
    return 0;
}

//----------------------------------------
static int set_exposure(int exposure)
{
    int ret = 0;
    uint8_t reg;
    if (exposure == -1) {
        // get exposure
        OV5640_RD_Reg(OV5640_REG_AEC_PK_EXPOSURE_HI, &reg);
        ret = ((int)reg & 0x0f) << 12;
        OV5640_RD_Reg(OV5640_REG_AEC_PK_EXPOSURE_MED, &reg);
        ret += (int)reg << 4;
        OV5640_RD_Reg(OV5640_REG_AEC_PK_EXPOSURE_LO, &reg);
        ret += (int)reg >> 4;
    }
    else if (exposure == -2) {
        // disable auto exposure and gain
        OV5640_WR_Reg(OV5640_REG_AEC_PK_MANUAL, 0x03);
    }
    else if (exposure > 0) {
        // disable auto exposure and gain
        OV5640_WR_Reg(OV5640_REG_AEC_PK_MANUAL, 0x03);
        // set manual exposure
        OV5640_WR_Reg(0x3212, 0x03); //start group 3
        OV5640_WR_Reg(OV5640_REG_AEC_PK_EXPOSURE_HI, (uint8_t)((exposure >> 12) & 0x0f));
        OV5640_WR_Reg(OV5640_REG_AEC_PK_EXPOSURE_MED, (uint8_t)((exposure >> 4) & 0xff));
        OV5640_WR_Reg(OV5640_REG_AEC_PK_EXPOSURE_LO, (uint8_t)(exposure << 4));
        OV5640_WR_Reg(0x3212,0x13); //end group 3
        OV5640_WR_Reg(0x3212,0xa3); //launch group 3
    }
    else {
        // enable auto exposure and gain
        OV5640_WR_Reg(OV5640_REG_AEC_PK_MANUAL, 0x00);
    }

    return ret;
}


//===============================
int ov5640_init(sensor_t *sensor)
{
    // Initialize sensor structure.
    sensor->is_init             = false;
    sensor->slv_addr            = OV5640_SLV_ADDR;
    sensor->reset               = reset;
    sensor->sleep               = sleep;
    sensor->read_reg            = read_reg;
    sensor->read_id             = read_id;
    sensor->write_reg           = write_reg;
    sensor->set_pixformat       = set_pixformat;
    sensor->set_framesize       = set_framesize;
    sensor->check_framesize     = ov5640_check_framesize;
    sensor->set_contrast        = set_contrast;
    sensor->set_brightness      = set_brightness;
    sensor->set_saturation      = set_saturation;
    sensor->set_quality         = set_quality;
    sensor->set_colorbar        = set_colorbar;
    sensor->set_hmirror         = set_hmirror;
    sensor->set_vflip           = set_vflip;
    sensor->set_exposure        = set_exposure;
    sensor->set_special_effect  = set_special_effect;
    sensor->set_light_mode      = set_light_mode;
    sensor->set_night_mode      = set_night_mode;
    sensor->deinit              = deinit;

    if (ov5640_check_framesize(sensor->framesize) < 0) sensor->framesize = FRAMESIZE_QVGA;

    // Open SCCB device
    deinit();
    ov5640_sensor = sensor;

    file_sccb = io_open("/dev/sccb0");
    if (file_sccb == 0) {
        LOGE(TAG, "Error opening SCCB");
        return -1;
    }
    file_ov5640 = sccb_get_device(file_sccb, OV5640_SLV_ADDR, REGLENGTH);
    if (file_ov5640 == 0) {
        io_close(file_sccb);
        file_sccb = 0;
        LOGE(TAG, "Error opening SCCB device");
        return -1;
    }
    sensor->scpp_handle = file_sccb;
    sensor->scpp_dev_handle = file_ov5640;

    mp_hal_delay_ms(5);
    read_id(&sensor->manuf_id, &sensor->chip_id);
    if (((sensor->chip_id & 0xFFF0) != 0x5640)) {
        LOGD(TAG, "Wrong camera ID (%04X)", sensor->chip_id);
        io_close(file_ov5640);
        io_close(file_sccb);
        file_sccb = 0;
        file_ov5640 = 0;
        sensor->scpp_handle = 0;
        sensor->scpp_dev_handle = 0;
        return -1;
    }

    sensor->is_init = true;

    return 0;
}
