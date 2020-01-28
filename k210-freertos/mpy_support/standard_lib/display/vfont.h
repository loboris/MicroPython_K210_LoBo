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


// A Vector font libarary beased upon the Hershey font set
// 
// Michael McElligott
// okio@users.sourceforge.net
// https://youtu.be/T0WgGcm7ujM

//  Copyright (c) 2005-2017  Michael McElligott
// 
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU LIBRARY GENERAL PUBLIC LICENSE
//  as published by the Free Software Foundation; either version 2
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU LIBRARY GENERAL PUBLIC LICENSE for more details.
//
//	You should have received a copy of the GNU Library General Public
//	License along with this library; if not, write to the Free
//	Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.



#ifndef _VFONT_H_
#define _VFONT_H_

# define HERSHEY_MAGIC  (uint32_t)0x53524548

typedef struct {
    uint16_t code;          // glyph character code
    uint16_t count;         // length of glyph data (bytes)
    uint8_t  left;          // left glyph margin relative to character 'R'
    uint8_t  right;         // right glyph margin relative to character 'R'
    uint8_t  top;           // top glyph margin relative to character 'R'
    uint8_t  bottom;        // bottom glyph margin relative to character 'R'
} __attribute__((packed)) char_glyph_t;

typedef struct {
    uint32_t magic;         // Hershey font ID
    uint16_t glyphCount;    // number of glyphs
    uint8_t  left;          // maximal left glyph margin relative to character 'R'
    uint8_t  right;         // maximal right glyph margin relative to character 'R'
    uint8_t  top;           // maximal top glyph margin relative to character 'R'
    uint8_t  bottom;        // maximal bottom glyph margin relative to character 'R'
    uint8_t  *glyphs;       // pointer to the glyphs data
} __attribute__((packed)) hfont_t;


enum _brush{
	BRUSH_POINT,			// .
	BRUSH_CIRCLE,			// O
	BRUSH_CIRCLE_FILLED,
	BRUSH_SQUARE,		
	BRUSH_SQUARE_FILLED,	
	BRUSH_TRIANGLE,			// north facing
	BRUSH_TRIANGLE_FILLED,	
	BRUSH_STROKE_1,			// slope up		/	
	BRUSH_STROKE_2,			// slope down	\ symbol
	BRUSH_STROKE_3,			// horizontal	-	
	BRUSH_STROKE_4,			// vertical		|	
	BRUSH_STROKE_5,			// 
	BRUSH_STROKE_6,			// 
	BRUSH_STAR,				// *
	BRUSH_X,				// X
	BRUSH_CARET,			// ^
	BRUSH_PLUS,				// +
	BRUSH_BITMAP,

	BRUSH_TOTAL,
	BRUSH_DISK = BRUSH_CIRCLE_FILLED		
};


#define RENDEROP_NONE			0x00
#define RENDEROP_SHEAR_X		0x01
#define RENDEROP_SHEAR_Y		0x02
#define RENDEROP_SHEAR_XYX		0x04
#define RENDEROP_ROTATE_STRING	0x08
#define RENDEROP_ROTATE_GLYPHS	0x10

#ifndef DEG2RAD
#define DEG2RAD(a)				((a)*((M_PI / 180.0f)))
#endif

#define CALC_PITCH_1(w)			(((w)>>3)+(((w)&0x07)!=0))	// 1bit packed, calculate number of storage bytes per row given width (of glyph)
#define CALC_PITCH_16(w)		((w)*sizeof(uint16_t))		// 16bit, 8 bits per byte

typedef struct{
	float x1;
	float y1;
	float x2;
	float y2;
} box_t;


typedef struct {
	float angle;
	float cos;
	float sin;
} rotate_t;

typedef struct {
	uint8_t *pixels;
	uint8_t width;
	uint8_t height;
	uint8_t stubUnused;
} image_t;
		
typedef struct {
	hfont_t *font;
	int x;				// initial rendering position
	int y;
	
	struct {			// current rendering position
		float x;
		float y;
	} pos;

	float xypad;			// horizontal/vertical padding. can be minus. eg; -0.5f

	struct {
		float glyph;		// vector scale/glyph size (2.0 = float size glyph)
		float horizontal;
		float vertical;
	} scale;

	struct {
		rotate_t string;
		rotate_t glyph;
	} rotate;
	
	struct {
		float angleX;
		float angleY;
		float cos;
		float sin;
		float tan;
	} shear;

	struct {
		float size;
		float step;
		float advanceMult;
		uint16_t type;
		uint16_t color;

		image_t image;
	} brush;
	
	uint16_t renderOp;
} vfont_t;

extern const uint8_t snowflake16x16[16][2];
extern const uint8_t smiley16x16[16][2];

void vfontInitialise(vfont_t *ctx);
void setFont(vfont_t *ctx, hfont_t *font);
hfont_t *getFont(vfont_t *ctx);

int setBrush(vfont_t *ctx, int brush);
float setBrushSize(vfont_t *ctx, float size);
void setBrushStep(vfont_t *ctx, float step);
float getBrushStep(vfont_t *ctx);
uint16_t setBrushColor(vfont_t *ctx, uint16_t color);
uint16_t getBrushColor(vfont_t *ctx);

void setGlyphScale(vfont_t *ctx, float scale);
float getGlyphScale(vfont_t *ctx);
void setGlyphPadding(vfont_t *ctx, float pad);
float getGlyphPadding(vfont_t *ctx);

void setRenderFilter(vfont_t *ctx, uint32_t mask);
uint32_t getRenderFilter(vfont_t *ctx);

void setAspect(vfont_t *ctx, float hori, float vert);
void setRotationAngle(vfont_t *ctx, float rotGlyph, float rotString);
void setShearAngle(vfont_t *ctx, float shrX, float shrY);

float getCharMetrics(vfont_t *ctx, hfont_t *font, uint16_t ch, box_t *box);
void getGlyphSize(vfont_t *ctx, uint16_t c, int *w, int *h);
void getMaxGlyphSize(vfont_t *ctx, int *w, int *h);
float getMaxGlyphHeight(vfont_t *ctx);
void getStringMetrics(vfont_t *ctx, unsigned char *text, box_t *box);

void vfontDrawString(vfont_t *ctx, unsigned char *text, int x, int y);

#endif


