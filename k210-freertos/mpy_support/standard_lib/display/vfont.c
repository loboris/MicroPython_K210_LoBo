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

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include <math.h>
#include "syslog.h"
#include "tft.h"
#include "vfont.h"

//-----------------------------------------------------------
static float distance(float x1, float y1, float x2, float y2)
{
    return sqrtf((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

//check if point2 is between point1 and point3 (the 3 points should be on the same line)
//------------------------------------------------------------------------------
static int isBetween(float x1, float y1, float x2, float y2, float x3, float y3)
{
    return ((int)(x1 - x2) * (int)(x3 - x2) <= 0) && ((int)(y1 - y2) * (int)(y3 - y2) <= 0);
}

//------------------------------------------------------------------------
static void rotateZ(rotate_t *rot, float x, float y, float *xr, float *yr)
{
    *xr = x * rot->cos - y * rot->sin;
    *yr = x * rot->sin + y * rot->cos;
}

//----------------------------------------------
static float char2float(vfont_t *ctx, uint8_t c)
{
    return ctx->scale.glyph * (float)(c - 'R');
}

//------------------------------------
static float scaledXYpad(vfont_t *ctx)
{
    float xypad =(ctx->scale.glyph * ctx->xypad * fabs(ctx->scale.horizontal)) + (ctx->brush.size / 2.0f);
    if (xypad < 1.0f) xypad = 1.0f;
    return xypad;
}

// Get glyph pointer for specific character code
// Returns NULL if not found
//--------------------------------------------------------
static char_glyph_t *find_glyph(hfont_t *font, uint16_t c)
{
    uint8_t *pglyphs = font->glyphs;
    char_glyph_t *glyph;
    for (int i=0; i<font->glyphCount; i++) {
        glyph = (char_glyph_t *)pglyphs;
        if (glyph->code == c) return glyph;
        pglyphs += (glyph->count + sizeof(char_glyph_t));
    }
    return NULL;
}


// returns scaled glyph horizontal width including padding
// or -1.0 if character's glyph was not found
// populates box structure with character's glyph metrics
//------------------------------------------------------------------------
float getCharMetrics(vfont_t *ctx, hfont_t *font, uint16_t ch, box_t *box)
{
    // find character's glyph
    char_glyph_t *glyph = find_glyph(font, ch);
    if (glyph == NULL) return -1.0f;

    float tm = 0.0f;
    float bm = 0.0f;
    float lm = 0.0f;
    float rm = 0.0f;
    float brushSize = ctx->brush.size / 2.0f;

    if (active_dstate->font_forceFixed) {
        rm = char2float(ctx, ctx->font->right) * fabs(ctx->scale.horizontal);
        lm = char2float(ctx, ctx->font->left) * fabs(ctx->scale.horizontal);
        tm = char2float(ctx, ctx->font->top) * fabs(ctx->scale.vertical);
        bm = char2float(ctx, ctx->font->bottom) * fabs(ctx->scale.vertical);
    }
    else {
        rm = char2float(ctx, glyph->right) * fabs(ctx->scale.horizontal);
        lm = char2float(ctx, glyph->left) * fabs(ctx->scale.horizontal);
        tm = char2float(ctx, glyph->top) * fabs(ctx->scale.vertical);
        bm = char2float(ctx, glyph->bottom) * fabs(ctx->scale.vertical);
    }

    box->x1 = lm - brushSize;
    box->y1 = tm - brushSize;
    box->x2 = rm + brushSize;
    box->y2 = bm + brushSize;

    return (rm - lm + scaledXYpad(ctx));
}

// Returns scaled glyph's width and height in w & h
//---------------------------------------------------------
void getGlyphSize(vfont_t *ctx, uint16_t c, int *w, int *h)
{
	box_t box = {0};
	if (getCharMetrics(ctx, ctx->font, c, &box) >= 0.0f) {
        if (w) *w = (int)(((box.x2 - box.x1) + 1.0f));
        if (h) *h = (int)(((box.y2 - box.y1) + 1.0f));
	}
}

// Returns scaled glyph's maximum width and height in w & h
//------------------------------------------------
void getMaxGlyphSize(vfont_t *ctx, int *w, int *h)
{
    if (w) *w = (ctx->font->right - ctx->font->left + 1) * ctx->scale.glyph * ctx->scale.horizontal;
    if (h) *h = (ctx->font->bottom - ctx->font->top + 1) * ctx->scale.glyph * ctx->scale.vertical;
}

// Returns scaled glyph's maximum height
//-----------------------------------
float getMaxGlyphHeight(vfont_t *ctx)
{
    return ((ctx->font->bottom - ctx->font->top + 1) * ctx->scale.glyph * ctx->scale.vertical);
}

//------------------------------------------------------------------
void getStringMetrics(vfont_t *ctx, unsigned char *text, box_t *box)
{
    float miny = 9999.0f;
    float maxy = -9999.0f;
    float minx = 9999.0f;
    float maxx = -9999.0f;
    unsigned char *ptext = text;
    uint16_t ch;

    while (*ptext){
        ch = *ptext++;
        // Process control characters
        if (ch == 12) {
            // '\f', map next character from map table 1
            ch = *ptext;
            if (ch) {
                ch = char_map_table1[ch];
                ptext++;
            }
            else continue;
        }
        if (ch == 7) {
            // '\a', map next character from map table 2
            ch = *ptext;
            if (ch) {
                ch = char_map_table2[ch];
                ptext++;
            }
            else continue;
        }
        else if (ch == 0x0A) {
            // ==== '\n', new line ====
            continue;
        }
        if (getCharMetrics(ctx, ctx->font, ch, box) >= 0.0f) {
            // only if glyph was found
            if (box->x1 < minx) minx = box->x1;
            if (box->x2 > maxx) maxx = box->x2;
            if (box->y1 < miny) miny = box->y1;
            if (box->y2 > maxy) maxy = box->y2;
        }
    }

    box->x1 = minx;
    box->y1 = miny;
    box->x2 = maxx;
    box->y2 = maxy;
}

//-----------------------------------------------------------------------------------
static void drawBrush(vfont_t *ctx, float xc, float yc, float radius, uint16_t color)
{

    int d = (int)radius>>1;
    if (ctx->brush.type == BRUSH_DISK) TFT_fillCircle(xc, yc, (int)radius, color);
    else if (ctx->brush.type == BRUSH_CIRCLE) TFT_drawCircle(xc, yc, radius, color);

    else if (ctx->brush.type == BRUSH_SQUARE_FILLED) TFT_fillRect(xc-radius, yc-radius, d, d, color);
    else if (ctx->brush.type == BRUSH_SQUARE) TFT_drawRect(xc-radius, yc-radius, d, d, color);
    else if (ctx->brush.type == BRUSH_TRIANGLE_FILLED) TFT_fillTriangle(xc-d, yc+d, xc, yc-d, xc+d, yc+d, color);
    else if (ctx->brush.type == BRUSH_TRIANGLE) TFT_drawTriangle(xc-d, yc+d, xc, yc-d, xc+d, yc+d, color);
    else if (ctx->brush.type == BRUSH_STROKE_1) {
        TFT_drawLine(xc-d, yc+d, xc+d, yc-d, color);        // slope up
        xc++;
        TFT_drawLine(xc-d, yc+d, xc+d, yc-d, color);
    }
    else if (ctx->brush.type == BRUSH_STROKE_2) {
        TFT_drawLine(xc-d, yc-d, xc+d, yc+d, color);        // slope down
        xc++;
        TFT_drawLine(xc-d, yc-d, xc+d, yc+d, color);
    }
    else if (ctx->brush.type == BRUSH_STROKE_3) {
        TFT_drawLine(xc-d, yc, xc+d, yc, color);            // horizontal
        yc++;
        TFT_drawLine(xc-d, yc, xc+d, yc, color);
    }
    else if (ctx->brush.type == BRUSH_STROKE_4) {
        TFT_drawLine(xc, yc-d, xc, yc+d, color);            // vertical
        xc++;
        TFT_drawLine(xc, yc-d, xc, yc+d, color);
    }
    else if (ctx->brush.type == BRUSH_STROKE_5) {
        TFT_drawLine(xc-d, yc+d, xc+d, yc-d, color);        // forward slope up with smaller siblings either side
        d = radius*0.3f;
        xc++;
        TFT_drawLine(xc-d, yc+d, xc+d, yc-d, color);
        xc -= 2;
        //yc -= 2;
        TFT_drawLine(xc-d, yc+d, xc+d, yc-d, color);
    }
    else if (ctx->brush.type == BRUSH_STROKE_6) {
        TFT_drawLine(xc-d, yc+d, xc+d, yc-d, color);        // BRUSH_STROKE_5 but thicker
        xc++;
        TFT_drawLine(xc-d, yc+d, xc+d, yc-d, color);
        d = radius*0.3f;
        xc++;
        TFT_drawLine(xc-d, yc+d, xc+d, yc-d, color);        // right side
        xc -= 2; yc -= 1;
        TFT_drawLine(xc-d, yc+d, xc+d, yc-d, color);        // left side
    }
    else if (ctx->brush.type == BRUSH_STAR) {
        TFT_drawLine(xc-d, yc+d, xc+d, yc-d, color);        // slope up
        TFT_drawLine(xc-d, yc-d, xc+d, yc+d, color);        // slope down
        TFT_drawLine(xc-d, yc, xc+d, yc, color);            // horizontal
        TFT_drawLine(xc, yc-d, xc, yc+d, color);            // vertical
    }
    else if (ctx->brush.type == BRUSH_X) {
        TFT_drawLine(xc-d, yc+d, xc+d, yc-d, color);        // slope up
        TFT_drawLine(xc-d, yc-d, xc+d, yc+d, color);        // slope down
    }
    else if (ctx->brush.type == BRUSH_PLUS) {
        TFT_drawLine(xc-d, yc, xc+d, yc, color);            // horizontal
        TFT_drawLine(xc, yc-d, xc, yc+d, color);            // vertical
    }
    else if (ctx->brush.type == BRUSH_CARET) {
        TFT_drawLine(xc-d, yc+d, xc, yc-d, color);          // left
        TFT_drawLine(xc, yc-d, xc+d, yc+d, color);          // right
    }
    else {
        // BRUSH_POINT
        TFT_drawPixel(xc, yc, color);
    }
}

// Draw vector from (x1,y1) to (x2,y2)
// (x1,y1) and (x2,y2) are points on SCREEN coordinates
//-----------------------------------------------------------------------------------------------
static void drawBrushVector(vfont_t *ctx, float x1, float y1, float x2, float y2, uint16_t color)
{
    if (ctx->brush.size > 1.0f) {
        float i = 0.0f;
        float x = x1;
        float y = y1;
        float bmul = ctx->brush.advanceMult;

        while ((distance(x, y, x2, y2) > bmul) && (isBetween(x1, y1, x, y, x2, y2))) {
            i += 1.0f;
            x = (x1 + i * bmul * (x2 - x1) / distance(x1, y1, x2, y2));
            y = (y1 + i * bmul * (y2 - y1) / distance(x1, y1, x2, y2));
            drawBrush(ctx, x, y, ctx->brush.size / 2.0f, color);
        }
    }
    else TFT_drawLine(x1, y1, x2, y2, color);
}

// Perform vector transformation(s) if needed
// and draw vector from (x1,y1) to (x2,y2)
// (x1,y1) and (x2,y2) are points on SCREEN coordinates
//--------------------------------------------------------------------------
static void drawVector(vfont_t *ctx, float x1, float y1, float x2, float y2)
{
    if (ctx->renderOp == RENDEROP_NONE) {
        drawBrushVector(ctx, x1, y1, x2, y2, ctx->brush.color);
        return;
    }
    else {
        // transform to 0
        x1 -= ctx->x; x2 -= ctx->x;
        y1 -= ctx->y; y2 -= ctx->y;
    }

    if (ctx->renderOp & RENDEROP_SHEAR_X) {
        x1 += y1 * ctx->shear.tan;
        x2 += y2 * ctx->shear.tan;
    }

    if (ctx->renderOp & RENDEROP_SHEAR_Y) {
        y1 = (x1 * ctx->shear.sin + y1 * ctx->shear.cos) + 1.0f;
        y2 = (x2 * ctx->shear.sin + y2 * ctx->shear.cos) + 1.0f;
    }

    if (ctx->renderOp & RENDEROP_ROTATE_GLYPHS) {
        //float x1r, y1r, x2r, y2r;
        // undo string transform
        x1 += ctx->x; x2 += ctx->x;
        y1 += ctx->y; y2 += ctx->y;

        // apply glyph transform
        x1 -= ctx->pos.x; x2 -= ctx->pos.x;
        y1 -= ctx->pos.y; y2 -= ctx->pos.y;

        rotateZ(&ctx->rotate.glyph, x1, y1, &x1, &y1);
        rotateZ(&ctx->rotate.glyph, x2, y2, &x2, &y2);

        x1 += ctx->pos.x - ctx->x; x2 += ctx->pos.x - ctx->x;
        y1 += ctx->pos.y - ctx->y; y2 += ctx->pos.y - ctx->y;
    }

    if (ctx->renderOp & RENDEROP_ROTATE_STRING) {
        rotateZ(&ctx->rotate.string, x1, y1, &x1, &y1);
        rotateZ(&ctx->rotate.string, x2, y2, &x2, &y2);
    }

    //  transform back
    x1 += ctx->x; x2 += ctx->x;
    y1 += ctx->y; y2 += ctx->y;

    // now draw the brush
    drawBrushVector(ctx, x1, y1, x2, y2, ctx->brush.color);
}

// Glyph is drawn relative to the glyph CENTER
// returns horizontal glyph advance
//------------------------------------------------------------
static inline int drawGlyph(vfont_t *ctx, char_glyph_t *glyph)
{
    uint8_t *hc = (uint8_t *)glyph + sizeof(char_glyph_t);
    uint8_t *hc_end = hc + glyph->count;

    float lm = 0.0f;
    float rm = 0.0f;
    float x_start = ctx->pos.x;

    if (active_dstate->font_forceFixed) {
        rm = char2float(ctx, ctx->font->right) * fabs(ctx->scale.horizontal);
        lm = char2float(ctx, ctx->font->left) * fabs(ctx->scale.horizontal);
    }
    else {
        rm = char2float(ctx, glyph->right) * fabs(ctx->scale.horizontal);
        lm = char2float(ctx, glyph->left) * fabs(ctx->scale.horizontal);
    }

    // Set the glyph's start X position
    // as 'lm' is always negative, this will advance the current x position to the glyphs senter
    ctx->pos.x -= lm;

    float x1 = 0.0f;
    float y1 = 0.0f;
    float x, y, x2, y2;
    int newPath = 1;

    while (hc < hc_end) {
        // Get the vertex points
        if ((*hc == ' ') && (*(hc+1) == 'R')) {
            hc += 2;
            newPath = 1;
        }
        else {
            x = char2float(ctx, *hc++) * ctx->scale.horizontal;
            y = char2float(ctx, *hc++) * ctx->scale.vertical;

            if (newPath) {
                newPath = 0;
                x1 = ctx->pos.x + x;
                y1 = ctx->pos.y + y;
            }
            else {
                x2 = ctx->pos.x + x;
                y2 = ctx->pos.y + y;

                drawVector(ctx, x1, y1, x2, y2);
                x1 = x2;
                y1 = y2;
            }
        }
    }
    // Set X position for the next glyph
    ctx->pos.x += rm + scaledXYpad(ctx);
    return (int)(ctx->pos.x - x_start);
}

//-------------------------------------------------------------------
void vfontDrawString(vfont_t *ctx, unsigned char *text, int x, int y)
{
    unsigned char *ptext = text;
    float yoffset = (char2float(ctx, ctx->font->top) * fabs(ctx->scale.vertical));

    active_dstate->TFT_Y = y;
    ctx->x = ctx->pos.x = x;
    ctx->y = ctx->pos.y = (y - yoffset + (ctx->brush.size / 2.0f));
    uint16_t ch;
    int start_x = x;
    int start_y = ctx->y;
    char_glyph_t *glyph;

    while (*ptext) {
        ch = *ptext++;
        // Process control characters
        if (ch == 12) {
            // '\f', map next character from map table 1
            ch = *ptext;
            if (ch) {
                ch = char_map_table1[ch];
                ptext++;
            }
            else continue;
        }
        if (ch == 7) {
            // '\a', map next character from map table 2
            ch = *ptext;
            if (ch) {
                ch = char_map_table2[ch];
                ptext++;
            }
            else continue;
        }
        else if (ch == 0x0A) {
            // ==== '\n', new line ====
            ctx->pos.x = start_x ;
            ctx->pos.y = start_y + getMaxGlyphHeight(ctx) + scaledXYpad(ctx);
            if ((int)ctx->pos.y > (active_dstate->dispWin.y2 - (int)ctx->pos.y)) break;
            active_dstate->TFT_Y += (int)(getMaxGlyphHeight(ctx) + scaledXYpad(ctx));
            continue;
        }

        glyph = find_glyph(ctx->font, ch);
        if (glyph == NULL) {
            ptext++;
            continue;
        }
        // If character is outside right margin wrap it if allowed
        if ((uint16_t)(ctx->pos.x + (glyph->right - glyph->left + 1)) > active_dstate->dispWin.x2) {
            // character is outside right margin
            if (active_dstate->text_wrap) {
                ctx->pos.x = start_x ;
                ctx->pos.y = start_y + getMaxGlyphHeight(ctx) + scaledXYpad(ctx);
                if ((int)ctx->pos.y > (active_dstate->dispWin.y2 - (int)ctx->pos.y)) break;
                active_dstate->TFT_Y += (int)(getMaxGlyphHeight(ctx) + scaledXYpad(ctx));
            }
            else break;
        }
        drawGlyph(ctx, glyph);
    }
    active_dstate->TFT_X = (int)ctx->pos.x;
}

//---------------------------------------
void setFont(vfont_t *ctx, hfont_t *font)
{
    if (font) ctx->font = font;
}

//----------------------------
hfont_t *getFont(vfont_t *ctx)
{
	return ctx->font;
}

//-----------------------------------
int setBrush(vfont_t *ctx, int brush)
{
	if (brush < BRUSH_TOTAL){
		int old = ctx->brush.type;
		ctx->brush.type = brush;
		return old;
	}
	return ctx->brush.type;
}


// calculate glyph stride
//-----------------------------------------------
static inline void brushCalcAdvance(vfont_t *ctx)
{
	ctx->brush.advanceMult = (ctx->brush.size * ctx->brush.step) / 100.0f;
}

//------------------------------------------
float setBrushSize(vfont_t *ctx, float size)
{
	if (size >= 0.5f){
		float old = ctx->brush.size;
		ctx->brush.size = size;
		brushCalcAdvance(ctx);
		return old;
	}
	return ctx->brush.size;
}

// 0.5 to 100.0
//-----------------------------------------
void setBrushStep(vfont_t *ctx, float step)
{
	if (step >= 0.5f){
		ctx->brush.step = step;
		brushCalcAdvance(ctx);
	}
}

//------------------------------
float getBrushStep(vfont_t *ctx)
{
	return ctx->brush.step;
}

//-------------------------------------------
void setGlyphScale(vfont_t *ctx, float scale)
{
	if (scale >= 0.1f)
		ctx->scale.glyph = scale;
}

//-------------------------------
float getGlyphScale(vfont_t *ctx)
{
	return ctx->scale.glyph;
}

// Extra space added to every glyph. (can be 0 or minus)
//-------------------------------------------
void setGlyphPadding(vfont_t *ctx, float pad)
{
	ctx->xypad = pad;
}

//---------------------------------
float getGlyphPadding(vfont_t *ctx)
{
	return ctx->xypad;
}

//--------------------------------------------------
uint16_t setBrushColor(vfont_t *ctx, uint16_t color)
{
	uint16_t old = ctx->brush.color;
	ctx->brush.color = color;
	return old;
}

//----------------------------------
uint16_t getBrushColor(vfont_t *ctx)
{
	return ctx->brush.color;
}

//---------------------------------------------
void setRenderFilter(vfont_t *ctx, uint32_t mask)
{
	ctx->renderOp = mask & 0xFFFF;
}

//------------------------------------
uint32_t getRenderFilter(vfont_t *ctx)
{
	return ctx->renderOp;
}

// when rotation from horizontal, when enabled via setRenderFilter()
//------------------------------------------------------------------
void setRotationAngle(vfont_t *ctx, float rotGlyph, float rotString)
{
	float oldG = ctx->rotate.glyph.angle;
	ctx->rotate.glyph.angle = fmodf(rotGlyph, 360.0f);

	if (oldG != ctx->rotate.glyph.angle){
		ctx->rotate.glyph.cos = cosf(DEG2RAD(ctx->rotate.glyph.angle));
		ctx->rotate.glyph.sin = sinf(DEG2RAD(ctx->rotate.glyph.angle));
	}
	
	float oldS = ctx->rotate.string.angle;
	ctx->rotate.string.angle = fmodf(rotString, 360.0f);
	
	if (oldS != ctx->rotate.string.angle){
		ctx->rotate.string.cos = cosf(DEG2RAD(ctx->rotate.string.angle));
		ctx->rotate.string.sin = sinf(DEG2RAD(ctx->rotate.string.angle));
	}
	
}

//------------------------------------------------------
void setShearAngle(vfont_t *ctx, float shrX, float shrY)
{
	ctx->shear.angleX = shrX;
	ctx->shear.angleY = shrY;
	
	ctx->shear.tan = -tanf(DEG2RAD(ctx->shear.angleX) /*/ 2.0f*/);
	ctx->shear.cos = cosf(DEG2RAD(ctx->shear.angleY));
	ctx->shear.sin = sinf(DEG2RAD(ctx->shear.angleY));
}

//--------------------------------------------------
void setAspect(vfont_t *ctx, float hori, float vert)
{
	if (hori >= 0.05f || hori <= -0.05f)
		ctx->scale.horizontal = hori;
	
	if (vert >= 0.05f || vert <= -0.05f)
		ctx->scale.vertical = vert;
}

//--------------------------------
void vfontInitialise(vfont_t *ctx)
{
	memset(ctx, 0, sizeof(vfont_t));

	// Set defaults
	setAspect(ctx, 1.0f, 1.0f);
	setGlyphPadding(ctx, 4.0f);
	setGlyphScale(ctx, 1.0f);
	setBrush(ctx, BRUSH_DISK);
	setBrushSize(ctx, 1.0f);
	setBrushStep(ctx, 10.0f);
	setBrushColor(ctx, TFT_CYAN);
	setRenderFilter(ctx, RENDEROP_NONE);
    setRotationAngle(ctx, 0.0f, 0.0f);
}

