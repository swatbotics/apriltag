#ifndef _PDFUTIL_H
#define _PDFUTIL_H

#include "image_u8.h"
#include "image_u32.h"
#include "zarray.h"
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pdf pdf_t;

pdf_t* pdf_create(double width, double height);

void pdf_destroy(pdf_t* pdf);

typedef enum pdf_font {
    PDF_FONT_COURIER,
    PDF_FONT_COURIER_BOLD,
    PDF_FONT_COURIER_ITALIC,
    PDF_FONT_COURIER_BOLD_ITALIC,
    PDF_FONT_HELVETICA,
    PDF_FONT_HELVETICA_BOLD,
    PDF_FONT_HELVETICA_ITALIC,
    PDF_FONT_HELVETICA_BOLD_ITALIC, 
    PDF_FONT_TIMES,
    PDF_FONT_TIMES_BOLD,
    PDF_FONT_TIMES_ITALIC,
    PDF_FONT_TIMES_BOLD_ITALIC,
    PDF_FONT_SYMBOL,
    PDF_FONT_ZAPF_DINGBATS
} pdf_font_t;

typedef enum pdf_stroke_type {
    PDF_STROKE_NONE,
    PDF_STROKE_REGULAR,
    PDF_STROKE_CLOSED
} pdf_stroke_type_t;

typedef enum pdf_join {
    PDF_JOIN_MITER,
    PDF_JOIN_ROUND,
    PDF_JOIN_BEVEL
} pdf_join_t;

typedef enum pdf_cap {
    PDF_CAP_BUTT,
    PDF_CAP_ROUND,
    PDF_CAP_SQUARE
} pdf_cap_t;

typedef struct pdf_stroke {
    double color_rgb[3];
    double width;
    pdf_cap_t cap;
    pdf_join_t join;
    pdf_stroke_type_t auto_close;
} pdf_stroke_t;

typedef enum pdf_fill_type {
    PDF_FILL_NONE,
    PDF_FILL_NONZERO,
    PDF_FILL_EVENODD
} pdf_fill_type_t;


typedef struct pdf_fill {
    double color_rgb[3];
    pdf_fill_type_t fill_rule;
} pdf_fill_t;


// should return 0 on success
typedef int (*pdf_write_callback)(size_t len, const char* data,
                                  void* userdata);

void pdf_end_page(pdf_t* pdf);

void pdf_text(pdf_t* pdf, pdf_font_t font,
              double size, double tx, double ty,
              const char* string);

int pdf_save(pdf_t* pdf, const char* filename);

int pdf_file_write(size_t len, const char* data, void* file);

int pdf_save_stream(pdf_t* pdf, pdf_write_callback cbk, void* userdata);

void pdf_image_gray(pdf_t* pdf,
                    double scale, // = points_per_sample
                    double x, double y,
                    const image_u8_t* image);

void pdf_image_rgb(pdf_t* pdf,
                   double scale, // = points_per_sample
                   double x, double y,
                   const image_u32_t* image); 
                   
typedef enum pdf_path_type {
    PDF_PATH_TYPE_MOVETO,
    PDF_PATH_TYPE_LINETO,
    PDF_PATH_TYPE_RECT,
    PDF_PATH_TYPE_CLOSE
} pdf_path_type_t;

typedef struct pdf_path_element {
    pdf_path_type_t type;
    double xydata[4];
} pdf_path_element_t;

zarray_t* pdf_path_create();
zarray_t* pdf_path_copy(zarray_t* path);
void pdf_path_destroy();

void pdf_get_stroke(const pdf_t* pdf, pdf_stroke_t* stroke);
void pdf_get_fill(const pdf_t* pdf, pdf_fill_t* fill);

void pdf_set_stroke(pdf_t* pdf, const pdf_stroke_t* stroke);
void pdf_set_fill(pdf_t* pdf, const pdf_fill_t* fill);

void pdf_path_move_to(zarray_t* path, double x, double y);
void pdf_path_line_to(zarray_t* path, double x, double y);
void pdf_path_rect(zarray_t* path, double x0, double y0, double w, double h);
void pdf_path_close(zarray_t* path);

void pdf_path_draw(pdf_t* pdf,
                   zarray_t* path,
                   pdf_stroke_type_t stroke_type,
                   pdf_fill_type_t fill_type);

void pdf_gstate_push(pdf_t* pdf);
void pdf_gstate_pop(pdf_t* pdf);

void pdf_ctm_concat(pdf_t* pdf,
                    double xx, double yx, 
                    double xy, double yy,
                    double tx, double ty);

void pdf_set_rgb(double rgb[3], double r, double g, double b);

pdf_stroke_t pdf_default_stroke(double r, double g, double b);
pdf_fill_t pdf_default_fill(double r, double g, double b);

void pdf_destroy(pdf_t* pdf);


#ifdef __cplusplus
}
#endif

#endif
