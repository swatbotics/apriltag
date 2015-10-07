#ifndef _PDFUTIL_H
#define _PDFUTIL_H

#include "image_u8.h"
#include "image_u32.h"
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

typedef void (*pdf_write_callback)(size_t len, const char* data,
                                   void* userdata);

void pdf_end_page(pdf_t* pdf);

void pdf_text(pdf_t* pdf, pdf_font_t font,
              double size, double tx, double ty,
              const char* string);

void pdf_save(pdf_t* pdf, const char* filename);

void pdf_file_write(size_t len, const char* data, void* file);

void pdf_save_stream(pdf_t* pdf, pdf_write_callback cbk, void* userdata);

void pdf_image_gray(pdf_t* pdf,
                    double scale, // = points_per_sample
                    double x, double y,
                    const image_u8_t* image);

void pdf_image_rgb(pdf_t* pdf,
                   double scale, // = points_per_sample
                   double x, double y,
                   const image_u32_t* image); 
                   

void pdf_destroy(pdf_t* pdf);

#ifdef __cplusplus
}
#endif

#endif
