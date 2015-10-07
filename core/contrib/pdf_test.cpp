#include "pdfutil.h"
#include <stdio.h>
#include <math.h>

image_u8_t* test_gray() {

  image_u8_t* image = image_u8_create(32, 32);

  uint8_t* dstrow = image->buf;

  for (int y=0; y<image->height; ++y) {
    double fy = (y+0.5)/image->height;
    fy = 2*fy - 1;
    uint8_t* dst = dstrow;
    for (int x=0; x<image->width; ++x) {
      double fx = (x+0.5)/image->width;
      fx = 2*fx - 1;
      double r2 = fx*fx + fy*fy;
      double r = sqrt(r2);
      double i = cos(r*16.0) * exp(-r2*4.0);
      *dst++ = (i*0.5+0.5) * 255.0;
    }
    dstrow += image->stride;
  }

  return image;
  
}

#define MAKE_RGB(r, g, b) ( ((b)<<16) | ((g)<<8) | ((r)<<0) )

image_u32_t* test_rgb() {

  image_u32_t* image = image_u32_create(256, 256);

  uint32_t* dstrow = image->buf;
  
  for (int y=0; y<image->height; ++y) {
    double fy = (y+0.5)/image->height;
    fy = 2*fy - 1;
    uint32_t* dst = dstrow;
    for (int x=0; x<image->width; ++x) {
      double fx = (x+0.5)/image->width;
      uint32_t r = 255 * (cos(2.0*fx*M_PI)*0.5+0.5);
      uint32_t g = 255 * (sin(fx*M_PI)*0.5+0.5);
      uint32_t b = 255 * (cos(fy*M_PI)*0.5+0.5);
      *dst++ = MAKE_RGB(r,g,b);
    }
    dstrow += image->stride;
  }
  
  return image;
  
}


int main(int argc, char** argv) {

  image_u8_t* gray = test_gray();
  image_u8_write_pnm(gray, "gray.pnm");

  image_u32_t* rgb = test_rgb();
  image_u32_write_pnm(rgb, "rgb.pnm");
  
  pdf_t* pdf = pdf_create(4.0*72, 4.0*72);

  pdf_text(pdf, PDF_FONT_COURIER, 12.0, 100.0, 100.0, "Hello World!");
  pdf_text(pdf, PDF_FONT_TIMES_BOLD_ITALIC, 24.0, 20.0, 50.0, "Hello World!");

  pdf_image_gray(pdf, 1, 50, 50, gray);

  pdf_save_stream(pdf, pdf_file_write, stdout);
  
  pdf_destroy(pdf);
  image_u8_destroy(gray);
  image_u32_destroy(rgb);

  
  return 0;

}
