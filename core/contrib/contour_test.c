#include "contour.h"
#include <stdio.h>

typedef struct string_image {
  int rows; 
  int cols;
  char buf[2048];
} string_image_t;

const string_image_t images[] = {


  { 6, 12,
    "            "
    "  1111111   "
    "  1  1  1 1 "
    "  1  1  1   "
    "  1111111   "
    "            " },


  { 8, 8,
    "        "
    " 111111 "
    " 1    1 "
    " 1 11 1 "
    " 1 11 1 "
    " 1    1 "
    " 111111 "
    "        "
  },


  { 8, 8,
    "        "
    " 111111 "
    " 1    1 "
    " 1 1    "
    " 1 111  "
    " 1      "
    " 111111 "
    "        "
  },

  { 6, 6,
    "111111"
    "1    1"
    "1 11 1"
    "1 11 1"
    "1    1"
    "111111"
  },

  { 35, 45,
    //0        1         2         3         4    
    //12345678901234567890123456789012345678901234
    "                                             "
    "  22:  333-   4444444444444444+   555=  66#  "
    " 21:  31-    417$           771+    51=  61# "
    " 2:   3-   447$               774+   5=   6# "
    " :   3-   41$    8888888888?    71+   5=   # "
    " :  31-  41$   88199999999918?   71+  51=  # "
    "   31-  41$   819&         991?   71+  51=   "
    "   3-   4$   81&             91?   7+   5=   "
    "  31-  4$   81&   AAAAAAAA@   91?   7+  51=  "
    "  3-  41$  81&   A1BBBBBBB1@   91?  71+  5=  "
    "  3-  4$  81&  AAB        BBA@  91?  7+  5=  "
    " 3-   4$  8&   A*           B@   9?  7+   5= "
    " 3-  4$  81&  A*   CCCCCC*   B@  91?  7+  5= "
    " 3-  4$  8&  A*   C1DDDDD1*  B1@  9?  7+  5= "
    " 3-  4$  8&  A*  C1      D1   B@  9?  7+  5= "
    " 3-  4$  8&  A*  C*       D   B@  9?  7+  5= "
    " 3-  4$  8&  A*  C*  EE*  D*  B@  9?  7+  5= "
    " 3-  4$  8&  A*  C*  E1*  D*  B@  9?  7+  5= " // 18
    " 3-  4$  8&  A*  C*  EE*  D*  B@  9?  7+  5= " // 19
    " 3-  4$  8&  A*  C*       D*  B@  9?  7+  5= " // 20
    " 3-  4$  8&  A*  C1*     D1*  B@  9?  7+  5= " // 21
    " 3-  4$  8&  A1*  C1DDDDD1*  B1@  9?  7+  5= " // 22
    " 3-  4$  81&  A*   CCCCCC*   B@  91?  7+  5= " // 23
    " 3-   4$  8&   A*           B@   9?  7+   5= " // 24
    "  3-  4$  81&   AAB*      BBA@  91?  7+  5=  " // 25
    "  3-  41$  81&   A1BBBBBBB1@   91?  71+  5=  " // 26
    "  31-  4$   81&   AAAAAAAA@   91?   7+  51=  " // 27
    "   3-   4$   81&             91?   7+   5=   " // 28
    "   31-  41$   819&         991?   71+  51=   " // 29
    " *  31-  41$   88199999999918?   71+  51=  * " // 30
    " *   3-   41$    8888888888?    71+   5=   * " // 31
    " F*   3-   447$               774+   5=   G* " // 32
    " F1*  31-    417$           771+    51=  G1* " // 33
    "  FF*  333-   4444444444444444+   555=  GG*  " // 34
    "                                             "
  },

  { 0, 0, "" }


};


int main(int argc, char** argv) {

  for (int k=0; images[k].rows; ++k) {
    
    const string_image_t* src = &(images[k]);

    image_u8_t* im = image_u8_create_alignment(src->cols, src->rows, 1);

    if (im->stride != im->width) {
      fprintf(stderr, "bad stride=%d (width=%d)\n", im->stride, im->width);
      exit(1);
    }

    int offs = 0;

    for (int i=0; i<src->rows; ++i) {
      for (int j=0; j<src->cols; ++j) {
        im->buf[offs] = (src->buf[offs] == ' ' ? 0 : 1);
        ++offs;
      }
    }

    zarray_t* contours = contour_detect(im);

    int nc = zarray_size(contours);
    
    printf("got %d contours\n", nc);
    printf("border  outer  coords   parent  perim\n");

    for (int i=0; i<nc; ++i) {
      const contour_info_t* ci;
      zarray_get_volatile(contours, i, &ci);
      int np = zarray_size(ci->points);
      const contour_point_t* p0;
      zarray_get_volatile(ci->points, 0, &p0);
      int i0 = p0->y + 1; int j0 = p0->x + 1;
      printf("%6d  %5d  (%2d,%2d)  %6d  %5d\n", i, ci->is_outer ? 1 : 0, i0, j0, ci->parent, np);
    }

    contour_destroy(contours);
    image_u8_destroy(im);

  }

  return 0;

}

