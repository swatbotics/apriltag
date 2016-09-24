#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "tag16h5.h"

typedef struct apriltag_family* (*factory_func_t)();

struct apriltag_family_info {
  const char* name;
  factory_func_t ctor;
};

static const struct apriltag_family_info lookup[] = {
  { "tag36h11", tag36h11_create },
  { "tag36h10", tag36h10_create },
  { "tag36artoolkit", tag36artoolkit_create },
  { "tag25h9", tag25h9_create },
  { "tag25h7", tag25h7_create },
  { "tag16h5", tag16h5_create },
  { NULL, NULL }
};

zarray_t* apriltag_family_list() {

  zarray_t* rval = zarray_create(sizeof(const char*));

  int i;

  for (i=0; lookup[i].name; ++i) {
    zarray_insert(rval, i, &lookup[i].name);
  }

  return rval;
  
}

apriltag_family_t* apriltag_family_create(const char* famname) {

  int i;

  for (i=0; lookup[i].name; ++i) {
    if (!strcmp(famname, lookup[i].name)) {
      return lookup[i].ctor();
    }
  }

  return NULL;

}


void apriltag_family_destroy(apriltag_family_t *tf) {
  
   free(tf->name);
   free(tf->codes);
   free(tf);
   
}
