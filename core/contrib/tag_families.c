#include "apriltag.h"
#include "tag36h11.h"
#include "tag16h5.h"
#include "tag25h9.h"
#include <string.h>

apriltag_family_t* tag_create_by_name(const char* name) {

  if (!strcmp(name, "tag36h11")) {
    return tag36h11_create();
  } else if (!strcmp(name, "tag16h5")) {
    return tag16h5_create();
  } else if (!strcmp(name, "tag25h9")) {
    return tag25h9_create();
  } else { // TODO: all the rest
    return NULL;
  }
  
}
