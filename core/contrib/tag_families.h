#ifndef _TAG_FAMILIES_H_
#define _TAG_FAMILIES_H_

typedef struct apriltag_family apriltag_family_t;

apriltag_family_t* tag_create_by_name(const char* name);

#endif
