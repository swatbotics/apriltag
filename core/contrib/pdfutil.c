#include "pdfutil.h"
#include "zarray.h"
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include <setjmp.h>

enum pdf_object_type {
  PDF_OBJECT_NULL=0,
  PDF_OBJECT_BOOLEAN,
  PDF_OBJECT_INTEGER,
  PDF_OBJECT_REAL,
  PDF_OBJECT_STRING,
  PDF_OBJECT_NAME,
  PDF_OBJECT_ARRAY,
  PDF_OBJECT_DICT,
  PDF_OBJECT_STREAM,
  PDF_OBJECT_REFERENCE,
};


typedef struct pdf_string {
  size_t length;
  char* data; // free'd on release
} pdf_string_t;

typedef struct pdf_string pdf_name_t;

typedef struct pdf_object pdf_object_t;

typedef struct pdf_dict_entry {
  pdf_object_t* key; // MUST BE OF NAME TYPE
  pdf_object_t* value; // 
} pdf_dict_entry_t;

typedef struct pdf_refspec { 
  int id;
  int revision;
} pdf_refspec_t;

typedef struct pdf_stream {
  pdf_object_t* dict; // NULL allowed
  zarray_t* content;
} pdf_stream_t;

struct pdf_object {
  int type;
  union {
    int           boolean;
    int           integer;
    double        real;
    pdf_string_t  string;
    pdf_name_t    name;
    zarray_t*     array; // of pdf_object_t*
    zarray_t*     dict; // of pdf_dict_entry_t
    pdf_stream_t  stream; // 
    pdf_refspec_t refspec;
  };
};

typedef struct pdf_toplevel_object {
  pdf_refspec_t refspec;
  pdf_object_t* object;
} pdf_toplevel_object_t;

typedef struct pdf_gstate {
  pdf_stroke_t stroke;
  pdf_fill_t fill;
} pdf_gstate_t;

struct pdf {
  int is_finished;
  pdf_refspec_t pages_ref;   
  pdf_object_t* pages_array; // stored in pages dict
  pdf_object_t* page_count;  // stored in pages dict
  pdf_object_t* cur_page;
  pdf_object_t* cur_page_contents;
  pdf_object_t* cur_page_fonts;
  zarray_t* toplevel_objects; // of pdf_toplevel_object_t*
  pdf_refspec_t fonts[14];
  zarray_t* gstate;
};

int pdf_objects_equal(const pdf_object_t* a,
                      const pdf_object_t* b) {

  if ((a && !b) || (b && !a)) {
    return 0;
  }

  if (a == b) { return 1; }

  if (a->type != b->type) { return 0; }

  switch (a->type) {
  case PDF_OBJECT_BOOLEAN:
    return (a->boolean != 0) == (b->boolean != 0);
  case PDF_OBJECT_INTEGER:
    return a->integer == b->integer;
  case PDF_OBJECT_REAL:
    return a->real == b->real;
  case PDF_OBJECT_STRING:
    return (a->string.length == b->string.length &&
            memcmp(a->string.data, b->string.data, a->string.length) == 0);
  case PDF_OBJECT_NAME:
    return (a->name.length == b->name.length &&
            memcmp(a->name.data, b->name.data, a->name.length) == 0);
  case PDF_OBJECT_ARRAY: {
    if (a->array->size != b->array->size) { return 0; }
    for (int i=0; i<a->array->size; ++i) {
      pdf_object_t* ai, *bi;
      zarray_get(a->array, i, &ai);
      zarray_get(b->array, i, &bi);
      if (!pdf_objects_equal(ai, bi)) { return 0; }
    }
    return 1;
  }
  case PDF_OBJECT_DICT: {
    if (a->dict->size != b->dict->size) { return 0; }
    for (int i=0; i<a->array->size; ++i) {
      pdf_dict_entry_t ai, bi;
      zarray_get(a->array, i, &ai);
      zarray_get(b->array, i, &bi);
      if (!pdf_objects_equal(ai.key, bi.key) ||
          !pdf_objects_equal(ai.value, bi.value)) {
        return 0;
      }
    }
    return 1;
  }
  case PDF_OBJECT_STREAM: {
    if (!pdf_objects_equal(a->stream.dict,
                           b->stream.dict)) {
      return 0;
    } else if (a->stream.content->size != b->stream.content->size) {
      return 0;
      return memcmp(a->stream.content->data,
                    b->stream.content->data,
                    a->stream.content->size) == 0;
    }
  }
  }

  return 0;

  
  
}

void pdf_object_destroy(pdf_object_t* o);

pdf_object_t* pdf_object_create() {
  pdf_object_t* o = calloc(1, sizeof(pdf_object_t));
  return o;
}

pdf_object_t* pdf_object_create_type(int t) {
  pdf_object_t* o = pdf_object_create();
  o->type = t;
  return o;
}

pdf_object_t* pdf_boolean_create(int b) {
  pdf_object_t* o = pdf_object_create_type(PDF_OBJECT_BOOLEAN);
  o->boolean = b;
  return o;
}

pdf_object_t* pdf_integer_create(int i) {
  pdf_object_t* o = pdf_object_create_type(PDF_OBJECT_INTEGER);
  o->integer = i;
  return o;
}

pdf_object_t* pdf_real_create(double d) {
  pdf_object_t* o = pdf_object_create_type(PDF_OBJECT_REAL);
  o->real = d;
  return o;
}

pdf_object_t* pdf_string_create(size_t len, const char* data) {
  pdf_object_t* o = pdf_object_create_type(PDF_OBJECT_STRING);
  o->string.length = len;
  o->string.data = (char*)malloc(len);
  memcpy(o->string.data, data, len);
  return o;
}

pdf_object_t* pdf_string_create_str(const char* data) {
  return pdf_string_create(strlen(data), data);
}

pdf_object_t* pdf_name_create(size_t len, const char* data) {
  pdf_object_t* o = pdf_object_create_type(PDF_OBJECT_NAME);
  o->name.length = len;
  o->name.data = (char*)malloc(len);
  memcpy(o->name.data, data, len);
  return o;
}

pdf_object_t* pdf_name_create_str(const char* data) {
  return pdf_name_create(strlen(data), data);
}

pdf_object_t* pdf_array_create() {
  pdf_object_t* o = pdf_object_create_type(PDF_OBJECT_ARRAY);
  o->array = zarray_create(sizeof(pdf_object_t*));
  return o;
}

void pdf_array_append(pdf_object_t* o, pdf_object_t* value) {
  assert(o->type == PDF_OBJECT_ARRAY);
  zarray_add(o->array, &value);
}

void pdf_array_append_unique(pdf_object_t* o, pdf_object_t* value) {
  assert(o->type == PDF_OBJECT_ARRAY);
  for (int i=0; i<o->array->size; ++i) {
    pdf_object_t* oi;
    zarray_get(o->array, i, &oi);
    if (pdf_objects_equal(oi, value)) {
      pdf_object_destroy(value);
      return;
    }
  }
  pdf_array_append(o, value);
}

pdf_object_t* pdf_dict_create() {
  pdf_object_t* o = pdf_object_create_type(PDF_OBJECT_DICT);
  o->dict = zarray_create(sizeof(pdf_dict_entry_t));
  return o;
}


pdf_dict_entry_t pdf_dict_get(pdf_object_t* o, int idx) {

  assert(o->type == PDF_OBJECT_DICT);
  assert(idx >= 0 && idx < zarray_size(o->dict));

  pdf_dict_entry_t rval;
  zarray_get(o->dict, idx, &rval);

  return rval;
  
}

int pdf_dict_lookup(pdf_object_t* o, size_t length, const char* data) {
  
  assert(o->type == PDF_OBJECT_DICT);

  for (int i=0; i<zarray_size(o->dict); ++i) {
    pdf_dict_entry_t keyval;
    zarray_get(o->dict, i, &keyval);
    assert(keyval.key->type == PDF_OBJECT_NAME);
    if (keyval.key->name.length == length &&
        memcmp(keyval.key->name.data, data, length) == 0) {
      return i;
    }
  }

  return -1;
  
}


int pdf_dict_lookup_str(pdf_object_t* o, const char* keystr) {

  size_t l = strlen(keystr);
  return pdf_dict_lookup(o, l, keystr);

}

void pdf_dict_insert(pdf_object_t* o,
                     pdf_object_t* key,
                     pdf_object_t* value) {

  assert(o->type == PDF_OBJECT_DICT);
  assert(key->type == PDF_OBJECT_NAME);

  
  pdf_dict_entry_t entry = { key, value };
  int i = pdf_dict_lookup(o, key->name.length, key->name.data);
  
  if (i >= 0) {
    pdf_dict_entry_t old;
    zarray_set(o->dict, i, &entry, &old);
    pdf_object_destroy(old.key);
    pdf_object_destroy(old.value);
  } else {
    zarray_add(o->dict, &entry);
  }


}

void pdf_dict_insert_str(pdf_object_t* o,
                          const char* key,
                          pdf_object_t* value) {

  pdf_dict_insert(o, pdf_name_create_str(key), value);

}


pdf_object_t* pdf_dict_lookup_create(pdf_object_t* o,
                                     size_t length,
                                     const char* keystr,
                                     pdf_object_t* (*fptr)()) {

  int i = pdf_dict_lookup_str(o, keystr);
  if (i >= 0) {
    return pdf_dict_get(o, i).value;
  } else {
    pdf_object_t* v = fptr();
    pdf_dict_insert(o, pdf_name_create(length, keystr), v);
    return v;
  }

}

  
pdf_object_t* pdf_dict_lookup_str_create(pdf_object_t* o,
                                         const char* keystr,
                                         pdf_object_t* (*fptr)()) {

  size_t l = strlen(keystr);
  return pdf_dict_lookup_create(o, l, keystr, fptr);

}

  
pdf_object_t* pdf_stream_create() {
  pdf_object_t* o = pdf_object_create_type(PDF_OBJECT_STREAM);
  o->stream.content = zarray_create(sizeof(char));
  
  return o;
}

void pdf_stream_set_dict(pdf_object_t* o, pdf_object_t* d) {
  assert(o->type == PDF_OBJECT_STREAM);
  if (o->stream.dict) { pdf_object_destroy(o->stream.dict); }
  o->stream.dict = d;
}

void pdf_stream_append(pdf_object_t* o, size_t len, const char* data) {
  assert(o->type == PDF_OBJECT_STREAM);
  int oldlen = zarray_size(o->stream.content);
  zarray_ensure_capacity(o->stream.content, oldlen + len);
  char* dst = o->stream.content->data + oldlen;
  memcpy(dst, data, len);
  o->stream.content->size = oldlen + len;
}

void pdf_stream_append_str(pdf_object_t* o, const char* data) {
  pdf_stream_append(o, strlen(data), data);
}

pdf_object_t* pdf_stream_create_data(size_t len, const char* data) {
  pdf_object_t* o = pdf_stream_create();
  pdf_stream_append(o, len, data);
  return o;
}

pdf_object_t* pdf_stream_create_str(const char* data) {
  pdf_object_t* o = pdf_stream_create();
  pdf_stream_append_str(o, data);
  return o;
}

pdf_refspec_t pdf_refspec_make(int id, int revision) {
  pdf_refspec_t rval = { id, revision };
  return rval;
}

pdf_object_t* pdf_reference_create(pdf_refspec_t refspec) {
  pdf_object_t* o = pdf_object_create_type(PDF_OBJECT_REFERENCE);
  o->refspec = refspec;
  return o;
}

pdf_object_t* pdf_reference_create_values(int id, int revision) {
  return pdf_reference_create(pdf_refspec_make(id, revision));
}

void pdf_object_destroy(pdf_object_t* o) {

  if (!o){ return; }
  
  switch(o->type) {
  case PDF_OBJECT_STRING:
    free(o->string.data);
    break;
  case PDF_OBJECT_NAME:
    free(o->name.data);
    break;
  case PDF_OBJECT_ARRAY:
    for (int i=0; i<zarray_size(o->array); ++i) {
      pdf_object_t* oi;
      zarray_get(o->array, i, &oi);
      pdf_object_destroy(oi);
    }
    zarray_destroy(o->array);
    break;
  case PDF_OBJECT_DICT:
    for (int i=0; i<zarray_size(o->dict); ++i) {
      pdf_dict_entry_t keyval;
      zarray_get(o->dict, i, &keyval);
      pdf_object_destroy(keyval.key);
      pdf_object_destroy(keyval.value);
    }
    zarray_destroy(o->dict);
    break;
  case PDF_OBJECT_STREAM:
    if (o->stream.dict) { pdf_object_destroy(o->stream.dict); }
    zarray_destroy(o->stream.content);
    break;
  }
  free(o);
}


pdf_object_t* make_rect(double x0, double y0, double x1, double y1) {

  pdf_object_t* rect = pdf_array_create();

  pdf_array_append(rect, pdf_real_create(x0));
  pdf_array_append(rect, pdf_real_create(y0));
  pdf_array_append(rect, pdf_real_create(x1));
  pdf_array_append(rect, pdf_real_create(y1));

  return rect;
  
}


pdf_refspec_t pdf_add_toplevel(pdf_t* pdf, pdf_object_t* object) {

  pdf_toplevel_object_t tlo;

  tlo.refspec.id = zarray_size(pdf->toplevel_objects) + 1;
  tlo.refspec.revision = 0;
  tlo.object = object;
  
  zarray_add(pdf->toplevel_objects, &tlo);

  return tlo.refspec;
  
}

int pdf_lookup_toplevel(pdf_t* pdf, pdf_refspec_t refspec) {

  for (int i=0; i<zarray_size(pdf->toplevel_objects); ++i) {
    pdf_toplevel_object_t tlo;
    zarray_get(pdf->toplevel_objects, i, &tlo);
    if (tlo.refspec.id == refspec.id &&
        tlo.refspec.revision == refspec.revision) {
      return i;
    }
  }

  return -1;
  
}


typedef struct pdf_writer {
  pdf_write_callback callback;
  void* userdata;
  size_t bytes_written;
    jmp_buf env;
} pdf_writer_t;

void pdf_write(pdf_writer_t* w, size_t len, const char* data) {
  int status = w->callback(len, data, w->userdata);
  if (status != 0) {
      
  }
  w->bytes_written += len;
}

void pdf_write_str(pdf_writer_t* w, const char* data) {
  pdf_write(w, strlen(data), data);
}

int pdf_is_regular(char c) {
  switch (c) {
  case '(':
  case ')':
  case '<':
  case '>':
  case '[':
  case ']':
  case '{':
  case '}':
  case '/':
  case '%':
    return 0;
  default:
    return !isspace(c);
  }
}

void pdf_write_object(pdf_writer_t* w, pdf_object_t* o) {

  char buf[1024];

  switch (o->type) {
  case PDF_OBJECT_BOOLEAN:
    pdf_write_str(w, o->boolean ? "true" : "false");
    break;
  case PDF_OBJECT_INTEGER: 
    snprintf(buf, 1024, "%d",
             o->integer); // TODO: check return value
    pdf_write_str(w, buf);
    break;
  case PDF_OBJECT_REAL:
    if (o->real == floor(o->real)) {
      snprintf(buf, 1024, "%.0f",
               o->real); // TODO: check return value
    } else {
      snprintf(buf, 1024, "%f",
               o->real); // TODO: check return value
    }
    pdf_write_str(w, buf);
    break;
  case PDF_OBJECT_STRING:
    pdf_write_str(w, "(");
    for (size_t i=0; i<o->string.length; ++i) {
      char c = o->string.data[i];
      if (c != '(' && c != ')' && c != '\\') {
        pdf_write(w, 1, &c);
      } else {
        snprintf(buf, 1024, "\\%03o", (int)c);
        pdf_write(w, 4, buf);
      }
    }
    pdf_write_str(w, ")");
    break;
  case PDF_OBJECT_NAME:
    pdf_write_str(w, "/");
    for (size_t i=0; i<o->name.length; ++i) {
      char c = o->name.data[i];
      if (pdf_is_regular(c) && c != '#') {
        pdf_write(w, 1, &c);
      } else {
        snprintf(buf, 1024, "#%02x", (int)c);
        pdf_write(w, 3, buf);
      }
    }
    break;
  case PDF_OBJECT_ARRAY: 
    pdf_write_str(w, "[ ");
    for (int i=0; i<zarray_size(o->array); ++i) {
      pdf_object_t* oi;
      zarray_get(o->array, i, &oi);
      pdf_write_object(w, oi);
      pdf_write_str(w, " ");
    };
    pdf_write_str(w, "]");
    break;
  case PDF_OBJECT_DICT:
    pdf_write_str(w, "<< ");
    for (int i=0; i<zarray_size(o->dict); ++i) {
      pdf_dict_entry_t keyval;
      zarray_get(o->dict, i, &keyval);
      pdf_write_object(w, keyval.key);
      pdf_write_str(w, " ");
      pdf_write_object(w, keyval.value);
      pdf_write_str(w, " ");
    }
    pdf_write_str(w, ">>");
    break;
  case PDF_OBJECT_STREAM: {
    int len = zarray_size(o->stream.content);
    if (!o->stream.dict) {
      o->stream.dict = pdf_dict_create();
    }
    pdf_dict_insert_str(o->stream.dict, "Length", pdf_integer_create(len));
    pdf_dict_insert_str(o->stream.dict, "Length", pdf_integer_create(len));
    pdf_write_object(w, o->stream.dict);
    pdf_write_str(w, "\n");
    pdf_write_str(w, "stream\n");
    pdf_write(w, zarray_size(o->stream.content), o->stream.content->data);
    pdf_write_str(w, "\n");
    pdf_write_str(w, "endstream\n");
    break;
  }
  case PDF_OBJECT_REFERENCE:
    snprintf(buf, 1024, "%d %d R", o->refspec.id,
             o->refspec.revision); // TODO: check return value
    pdf_write_str(w, buf);
    break;
  default:
    pdf_write_str(w, "null");
    break;
  }

}

int pdf_write_full(pdf_writer_t* w, pdf_t* pdf) {

  size_t nobj = zarray_size(pdf->toplevel_objects);
  size_t toplevel_offsets[nobj];

  pdf_write_str(w, "%PDF-1.1\n");

  pdf_write_str(w, "% (\xE2\x95\xAF\xC2\xB0\xE2\x96\xA1\xC2\xB0"
                "\xEF\xBC\x89\xE2\x95\xAF\xEF\xB8\xB5 "
                "\xE2\x94\xBB\xE2\x94\x81\xE2\x94\xBB\n\n");

  char buf[1024];
  
  // write toplevel objects
  for (size_t i=0; i<nobj; ++i) {

    pdf_toplevel_object_t* tli;
    zarray_get_volatile(pdf->toplevel_objects, i, &tli);

    toplevel_offsets[i] = w->bytes_written;

    snprintf(buf, 1024, "%d %d obj\n", tli->refspec.id, tli->refspec.revision);
    pdf_write_str(w, buf);
    
    pdf_write_object(w, tli->object);
    if (tli->object->type != PDF_OBJECT_STREAM) {
      pdf_write_str(w, "\n");
    }
    pdf_write_str(w, "endobj\n\n");
    
  }

  // write xref
  size_t xref_start = w->bytes_written;
    
  pdf_write_str(w, "xref\n");
  snprintf(buf, 1024, "%d %d\n", 0, (int)nobj+1);
  pdf_write_str(w, buf);
  pdf_write_str(w, "0000000000 65535 f \n");
  for (size_t i=0; i<nobj; ++i) {
    snprintf(buf, 1024, "%010d 00000 n \n", (int)toplevel_offsets[i]);
    pdf_write_str(w, buf);
  }
  pdf_write_str(w, "trailer\n");
  pdf_write_str(w, "<< /Root 1 0 R");
  snprintf(buf, 1024, " /Size %d ", (int)nobj+1);
  pdf_write_str(w, buf);
  pdf_write_str(w, ">>\n");
  pdf_write_str(w, "startxref\n");
  snprintf(buf, 1024, "%d\n", (int)xref_start);
  pdf_write_str(w, buf);
  pdf_write_str(w, "%%EOF\n");

  return 0;
  
}
void pdf_finish_page(pdf_t* pdf) {
    
  // deal with anything left on this page
  if (pdf->cur_page) {
      if (zarray_size(pdf->gstate) > 1) {
          fprintf(stderr, "warning: the graphics state stack was unbalanced in pdf_end_page\n");
          while (zarray_size(pdf->gstate) > 1) {
              pdf_gstate_pop(pdf);
          }
      }
  }

}

pdf_gstate_t* pdf_cur_gstate(pdf_t* pdf) {

    pdf_gstate_t* cur_gstate;

    zarray_get_volatile(pdf->gstate, zarray_size(pdf->gstate)-1, &cur_gstate);

    return cur_gstate;

}

void pdf_end_page(pdf_t* pdf) {

  pdf_finish_page(pdf);

  pdf_gstate_t* base_gstate = pdf_cur_gstate(pdf);

  pdf_stroke_t old_stroke = base_gstate->stroke;
  base_gstate->stroke = pdf_default_stroke(0, 0, 0);

  pdf_fill_t old_fill = base_gstate->fill;
  base_gstate->fill = pdf_default_fill(0, 0, 0);
  
  //////////////////////////////////////////////////
  // now create the new page

  pdf->cur_page = pdf_dict_create();

  pdf_refspec_t page_ref = pdf_add_toplevel(pdf, pdf->cur_page);

  pdf_array_append(pdf->pages_array, pdf_reference_create(page_ref));
  pdf->page_count->integer++;

  pdf_dict_insert_str(pdf->cur_page, "Type", pdf_name_create_str("Page"));
  pdf_dict_insert_str(pdf->cur_page, "Parent", pdf_reference_create(pdf->pages_ref));

  pdf->cur_page_contents = pdf_stream_create();
  //pdf->cur_page_contents = pdf_stream_create_str("0.9 0.5 0.0 rg 100 400 300 300 re f q 0.1 0.9 0.5 rg 100 200 200 200 re f Q 350 200 50 50 re f");

  pdf_refspec_t contents_ref = pdf_add_toplevel(pdf, pdf->cur_page_contents);

  pdf_dict_insert_str(pdf->cur_page, "Contents", pdf_reference_create(contents_ref));

  pdf->cur_page_fonts = NULL;

  pdf_set_stroke(pdf, &old_stroke);
  pdf_set_fill(pdf, &old_fill);
  


}

pdf_t* pdf_create(double width, double height) {

  pdf_t* pdf = calloc(1, sizeof(pdf_t));

  pdf->toplevel_objects = zarray_create(sizeof(pdf_toplevel_object_t));

  pdf_object_t* catalog = pdf_dict_create();
  pdf_add_toplevel(pdf, catalog);

  pdf_object_t* pagesdict = pdf_dict_create();
  pdf->pages_ref = pdf_add_toplevel(pdf, pagesdict);

  ////////////////////////////////////////
  // catalog
  
  pdf_dict_insert_str(catalog, "Type", pdf_name_create_str("Catalog"));
  pdf_dict_insert_str(catalog, "Pages", pdf_reference_create(pdf->pages_ref));

  ////////////////////////////////////////
  // pages dict

  pdf->pages_array = pdf_array_create();
  pdf->page_count = pdf_integer_create(0);
    
  pdf_dict_insert_str(pagesdict, "Type", pdf_name_create_str("Pages"));
  pdf_dict_insert_str(pagesdict, "Kids", pdf->pages_array);
  pdf_dict_insert_str(pagesdict, "Count", pdf->page_count);
  pdf_dict_insert_str(pagesdict, "MediaBox", make_rect(0, 0, width, height));

  pdf->gstate = zarray_create(sizeof(pdf_gstate_t));

  pdf_gstate_t base_gstate;
  base_gstate.stroke = pdf_default_stroke(0, 0, 0);
  base_gstate.fill = pdf_default_fill(0, 0, 0);
  zarray_add(pdf->gstate, &base_gstate);

  pdf_end_page(pdf);

  return pdf;
  
}


void pdf_destroy(pdf_t* pdf) {

  for (int i=0; i<zarray_size(pdf->toplevel_objects); ++i) {
    pdf_toplevel_object_t* tli;
    zarray_get_volatile(pdf->toplevel_objects, i, &tli);
    pdf_object_destroy(tli->object);
  }

  zarray_destroy(pdf->toplevel_objects);

  zarray_destroy(pdf->gstate);

  free(pdf);
  
}

int pdf_stream_write(size_t len,
                     const char* data,
                     void* s) {
    
  pdf_object_t* stream  = (pdf_object_t*)s;

  pdf_stream_append(stream, len, data);

  return 0;

}

int pdf_file_write(size_t len,
                    const char* data,
                    void* fp) {
    
    return fwrite(data, len, 1, (FILE*)fp);
  
}


int pdf_save_stream(pdf_t* pdf,
                     pdf_write_callback cbk,
                     void* userdata) {

    pdf_finish_page(pdf);

  pdf_writer_t w = { cbk, userdata, 0 };
  
  int status = setjmp(w.env);

  if (status != 0) {
      return status;
  }

  return pdf_write_full(&w, pdf);
  
}

int pdf_save(pdf_t* pdf,
             const char* filename) {

    FILE* fp = fopen(filename, "wb");
    if (!fp) { return -1; }
    
    int status = pdf_save_stream(pdf, pdf_file_write, fp);
    
    fclose(fp);

    return status;
    
}


const char* pdf_font_name(pdf_font_t font) {
  switch (font) {
  case PDF_FONT_COURIER:
    return "Courier";
  case PDF_FONT_COURIER_BOLD:
    return "Courier-Bold";
  case PDF_FONT_COURIER_ITALIC:
    return "Courier-Oblique";
  case PDF_FONT_COURIER_BOLD_ITALIC:
    return "Courier-BoldOblique";
  case PDF_FONT_HELVETICA:
    return "Helvetica";
  case PDF_FONT_HELVETICA_BOLD:
    return "Helvetica-Bold";
  case PDF_FONT_HELVETICA_ITALIC:
    return "Helvetica-Oblique";
  case PDF_FONT_HELVETICA_BOLD_ITALIC: 
    return "Helvetica-BoldOblique";
  case PDF_FONT_TIMES:
    return "Times-Roman";
  case PDF_FONT_TIMES_BOLD:
    return "Times-Bold";
  case PDF_FONT_TIMES_ITALIC:
    return "Times-Italic";
  case PDF_FONT_TIMES_BOLD_ITALIC:
    return "Times-BoldItalic";
  case PDF_FONT_SYMBOL:
    return "Symbol";
  case PDF_FONT_ZAPF_DINGBATS:
    return "ZapfDingbats";
  }
}

void pdf_stream_append_object(pdf_object_t* s, pdf_object_t* o) {

  assert(s->type == PDF_OBJECT_STREAM);

  pdf_writer_t w = { pdf_stream_write, s, 0 };

  pdf_write_object(&w, o);
  
}
                              
void pdf_stream_append_boolean(pdf_object_t* s, int val) {
  pdf_object_t o;
  o.type = PDF_OBJECT_BOOLEAN;
  o.boolean = val;
  pdf_stream_append_object(s, &o);
}

void pdf_stream_append_integer(pdf_object_t* s, int val) {
  pdf_object_t o;
  o.type = PDF_OBJECT_INTEGER;
  o.integer = val;
  pdf_stream_append_object(s, &o);
}

void pdf_stream_append_real(pdf_object_t* s, double val) {
  pdf_object_t o;
  o.type = PDF_OBJECT_REAL;
  o.real = val;
  pdf_stream_append_object(s, &o);
}

void pdf_stream_append_reals(pdf_object_t* s, const double* vals, int count) {
    for (int i=0; i<count; ++i) {
        if (i) { pdf_stream_append_str(s, " "); }
        pdf_stream_append_real(s, vals[i]);
    }
}

void pdf_stream_append_name(pdf_object_t* s, const char* val) {
  pdf_object_t o;
  o.type = PDF_OBJECT_NAME;
  o.name.data = (char*)val;
  o.name.length = strlen(val);
  pdf_stream_append_object(s, &o);
}

void pdf_stream_append_string(pdf_object_t* s, const char* val) {
  pdf_object_t o;
  o.type = PDF_OBJECT_STRING;
  o.string.data = (char*)val;
  o.string.length = strlen(val);
  pdf_stream_append_object(s, &o);
}


void pdf_cur_page_newline(pdf_t* pdf) {

  zarray_t* sc = pdf->cur_page_contents->stream.content;
  
  if (sc->size &&
      !isspace(sc->data[sc->size-1])) {
    pdf_stream_append_str(pdf->cur_page_contents, "\n");
  }

}

void pdf_image_gray(pdf_t* pdf,
                    double samples_per_pt,
                    double x, double y,
                    const image_u8_t* image) {

  pdf_object_t* idict = pdf_dict_create();
  pdf_dict_insert_str(idict, "Type", pdf_name_create_str("XObject"));
  pdf_dict_insert_str(idict, "Subtype", pdf_name_create_str("Image"));
  pdf_dict_insert_str(idict, "Width", pdf_integer_create(image->width));
  pdf_dict_insert_str(idict, "Height", pdf_integer_create(image->height));
  pdf_dict_insert_str(idict, "ColorSpace", pdf_name_create_str("DeviceGray"));
  pdf_dict_insert_str(idict, "Interpolate", pdf_boolean_create(0));
  pdf_dict_insert_str(idict, "BitsPerComponent", pdf_integer_create(8));

  pdf_object_t* xobj = pdf_stream_create();
  pdf_stream_set_dict(xobj, idict);

  const char* src = (const char*)image->buf;

  for (int y=0; y<image->height; ++y) {
    pdf_stream_append(xobj, image->width, src);
    src += image->stride;
  }
  
  pdf_refspec_t r = pdf_add_toplevel(pdf, xobj);

  pdf_object_t* resources = pdf_dict_lookup_str_create(pdf->cur_page, "Resources",
                                                       pdf_dict_create);

  pdf_object_t* procset = pdf_dict_lookup_str_create(resources, "ProcSet",
                                                     pdf_array_create);

  pdf_array_append_unique(procset, pdf_name_create_str("PDF"));
  pdf_array_append_unique(procset, pdf_name_create_str("ImageB"));

  pdf_object_t* xdict = pdf_dict_lookup_str_create(resources, "XObject",
                                                   pdf_dict_create);

  char oname[1024];
  snprintf(oname, 1024, "Im%d", zarray_size(xdict->dict)+1);

  pdf_dict_insert_str(xdict, oname, pdf_reference_create(r));

  double cm[6] = {
      samples_per_pt*image->width, 0,
      0, samples_per_pt*image->height,
      x, y
  };
  
  pdf_cur_page_newline(pdf);
  pdf_stream_append_str(pdf->cur_page_contents, "q ");
  pdf_stream_append_reals(pdf->cur_page_contents, cm, 6);
  pdf_stream_append_str(pdf->cur_page_contents, " cm ");
  pdf_stream_append_name(pdf->cur_page_contents, oname);
  pdf_stream_append_str(pdf->cur_page_contents, " Do ");
  pdf_stream_append_str(pdf->cur_page_contents, "Q");

  
  
}

void pdf_text(pdf_t* pdf, pdf_font_t font,
              double size, double tx, double ty,
              const char* string) {

  if ((int)font < 0 || (int)font >= 14) {
    return;
  }

  char fstr[1024];
  snprintf(fstr, 1024, "F%X", (int)font+1);

  // check out the current pages resources to see if this font is in there
  pdf_object_t* resources = pdf_dict_lookup_str_create(pdf->cur_page, "Resources",
                                                       pdf_dict_create);

  if (!pdf->cur_page_fonts) {
    pdf->cur_page_fonts = pdf_dict_create();
    pdf_dict_insert_str(resources, "Font", pdf->cur_page_fonts);
  }

  pdf_refspec_t fr = pdf->fonts[font];
  if (fr.id == 0) {
    pdf_object_t* fdict = pdf_dict_create();
    pdf->fonts[font] = fr = pdf_add_toplevel(pdf, fdict);
    pdf_dict_insert_str(fdict, "Type", pdf_name_create_str("Font"));
    pdf_dict_insert_str(fdict, "Subtype", pdf_name_create_str("Type1"));
    pdf_dict_insert_str(fdict, "BaseFont", pdf_name_create_str(pdf_font_name(font)));
  }


  if (pdf_dict_lookup_str(pdf->cur_page_fonts, fstr) < 0) {
    pdf_dict_insert_str(pdf->cur_page_fonts, fstr,
                        pdf_reference_create(fr));
  }

  pdf_cur_page_newline(pdf);

  pdf_stream_append_str(pdf->cur_page_contents, "BT ");
  pdf_stream_append_name(pdf->cur_page_contents, fstr);
  pdf_stream_append_str(pdf->cur_page_contents, " ");
  pdf_stream_append_real(pdf->cur_page_contents, size);
  pdf_stream_append_str(pdf->cur_page_contents, " Tf ");
  pdf_stream_append_real(pdf->cur_page_contents, tx);
  pdf_stream_append_str(pdf->cur_page_contents, " ");
  pdf_stream_append_real(pdf->cur_page_contents, ty);
  pdf_stream_append_str(pdf->cur_page_contents, " Td ");
  pdf_stream_append_string(pdf->cur_page_contents, string);
  pdf_stream_append_str(pdf->cur_page_contents, " Tj ET");


}

void pdf_path_rect(zarray_t* path, double x, double y, double w, double h) {
    pdf_path_element_t element = { PDF_PATH_TYPE_RECT, { x, y, w, h } };
    zarray_add(path, &element);
}

void pdf_path_move_to(zarray_t* path, double x, double y) {
    pdf_path_element_t element = { PDF_PATH_TYPE_MOVETO, { x, y } };
    zarray_add(path, &element);
}

void pdf_path_line_to(zarray_t* path, double x, double y) {
    pdf_path_element_t element = { PDF_PATH_TYPE_LINETO, { x, y } };
    zarray_add(path, &element);
}

void pdf_path_close(zarray_t* path) {
    pdf_path_element_t element = { PDF_PATH_TYPE_CLOSE, { 0, 0 } };
    zarray_add(path, &element);
}

void pdf_set_color(pdf_t* pdf,
                   double cur_rgb[3],
                   const double new_rgb[3],
                   const char* op) {
    
    size_t sz = sizeof(double)*3;


    if (memcmp(cur_rgb, new_rgb, sz) == 0) { return; }

    pdf_cur_page_newline(pdf);
    pdf_stream_append_reals(pdf->cur_page_contents, new_rgb, 3);
    pdf_stream_append_str(pdf->cur_page_contents, op);
    memcpy(cur_rgb, new_rgb, sz);

}

    

void pdf_set_stroke(pdf_t* pdf,
                    const pdf_stroke_t* s_new) {

    pdf_gstate_t* cur_gstate = pdf_cur_gstate(pdf);
    pdf_stroke_t* s_cur = &cur_gstate->stroke;

    pdf_set_color(pdf,
                  s_cur->color_rgb,
                  s_new->color_rgb,
                  " RG\n");
    
    if (s_new->width != s_cur->width) {
        pdf_cur_page_newline(pdf);
        pdf_stream_append_real(pdf->cur_page_contents, s_new->width);
        pdf_stream_append_str(pdf->cur_page_contents, " w ");
        s_cur->width = s_new->width;
    }
    
    if (s_new->cap != s_cur->cap) {
        pdf_cur_page_newline(pdf);
        pdf_stream_append_integer(pdf->cur_page_contents, (int)s_new->cap);
        pdf_stream_append_str(pdf->cur_page_contents, " J ");
        s_cur->cap = s_new->cap;
    }
    
    if (s_new->join != s_cur->join) {
        pdf_cur_page_newline(pdf);
        pdf_stream_append_integer(pdf->cur_page_contents, (int)s_new->join);
        pdf_stream_append_str(pdf->cur_page_contents, " j\n");
        s_cur->join = s_new->join;
    }
    
}

void pdf_set_fill(pdf_t* pdf,
                  const pdf_fill_t* f_new) {

    pdf_gstate_t* cur_gstate = pdf_cur_gstate(pdf);
    pdf_fill_t* f_cur = &cur_gstate->fill;

    pdf_set_color(pdf,
                  f_cur->color_rgb,
                  f_new->color_rgb,
                  " rg\n");

}

void pdf_path_draw(pdf_t* pdf,
                   zarray_t* path,
                   pdf_stroke_type_t stroke_type,
                   pdf_fill_type_t fill_type) {

    if (stroke_type == PDF_STROKE_NONE &&
        fill_type == PDF_FILL_NONE) {
        return;
    }

    pdf_cur_page_newline(pdf);

    for (int i=0; i<zarray_size(path); ++i) {

        pdf_path_element_t element;
        zarray_get(path, i, &element);

        if (i) {
            pdf_stream_append_str(pdf->cur_page_contents, " ");
        }

        switch (element.type) {
        case PDF_PATH_TYPE_RECT:
            pdf_stream_append_reals(pdf->cur_page_contents, element.xydata, 4);
            pdf_stream_append_str(pdf->cur_page_contents, " re");
            break;
        case PDF_PATH_TYPE_MOVETO:
        case PDF_PATH_TYPE_LINETO:
            pdf_stream_append_reals(pdf->cur_page_contents, element.xydata, 2);
            pdf_stream_append_str(pdf->cur_page_contents,
                                  (element.type == PDF_PATH_TYPE_MOVETO ? " m" : " l"));
            break;
        case PDF_PATH_TYPE_CLOSE:
            pdf_stream_append_str(pdf->cur_page_contents, "h");
            break;
        default:
            fprintf(stderr, "warning pdf path element %d has invalid type %d\n",
                    i, (int)element.type);
            break;
        }
        
    }

    const char* op = NULL;

    if (stroke_type == PDF_STROKE_CLOSED) { // closed stroke
        if (fill_type == PDF_FILL_NONZERO) {
            op = "b";
        } else if (fill_type == PDF_FILL_EVENODD) {
            op = "b*";
        } else  {
            op = "s";
        }
    } else if (stroke_type == PDF_STROKE_REGULAR) {
        if (fill_type == PDF_FILL_NONZERO) {
            op = "B";
        } else if (fill_type == PDF_FILL_EVENODD) {
            op = "B*";
        } else { // unclosed stroke only
            op = "S";
        }
    } else { // fill only
        if (fill_type == PDF_FILL_NONZERO) {
            op = "f";
        } else {
            op = "f*";
        }
    }

    pdf_stream_append_str(pdf->cur_page_contents, " ");
    pdf_stream_append_str(pdf->cur_page_contents, op);

    pdf_path_destroy(path);
    
}

void pdf_set_rgb(double rgb[3], double r, double g, double b) {
    rgb[0] = r;
    rgb[1] = g;
    rgb[2] = b;
}


pdf_stroke_t pdf_default_stroke(double r, double g, double b) {
    
    static pdf_stroke_t stroke = {
        { 0, 0, 0 },
        1.0,
        PDF_CAP_BUTT,
        PDF_JOIN_MITER,
    };

    pdf_set_rgb(stroke.color_rgb, r, g, b);

    return stroke;

}
        
pdf_fill_t pdf_default_fill(double r, double g, double b) {

    static pdf_fill_t fill = {
        { 0, 0, 0 },
    };

    pdf_set_rgb(fill.color_rgb, r, g, b);

    return fill;

}
    
zarray_t* pdf_path_create() {
    return zarray_create(sizeof(pdf_path_element_t));
}

zarray_t* pdf_path_copy(zarray_t* path) {
    return zarray_copy(path);
}

void pdf_path_destroy(zarray_t* path) {
    zarray_destroy(path);
}


void pdf_gstate_push(pdf_t* pdf);
void pdf_gstate_pop(pdf_t* pdf);

void pdf_ctm_concat(pdf_t* pdf,
                    double xx, double yx, 
                    double xy, double yy,
                    double tx, double ty) {

    double cm[6] = {
        xx, yx,
        xy, yy,
        tx, ty
    };

    pdf_cur_page_newline(pdf);
    pdf_stream_append_reals(pdf->cur_page_contents, cm, 6); 
    pdf_stream_append_str(pdf->cur_page_contents, " cm");
   
}

void pdf_gstate_push(pdf_t* pdf) {

    pdf_cur_page_newline(pdf);
    pdf_stream_append_str(pdf->cur_page_contents, "q");

    pdf_gstate_t* cur_gstate = pdf_cur_gstate(pdf);
    zarray_add(pdf->gstate, &cur_gstate);
    
}

void pdf_gstate_pop(pdf_t* pdf) {

    if (zarray_size(pdf->gstate) <= 1) {
        fprintf(stderr, "warning: unbalanced gstate stack in pdf_gstate_pop!\n");
    } else {
        zarray_truncate(pdf->gstate,
                        zarray_size(pdf->gstate)-1);
    }
    
    pdf_cur_page_newline(pdf);
    pdf_stream_append_str(pdf->cur_page_contents, "Q");
    
}

