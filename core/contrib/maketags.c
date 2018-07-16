#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>

#include "apriltag.h"
#include "apriltag_family.h"
#include "apriltag_vis.h"
#include "image_u8.h"
#include "contour.h"

#include "zarray.h"
#include "getopt.h"

#include "pdfutil.h"

typedef struct unit {
    const char* name;
    double points;
} unit_t;

static const unit_t units[] = {
  { "pt", 1 },
  { "in", 72 },
  { "ft", 72*12 },
  { "cm", 28.346456693 },
  { "mm", 2.8346456693 },
  { "cm", 28.346456693 },
  { "m", 283.46456693 },
  { NULL, 0 }
};

const unit_t* lookup_unit(const char* unit) {

    for (int i=0; units[i].name; ++i) {
        if (!strcmp(unit, units[i].name)) {
            return units+i;
        }
    }

    fprintf(stderr, "error: unknown unit %s\n", unit);
    exit(1);
    
}

typedef struct paper {
    const char* name;
    double width, height;
    const char* unit_name;
} paper_t;


static const paper_t papers[] = {
    { "letter", 612, 792, "in" },
    { "tabloid", 792, 1224, "in" },
    { "a6", 297.5, 419.6, "mm" },
    { "a5", 419.3, 595.4, "mm" },
    { "a4", 595, 842, "mm" },
    { "a3", 841.5, 1190.7, "mm" },
    { "a2", 1190, 1684, "mm" },
    { "a1", 1683, 2384.2, "mm" },
    { "a0", 2382.8, 3370.8, "mm" },
    { NULL, 0, 0 }
};
    
const unit_t* lookup_paper(const char* paper, double paper_dims[2]) {

    for (int i=0; papers[i].name; ++i) {
        if (!strcmp(paper, papers[i].name)) {
            paper_dims[0] = papers[i].width;
            paper_dims[1] = papers[i].height;
            return lookup_unit(papers[i].unit_name);
        }
    }

    fprintf(stderr, "unknown paper type %s\n\n", paper);
    fprintf(stderr, "choose one of: ");
    for (int i=0; papers[i].name; ++i) {
        fprintf(stderr, "%s%s",
                i == 0 ? "" : ", ",
                papers[i].name);
    }
    fprintf(stderr, "\n");
    
    exit(1);
    
}

const unit_t* parse_unit(const char* meas, double* quantity) {
    double value;
    char unit[6];
    if (sscanf(meas, "%lf %5s", &value, unit) != 2) {
        fprintf(stderr, "error parsing measurement: %s\n", meas);
        exit(1);
    }
    const unit_t* u = lookup_unit(unit);
    *quantity = value * u->points;
    return u;
}

const unit_t* parse_size(const char* mpair_orig, double dims[2]) {

    char* mpair = strdup(mpair_orig);
    char* comma = strchr(mpair, ',');

    if (!comma || *comma != ',') {
        fprintf(stderr, "error parsing size: %s\n", mpair);
        exit(1);
    }

    *comma = 0;

    parse_unit(mpair, dims+0);
    const unit_t* rval = parse_unit(comma+1, dims+1);

    free(mpair);

    return rval;

}

typedef struct options {

    apriltag_family_t* family;

    uint32_t max_id;
    
    double paper_dims[2];
    int base_px;

    const char* tagsize_str;
    double tagsize;
    double px;
    
    double border;

    double fontsize;
    
    int grid_dims[2];
    
    double tag_step[2];
    
    double p0[2];
    
    double lgray;
    
    const char* labelfmt_str;
    const char* output_filename;

    const unit_t* paper_unit;
    const unit_t* tag_unit;
    const unit_t* font_unit;

    char tagsize_buf[1024];
    
} options_t;

void get_options(int argc, char** argv, options_t* opts) {

    getopt_t* getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, 'M', "maxid", "0", "Maximum # of tags to output (0 = include all)");

    getopt_add_string(getopt, 'p', "paper", "", "Paper size (choose from letter, tabloid, a6, a5, a4, a3, a2, a1, a0)");
    getopt_add_string(getopt, 'd', "paperdims", "", "Paper dimensions (e.g. 8.5in,11in)");
    getopt_add_bool(getopt, 'L', "landscape", 0, "Landscape mode (swap paper dims)");
    
    getopt_add_double(getopt, 'm', "margin", "0.25in", "Margin around paper edges");
    getopt_add_string(getopt, 't', "tagsize", "", "Tag size (including black border)");
    getopt_add_double(getopt, 'b', "border", "1", "White border around tags (measured in pixels)");
    getopt_add_bool(getopt, 'l', "label", 0, "Include labels");
    getopt_add_string(getopt, 'F', "labelfmt", "%d", "Label format (%d=id, %f=family, %t=tagsize)");
    getopt_add_string(getopt, 's', "fontsize", "12pt", "Font size for labels");
    getopt_add_double(getopt, 'g', "labelgray", "0.75", "Label grayscale (0=black, 1=white)");
    getopt_add_double(getopt, 'o', "output", "tags.pdf", "Output filename");

    if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options] <input files>\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    const char* family_str = getopt_get_string(getopt, "family");
    const char* paper_str = getopt_get_string(getopt, "paper");
    const char* paperdims_str = getopt_get_string(getopt, "paperdims");
    int is_landscape = getopt_get_bool(getopt, "landscape");

    opts->paper_unit = opts->tag_unit = opts->font_unit = units + 0;

    opts->max_id = getopt_get_int(getopt, "maxid");   
    
    opts->tagsize_str = getopt_get_string(getopt, "tagsize");
    
    opts->labelfmt_str = getopt_get_string(getopt, "labelfmt");

    double margin;
    parse_unit(getopt_get_string(getopt, "margin"), &margin);

    opts->output_filename = getopt_get_string(getopt, "output");

    opts->font_unit = parse_unit(getopt_get_string(getopt, "fontsize"), &opts->fontsize);
    
    opts->lgray = getopt_get_double(getopt, "labelgray");
    
    double border_px = getopt_get_double(getopt, "border");

    int do_labels = getopt_get_bool(getopt, "label");
    if (!do_labels) { opts->fontsize = 0; }

    //////////////////////////////////////////////////

    opts->family = apriltag_family_create(family_str);

    if (!opts->family) {
        fprintf(stderr, "unknown tag family %s\n\n", family_str);
        fprintf(stderr, "choose one of: ");
        zarray_t* families = apriltag_family_list();
        for (int i=0; i<zarray_size(families); ++i) {
            char* str;
            zarray_get(families, i, &str);
            fprintf(stderr, "%s%s", (i == 0 ? ""  : ", "), str);
        }
        fprintf(stderr, "\n");
        exit(1);
    }

    if (!opts->max_id || opts->max_id > opts->family->ncodes) {
        opts->max_id = opts->family->ncodes;
    }

    if (*paper_str) {
        opts->paper_unit = lookup_paper(paper_str, opts->paper_dims);
        if (*paperdims_str) {
            fprintf(stderr, "warning: ignoring paperdims because paper was set!\n");
        }
    } else if (*paperdims_str) {
        opts->paper_unit = parse_size(paperdims_str, opts->paper_dims);
    } else {
        opts->paper_unit = lookup_paper(papers[0].name, opts->paper_dims);
    }

    if (is_landscape) {
        double tmp = opts->paper_dims[0];
        opts->paper_dims[0] = opts->paper_dims[1];
        opts->paper_dims[1] = tmp;
    }

    opts->base_px = opts->family->d + 2*opts->family->black_border;

    if (*opts->tagsize_str) {
            
        opts->tag_unit = parse_unit(opts->tagsize_str, &opts->tagsize);
        opts->px = opts->tagsize / opts->base_px;
        opts->border = border_px * opts->px;
            
    } else {
            
        double total_px = opts->base_px + 2*border_px;

        opts->tag_unit = opts->paper_unit;
            
        opts->px = opts->paper_dims[0] / total_px;
        opts->border = border_px * opts->px;
        opts->tagsize = opts->paper_dims[0] - 2*opts->border;

        snprintf(opts->tagsize_buf, 1024, "%g %s",
                 opts->tagsize / opts->tag_unit->points,
                 opts->tag_unit->name);
        
        opts->tagsize_str = opts->tagsize_buf;
            
    }
    
    const double dir[2] = { 1, -1 };

    opts->tag_step[0] = opts->tagsize + opts->border * 2;
    opts->tag_step[1] = opts->tag_step[0] + opts->fontsize;
    
    for (int i=0; i<2; ++i) {

        double d = opts->paper_dims[i] - 2 * margin;
        int cnt = floor(d / opts->tag_step[i]);
        if (cnt < 1) { cnt = 1; }
        
        opts->grid_dims[i] = cnt;
        
        opts->p0[i] = ( 0.5*opts->paper_dims[i] -
                        dir[i]*0.5*(cnt*opts->tag_step[i]) );

    }

    printf("using tag family %s of size %d\n",
           family_str, opts->family->d);
    
    printf("using paper size of (%g %s, %g %s) with margins of %g pt\n",
           opts->paper_dims[0] / opts->paper_unit->points,
           opts->paper_unit->name,
           opts->paper_dims[1] / opts->paper_unit->points,
           opts->paper_unit->name,
           margin);

    printf("tag size with border is %g %s\n",
           opts->tagsize / opts->tag_unit->points,
           opts->tag_unit->name);
    
    printf("output will have %d rows x %d cols\n",
           opts->grid_dims[0], opts->grid_dims[1]);

    if (opts->fontsize) {
        printf("showing labels at font size %g %s with format \"%s\"\n",
               opts->fontsize / opts->font_unit->points,
               opts->font_unit->name,
               opts->labelfmt_str);
    
    }
    
    printf("will output to %s\n", opts->output_filename);
    

}

void format_label(char* buf, size_t count,
                  const char* fmt,
                  unsigned int idx,
                  const char* name,
                  const char* size) {

    char* end = buf + count-1;

    while (buf < end) {
        
        if (!*fmt) { break; }
        
        if (*fmt == '%') {
            
            const char* to_append = 0;
            char tmp[1024];

            ++fmt;

            if (*fmt == 'd') {
                snprintf(tmp, 1024, "%u", idx);
                to_append = tmp;
            } else if (*fmt == 't') {
                to_append = size;
            } else if (*fmt == 'f') {
                to_append = name;
            } else if (*fmt == '%') {
                to_append = "%";
            } else {
                to_append = "%";
                --fmt;
            }

            while (buf < end && *to_append) {
                *buf++ = *to_append++;
            }
            
        } else {
            
            *buf++ = *fmt;
            
        }

        ++fmt;
        
    }

    *buf = 0;

}

int main(int argc, char** argv) {

    options_t opts;
    get_options(argc, argv, &opts);
    
    pdf_t* pdf = pdf_create(opts.paper_dims[0], opts.paper_dims[1]);

    int row = 0;
    int col = 0;

    pdf_fill_t label_fill =
        pdf_default_fill(opts.lgray, opts.lgray, opts.lgray);

    for (uint32_t i=0; i<opts.max_id; ++i) {

        if (i) {
            col += 1;
            if (col >= opts.grid_dims[0]) {
                col = 0;
                row += 1;
                if (row >= opts.grid_dims[1]) {
                    row = 0;
                    pdf_end_page(pdf);
                }
            }
        }
        
        uint64_t code = opts.family->codes[i];

        image_u8_t* image = apriltag_vis_texture2(opts.family, code, 0, 255, 0);

        zarray_t* contours = contour_line_sweep(image);

        double tx = opts.p0[0] + col*opts.tag_step[0];
        double ty = opts.p0[1] - (row+1)*opts.tag_step[1];

        pdf_gstate_push(pdf);

        if (opts.fontsize) {

            char label_string[1024];

            format_label(label_string, 1024,
                         opts.labelfmt_str,
                         i, opts.family->name, opts.tagsize_str);

            printf("label_string = %s\n", label_string);
                         
            pdf_gstate_push(pdf);
            pdf_set_fill(pdf, &label_fill);
            pdf_text(pdf, PDF_FONT_HELVETICA, opts.fontsize,
                     tx+opts.border, ty, label_string);
            pdf_gstate_pop(pdf);
            
        }

        pdf_ctm_concat(pdf, opts.px, 0, 0, opts.px,
                       tx+opts.border, ty+opts.border+opts.fontsize);
        
        zarray_t* path = pdf_path_create();
        pdf_path_rect(path, 0, 0, opts.base_px, opts.base_px);
    
        for (int i=0; i<zarray_size(contours); ++i) {

            zarray_t* contour;
            zarray_get(contours, i, &contour);

            for (int j=0; j<zarray_size(contour); ++j) {
                
                const contour_point_t* pj;
                zarray_get_volatile(contour, j, &pj);
                
                int py = image->height - pj->y;
                
                if (j == 0) {
                    pdf_path_move_to(path, pj->x, py);
                } else {
                    pdf_path_line_to(path, pj->x, py);
                }
                
            }
            
            pdf_path_close(path);

        }

        pdf_path_draw(pdf, path, PDF_STROKE_NONE, PDF_FILL_NONZERO);
        pdf_gstate_pop(pdf);

        contour_line_sweep_destroy(contours);
        image_u8_destroy(image);
        
    }

    pdf_save(pdf, opts.output_filename);
    
    return 0;

}
