// SPDX-License-Identifier
// Copyright (c) 2019 David Lechner
// Copyright (c) 2013, 2014 Damien P. George

// class Image
//
// Image manipulation on ev3dev using the GRX3 graphics library. This can be
// used for both in-memory images and writing directly to the screen.

#include <string.h>

#include <grx-3.0.h>

#include <pbio/light.h>

#include "py/mpconfig.h"
#include "py/misc.h"
#include "py/mpprint.h"
#include "py/obj.h"
#include "py/runtime.h"

#include "pb_ev3dev_types.h"
#include "modparameters.h"
#include "pbkwarg.h"

typedef struct _ev3dev_Image_obj_t {
    mp_obj_base_t base;
    mp_obj_t width;
    mp_obj_t height;
    gboolean cleared;
    GrxContext *context;
    GrxTextOptions *text_options;
    gint print_x;
    gint print_y;
} ev3dev_Image_obj_t;

// map Pybricks color enum to GRX color value using standard web CSS values
STATIC GrxColor map_color(mp_obj_t *obj) {
    if (obj == mp_const_none) {
        return GRX_COLOR_NONE;
    }

    pbio_light_color_t color = pb_type_enum_get_value(obj, &pb_enum_type_Color);

    switch (color) {
    case PBIO_LIGHT_COLOR_NONE:
        return GRX_COLOR_NONE;
    case PBIO_LIGHT_COLOR_BLACK:
        return GRX_COLOR_BLACK;
    case PBIO_LIGHT_COLOR_BLUE:
        return grx_color_get(0, 0, 255);
    case PBIO_LIGHT_COLOR_GREEN:
        return grx_color_get(0, 128, 0);
    case PBIO_LIGHT_COLOR_YELLOW:
        return grx_color_get(255, 255, 0);
    case PBIO_LIGHT_COLOR_RED:
        return grx_color_get(255, 0, 0);
    case PBIO_LIGHT_COLOR_WHITE:
        return GRX_COLOR_WHITE;
    case PBIO_LIGHT_COLOR_BROWN:
        return grx_color_get(165, 42, 42);
    case PBIO_LIGHT_COLOR_ORANGE:
        return grx_color_get(255, 165, 0);
    case PBIO_LIGHT_COLOR_PURPLE:
        return grx_color_get(128, 0, 128);
    }
    return grx_color_get_black();
}

STATIC mp_obj_t ev3dev_Image_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    if (n_args < 1) {
        mp_raise_TypeError("Requires one argument");
    }

    enum { ARG_sub, ARG_x1, ARG_y1, ARG_x2, ARG_y2 };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_sub, MP_ARG_BOOL | MP_ARG_KW_ONLY, {.u_bool = FALSE} },
        { MP_QSTR_x1, MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = 0} },
        { MP_QSTR_y1, MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = 0} },
        { MP_QSTR_x2, MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = 0} },
        { MP_QSTR_y2, MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = 0} },
    };

    mp_arg_val_t arg_vals[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args - 1, n_kw, &args[1], MP_ARRAY_SIZE(allowed_args), allowed_args, arg_vals);

    GrxContext *context = NULL;

    if (mp_obj_is_qstr(args[0]) && MP_OBJ_QSTR_VALUE(args[0]) == MP_QSTR__screen_) {
        // special case '_screen_' creates image that draws directly to screen
        context = grx_context_ref(grx_get_screen_context());
    }
    else if (mp_obj_is_str(args[0])) {
        const char *filename = mp_obj_str_get_str(args[0]);

        // add file extension if missing
        char *filename_ext = NULL;
        if (!g_str_has_suffix(filename, ".png") && !g_str_has_suffix(filename, ".PNG")) {
            filename_ext = g_strconcat(filename, ".png", NULL);
            filename = filename_ext;
        }

        gint w, h;
        if (!grx_query_png_file(filename, &w, &h)) {
            mp_obj_t ex = mp_obj_new_exception_msg_varg(&mp_type_OSError,
                "'%s' is not a .png file", filename);
            g_free(filename_ext);
            nlr_raise(ex);
        }

        context = grx_context_new(w, h, NULL, NULL);
        if (!context) {
            g_free(filename_ext);
            mp_raise_msg(&mp_type_RuntimeError, "failed to allocate context for image");
        }

        GError *error = NULL;
        if (!grx_context_load_from_png(context, filename, FALSE, &error)) {
            mp_obj_t ex = mp_obj_new_exception_msg_varg(&mp_type_OSError,
                "Failed to load '%s': %s", filename, error->message);
            g_free(filename_ext);
            g_error_free(error);
            nlr_raise(ex);
        }

        g_free(filename_ext);
    }
    else if (mp_obj_is_type(args[0], &pb_type_ev3dev_Image)) {
        ev3dev_Image_obj_t *image = MP_OBJ_TO_PTR(args[0]);
        if (arg_vals[ARG_sub].u_bool) {
            context = grx_context_new_subcontext(arg_vals[ARG_x1].u_int, arg_vals[ARG_y1].u_int,
                arg_vals[ARG_x2].u_int, arg_vals[ARG_y2].u_int, image->context, NULL);
        }
        else {
            gint w = grx_context_get_width(image->context);
            gint h = grx_context_get_height(image->context);
            context = grx_context_new(w, h, NULL, NULL);
            grx_context_bit_blt(context, 0, 0, image->context, 0, 0,
                w - 1, h - 1, GRX_COLOR_MODE_WRITE);
        }
    }

    if (!context) {
        mp_raise_TypeError("Argument must be str or Image");
    }

    ev3dev_Image_obj_t *self = m_new_obj_with_finaliser(ev3dev_Image_obj_t);

    self->base.type = &pb_type_ev3dev_Image;
    self->context = context;
    self->width = mp_obj_new_int(grx_context_get_width(self->context));
    self->height = mp_obj_new_int(grx_context_get_height(self->context));

    pb_type_ev3dev_Font_init();
    GrxFont *font = pb_ev3dev_Font_obj_get_font(pb_const_ev3dev_font_DEFAULT);
    self->text_options = grx_text_options_new(font, GRX_COLOR_BLACK);

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t ev3dev_Image___del__(mp_obj_t self_in) {
    ev3dev_Image_obj_t *self = MP_OBJ_TO_PTR(self_in);
    grx_text_options_unref(self->text_options);
    grx_context_unref(self->context);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(ev3dev_Image___del___obj, ev3dev_Image___del__);

// Ensure that screen has been cleared before we start drawing anything else
STATIC void clear_once(ev3dev_Image_obj_t *self) {
    if (self->cleared) {
        return;
    }

    GrxContext *screen = grx_get_screen_context();
    if (self->context == screen || self->context->root == screen) {
        // HACK: stop the startup animation from pbinit
        extern void pbricks_end_startup_animation();
        pbricks_end_startup_animation();
    }

    grx_context_clear(self->context, GRX_COLOR_WHITE);
    self->cleared = TRUE;
}

STATIC mp_obj_t ev3dev_Image_clear(mp_obj_t self_in) {
    ev3dev_Image_obj_t *self = MP_OBJ_TO_PTR(self_in);
    clear_once(self);
    grx_context_clear(self->context, GRX_COLOR_WHITE);
    self->print_x = 0;
    self->print_y = 0;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(ev3dev_Image_clear_obj, ev3dev_Image_clear);

STATIC mp_obj_t ev3dev_Image_draw_pixel(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    PB_PARSE_ARGS_METHOD(n_args, pos_args, kw_args,
        ev3dev_Image_obj_t, self,
        PB_ARG_REQUIRED(x),
        PB_ARG_REQUIRED(y),
        PB_ARG_DEFAULT_ENUM(color, pb_const_black)
    );

    mp_int_t _x = mp_obj_get_int(x);
    mp_int_t _y = mp_obj_get_int(y);
    GrxColor _color = map_color(color);

    clear_once(self);
    grx_set_current_context(self->context);
    grx_draw_pixel(_x, _y, _color);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ev3dev_Image_draw_pixel_obj, 1, ev3dev_Image_draw_pixel);

STATIC mp_obj_t ev3dev_Image_draw_line(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    PB_PARSE_ARGS_METHOD(n_args, pos_args, kw_args,
        ev3dev_Image_obj_t, self,
        PB_ARG_REQUIRED(x1),
        PB_ARG_REQUIRED(y1),
        PB_ARG_REQUIRED(x2),
        PB_ARG_REQUIRED(y2),
        PB_ARG_DEFAULT_ENUM(color, pb_const_black)
    );

    mp_int_t _x1 = mp_obj_get_int(x1);
    mp_int_t _y1 = mp_obj_get_int(y1);
    mp_int_t _x2 = mp_obj_get_int(x2);
    mp_int_t _y2 = mp_obj_get_int(y2);
    GrxColor _color = map_color(color);

    clear_once(self);
    grx_set_current_context(self->context);
    grx_draw_line(_x1, _y1, _x2, _y2, _color);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ev3dev_Image_draw_line_obj, 1, ev3dev_Image_draw_line);

STATIC mp_obj_t ev3dev_Image_draw_box(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    PB_PARSE_ARGS_METHOD(n_args, pos_args, kw_args,
        ev3dev_Image_obj_t, self,
        PB_ARG_REQUIRED(x1),
        PB_ARG_REQUIRED(y1),
        PB_ARG_REQUIRED(x2),
        PB_ARG_REQUIRED(y2),
        PB_ARG_DEFAULT_INT(r, 0),
        PB_ARG_DEFAULT_FALSE(fill),
        PB_ARG_DEFAULT_ENUM(color, pb_const_black)
    );

    mp_int_t _x1 = mp_obj_get_int(x1);
    mp_int_t _y1 = mp_obj_get_int(y1);
    mp_int_t _x2 = mp_obj_get_int(x2);
    mp_int_t _y2 = mp_obj_get_int(y2);
    mp_int_t _r = mp_obj_get_int(r);
    GrxColor _color = map_color(color);

    clear_once(self);
    grx_set_current_context(self->context);
    if (mp_obj_is_true(fill)) {
        if (_r > 0) {
            grx_draw_filled_rounded_box(_x1, _y1, _x2, _y2, _r, _color);
        }
        else {
            grx_draw_filled_box(_x1, _y1, _x2, _y2, _color);
        }
    }
    else {
        if (_r > 0) {
            grx_draw_rounded_box(_x1, _y1, _x2, _y2, _r, _color);
        }
        else {
            grx_draw_box(_x1, _y1, _x2, _y2, _color);
        }
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ev3dev_Image_draw_box_obj, 1, ev3dev_Image_draw_box);

STATIC mp_obj_t ev3dev_Image_draw_circle(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_x, ARG_y, ARG_r, ARG_fill, ARG_color };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_x, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_y, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_r, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_fill, MP_ARG_BOOL, {.u_bool = FALSE} },
        { MP_QSTR_color, MP_ARG_OBJ, {.u_rom_obj = MP_ROM_PTR(&pb_const_black)} },
    };
    mp_arg_val_t arg_vals[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, arg_vals);

    ev3dev_Image_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);

    mp_int_t x = arg_vals[ARG_x].u_int;
    mp_int_t y = arg_vals[ARG_y].u_int;
    mp_int_t r = arg_vals[ARG_r].u_int;
    bool fill = arg_vals[ARG_fill].u_bool;
    GrxColor color = map_color(arg_vals[ARG_color].u_obj);

    clear_once(self);
    grx_set_current_context(self->context);
    if (fill) {
        grx_draw_filled_circle(x, y, r, color);
    }
    else {
        grx_draw_circle(x, y, r, color);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ev3dev_Image_draw_circle_obj, 1, ev3dev_Image_draw_circle);

STATIC mp_obj_t ev3dev_Image_draw_image(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_x, ARG_y, ARG_image, ARG_color };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_x, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_y, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_image, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_color, MP_ARG_OBJ, {.u_rom_obj = mp_const_none} },
    };
    mp_arg_val_t arg_vals[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, arg_vals);

    ev3dev_Image_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);

    mp_int_t x = arg_vals[ARG_x].u_int;
    mp_int_t y = arg_vals[ARG_y].u_int;
    if (!mp_obj_is_type(arg_vals[ARG_image].u_obj, &pb_type_ev3dev_Image)) {
        mp_raise_TypeError("Image object is required");
    }
    ev3dev_Image_obj_t *image = MP_OBJ_TO_PTR(arg_vals[ARG_image].u_obj);
    GrxColor color = map_color(arg_vals[ARG_color].u_obj);

    clear_once(self);
    grx_context_bit_blt(self->context, x, y, image->context, 0, 0,
        grx_context_get_max_x(image->context), grx_context_get_max_y(image->context),
        color == GRX_COLOR_NONE ? GRX_COLOR_MODE_WRITE : grx_color_to_image_mode(color));

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ev3dev_Image_draw_image_obj, 1, ev3dev_Image_draw_image);

STATIC mp_obj_t ev3dev_Image_draw_text(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    PB_PARSE_ARGS_METHOD(n_args, pos_args, kw_args,
        ev3dev_Image_obj_t, self,
        PB_ARG_REQUIRED(x),
        PB_ARG_REQUIRED(y),
        PB_ARG_REQUIRED(text),
        PB_ARG_DEFAULT_ENUM(color, pb_const_black)
    );

    mp_int_t _x = mp_obj_get_int(x);
    mp_int_t _y = mp_obj_get_int(y);
    const char *_text = mp_obj_str_get_str(text);
    GrxColor _color = map_color(color);

    clear_once(self);
    grx_set_current_context(self->context);
    grx_text_options_set_fg_color(self->text_options, _color);
    grx_draw_text(_text, _x, _y, self->text_options);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ev3dev_Image_draw_text_obj, 1, ev3dev_Image_draw_text);

STATIC mp_obj_t ev3dev_Image_set_font(mp_obj_t self_in, mp_obj_t font_in) {
    ev3dev_Image_obj_t *self = MP_OBJ_TO_PTR(self_in);
    GrxFont *font = pb_ev3dev_Font_obj_get_font(font_in);

    grx_text_options_set_font(self->text_options, font);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(ev3dev_Image_set_font_obj, ev3dev_Image_set_font);

// copy of mp_builtin_print modified to print to vstr
STATIC mp_obj_t ev3dev_Image_print(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_sep, ARG_end };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_sep, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_rom_obj = MP_ROM_QSTR(MP_QSTR__space_)} },
        { MP_QSTR_end, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_rom_obj = MP_ROM_QSTR(MP_QSTR__0x0a_)} },
    };

    // parse args (a union is used to reduce the amount of C stack that is needed)
    union {
        mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
        size_t len[2];
    } u;
    mp_arg_parse_all(0, NULL, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, u.args);

    ev3dev_Image_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);

    // extract the objects first because we are going to use the other part of the union
    mp_obj_t sep = u.args[ARG_sep].u_obj;
    mp_obj_t end = u.args[ARG_end].u_obj;
    const char *sep_data = mp_obj_str_get_data(sep, &u.len[0]);
    const char *end_data = mp_obj_str_get_data(end, &u.len[1]);

    vstr_t vstr;
    mp_print_t print;
    vstr_init_print(&vstr, 16, &print);

    for (size_t i = 1; i < n_args; i++) {
        if (i > 1) {
            mp_print_strn(&print, sep_data, u.len[0], 0, 0, 0);
        }
        mp_obj_print_helper(&print, pos_args[i], PRINT_STR);
    }
    mp_print_strn(&print, end_data, u.len[1], 0, 0, 0);

    clear_once(self);
    grx_set_current_context(self->context);
    GrxFont *font = grx_text_options_get_font(self->text_options);
    gint font_height = grx_font_get_height(font);
    gchar **lines = g_strsplit(vstr_null_terminated_str(&vstr), "\n", -1);
    for (gchar **l = lines; *l; l++) {
        // if this is not the first line, then we had a newline and need to advance
        if (l != lines) {
            self->print_x = 0;
            self->print_y += font_height;
        }
        // if printing would run off of the bottom of the screen, scroll
        // everything on the screen up enough to fit one more line
        gint screen_height = grx_get_height();
        gint over = self->print_y + font_height - screen_height;
        if (over > 0) {
            gint max_x = grx_get_max_x();
            for (int y = 0; y < screen_height; y++) {
                const GrxColor *scan_line = grx_get_scanline(0, max_x, y + over, NULL);
                if (scan_line) {
                    grx_put_scanline(0, max_x, y, scan_line, GRX_COLOR_MODE_WRITE);
                }
                else {
                    grx_draw_hline(0, max_x, y, GRX_COLOR_WHITE);
                }
            }
            self->print_y -= over;
        }
        grx_draw_text(*l, self->print_x, self->print_y, self->text_options);
        self->print_x += grx_font_get_text_width(font, *l);
    }
    g_strfreev(lines);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(ev3dev_Image_print_obj, 1, ev3dev_Image_print);

STATIC mp_obj_t ev3dev_Image_save(mp_obj_t self_in, mp_obj_t filename_in) {
    ev3dev_Image_obj_t *self = MP_OBJ_TO_PTR(self_in);
    const char *filename = mp_obj_str_get_str(filename_in);

    // add file extension if missing
    char *filename_ext = NULL;
    if (!g_str_has_suffix(filename, ".png") && !g_str_has_suffix(filename, ".PNG")) {
        filename_ext = g_strconcat(filename, ".png", NULL);
        filename = filename_ext;
    }

    GError *error = NULL;
    gboolean ok = grx_context_save_to_png(self->context, filename, &error);
    g_free(filename_ext);
    if (!ok) {
        mp_obj_t ex = mp_obj_new_exception_msg_varg(&mp_type_OSError,
            "Failed to save image: %s", error->message);
        g_error_free(error);
        nlr_raise(ex);
    }

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(ev3dev_Image_save_obj, ev3dev_Image_save);

STATIC const mp_rom_map_elem_t ev3dev_Image_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___del__),     MP_ROM_PTR(&ev3dev_Image___del___obj)                  },
    { MP_ROM_QSTR(MP_QSTR_clear),       MP_ROM_PTR(&ev3dev_Image_clear_obj)                    },
    { MP_ROM_QSTR(MP_QSTR_draw_pixel),  MP_ROM_PTR(&ev3dev_Image_draw_pixel_obj)               },
    { MP_ROM_QSTR(MP_QSTR_draw_line),   MP_ROM_PTR(&ev3dev_Image_draw_line_obj)                },
    { MP_ROM_QSTR(MP_QSTR_draw_box),    MP_ROM_PTR(&ev3dev_Image_draw_box_obj)                 },
    { MP_ROM_QSTR(MP_QSTR_draw_circle), MP_ROM_PTR(&ev3dev_Image_draw_circle_obj)              },
    { MP_ROM_QSTR(MP_QSTR_draw_image),  MP_ROM_PTR(&ev3dev_Image_draw_image_obj)               },
    { MP_ROM_QSTR(MP_QSTR_draw_text),   MP_ROM_PTR(&ev3dev_Image_draw_text_obj)                },
    { MP_ROM_QSTR(MP_QSTR_set_font),    MP_ROM_PTR(&ev3dev_Image_set_font_obj)                 },
    { MP_ROM_QSTR(MP_QSTR_print),       MP_ROM_PTR(&ev3dev_Image_print_obj)                    },
    { MP_ROM_QSTR(MP_QSTR_save),        MP_ROM_PTR(&ev3dev_Image_save_obj)                     },
    { MP_ROM_QSTR(MP_QSTR_width),       MP_ROM_ATTRIBUTE_OFFSET(ev3dev_Image_obj_t, width)     },
    { MP_ROM_QSTR(MP_QSTR_height),      MP_ROM_ATTRIBUTE_OFFSET(ev3dev_Image_obj_t, height)    },
};
STATIC MP_DEFINE_CONST_DICT(ev3dev_Image_locals_dict, ev3dev_Image_locals_dict_table);

const mp_obj_type_t pb_type_ev3dev_Image = {
    { &mp_type_type },
    .name = MP_QSTR_Image,
    .make_new = ev3dev_Image_make_new,
    .locals_dict = (mp_obj_dict_t*)&ev3dev_Image_locals_dict,
};
