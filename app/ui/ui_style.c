#include "ui_style.h"

lv_style_t ui_style_screen;
lv_style_t ui_style_panel;
lv_style_t ui_style_button;
lv_style_t ui_style_button_checked;
lv_style_t ui_style_control;

void ui_style_init(void)
{
    lv_style_init(&ui_style_screen);
    lv_style_set_bg_color(&ui_style_screen, lv_color_hex(UI_COLOR_BG));
    lv_style_set_bg_opa(&ui_style_screen, LV_OPA_COVER);
    lv_style_set_text_color(&ui_style_screen, lv_color_hex(UI_COLOR_TEXT));
    lv_style_set_text_font(&ui_style_screen, UI_FONT_BODY);

    lv_style_init(&ui_style_panel);
    lv_style_set_bg_color(&ui_style_panel, lv_color_hex(UI_COLOR_PANEL));
    lv_style_set_bg_opa(&ui_style_panel, LV_OPA_COVER);
    lv_style_set_border_width(&ui_style_panel, 0);
    lv_style_set_radius(&ui_style_panel, 0);
    lv_style_set_pad_all(&ui_style_panel, 0);

    lv_style_init(&ui_style_button);
    lv_style_set_bg_color(&ui_style_button, lv_color_hex(UI_COLOR_PANEL_ALT));
    lv_style_set_bg_opa(&ui_style_button, LV_OPA_COVER);
    lv_style_set_border_color(&ui_style_button, lv_color_hex(UI_COLOR_BORDER));
    lv_style_set_border_width(&ui_style_button, 1);
    lv_style_set_radius(&ui_style_button, 6);
    lv_style_set_text_color(&ui_style_button, lv_color_hex(UI_COLOR_TEXT));
    lv_style_set_text_font(&ui_style_button, UI_FONT_CONTROL);
    lv_style_set_height(&ui_style_button, UI_BUTTON_HEIGHT);
    lv_style_set_pad_hor(&ui_style_button, UI_BUTTON_PAD_HOR);
    lv_style_set_shadow_width(&ui_style_button, 0);

    lv_style_init(&ui_style_button_checked);
    lv_style_set_bg_color(&ui_style_button_checked, lv_color_hex(UI_COLOR_ACCENT));
    lv_style_set_border_color(&ui_style_button_checked, lv_color_hex(UI_COLOR_ACCENT));
    lv_style_set_text_color(&ui_style_button_checked, lv_color_hex(UI_COLOR_BG));

    lv_style_init(&ui_style_control);
    lv_style_set_bg_color(&ui_style_control, lv_color_hex(UI_COLOR_PANEL_ALT));
    lv_style_set_border_color(&ui_style_control, lv_color_hex(UI_COLOR_BORDER));
    lv_style_set_text_font(&ui_style_control, UI_FONT_CONTROL);
    lv_style_set_pad_ver(&ui_style_control, UI_CONTROL_PAD_VER);
}

lv_obj_t *ui_style_create_panel(lv_obj_t *parent)
{
    lv_obj_t *panel = lv_obj_create(parent);
    lv_obj_add_style(panel, &ui_style_panel, 0);
    lv_obj_remove_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    return panel;
}

lv_obj_t *ui_style_create_label(lv_obj_t *parent, const char *text,
                                const lv_font_t *font, uint32_t color_hex)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(color_hex), 0);
    return label;
}

lv_obj_t *ui_style_create_button(lv_obj_t *parent, const char *text,
                                 lv_obj_t **label_out)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_t *label;

    lv_obj_remove_style_all(button);
    lv_obj_add_style(button, &ui_style_button, 0);
    lv_obj_add_style(button, &ui_style_button_checked, LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(button, lv_color_hex(UI_COLOR_BORDER), LV_STATE_PRESSED);

    label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_center(label);
    if (label_out != NULL) {
        *label_out = label;
    }
    return button;
}
