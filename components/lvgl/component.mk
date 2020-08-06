#
# Component Makefile (for esp-idf)
#
# This Makefile should, at the very least, just include $(SDK_PATH)/make/component.mk. By default,
# this will take the sources in this directory, compile them and link them into
# lib(subdirectory_name).a in the build directory. This behaviour is entirely configurable,
# please read the SDK documents if you need to do this.
#



COMPONENT_SRCDIRS := lvgl/src/lv_core lvgl/src/lv_draw lvgl/src/lv_font lvgl/src/lv_hal lvgl/src/lv_misc lvgl/src/lv_widgets lvgl/src/lv_themes
COMPONENT_ADD_INCLUDEDIRS := lvgl
COMPONENT_ADD_LDFLAGS := -llvgl
COMPONENT_OBJEXCLUDE := 



