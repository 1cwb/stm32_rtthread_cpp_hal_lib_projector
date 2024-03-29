CURRENT_DIR := $(CURDIR)/rtos

SUBDIRS := $(shell find $(CURRENT_DIR) -maxdepth 4 -type d)

CURRENT_SRC_DIR := $(SUBDIRS)

SRC_C_FILES += $(foreach dir, $(CURRENT_SRC_DIR), $(wildcard $(dir)/*.c))
SRC_INCFILES += $(foreach dir, $(CURRENT_SRC_DIR), $(wildcard $(dir)/*.h))
SRC_CXX_FILES += $(foreach dir, $(CURRENT_SRC_DIR), $(wildcard $(dir)/*.cpp))
LINK_FILES += $(foreach dir, $(CURRENT_SRC_DIR), $(wildcard $(dir)/*.ld))
SRC_AMSFILES +=  $(foreach dir, $(CURRENT_SRC_DIR), $(wildcard $(dir)/*.s))
SRC_AMSFILES +=  $(foreach dir, $(CURRENT_SRC_DIR), $(wildcard $(dir)/*.S))
SRC_INCDIR += $(SUBDIRS)
