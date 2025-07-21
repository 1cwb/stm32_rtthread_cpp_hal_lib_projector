ifeq ($(BOARD_TYPE), $(RC_FLY_BOARD))
CURRENT_DIR := $(CURDIR)/project/rc-fly
SUBDIRS := $(shell find $(CURRENT_DIR) -maxdepth 3 -type d)

CURRENT_SRC_DIR := $(SUBDIRS)

SRC_C_FILES += $(foreach dir, $(CURRENT_SRC_DIR), $(wildcard $(dir)/*.c))
SRC_INCFILES += $(foreach dir, $(CURRENT_SRC_DIR), $(wildcard $(dir)/*.h))
SRC_CXX_FILES += $(foreach dir, $(CURRENT_SRC_DIR), $(wildcard $(dir)/*.cpp))
LINK_FILES += $(foreach dir, $(CURRENT_SRC_DIR), $(wildcard $(dir)/*.ld))
SRC_AMSFILES +=  $(foreach dir, $(CURRENT_SRC_DIR), $(wildcard $(dir)/*.s))

SRC_INCDIR += $(SUBDIRS)
endif