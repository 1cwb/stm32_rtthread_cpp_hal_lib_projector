CURRENT_DIR := $(CURDIR)/SYSTEM

SUBDIRS := $(shell find $(CURRENT_DIR) -maxdepth 3 -type d)

CURRENT_SRC_DIR := $(SUBDIRS)

SRC_C_FILES += $(foreach dir, $(CURRENT_SRC_DIR), $(wildcard $(dir)/*.c))
SRC_INCFILES += $(foreach dir, $(CURRENT_SRC_DIR), $(wildcard $(dir)/*.h))
SRC_CXX_FILES += $(foreach dir, $(CURRENT_SRC_DIR), $(wildcard $(dir)/*.cpp))
LINK_FILES += $(foreach dir, $(CURRENT_SRC_DIR), $(wildcard $(dir)/*.ld))
SRC_AMSFILES +=  $(foreach dir, $(CURRENT_SRC_DIR), $(wildcard $(dir)/*.s))

SRC_INCDIR += $(SUBDIRS)
