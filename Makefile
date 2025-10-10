# for sub directory
export SRC_AMSFILES :=
export SRC_C_FILES :=
export SRC_INCFILES :=
export SRC_CXX_FILES :=
export SRC_HPP_FILES :=
export SRC_INCDIR :=
export LINK_FILES :=
export BOARD_TYPE :=
#######################################BOARD_TYPE#############################################
export AP_FLY_BOARD := 1
export RC_FLY_BOARD := 2
export DEMO_BOARD := 3
BOARD_TYPE := $(RC_FLY_BOARD)

# Build configuration: 0 for Release, 1 for Debug.
# To build with debug info, run: make DEBUG=1
DEBUG ?= 0

########################################MEM_MAP################################################
APP_FLASH_ORIGIN := 0x8032000
APP_FLASH_LEN := 1848K
DEFINE += -DH7XX_BOOTLOADER_ADDR=0x1FF09800
DEFINE += -DAPP_VTABLE_ADDR=$(APP_FLASH_ORIGIN)
DEFINE += -DDEBUG_UART_BOUNDRATE=921600
######################################SUB_MK###################################################
# 要编译的文件夹，用空格分隔
ifdef BOOT
BUILD_DIR := $(CURDIR)/bootloader $(CURDIR)/core $(CURDIR)/stm32h7hallib
else
BUILD_DIR := $(CURDIR)/startup $(CURDIR)/core $(CURDIR)/stm32h7hallib $(CURDIR)/device $(CURDIR)/project $(CURDIR)/rtos $(CURDIR)/system $(CURDIR)/3rdlib
endif

#$(info \"$(BUILD_DIR)\")
SUBDIRS := $(shell find $(BUILD_DIR) -maxdepth 4 -type d)
SUBMK += $(foreach dir, $(SUBDIRS), $(wildcard $(dir)/*.mk))
include $(SUBMK)
#$(shell sleep 5)
################################################################################################
OUTPUT := $(CURDIR)/output
ifdef BOOT
TARGET           	?= STM32H7_BOOT
OUTPUTDIR += $(OUTPUT)/bootloader
else
TARGET           	?= STM32H7_APP
OUTPUTDIR += $(OUTPUT)/app
endif

define ensure_dir
    @if [ ! -d \"$(1)\" ]; then \
        echo \"创建目录: $(1)\"; \
        mkdir -p $(1); \
    else \
        echo \"目录已存在: $(1)\"; \
    fi
endef

###################################################COMPILE######################################
CROSS_COMPILE    	?= arm-none-eabi-

CC               	:= $(CROSS_COMPILE)gcc
CXX              	:= $(CROSS_COMPILE)g++
LD               	:= $(CROSS_COMPILE)ld
OBJCOPY          	:= $(CROSS_COMPILE)objcopy
OBJDUMP          	:= $(CROSS_COMPILE)objdump
SIZEINFO            := $(CROSS_COMPILE)size

#############################################################ARM################################
#softvfp 软浮点
#fpv5-d16 or fpv5-sp-d16 cortex-M7 单精度硬件浮点
#fpv4-d16 or fpv4-sp-d16 cortex-M4 单精度硬件浮点
#fpv5-dp-d16 cortex-M7 双精度硬件浮点

CPU           		:= -mcpu=cortex-m7
FPU        			:= -mfpu=fpv5-sp-d16
FLOAT_ABT 			:= -mfloat-abi=hard
ARM_INSTRUCTION 	:= -mthumb
MCU_FLAGS       	:= $(CPU) $(ARM_INSTRUCTION) $(FPU) $(FLOAT_ABT) -ftree-vectorize

##################################################COMPILE_FLAGS#################################
ifeq ($(DEBUG), 1)
# --- DEBUG FLAGS ---
C_COMPILE_FLAGS 	:= -lc -lm -lnosys -std=c11 \
                   -Wall \
                   -fno-common \
                   -fmessage-length=0 \
                   -O0 \
                   -g3 \
                   -flto \
                   -fno-stack-protector

CXX_COMPILE_FLAGS 	:= -lc -lm -lnosys \
                   -std=c++17 \
                   -fno-rtti \
                   -fno-exceptions \
                   -fno-builtin \
                   -fno-threadsafe-statics \
                   -fno-use-cxa-atexit \
                   -Wall \
                   -O0 \
                   -g3 \
                   -flto \
                   -fvisibility=hidden

EXTRA_LINK_FLAGS	:= \
                   -L/opt/gcc-arm-none-eabi-10.3-2021.10/arm-none-eabi/lib/thumb/v7e-m+fp/hard \
                   -Wl,-Map=$(OUTPUTDIR)/$(TARGET).map \
                   -Wl,--cref \
                   -Wl,--no-warn-mismatch

else
# --- RELEASE FLAGS ---
C_COMPILE_FLAGS 	:= -lc -lm -lnosys -std=c11 \
                   -Wall \
                   -ffunction-sections \
                   -fdata-sections \
                   -fno-common \
                   -fmessage-length=0 \
                   -Og \
                   -flto \
                   -fno-stack-protector \
                   -fomit-frame-pointer

CXX_COMPILE_FLAGS 	:= -lc -lm -lnosys \
                   -std=c++17 \
                   -fno-rtti \
                   -fno-exceptions \
                   -fno-builtin \
                   -fno-threadsafe-statics \
                   -fno-use-cxa-atexit \
                   -Wall \
                   -ffunction-sections \
                   -fdata-sections \
                   -Og \
                   -flto \
                   -fomit-frame-pointer \
                   -fvisibility=hidden

EXTRA_LINK_FLAGS	:= \
                   -L/opt/gcc-arm-none-eabi-10.3-2021.10/arm-none-eabi/lib/thumb/v7e-m+fp/hard \
                   -Wl,--gc-sections \
                   -Wl,--strip-all \
                   -Wl,-Map=$(OUTPUTDIR)/$(TARGET).map \
                   -Wl,--cref \
                   -Wl,--no-warn-mismatch
endif

# --- COMMON LINK FLAGS ---
EXTRA_LINK_FLAGS	+= \
                   -lc \
                   -lm \
                   -lstdc++ \
                   -lnosys \
                   -T$(LINK_FILES) \
                   -specs=nano.specs \
                   -specs=nosys.specs \
				   -u _printf_float \
           		   -u _scanf_float \
                   -flto \
                   -Wl,--defsym=_app_flash_origin=$(APP_FLASH_ORIGIN) \
                   -Wl,--defsym=_app_flash_len=$(APP_FLASH_LEN)

# ASM 编译标志保持不变
ASM_COMPILE_FLAGS 	:= -x assembler-with-cpp -Wa,-mimplicit-it=thumb
#################################################################################################
ifeq ($(BOARD_TYPE), $(RC_FLY_BOARD))
DEFINE    +=-DSTM32H743xx \
			-DHSE_VALUE=8000000 \
			-DCSI_VALUE=4000000 \
			-DHSI_VALUE=64000000 \
			-DPLLM_VALUE=2 \
			-DPLLN_VALUE=240 \
			-DPLLP_VALUE=2 \
			-DPLLQ_VALUE=4 \
			-DPLLR_VALUE=2 \
			-DPLL2M_VALUE=1 \
			-DPLL2N_VALUE=25 \
			-DPLL2P_VALUE=2 \
			-DPLL2Q_VALUE=1 \
			-DPLL2R_VALUE=1
else ifeq ($(BOARD_TYPE), $(DEMO_BOARD))
DEFINE    +=-DSTM32H743xx \
			-DHSE_VALUE=25000000 \
			-DCSI_VALUE=4000000 \
			-DHSI_VALUE=64000000 \
			-DPLLM_VALUE=5 \
			-DPLLN_VALUE=192 \
			-DPLLP_VALUE=2 \
			-DPLLQ_VALUE=4 \
			-DPLLR_VALUE=2 \
			-DPLL2M_VALUE=5 \
			-DPLL2N_VALUE=40 \
			-DPLL2P_VALUE=2 \
			-DPLL2Q_VALUE=2 \
			-DPLL2R_VALUE=2 \
			-DUSE_SDRAM=1 \
			-DSDRAM_ORIGIN=0xC0000000 \
			-DSDRAM_LEN=16384
#else ifeq ($(BOARD_TYPE), $(AP_FLY_BOARD))
#DEFINE    +=-DSTM32H743xx \
#			-DHSE_VALUE=8000000 \
#			-DCSI_VALUE=4000000 \
#			-DHSI_VALUE=64000000 \
#			-DPLLM_VALUE=2 \
#			-DPLLN_VALUE=240 \
#			-DPLLP_VALUE=2 \
#			-DPLLQ_VALUE=4 \
#			-DPLLR_VALUE=2 \
#			-DPLL2M_VALUE=2 \
#			-DPLL2N_VALUE=25 \
#			-DPLL2P_VALUE=2 \
#			-DPLL2Q_VALUE=1 \
#			-DPLL2R_VALUE=1
#endif
else ifeq ($(BOARD_TYPE), $(AP_FLY_BOARD))
DEFINE    +=-DSTM32H743xx \
			-DHSE_VALUE=16000000 \
			-DCSI_VALUE=4000000 \
			-DHSI_VALUE=64000000 \
			-DPLLM_VALUE=4 \
			-DPLLN_VALUE=240 \
			-DPLLP_VALUE=2 \
			-DPLLQ_VALUE=4 \
			-DPLLR_VALUE=2 \
			-DPLL2M_VALUE=2 \
			-DPLL2N_VALUE=25 \
			-DPLL2P_VALUE=2 \
			-DPLL2Q_VALUE=1 \
			-DPLL2R_VALUE=1
endif
DEFINE    +=-DTHREAD_TICK_PER_SECOND=1000
###############################################################
CFLAGS 				+= $(MCU_FLAGS) $(C_COMPILE_FLAGS) $(DEFINE)
CXXFLAGS 			+= $(MCU_FLAGS) $(CXX_COMPILE_FLAGS) $(DEFINE)
ASMFLAGS			+= $(MCU_FLAGS) $(ASM_COMPILE_FLAGS) $(DEFINE)

LFLAGS += $(MCU_FLAGS)  $(EXTRA_LINK_FLAGS)

INCLUDE 		 := $(patsubst %, -I %, $(SRC_INCDIR))
SFILES			 := $(SRC_AMSFILES)
CFILES			 := $(SRC_C_FILES)
CXXFILES         := $(SRC_CXX_FILES)
HPPFILES         := $(SRC_HPP_FILES)

SFILENAME  		 := $(notdir $(SFILES))
CFILENAME 		 := $(notdir $(CFILES))
CXXFILENAME		 := $(notdir $(CXXFILES))
HPPFILENAME      := $(notdir $(HPPFILES))

SOBJS		 	 := $(patsubst %, $(OUTPUTDIR)/%, $(SFILENAME:.s=.o))
COBJS		 	 := $(patsubst %, $(OUTPUTDIR)/%, $(CFILENAME:.c=.o))
CXXOBJS          := $(patsubst %, $(OUTPUTDIR)/%, $(CXXFILENAME:.cpp=.o))
HPPOBJS          := $(patsubst %, $(OUTPUTDIR)/%, $(HPPFILENAME:.hpp=.o))
OBJS			 := $(SOBJS) $(COBJS) $(CXXOBJS)

SRCDIRS          := $(dir $(SFILES)) $(dir $(CFILES)) $(dir $(CXXFILES))
VPATH			 := $(SRCDIRS)

.PHONY: clean

define analyze_elf_sections_color
    @echo "\n\033[1;34m--- Memory Usage Summary (Bytes/KB) ---\033[0m"
    @$(SIZEINFO) $(1) | awk '\
    NR==1 {printf "\033[1;33m%-15s %10s %8s\033[0m\n", "Section", "Bytes", "KB"} \
    NR>1 {printf "\033[1;36m%-15s \033[1;33m%10d \033[1;35m%7.2fK\033[0m\n", $$1, $$2, $$2/1024}'
    
    @echo "\n\033[1;34m--- Largest Sections ---\033[0m"
    @$(SIZEINFO) -A $(1) | sort -k2 -n -r | head -n 5 | awk '\
    BEGIN {printf "\033[1;33m%-20s %10s %8s\033[0m\n", "Section", "Bytes", "KB"} \
    {printf "\033[1;32m%-20s \033[1;35m%10d \033[1;31m%7.2fK\033[0m\n", $$1, $$2, $$2/1024}'
endef

#$(info \"SFILES = $(SFILES) \")
#$(info \"CFILES = $(CFILES) \")

$(OUTPUTDIR)/$(TARGET).elf:$(OBJS)
	$(CC) -o $(OUTPUTDIR)/$(TARGET).elf $^ $(LFLAGS)
	$(OBJCOPY) -O binary -S $(OUTPUTDIR)/$(TARGET).elf $(OUTPUTDIR)/$(TARGET).bin
	$(OBJDUMP) -D -m arm $(OUTPUTDIR)/$(TARGET).elf > $(OUTPUTDIR)/$(TARGET).dis
	cp $(OUTPUTDIR)/$(TARGET).bin /mnt/e/STM32/
	$(call analyze_elf_sections_color,$@)  # 注意这里使用$@作为参数
	sync

$(SOBJS) : $(OUTPUTDIR)/%.o : %.s
	$(call ensure_dir,$(OUTPUTDIR))
	$(CC) -c $(ASMFLAGS) -o $@ $<

$(COBJS) : $(OUTPUTDIR)/%.o : %.c
	$(CC) -c $(CFLAGS) $(INCLUDE) -o $@ $<

$(CXXOBJS) : $(OUTPUTDIR)/%.o : %.cpp
	$(CXX) -c $(CXXFLAGS) $(INCLUDE) -o $@ $<

$(HPPOBJS) : $(OUTPUTDIR)/%.o : %.hpp
	$(CXX) -c $(CXXFLAGS) $(INCLUDE) -o $@ $<

clean:
	rm -rf $(OUTPUT)/*