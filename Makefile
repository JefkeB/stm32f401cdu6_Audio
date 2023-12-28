## keep this name in sync with the 1 used in
## launch.json
## "executable": ".\\build\\${workspaceFolderBasename}.elf",
PROJ_NAME=hw_debug
VERBOSE = @

OPENOCD_CONFIG ?= "ENTER_A_VALID_CONFIG"


PROCESSOR_MODEL = STM32F401xE
PROCESSOR_LINKER_SCRIPT = LinkerScript

PACKAGE_DIR = _package_

####
DEFINES_ = 

PROCESSOR_LINKER_SCRIPT = STM32F401CDUx_FLASH
# LINKER_SCRIPT = $(SOURCE_SUB_DIR)/$(PACKAGE_DIR)/$(PROCESSOR_VARIANT)/gcc/gcc/$(PROCESSOR_LINKER_SCRIPT).ld
LINKER_SCRIPT = $(PROCESSOR_LINKER_SCRIPT).ld

##
## Application Source files
##
SOURCE_SUB_DIR = .

_SRCS_ =  \
	Core/Src/dual240.c \
	Core/Src/font8.c \
	Core/Src/main.c \
	Core/Src/st7789.c \
	Core/Src/stm32f4xx_hal_msp.c \
	Core/Src/stm32f4xx_it.c \
	Core/Src/syscalls.c \
	Core/Src/sysmem.c \
	Core/Src/system_stm32f4xx.c \
	Core/Src/uvmete240.c \
	\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_dma.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_exti.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_gpio.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_rcc.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_tim.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_utils.c \
	\
	Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
	Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
	Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c  \
	Middlewares/ST/STM32_USB_Device_Library/Class/AUDIO/Src/usbd_audio.c \
	\
	USB_DEVICE/App/usb_device.c \
	USB_DEVICE/App/usbd_audio_if.c \
	USB_DEVICE/App/usbd_desc.c \
	USB_DEVICE/Target/usbd_conf.c 

	
INCLUDE_DIRS_ += \
	Core/Inc \
	\
	USB_DEVICE/App \
	USB_DEVICE/Target \
	\
	Drivers/STM32F4xx_HAL_Driver/Inc \
	Drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
	Drivers/CMSIS/Device/ST/STM32F4xx/Include \
	Drivers/CMSIS/Include \
	\
	Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
	Middlewares/ST/STM32_USB_Device_Library/Class/AUDIO/Inc	

##
## other system source files
##
_SRCS_OTHER_ = \
	Core/Startup/startup_stm32f401cdux.s 
	
##	arm-none-eabi-gcc -mcpu=cortex-m4 -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"


INCLUDE_DIRS_ += \
	../Utilities/STM32F4xx-Nucleo 
 
##
## HAL source files
##
SOURCE_HAL_SUB_DIR = HAL_Driver/Src/

_SRCS_HAL_ = \

INCLUDE_DIRS_ += \
	../HAL_Driver/inc/Legacy \
	../HAL_Driver/inc 
	
	 
##
## Include files	
##
_INCLUDE_DIRS_ = \
	

# all source file	
SRCS := $(addprefix $(SOURCE_SUB_DIR)/,$(_SRCS_))
SRCS += $(addprefix $(SOURCE_HAL_SUB_DIR)/,$(_SRCS_HAL_)) 
SRCS += $(_SRCS_OTHER_) 
SRCS += $(addprefix $(FREERTOS_SUB_DIR)/,$(_SRCS_FREERTOS_)) $(addprefix $(FREERTOS_SUB_DIR)/,$(_SRC_SYSVIEW_))
	
# make -I"src/INCLUDEFILEPATH" from each line in the includes collection	
INCLUDE_DIRS := $(addprefix -I"$(SOURCE_SUB_DIR)/,$(addsuffix ", $(INCLUDE_DIRS_))) 
INCLUDE_DIRS += $(addprefix -I"$(SOURCE_SUB_DIR)/,$(addsuffix ", $(_INCLUDE_DIRS_))) 
INCLUDE_DIRS += $(addprefix -I"$(FREERTOS_SUB_DIR)/,$(addsuffix ", $(_INLCUDE_FREERTOS_))) 
INCLUDE_DIRS += $(addprefix -I"$(FREERTOS_SUB_DIR)/,$(addsuffix ", $(_INCLUDE_SYSVIEW_))) 
	
####	   
CC=arm-none-eabi-gcc
CPP=arm-none-eabi-g++
OBJCOPY=arm-none-eabi-objcopy
OBJSIZE=arm-none-eabi-size

####
OBJDIR = build


_DEFINES = \
	-DDEBUG \
  \
 -DSTM32F401xE \
 -DSTM32 \
 -DSTM32F4 \
 -DDEBUG \
 -DUSE_HAL_DRIVER \
 -DUSE_FULL_LL_DRIVER \
 $(OPTIONZ)
	
DEFINES = $(DEFINES_) $(_DEFINES)	

MCU = \
	-mcpu=cortex-m4 \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard

####
CFLAGS = \
	-mthumb \
	-Os \
	-ffunction-sections \
	-fdata-sections \
	-mlong-calls \
	-g3 \
	-Wall \
	$(MCU) \
	-c \
	-std=gnu11 \
	-fstack-usage \
	--specs=nano.specs

####	
CPPFLAGS = \
	-mthumb \
	-Os \
	-ffunction-sections \
	-fdata-sections \
	-fno-rtti \
	-fno-exceptions \
	-mlong-calls \
	-g3 \
	-Wall \
	$(MCU) \
	-c 

####	
LDFLAGS =\
	-mthumb \
	-Wl,-Map="$(PROJ_NAME).map" \
	--specs=nano.specs \
	-Wl,--start-group \
	-lm  \
	-Wl,--end-group \
	-L"source/CMSIS/Lib/GCC"  \
	-Wl,--gc-sections \
	$(MCU) \
	-Wl,--entry=Reset_Handler \
	-Wl,--cref \
	-mthumb \
	-T$(LINKER_SCRIPT) \
	-Wl,--defsym,STACK_SIZE=0x0400 \
##	-Xlinker --section-start=.text_nietInGebruik=0x8100 \
##	-Xlinker --section-start=.device_information_nietInGebruik=0x008000  

####
CFLAGS += $(INCLUDE_DIRS) $(DEFINES)
CPPFLAGS += $(INCLUDE_DIRS) $(DEFINES)

####
OBJS := $(SRCS:.c=.o)
OBJS := $(OBJS:.cpp=.o)
OBJS := $(OBJS:.s=.o)
OBJS := $(addprefix $(OBJDIR)/,$(OBJS))

all: $(OBJDIR)/$(PROJ_NAME).elf $(OBJDIR)/$(PROJ_NAME).hex $(OBJDIR)/$(PROJ_NAME).bin

$(OBJDIR)/%.elf: $(OBJS)
	@echo Linking ...
	$(VERBOSE)$(CPP) -o $@ $^ $(LDFLAGS)	

%.hex: %.elf
	$(VERBOSE)$(OBJCOPY) -O ihex $^ $@

%.bin: %.elf
	$(VERBOSE)$(OBJCOPY) -O binary $^ $@
	$(VERBOSE)$(OBJSIZE) $^ 		

	
$(OBJDIR)/%.o: %.cpp
	@echo Building cpp file: $<
	$(VERBOSE)mkdir -p $(dir $@)
	$(VERBOSE)$(CPP) -c $(CPPFLAGS) -o $@ $^

$(OBJDIR)/%.o: %.c
	@echo Building c file: $<
	$(VERBOSE)mkdir -p $(dir $@)
	$(VERBOSE)$(CC) -c $(CFLAGS) -o $@ $^

$(OBJDIR)/%.o: %.s
	@echo Building s file: $<
	$(VERBOSE)mkdir -p $(dir $@)
	$(VERBOSE)$(CC) -c $(CFLAGS) -o $@ $^

$(OBJDIR):
	mkdir -p $@

clean:
	rm -fr build

program:
	openocd -f $(OPENOCD_CONFIG)

#######################################
# Flash binary
#######################################
flash:
	./tools/st-flash.exe --reset write build/$(PROJ_NAME).bin 0x08000000

# Dependencies
$(OBJDIR)/$(PROJ_NAME).elf: $(OBJS) | $(OBJDIR)


