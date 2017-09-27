################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../src/ \
../src/ASF/ \
../src/ASF/avr32/ \
../src/ASF/avr32/drivers/ \
../src/ASF/avr32/drivers/intc/ \
../src/ASF/avr32/utils/ \
../src/ASF/avr32/utils/header_files/ \
../src/ASF/avr32/utils/make/ \
../src/ASF/avr32/utils/preprocessor/ \
../src/ASF/avr32/utils/startup/ \
../src/ASF/common/ \
../src/ASF/common/boards/ \
../src/ASF/common/boards/user_board/ \
../src/ASF/common/utils/ \
../src/ASF/common/utils/interrupt/ \
../src/config/


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../src/accelerometer.c \
../src/my_INTC.c \
../src/XNL.c \
../src/ASF/common/boards/user_board/init.c


PREPROCESSING_SRCS +=  \
../src/exception.x \
../src/ASF/avr32/utils/startup/startup_uc3.S \
../src/ASF/avr32/utils/startup/trampoline_uc3.S


ASM_SRCS += 


OBJS +=  \
src/accelerometer.o \
src/exception.o \
src/my_INTC.o \
src/XNL.o \
src/ASF/avr32/utils/startup/startup_uc3.o \
src/ASF/avr32/utils/startup/trampoline_uc3.o \
src/ASF/common/boards/user_board/init.o

OBJS_AS_ARGS +=  \
src/accelerometer.o \
src/exception.o \
src/my_INTC.o \
src/XNL.o \
src/ASF/avr32/utils/startup/startup_uc3.o \
src/ASF/avr32/utils/startup/trampoline_uc3.o \
src/ASF/common/boards/user_board/init.o

C_DEPS +=  \
src/accelerometer.d \
src/exception.d \
src/my_INTC.d \
src/XNL.d \
src/ASF/avr32/utils/startup/startup_uc3.d \
src/ASF/avr32/utils/startup/trampoline_uc3.d \
src/ASF/common/boards/user_board/init.d

C_DEPS_AS_ARGS +=  \
src/accelerometer.d \
src/exception.d \
src/my_INTC.d \
src/XNL.d \
src/ASF/avr32/utils/startup/startup_uc3.d \
src/ASF/avr32/utils/startup/trampoline_uc3.d \
src/ASF/common/boards/user_board/init.d

OUTPUT_FILE_PATH +=accelerometer_demo_6_2.elf

OUTPUT_FILE_PATH_AS_ARGS +=accelerometer_demo_6_2.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+=  \
../src/link_uc3b0512.lds


# AVR32/GNU C Compiler









src/%.o: ../src/%.c
	@echo Building file: $<
	@echo Invoking: AVR32/GNU C Compiler : 4.4.7
	$(QUOTE)D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr32\avr32-gnu-toolchain\bin\avr32-gcc.exe$(QUOTE)  -x c -DDEBUG -DBOARD=USER_BOARD  -I"../common/applications/user_application/user_board/config" -I"../src/ASF/avr32/utils" -I"../src/config" -I"../src/ASF/common/boards" -I"../src/ASF/avr32/utils/preprocessor" -I"../src/ASF/avr32/drivers/intc" -I"../src/ASF/common/utils" -I"../src" -I"../src/ASF/common/boards/user_board" -I"D:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\UC3B_DFP\1.0.29\include\AT32UC3B0512"  -O1 -fdata-sections -ffunction-sections -masm-addr-pseudos -g2 -Wall -mpart=uc3b0512 -c -std=gnu99 -fno-strict-aliasing -Wstrict-prototypes -Wmissing-prototypes -Werror-implicit-function-declaration -Wpointer-arith -mrelax -mno-cond-exec-before-reload -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

src/ASF/common/boards/user_board/%.o: ../src/ASF/common/boards/user_board/%.c
	@echo Building file: $<
	@echo Invoking: AVR32/GNU C Compiler : 4.4.7
	$(QUOTE)D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr32\avr32-gnu-toolchain\bin\avr32-gcc.exe$(QUOTE)  -x c -DDEBUG -DBOARD=USER_BOARD  -I"../common/applications/user_application/user_board/config" -I"../src/ASF/avr32/utils" -I"../src/config" -I"../src/ASF/common/boards" -I"../src/ASF/avr32/utils/preprocessor" -I"../src/ASF/avr32/drivers/intc" -I"../src/ASF/common/utils" -I"../src" -I"../src/ASF/common/boards/user_board" -I"D:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\UC3B_DFP\1.0.29\include\AT32UC3B0512"  -O1 -fdata-sections -ffunction-sections -masm-addr-pseudos -g2 -Wall -mpart=uc3b0512 -c -std=gnu99 -fno-strict-aliasing -Wstrict-prototypes -Wmissing-prototypes -Werror-implicit-function-declaration -Wpointer-arith -mrelax -mno-cond-exec-before-reload -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler

src/%.o: ../src/%.S
	@echo Building file: $<
	@echo Invoking: AVR32/GNU Preprocessing Assembler : 4.4.7
	$(QUOTE)D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr32\avr32-gnu-toolchain\bin\avr32-gcc.exe$(QUOTE) -x assembler-with-cpp -c -mpart=uc3b0512 -mrelax -DBOARD=USER_BOARD -I "../common/applications/user_application/user_board/config" -I "../src/ASF/avr32/utils" -I "../src/config" -I "../src/ASF/common/boards" -I "../src/ASF/avr32/utils/preprocessor" -I "../src/ASF/avr32/drivers/intc" -I "../src/ASF/common/utils" -I "../src" -I "../src/ASF/common/boards/user_board" -I "D:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\UC3B_DFP\1.0.29\include\AT32UC3B0512"  -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -Wa,-g   -o "$@" "$<" 
	@echo Finished building: $<
	

src/%.o: ../src/%.x
	@echo Building file: $<
	@echo Invoking: AVR32/GNU Preprocessing Assembler : 4.4.7
	$(QUOTE)D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr32\avr32-gnu-toolchain\bin\avr32-gcc.exe$(QUOTE) -x assembler-with-cpp -c -mpart=uc3b0512 -mrelax -DBOARD=USER_BOARD -I "../common/applications/user_application/user_board/config" -I "../src/ASF/avr32/utils" -I "../src/config" -I "../src/ASF/common/boards" -I "../src/ASF/avr32/utils/preprocessor" -I "../src/ASF/avr32/drivers/intc" -I "../src/ASF/common/utils" -I "../src" -I "../src/ASF/common/boards/user_board" -I "D:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\UC3B_DFP\1.0.29\include\AT32UC3B0512"  -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -Wa,-g   -o "$@" "$<" 
	@echo Finished building: $<
	

src/%.o: ../src/%.X
	@echo Building file: $<
	@echo Invoking: AVR32/GNU Preprocessing Assembler : 4.4.7
	$(QUOTE)D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr32\avr32-gnu-toolchain\bin\avr32-gcc.exe$(QUOTE) -x assembler-with-cpp -c -mpart=uc3b0512 -mrelax -DBOARD=USER_BOARD -I "../common/applications/user_application/user_board/config" -I "../src/ASF/avr32/utils" -I "../src/config" -I "../src/ASF/common/boards" -I "../src/ASF/avr32/utils/preprocessor" -I "../src/ASF/avr32/drivers/intc" -I "../src/ASF/common/utils" -I "../src" -I "../src/ASF/common/boards/user_board" -I "D:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\UC3B_DFP\1.0.29\include\AT32UC3B0512"  -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -Wa,-g   -o "$@" "$<" 
	@echo Finished building: $<
	

src/ASF/avr32/utils/startup/%.o: ../src/ASF/avr32/utils/startup/%.S
	@echo Building file: $<
	@echo Invoking: AVR32/GNU Preprocessing Assembler : 4.4.7
	$(QUOTE)D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr32\avr32-gnu-toolchain\bin\avr32-gcc.exe$(QUOTE) -x assembler-with-cpp -c -mpart=uc3b0512 -mrelax -DBOARD=USER_BOARD -I "../common/applications/user_application/user_board/config" -I "../src/ASF/avr32/utils" -I "../src/config" -I "../src/ASF/common/boards" -I "../src/ASF/avr32/utils/preprocessor" -I "../src/ASF/avr32/drivers/intc" -I "../src/ASF/common/utils" -I "../src" -I "../src/ASF/common/boards/user_board" -I "D:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\UC3B_DFP\1.0.29\include\AT32UC3B0512"  -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -Wa,-g   -o "$@" "$<" 
	@echo Finished building: $<
	

src/ASF/avr32/utils/startup/%.o: ../src/ASF/avr32/utils/startup/%.x
	@echo Building file: $<
	@echo Invoking: AVR32/GNU Preprocessing Assembler : 4.4.7
	$(QUOTE)D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr32\avr32-gnu-toolchain\bin\avr32-gcc.exe$(QUOTE) -x assembler-with-cpp -c -mpart=uc3b0512 -mrelax -DBOARD=USER_BOARD -I "../common/applications/user_application/user_board/config" -I "../src/ASF/avr32/utils" -I "../src/config" -I "../src/ASF/common/boards" -I "../src/ASF/avr32/utils/preprocessor" -I "../src/ASF/avr32/drivers/intc" -I "../src/ASF/common/utils" -I "../src" -I "../src/ASF/common/boards/user_board" -I "D:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\UC3B_DFP\1.0.29\include\AT32UC3B0512"  -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -Wa,-g   -o "$@" "$<" 
	@echo Finished building: $<
	

src/ASF/avr32/utils/startup/%.o: ../src/ASF/avr32/utils/startup/%.X
	@echo Building file: $<
	@echo Invoking: AVR32/GNU Preprocessing Assembler : 4.4.7
	$(QUOTE)D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr32\avr32-gnu-toolchain\bin\avr32-gcc.exe$(QUOTE) -x assembler-with-cpp -c -mpart=uc3b0512 -mrelax -DBOARD=USER_BOARD -I "../common/applications/user_application/user_board/config" -I "../src/ASF/avr32/utils" -I "../src/config" -I "../src/ASF/common/boards" -I "../src/ASF/avr32/utils/preprocessor" -I "../src/ASF/avr32/drivers/intc" -I "../src/ASF/common/utils" -I "../src" -I "../src/ASF/common/boards/user_board" -I "D:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\UC3B_DFP\1.0.29\include\AT32UC3B0512"  -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -Wa,-g   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR32/GNU Linker : 4.4.7
	$(QUOTE)D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr32\avr32-gnu-toolchain\bin\avr32-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -nostartfiles -Wl,-Map="accelerometer_demo_6_2.map" -Wl,--start-group -lm  -Wl,--end-group -Wl,--gc-sections --rodata-writable -Wl,--direct-data -mpart=uc3b0512 -Wl,--relax -Wl,-e,_trampoline -T../src/link_uc3b0512.lds  
	@echo Finished building target: $@
	"D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr32\avr32-gnu-toolchain\bin\avr32-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature "accelerometer_demo_6_2.elf" "accelerometer_demo_6_2.hex"
	"D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr32\avr32-gnu-toolchain\bin\avr32-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "accelerometer_demo_6_2.elf" "accelerometer_demo_6_2.eep" || exit 0
	"D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr32\avr32-gnu-toolchain\bin\avr32-objdump.exe" -h -S "accelerometer_demo_6_2.elf" > "accelerometer_demo_6_2.lss"
	"D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr32\avr32-gnu-toolchain\bin\avr32-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "accelerometer_demo_6_2.elf" "accelerometer_demo_6_2.srec"
	"D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr32\avr32-gnu-toolchain\bin\avr32-size.exe" "accelerometer_demo_6_2.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "accelerometer_demo_6_2.elf" "accelerometer_demo_6_2.a" "accelerometer_demo_6_2.hex" "accelerometer_demo_6_2.lss" "accelerometer_demo_6_2.eep" "accelerometer_demo_6_2.map" "accelerometer_demo_6_2.srec" "accelerometer_demo_6_2.usersignatures"
	