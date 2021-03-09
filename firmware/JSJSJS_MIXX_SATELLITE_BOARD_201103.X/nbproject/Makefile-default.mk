#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/JSJSJS_MIXX_SATELLITE_BOARD_201103.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/JSJSJS_MIXX_SATELLITE_BOARD_201103.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/system_config/default/framework/driver/can/src/drv_can_mapping.c ../src/system_config/default/framework/driver/can/src/drv_can_static.c ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c ../src/system_config/default/framework/system/devcon/src/sys_devcon.c ../src/system_config/default/framework/system/devcon/src/sys_devcon_pic32mz.c ../src/system_config/default/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/system_exceptions.c ../src/system_config/default/system_tasks.c ../src/LAN2CAN.c ../src/function_initialization.c ../src/function_utilities.c ../src/HX711.c ../src/main.c ../../../../../../microchip/harmony/v2_06/framework/driver/ethmac/src/dynamic/drv_ethmac.c ../../../../../../microchip/harmony/v2_06/framework/driver/ethmac/src/dynamic/drv_ethmac_lib.c ../../../../../../microchip/harmony/v2_06/framework/driver/ethphy/src/dynamic/drv_ethphy.c ../../../../../../microchip/harmony/v2_06/framework/driver/ethphy/src/dynamic/drv_extphy_smsc8740.c ../../../../../../microchip/harmony/v2_06/framework/driver/miim/src/dynamic/drv_miim.c ../../../../../../microchip/harmony/v2_06/framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c ../../../../../../microchip/harmony/v2_06/framework/system/random/src/sys_random.c ../../../../../../microchip/harmony/v2_06/framework/system/tmr/src/sys_tmr.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/common/helpers.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/ipv4.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcp.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_heap_alloc.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_heap_internal.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/arp.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/dhcp.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/dns.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/icmp.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/hash_fnv.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/oahash.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_helpers.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_helper_c32.S ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_manager.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_notify.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_packet.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/337818580/drv_can_mapping.o ${OBJECTDIR}/_ext/337818580/drv_can_static.o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ${OBJECTDIR}/_ext/340578644/sys_devcon.o ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1360937237/LAN2CAN.o ${OBJECTDIR}/_ext/1360937237/function_initialization.o ${OBJECTDIR}/_ext/1360937237/function_utilities.o ${OBJECTDIR}/_ext/1360937237/HX711.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1486592573/drv_ethmac.o ${OBJECTDIR}/_ext/1486592573/drv_ethmac_lib.o ${OBJECTDIR}/_ext/1632220917/drv_ethphy.o ${OBJECTDIR}/_ext/1632220917/drv_extphy_smsc8740.o ${OBJECTDIR}/_ext/1426996077/drv_miim.o ${OBJECTDIR}/_ext/709153290/drv_tmr.o ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o ${OBJECTDIR}/_ext/655575975/sys_random.o ${OBJECTDIR}/_ext/910924237/sys_tmr.o ${OBJECTDIR}/_ext/2030780900/helpers.o ${OBJECTDIR}/_ext/361655554/ipv4.o ${OBJECTDIR}/_ext/361655554/tcp.o ${OBJECTDIR}/_ext/361655554/tcpip_heap_alloc.o ${OBJECTDIR}/_ext/361655554/tcpip_heap_internal.o ${OBJECTDIR}/_ext/361655554/arp.o ${OBJECTDIR}/_ext/361655554/dhcp.o ${OBJECTDIR}/_ext/361655554/dns.o ${OBJECTDIR}/_ext/361655554/icmp.o ${OBJECTDIR}/_ext/361655554/hash_fnv.o ${OBJECTDIR}/_ext/361655554/oahash.o ${OBJECTDIR}/_ext/361655554/tcpip_helpers.o ${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o ${OBJECTDIR}/_ext/361655554/tcpip_manager.o ${OBJECTDIR}/_ext/361655554/tcpip_notify.o ${OBJECTDIR}/_ext/361655554/tcpip_packet.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/337818580/drv_can_mapping.o.d ${OBJECTDIR}/_ext/337818580/drv_can_static.o.d ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d ${OBJECTDIR}/_ext/340578644/sys_devcon.o.d ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o.d ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.d ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d ${OBJECTDIR}/_ext/1688732426/system_init.o.d ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d ${OBJECTDIR}/_ext/1360937237/LAN2CAN.o.d ${OBJECTDIR}/_ext/1360937237/function_initialization.o.d ${OBJECTDIR}/_ext/1360937237/function_utilities.o.d ${OBJECTDIR}/_ext/1360937237/HX711.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1486592573/drv_ethmac.o.d ${OBJECTDIR}/_ext/1486592573/drv_ethmac_lib.o.d ${OBJECTDIR}/_ext/1632220917/drv_ethphy.o.d ${OBJECTDIR}/_ext/1632220917/drv_extphy_smsc8740.o.d ${OBJECTDIR}/_ext/1426996077/drv_miim.o.d ${OBJECTDIR}/_ext/709153290/drv_tmr.o.d ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o.d ${OBJECTDIR}/_ext/655575975/sys_random.o.d ${OBJECTDIR}/_ext/910924237/sys_tmr.o.d ${OBJECTDIR}/_ext/2030780900/helpers.o.d ${OBJECTDIR}/_ext/361655554/ipv4.o.d ${OBJECTDIR}/_ext/361655554/tcp.o.d ${OBJECTDIR}/_ext/361655554/tcpip_heap_alloc.o.d ${OBJECTDIR}/_ext/361655554/tcpip_heap_internal.o.d ${OBJECTDIR}/_ext/361655554/arp.o.d ${OBJECTDIR}/_ext/361655554/dhcp.o.d ${OBJECTDIR}/_ext/361655554/dns.o.d ${OBJECTDIR}/_ext/361655554/icmp.o.d ${OBJECTDIR}/_ext/361655554/hash_fnv.o.d ${OBJECTDIR}/_ext/361655554/oahash.o.d ${OBJECTDIR}/_ext/361655554/tcpip_helpers.o.d ${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o.d ${OBJECTDIR}/_ext/361655554/tcpip_manager.o.d ${OBJECTDIR}/_ext/361655554/tcpip_notify.o.d ${OBJECTDIR}/_ext/361655554/tcpip_packet.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/337818580/drv_can_mapping.o ${OBJECTDIR}/_ext/337818580/drv_can_static.o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ${OBJECTDIR}/_ext/340578644/sys_devcon.o ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1360937237/LAN2CAN.o ${OBJECTDIR}/_ext/1360937237/function_initialization.o ${OBJECTDIR}/_ext/1360937237/function_utilities.o ${OBJECTDIR}/_ext/1360937237/HX711.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1486592573/drv_ethmac.o ${OBJECTDIR}/_ext/1486592573/drv_ethmac_lib.o ${OBJECTDIR}/_ext/1632220917/drv_ethphy.o ${OBJECTDIR}/_ext/1632220917/drv_extphy_smsc8740.o ${OBJECTDIR}/_ext/1426996077/drv_miim.o ${OBJECTDIR}/_ext/709153290/drv_tmr.o ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o ${OBJECTDIR}/_ext/655575975/sys_random.o ${OBJECTDIR}/_ext/910924237/sys_tmr.o ${OBJECTDIR}/_ext/2030780900/helpers.o ${OBJECTDIR}/_ext/361655554/ipv4.o ${OBJECTDIR}/_ext/361655554/tcp.o ${OBJECTDIR}/_ext/361655554/tcpip_heap_alloc.o ${OBJECTDIR}/_ext/361655554/tcpip_heap_internal.o ${OBJECTDIR}/_ext/361655554/arp.o ${OBJECTDIR}/_ext/361655554/dhcp.o ${OBJECTDIR}/_ext/361655554/dns.o ${OBJECTDIR}/_ext/361655554/icmp.o ${OBJECTDIR}/_ext/361655554/hash_fnv.o ${OBJECTDIR}/_ext/361655554/oahash.o ${OBJECTDIR}/_ext/361655554/tcpip_helpers.o ${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o ${OBJECTDIR}/_ext/361655554/tcpip_manager.o ${OBJECTDIR}/_ext/361655554/tcpip_notify.o ${OBJECTDIR}/_ext/361655554/tcpip_packet.o

# Source Files
SOURCEFILES=../src/system_config/default/framework/driver/can/src/drv_can_mapping.c ../src/system_config/default/framework/driver/can/src/drv_can_static.c ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c ../src/system_config/default/framework/system/devcon/src/sys_devcon.c ../src/system_config/default/framework/system/devcon/src/sys_devcon_pic32mz.c ../src/system_config/default/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/system_exceptions.c ../src/system_config/default/system_tasks.c ../src/LAN2CAN.c ../src/function_initialization.c ../src/function_utilities.c ../src/HX711.c ../src/main.c ../../../../../../microchip/harmony/v2_06/framework/driver/ethmac/src/dynamic/drv_ethmac.c ../../../../../../microchip/harmony/v2_06/framework/driver/ethmac/src/dynamic/drv_ethmac_lib.c ../../../../../../microchip/harmony/v2_06/framework/driver/ethphy/src/dynamic/drv_ethphy.c ../../../../../../microchip/harmony/v2_06/framework/driver/ethphy/src/dynamic/drv_extphy_smsc8740.c ../../../../../../microchip/harmony/v2_06/framework/driver/miim/src/dynamic/drv_miim.c ../../../../../../microchip/harmony/v2_06/framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c ../../../../../../microchip/harmony/v2_06/framework/system/random/src/sys_random.c ../../../../../../microchip/harmony/v2_06/framework/system/tmr/src/sys_tmr.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/common/helpers.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/ipv4.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcp.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_heap_alloc.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_heap_internal.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/arp.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/dhcp.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/dns.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/icmp.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/hash_fnv.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/oahash.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_helpers.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_helper_c32.S ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_manager.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_notify.c ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_packet.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/JSJSJS_MIXX_SATELLITE_BOARD_201103.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MZ2048EFH064
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o: ../src/system_config/default/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/340578644" 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1 -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o ../src/system_config/default/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD4=1
	
${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_helper_c32.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o.ok ${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o.d" "${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1 -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o.d"  -o ${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_helper_c32.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD4=1
	
else
${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o: ../src/system_config/default/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/340578644" 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o ../src/system_config/default/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.asm.d",--gdwarf-2
	
${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_helper_c32.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o.ok ${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o.d" "${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o.d"  -o ${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_helper_c32.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/361655554/tcpip_helper_c32.o.asm.d",--gdwarf-2
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/337818580/drv_can_mapping.o: ../src/system_config/default/framework/driver/can/src/drv_can_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/337818580" 
	@${RM} ${OBJECTDIR}/_ext/337818580/drv_can_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/337818580/drv_can_mapping.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/337818580/drv_can_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/337818580/drv_can_mapping.o.d" -o ${OBJECTDIR}/_ext/337818580/drv_can_mapping.o ../src/system_config/default/framework/driver/can/src/drv_can_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/337818580/drv_can_static.o: ../src/system_config/default/framework/driver/can/src/drv_can_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/337818580" 
	@${RM} ${OBJECTDIR}/_ext/337818580/drv_can_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/337818580/drv_can_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/337818580/drv_can_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/337818580/drv_can_static.o.d" -o ${OBJECTDIR}/_ext/337818580/drv_can_static.o ../src/system_config/default/framework/driver/can/src/drv_can_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o: ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/340578644/sys_devcon.o: ../src/system_config/default/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/340578644" 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/340578644/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/340578644/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/340578644/sys_devcon.o ../src/system_config/default/framework/system/devcon/src/sys_devcon.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o: ../src/system_config/default/framework/system/devcon/src/sys_devcon_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/340578644" 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o ../src/system_config/default/framework/system/devcon/src/sys_devcon_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_exceptions.o: ../src/system_config/default/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ../src/system_config/default/system_exceptions.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/LAN2CAN.o: ../src/LAN2CAN.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/LAN2CAN.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/LAN2CAN.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/LAN2CAN.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/LAN2CAN.o.d" -o ${OBJECTDIR}/_ext/1360937237/LAN2CAN.o ../src/LAN2CAN.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/function_initialization.o: ../src/function_initialization.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/function_initialization.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/function_initialization.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/function_initialization.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/function_initialization.o.d" -o ${OBJECTDIR}/_ext/1360937237/function_initialization.o ../src/function_initialization.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/function_utilities.o: ../src/function_utilities.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/function_utilities.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/function_utilities.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/function_utilities.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/function_utilities.o.d" -o ${OBJECTDIR}/_ext/1360937237/function_utilities.o ../src/function_utilities.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/HX711.o: ../src/HX711.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/HX711.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/HX711.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/HX711.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/HX711.o.d" -o ${OBJECTDIR}/_ext/1360937237/HX711.o ../src/HX711.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1486592573/drv_ethmac.o: ../../../../../../microchip/harmony/v2_06/framework/driver/ethmac/src/dynamic/drv_ethmac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1486592573" 
	@${RM} ${OBJECTDIR}/_ext/1486592573/drv_ethmac.o.d 
	@${RM} ${OBJECTDIR}/_ext/1486592573/drv_ethmac.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1486592573/drv_ethmac.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1486592573/drv_ethmac.o.d" -o ${OBJECTDIR}/_ext/1486592573/drv_ethmac.o ../../../../../../microchip/harmony/v2_06/framework/driver/ethmac/src/dynamic/drv_ethmac.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1486592573/drv_ethmac_lib.o: ../../../../../../microchip/harmony/v2_06/framework/driver/ethmac/src/dynamic/drv_ethmac_lib.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1486592573" 
	@${RM} ${OBJECTDIR}/_ext/1486592573/drv_ethmac_lib.o.d 
	@${RM} ${OBJECTDIR}/_ext/1486592573/drv_ethmac_lib.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1486592573/drv_ethmac_lib.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1486592573/drv_ethmac_lib.o.d" -o ${OBJECTDIR}/_ext/1486592573/drv_ethmac_lib.o ../../../../../../microchip/harmony/v2_06/framework/driver/ethmac/src/dynamic/drv_ethmac_lib.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1632220917/drv_ethphy.o: ../../../../../../microchip/harmony/v2_06/framework/driver/ethphy/src/dynamic/drv_ethphy.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1632220917" 
	@${RM} ${OBJECTDIR}/_ext/1632220917/drv_ethphy.o.d 
	@${RM} ${OBJECTDIR}/_ext/1632220917/drv_ethphy.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1632220917/drv_ethphy.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1632220917/drv_ethphy.o.d" -o ${OBJECTDIR}/_ext/1632220917/drv_ethphy.o ../../../../../../microchip/harmony/v2_06/framework/driver/ethphy/src/dynamic/drv_ethphy.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1632220917/drv_extphy_smsc8740.o: ../../../../../../microchip/harmony/v2_06/framework/driver/ethphy/src/dynamic/drv_extphy_smsc8740.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1632220917" 
	@${RM} ${OBJECTDIR}/_ext/1632220917/drv_extphy_smsc8740.o.d 
	@${RM} ${OBJECTDIR}/_ext/1632220917/drv_extphy_smsc8740.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1632220917/drv_extphy_smsc8740.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1632220917/drv_extphy_smsc8740.o.d" -o ${OBJECTDIR}/_ext/1632220917/drv_extphy_smsc8740.o ../../../../../../microchip/harmony/v2_06/framework/driver/ethphy/src/dynamic/drv_extphy_smsc8740.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1426996077/drv_miim.o: ../../../../../../microchip/harmony/v2_06/framework/driver/miim/src/dynamic/drv_miim.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1426996077" 
	@${RM} ${OBJECTDIR}/_ext/1426996077/drv_miim.o.d 
	@${RM} ${OBJECTDIR}/_ext/1426996077/drv_miim.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1426996077/drv_miim.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1426996077/drv_miim.o.d" -o ${OBJECTDIR}/_ext/1426996077/drv_miim.o ../../../../../../microchip/harmony/v2_06/framework/driver/miim/src/dynamic/drv_miim.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/709153290/drv_tmr.o: ../../../../../../microchip/harmony/v2_06/framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/709153290" 
	@${RM} ${OBJECTDIR}/_ext/709153290/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/709153290/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/709153290/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/709153290/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/709153290/drv_tmr.o ../../../../../../microchip/harmony/v2_06/framework/driver/tmr/src/dynamic/drv_tmr.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/231205469/sys_int_pic32.o: ../../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/231205469" 
	@${RM} ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/231205469/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/231205469/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o ../../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/655575975/sys_random.o: ../../../../../../microchip/harmony/v2_06/framework/system/random/src/sys_random.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/655575975" 
	@${RM} ${OBJECTDIR}/_ext/655575975/sys_random.o.d 
	@${RM} ${OBJECTDIR}/_ext/655575975/sys_random.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/655575975/sys_random.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/655575975/sys_random.o.d" -o ${OBJECTDIR}/_ext/655575975/sys_random.o ../../../../../../microchip/harmony/v2_06/framework/system/random/src/sys_random.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/910924237/sys_tmr.o: ../../../../../../microchip/harmony/v2_06/framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/910924237" 
	@${RM} ${OBJECTDIR}/_ext/910924237/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/910924237/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/910924237/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/910924237/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/910924237/sys_tmr.o ../../../../../../microchip/harmony/v2_06/framework/system/tmr/src/sys_tmr.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2030780900/helpers.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/common/helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2030780900" 
	@${RM} ${OBJECTDIR}/_ext/2030780900/helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/2030780900/helpers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2030780900/helpers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/2030780900/helpers.o.d" -o ${OBJECTDIR}/_ext/2030780900/helpers.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/common/helpers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/ipv4.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/ipv4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/ipv4.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/ipv4.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/ipv4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/ipv4.o.d" -o ${OBJECTDIR}/_ext/361655554/ipv4.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/ipv4.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/tcp.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/tcp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/tcp.o.d" -o ${OBJECTDIR}/_ext/361655554/tcp.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcp.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/tcpip_heap_alloc.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_heap_alloc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_heap_alloc.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_heap_alloc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/tcpip_heap_alloc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/tcpip_heap_alloc.o.d" -o ${OBJECTDIR}/_ext/361655554/tcpip_heap_alloc.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_heap_alloc.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/tcpip_heap_internal.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_heap_internal.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_heap_internal.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_heap_internal.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/tcpip_heap_internal.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/tcpip_heap_internal.o.d" -o ${OBJECTDIR}/_ext/361655554/tcpip_heap_internal.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_heap_internal.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/arp.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/arp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/arp.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/arp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/arp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/arp.o.d" -o ${OBJECTDIR}/_ext/361655554/arp.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/arp.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/dhcp.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/dhcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/dhcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/dhcp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/dhcp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/dhcp.o.d" -o ${OBJECTDIR}/_ext/361655554/dhcp.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/dhcp.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/dns.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/dns.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/dns.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/dns.o.d" -o ${OBJECTDIR}/_ext/361655554/dns.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/dns.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/icmp.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/icmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/icmp.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/icmp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/icmp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/icmp.o.d" -o ${OBJECTDIR}/_ext/361655554/icmp.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/icmp.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/hash_fnv.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/hash_fnv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/hash_fnv.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/hash_fnv.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/hash_fnv.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/hash_fnv.o.d" -o ${OBJECTDIR}/_ext/361655554/hash_fnv.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/hash_fnv.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/oahash.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/oahash.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/oahash.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/oahash.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/oahash.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/oahash.o.d" -o ${OBJECTDIR}/_ext/361655554/oahash.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/oahash.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/tcpip_helpers.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_helpers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/tcpip_helpers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/tcpip_helpers.o.d" -o ${OBJECTDIR}/_ext/361655554/tcpip_helpers.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_helpers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/tcpip_manager.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_manager.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/tcpip_manager.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/tcpip_manager.o.d" -o ${OBJECTDIR}/_ext/361655554/tcpip_manager.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_manager.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/tcpip_notify.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_notify.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_notify.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_notify.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/tcpip_notify.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/tcpip_notify.o.d" -o ${OBJECTDIR}/_ext/361655554/tcpip_notify.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_notify.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/tcpip_packet.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_packet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_packet.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_packet.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/tcpip_packet.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD4=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/tcpip_packet.o.d" -o ${OBJECTDIR}/_ext/361655554/tcpip_packet.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_packet.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
else
${OBJECTDIR}/_ext/337818580/drv_can_mapping.o: ../src/system_config/default/framework/driver/can/src/drv_can_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/337818580" 
	@${RM} ${OBJECTDIR}/_ext/337818580/drv_can_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/337818580/drv_can_mapping.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/337818580/drv_can_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/337818580/drv_can_mapping.o.d" -o ${OBJECTDIR}/_ext/337818580/drv_can_mapping.o ../src/system_config/default/framework/driver/can/src/drv_can_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/337818580/drv_can_static.o: ../src/system_config/default/framework/driver/can/src/drv_can_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/337818580" 
	@${RM} ${OBJECTDIR}/_ext/337818580/drv_can_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/337818580/drv_can_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/337818580/drv_can_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/337818580/drv_can_static.o.d" -o ${OBJECTDIR}/_ext/337818580/drv_can_static.o ../src/system_config/default/framework/driver/can/src/drv_can_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o: ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/340578644/sys_devcon.o: ../src/system_config/default/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/340578644" 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/340578644/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/340578644/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/340578644/sys_devcon.o ../src/system_config/default/framework/system/devcon/src/sys_devcon.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o: ../src/system_config/default/framework/system/devcon/src/sys_devcon_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/340578644" 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o ../src/system_config/default/framework/system/devcon/src/sys_devcon_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_exceptions.o: ../src/system_config/default/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ../src/system_config/default/system_exceptions.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/LAN2CAN.o: ../src/LAN2CAN.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/LAN2CAN.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/LAN2CAN.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/LAN2CAN.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/LAN2CAN.o.d" -o ${OBJECTDIR}/_ext/1360937237/LAN2CAN.o ../src/LAN2CAN.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/function_initialization.o: ../src/function_initialization.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/function_initialization.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/function_initialization.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/function_initialization.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/function_initialization.o.d" -o ${OBJECTDIR}/_ext/1360937237/function_initialization.o ../src/function_initialization.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/function_utilities.o: ../src/function_utilities.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/function_utilities.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/function_utilities.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/function_utilities.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/function_utilities.o.d" -o ${OBJECTDIR}/_ext/1360937237/function_utilities.o ../src/function_utilities.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/HX711.o: ../src/HX711.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/HX711.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/HX711.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/HX711.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/HX711.o.d" -o ${OBJECTDIR}/_ext/1360937237/HX711.o ../src/HX711.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1486592573/drv_ethmac.o: ../../../../../../microchip/harmony/v2_06/framework/driver/ethmac/src/dynamic/drv_ethmac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1486592573" 
	@${RM} ${OBJECTDIR}/_ext/1486592573/drv_ethmac.o.d 
	@${RM} ${OBJECTDIR}/_ext/1486592573/drv_ethmac.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1486592573/drv_ethmac.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1486592573/drv_ethmac.o.d" -o ${OBJECTDIR}/_ext/1486592573/drv_ethmac.o ../../../../../../microchip/harmony/v2_06/framework/driver/ethmac/src/dynamic/drv_ethmac.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1486592573/drv_ethmac_lib.o: ../../../../../../microchip/harmony/v2_06/framework/driver/ethmac/src/dynamic/drv_ethmac_lib.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1486592573" 
	@${RM} ${OBJECTDIR}/_ext/1486592573/drv_ethmac_lib.o.d 
	@${RM} ${OBJECTDIR}/_ext/1486592573/drv_ethmac_lib.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1486592573/drv_ethmac_lib.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1486592573/drv_ethmac_lib.o.d" -o ${OBJECTDIR}/_ext/1486592573/drv_ethmac_lib.o ../../../../../../microchip/harmony/v2_06/framework/driver/ethmac/src/dynamic/drv_ethmac_lib.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1632220917/drv_ethphy.o: ../../../../../../microchip/harmony/v2_06/framework/driver/ethphy/src/dynamic/drv_ethphy.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1632220917" 
	@${RM} ${OBJECTDIR}/_ext/1632220917/drv_ethphy.o.d 
	@${RM} ${OBJECTDIR}/_ext/1632220917/drv_ethphy.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1632220917/drv_ethphy.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1632220917/drv_ethphy.o.d" -o ${OBJECTDIR}/_ext/1632220917/drv_ethphy.o ../../../../../../microchip/harmony/v2_06/framework/driver/ethphy/src/dynamic/drv_ethphy.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1632220917/drv_extphy_smsc8740.o: ../../../../../../microchip/harmony/v2_06/framework/driver/ethphy/src/dynamic/drv_extphy_smsc8740.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1632220917" 
	@${RM} ${OBJECTDIR}/_ext/1632220917/drv_extphy_smsc8740.o.d 
	@${RM} ${OBJECTDIR}/_ext/1632220917/drv_extphy_smsc8740.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1632220917/drv_extphy_smsc8740.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1632220917/drv_extphy_smsc8740.o.d" -o ${OBJECTDIR}/_ext/1632220917/drv_extphy_smsc8740.o ../../../../../../microchip/harmony/v2_06/framework/driver/ethphy/src/dynamic/drv_extphy_smsc8740.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1426996077/drv_miim.o: ../../../../../../microchip/harmony/v2_06/framework/driver/miim/src/dynamic/drv_miim.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1426996077" 
	@${RM} ${OBJECTDIR}/_ext/1426996077/drv_miim.o.d 
	@${RM} ${OBJECTDIR}/_ext/1426996077/drv_miim.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1426996077/drv_miim.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/1426996077/drv_miim.o.d" -o ${OBJECTDIR}/_ext/1426996077/drv_miim.o ../../../../../../microchip/harmony/v2_06/framework/driver/miim/src/dynamic/drv_miim.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/709153290/drv_tmr.o: ../../../../../../microchip/harmony/v2_06/framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/709153290" 
	@${RM} ${OBJECTDIR}/_ext/709153290/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/709153290/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/709153290/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/709153290/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/709153290/drv_tmr.o ../../../../../../microchip/harmony/v2_06/framework/driver/tmr/src/dynamic/drv_tmr.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/231205469/sys_int_pic32.o: ../../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/231205469" 
	@${RM} ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/231205469/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/231205469/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o ../../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/655575975/sys_random.o: ../../../../../../microchip/harmony/v2_06/framework/system/random/src/sys_random.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/655575975" 
	@${RM} ${OBJECTDIR}/_ext/655575975/sys_random.o.d 
	@${RM} ${OBJECTDIR}/_ext/655575975/sys_random.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/655575975/sys_random.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/655575975/sys_random.o.d" -o ${OBJECTDIR}/_ext/655575975/sys_random.o ../../../../../../microchip/harmony/v2_06/framework/system/random/src/sys_random.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/910924237/sys_tmr.o: ../../../../../../microchip/harmony/v2_06/framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/910924237" 
	@${RM} ${OBJECTDIR}/_ext/910924237/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/910924237/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/910924237/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/910924237/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/910924237/sys_tmr.o ../../../../../../microchip/harmony/v2_06/framework/system/tmr/src/sys_tmr.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2030780900/helpers.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/common/helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2030780900" 
	@${RM} ${OBJECTDIR}/_ext/2030780900/helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/2030780900/helpers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2030780900/helpers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/2030780900/helpers.o.d" -o ${OBJECTDIR}/_ext/2030780900/helpers.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/common/helpers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/ipv4.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/ipv4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/ipv4.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/ipv4.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/ipv4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/ipv4.o.d" -o ${OBJECTDIR}/_ext/361655554/ipv4.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/ipv4.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/tcp.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/tcp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/tcp.o.d" -o ${OBJECTDIR}/_ext/361655554/tcp.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcp.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/tcpip_heap_alloc.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_heap_alloc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_heap_alloc.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_heap_alloc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/tcpip_heap_alloc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/tcpip_heap_alloc.o.d" -o ${OBJECTDIR}/_ext/361655554/tcpip_heap_alloc.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_heap_alloc.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/tcpip_heap_internal.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_heap_internal.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_heap_internal.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_heap_internal.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/tcpip_heap_internal.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/tcpip_heap_internal.o.d" -o ${OBJECTDIR}/_ext/361655554/tcpip_heap_internal.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_heap_internal.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/arp.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/arp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/arp.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/arp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/arp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/arp.o.d" -o ${OBJECTDIR}/_ext/361655554/arp.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/arp.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/dhcp.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/dhcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/dhcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/dhcp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/dhcp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/dhcp.o.d" -o ${OBJECTDIR}/_ext/361655554/dhcp.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/dhcp.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/dns.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/dns.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/dns.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/dns.o.d" -o ${OBJECTDIR}/_ext/361655554/dns.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/dns.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/icmp.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/icmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/icmp.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/icmp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/icmp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/icmp.o.d" -o ${OBJECTDIR}/_ext/361655554/icmp.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/icmp.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/hash_fnv.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/hash_fnv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/hash_fnv.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/hash_fnv.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/hash_fnv.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/hash_fnv.o.d" -o ${OBJECTDIR}/_ext/361655554/hash_fnv.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/hash_fnv.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/oahash.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/oahash.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/oahash.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/oahash.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/oahash.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/oahash.o.d" -o ${OBJECTDIR}/_ext/361655554/oahash.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/oahash.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/tcpip_helpers.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_helpers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/tcpip_helpers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/tcpip_helpers.o.d" -o ${OBJECTDIR}/_ext/361655554/tcpip_helpers.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_helpers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/tcpip_manager.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_manager.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/tcpip_manager.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/tcpip_manager.o.d" -o ${OBJECTDIR}/_ext/361655554/tcpip_manager.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_manager.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/tcpip_notify.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_notify.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_notify.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_notify.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/tcpip_notify.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/tcpip_notify.o.d" -o ${OBJECTDIR}/_ext/361655554/tcpip_notify.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_notify.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/361655554/tcpip_packet.o: ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_packet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361655554" 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_packet.o.d 
	@${RM} ${OBJECTDIR}/_ext/361655554/tcpip_packet.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361655554/tcpip_packet.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../../microchip/harmony/v2_06/framework" -MMD -MF "${OBJECTDIR}/_ext/361655554/tcpip_packet.o.d" -o ${OBJECTDIR}/_ext/361655554/tcpip_packet.o ../../../../../../microchip/harmony/v2_06/framework/tcpip/src/tcpip_packet.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/JSJSJS_MIXX_SATELLITE_BOARD_201103.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../microchip/harmony/v2_06/bin/framework/peripheral/PIC32MZ2048EFH064_peripherals.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_ICD4=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/JSJSJS_MIXX_SATELLITE_BOARD_201103.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ..\..\..\..\..\..\microchip\harmony\v2_06\bin\framework\peripheral\PIC32MZ2048EFH064_peripherals.a      -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x0:0x37F   -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=__MPLAB_DEBUGGER_ICD4=1,--defsym=_min_heap_size=44960,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/JSJSJS_MIXX_SATELLITE_BOARD_201103.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../microchip/harmony/v2_06/bin/framework/peripheral/PIC32MZ2048EFH064_peripherals.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/JSJSJS_MIXX_SATELLITE_BOARD_201103.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ..\..\..\..\..\..\microchip\harmony\v2_06\bin\framework\peripheral\PIC32MZ2048EFH064_peripherals.a      -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=44960,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/JSJSJS_MIXX_SATELLITE_BOARD_201103.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
