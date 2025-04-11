/* Automatically generated file; DO NOT EDIT */
/* Espressif IoT Development Framework Linker Script */
/* Generated from: /Users/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/esp_system/ld/esp32/sections.ld.in */

/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*  Default entry point:  */
ENTRY(call_start_cpu0);

SECTIONS
{
  /* RTC fast memory holds RTC wake stub code,
     including from any source file named rtc_wake_stub*.c
  */
  .rtc.text :
  {
    . = ALIGN(4);

    *(.rtc.literal .rtc.text .rtc.text.*)

    *rtc_wake_stub*.*(.literal .text .literal.* .text.*)
    _rtc_text_end = ABSOLUTE(.);
  } > rtc_iram_seg

  /*
    This section is required to skip rtc.text area because rtc_iram_seg and
    rtc_data_seg are reflect the same address space on different buses.
  */
  .rtc.dummy :
  {
    _rtc_dummy_start = ABSOLUTE(.);
    _rtc_fast_start = ABSOLUTE(.);
    . = SIZEOF(.rtc.text);
    _rtc_dummy_end = ABSOLUTE(.);
  } > rtc_data_seg

  /* This section located in RTC FAST Memory area.
     It holds data marked with RTC_FAST_ATTR attribute.
     See the file "esp_attr.h" for more information.
  */
  .rtc.force_fast :
  {
    . = ALIGN(4);
    _rtc_force_fast_start = ABSOLUTE(.);

    _coredump_rtc_fast_start = ABSOLUTE(.);
    *(.rtc.fast.coredump .rtc.fast.coredump.*)
    _coredump_rtc_fast_end = ABSOLUTE(.);

    *(.rtc.force_fast .rtc.force_fast.*)
    . = ALIGN(4) ;

    _rtc_force_fast_end = ABSOLUTE(.);
  } > rtc_data_seg

  /* RTC data section holds RTC wake stub
     data/rodata, including from any source file
     named rtc_wake_stub*.c and the data marked with
     RTC_DATA_ATTR, RTC_RODATA_ATTR attributes.
     The memory location of the data is dependent on
     CONFIG_ESP32_RTCDATA_IN_FAST_MEM option.
  */
  .rtc.data :
  {
    _rtc_data_start = ABSOLUTE(.);

    _coredump_rtc_start = ABSOLUTE(.);
    *(.rtc.coredump .rtc.coredump.*)
    _coredump_rtc_end = ABSOLUTE(.);
    *(.rtc.data .rtc.data.*)
    *(.rtc.rodata .rtc.rodata.*)

    *rtc_wake_stub*.*(.data .rodata .data.* .rodata.*)
    _rtc_data_end = ABSOLUTE(.);

  } > rtc_data_location

  /* RTC bss, from any source file named rtc_wake_stub*.c */
  .rtc.bss (NOLOAD) :
  {
    _rtc_bss_start = ABSOLUTE(.);
    *rtc_wake_stub*.*(.bss .bss.*)
    *rtc_wake_stub*.*(COMMON)

    *(.rtc.bss)

    _rtc_bss_end = ABSOLUTE(.);
  } > rtc_data_location

  /* This section holds data that should not be initialized at power up
     and will be retained during deep sleep.
     User data marked with RTC_NOINIT_ATTR will be placed
     into this section. See the file "esp_attr.h" for more information.
	 The memory location of the data is dependent on
     CONFIG_ESP32_RTCDATA_IN_FAST_MEM option.
  */
  .rtc_noinit (NOLOAD):
  {
    . = ALIGN(4);
    _rtc_noinit_start = ABSOLUTE(.);
    *(.rtc_noinit .rtc_noinit.*)
    . = ALIGN(4) ;
    _rtc_noinit_end = ABSOLUTE(.);
  } > rtc_data_location

  /* This section located in RTC SLOW Memory area.
     It holds data marked with RTC_SLOW_ATTR attribute.
     See the file "esp_attr.h" for more information.
  */
  .rtc.force_slow :
  {
    . = ALIGN(4);
    _rtc_force_slow_start = ABSOLUTE(.);
    *(.rtc.force_slow .rtc.force_slow.*)
    . = ALIGN(4) ;
    _rtc_force_slow_end = ABSOLUTE(.);
  } > rtc_slow_seg

  /**
   * This section holds RTC FAST data that should have fixed addresses.
   * The data are not initialized at power-up and are retained during deep sleep.
   */
  .rtc_fast_reserved (NOLOAD):
  {
    . = ALIGN(4);
    _rtc_fast_reserved_start = ABSOLUTE(.);
    /* New data can only be added here to ensure existing data are not moved.
       Because data have adhered to the end of the segment and code is relied on it.
       >> put new data here << */
    KEEP(*(.bootloader_data_rtc_mem .bootloader_data_rtc_mem.*))
    _rtc_fast_reserved_end = ABSOLUTE(.);
  } > rtc_fast_reserved_seg

  _rtc_fast_reserved_length = _rtc_fast_reserved_end - _rtc_fast_reserved_start;
  ASSERT((_rtc_fast_reserved_length <= LENGTH(rtc_fast_reserved_seg)),
          "RTC FAST reserved segment data does not fit.")

  /**
   * This section holds RTC SLOW data that should have fixed addresses.
   * The data are not initialized at power-up and are retained during deep sleep.
   */
  .rtc_slow_reserved (NOLOAD):
  {
    . = ALIGN(4);
    _rtc_slow_reserved_start = ABSOLUTE(.);
    /* New data can only be added here to ensure existing data are not moved.
       Because data have adhered to the end of the segment and code is relied on it.
       >> put new data here << */

    *(.rtc_timer_data_in_rtc_mem .rtc_timer_data_in_rtc_mem.*)
    _rtc_slow_reserved_end = ABSOLUTE(.);
  } > rtc_slow_reserved_seg

  _rtc_slow_reserved_length = _rtc_slow_reserved_end - _rtc_slow_reserved_start;
  _rtc_reserved_length = _rtc_slow_reserved_length;
  ASSERT((_rtc_slow_reserved_length <= LENGTH(rtc_slow_reserved_seg)),
          "RTC SLOW reserved segment data does not fit.")

  /* Get size of rtc slow data based on rtc_data_location alias */
  _rtc_slow_length = (ORIGIN(rtc_slow_seg) == ORIGIN(rtc_data_location))
                        ? (_rtc_force_slow_end - _rtc_data_start)
                        : (_rtc_force_slow_end - _rtc_force_slow_start);

  _rtc_fast_length = (ORIGIN(rtc_slow_seg) == ORIGIN(rtc_data_location))
                        ? (_rtc_force_fast_end - _rtc_fast_start)
                        : (_rtc_noinit_end - _rtc_fast_start);

  ASSERT((_rtc_slow_length <= LENGTH(rtc_slow_seg)),
          "RTC_SLOW segment data does not fit.")

  ASSERT((_rtc_fast_length <= LENGTH(rtc_data_seg)),
          "RTC_FAST segment data does not fit.")

  /* Send .iram0 code to iram */
  .iram0.vectors :
  {
    _iram_start = ABSOLUTE(.);
    /* Vectors go to IRAM */
    _vector_table = ABSOLUTE(.);
    /* Vectors according to builds/RF-2015.2-win32/esp108_v1_2_s5_512int_2/config.html */
    . = 0x0;
    KEEP(*(.WindowVectors.text));
    . = 0x180;
    KEEP(*(.Level2InterruptVector.text));
    . = 0x1c0;
    KEEP(*(.Level3InterruptVector.text));
    . = 0x200;
    KEEP(*(.Level4InterruptVector.text));
    . = 0x240;
    KEEP(*(.Level5InterruptVector.text));
    . = 0x280;
    KEEP(*(.DebugExceptionVector.text));
    . = 0x2c0;
    KEEP(*(.NMIExceptionVector.text));
    . = 0x300;
    KEEP(*(.KernelExceptionVector.text));
    . = 0x340;
    KEEP(*(.UserExceptionVector.text));
    . = 0x3C0;
    KEEP(*(.DoubleExceptionVector.text));
    . = 0x400;
    _invalid_pc_placeholder = ABSOLUTE(.);
    *(.*Vector.literal)

    *(.UserEnter.literal);
    *(.UserEnter.text);
    . = ALIGN (16);
    *(.entry.text)
    *(.init.literal)
    *(.init)

    _init_end = ABSOLUTE(.);
  } > iram0_0_seg

  .iram0.text :
  {
    /* Code marked as runnning out of IRAM */
    _iram_text_start = ABSOLUTE(.);

    *(.iram1 .iram1.*)
    *libapp_trace.a:app_trace.*(.literal .literal.* .text .text.*)
    *libapp_trace.a:app_trace_util.*(.literal .literal.* .text .text.*)
    *libapp_trace.a:port_uart.*(.literal .literal.* .text .text.*)
    *libc_nano.a:creat.*(.literal .literal.* .text .text.*)
    *libc_nano.a:isatty.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-abs.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-asctime.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-asctime_r.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-atoi.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-atol.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-bzero.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-close.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-creat.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-ctime.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-ctime_r.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-ctype_.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-div.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-environ.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-envlock.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-fclose.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-fflush.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-findfp.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-fputwc.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-fvwrite.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-fwalk.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-getenv_r.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-gettzinfo.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-gmtime.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-gmtime_r.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-impure.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-isalnum.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-isalpha.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-isascii.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-isblank.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-iscntrl.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-isdigit.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-isgraph.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-islower.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-isprint.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-ispunct.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-isspace.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-isupper.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-itoa.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-labs.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-lcltime.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-lcltime_r.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-ldiv.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-longjmp.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-makebuf.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-memccpy.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-memchr.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-memcmp.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-memcpy.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-memmove.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-memrchr.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-memset.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-mktime.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-month_lengths.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-open.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-quorem.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-raise.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-rand.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-rand_r.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-read.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-refill.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-rshift.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-s_fpclassify.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-sbrk.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-sccl.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-setjmp.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-sf_nan.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-srand.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-stdio.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strcasecmp.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strcasestr.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strcat.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strchr.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strcmp.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strcoll.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strcpy.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strcspn.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strdup.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strdup_r.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strftime.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strlcat.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strlcpy.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strlen.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strlwr.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strncasecmp.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strncat.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strncmp.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strncpy.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strndup.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strndup_r.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strnlen.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strptime.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strrchr.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strsep.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strspn.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strstr.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strtok_r.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strtol.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strtoul.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-strupr.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-sysclose.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-sysopen.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-sysread.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-syssbrk.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-system.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-systimes.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-syswrite.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-time.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-timelocal.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-toascii.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-tolower.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-toupper.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-tzcalc_limits.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-tzlock.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-tzset.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-tzset_r.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-tzvars.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-ungetc.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-utoa.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-wbuf.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-wcrtomb.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-wctomb_r.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lib_a-wsetup.*(.literal .literal.* .text .text.*)
    *libc_nano.a:lock.*(.literal .literal.* .text .text.*)
    *libdriver.a:gptimer.*(.literal.gptimer_default_isr .text.gptimer_default_isr)
    *libesp_hw_support.a:cpu.*(.literal.esp_cpu_compare_and_set .text.esp_cpu_compare_and_set)
    *libesp_hw_support.a:cpu.*(.literal.esp_cpu_reset .text.esp_cpu_reset)
    *libesp_hw_support.a:cpu.*(.literal.esp_cpu_stall .text.esp_cpu_stall)
    *libesp_hw_support.a:cpu.*(.literal.esp_cpu_unstall .text.esp_cpu_unstall)
    *libesp_hw_support.a:cpu.*(.literal.esp_cpu_wait_for_intr .text.esp_cpu_wait_for_intr)
    *libesp_hw_support.a:esp_gpio_reserve.*(.literal.esp_gpio_is_pin_reserved .text.esp_gpio_is_pin_reserved)
    *libesp_hw_support.a:esp_gpio_reserve.*(.literal.esp_gpio_reserve_pins .text.esp_gpio_reserve_pins)
    *libesp_hw_support.a:esp_memory_utils.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:rtc_clk.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:rtc_sleep.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:rtc_time.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:rtc_wdt.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:sar_periph_ctrl.*(.literal.sar_periph_ctrl_power_enable .text.sar_periph_ctrl_power_enable)
    *libesp_mm.a:cache_esp32.*(.literal .literal.* .text .text.*)
    *libesp_mm.a:esp_cache.*(.literal .literal.* .text .text.*)
    *libesp_ringbuf.a:(EXCLUDE_FILE(*libesp_ringbuf.a:ringbuf.*) .literal EXCLUDE_FILE(*libesp_ringbuf.a:ringbuf.*) .literal.* EXCLUDE_FILE(*libesp_ringbuf.a:ringbuf.*) .text EXCLUDE_FILE(*libesp_ringbuf.a:ringbuf.*) .text.*)
    *libesp_ringbuf.a:ringbuf.*(.literal.prvCopyItemAllowSplit .literal.prvCopyItemByteBuf .literal.prvCopyItemNoSplit .literal.prvReceiveGenericFromISR .literal.vRingbufferReturnItemFromISR .literal.xRingbufferReceiveFromISR .literal.xRingbufferReceiveSplitFromISR .literal.xRingbufferReceiveUpToFromISR .literal.xRingbufferSendFromISR .text .text.prvAcquireItemNoSplit .text.prvCheckItemAvail .text.prvCheckItemFitsByteBuffer .text.prvCheckItemFitsDefault .text.prvCopyItemAllowSplit .text.prvCopyItemByteBuf .text.prvCopyItemNoSplit .text.prvGetItemByteBuf .text.prvGetItemDefault .text.prvReceiveGenericFromISR .text.prvReturnItemByteBuf .text.prvReturnItemDefault .text.prvSendItemDoneNoSplit .text.vRingbufferReturnItemFromISR .text.xRingbufferReceiveFromISR .text.xRingbufferReceiveSplitFromISR .text.xRingbufferReceiveUpToFromISR .text.xRingbufferSendFromISR)
    *libesp_rom.a:esp_rom_spiflash.*(.literal .literal.* .text .text.*)
    *libesp_system.a:esp_err.*(.literal .literal.* .text .text.*)
    *libesp_system.a:esp_system_chip.*(.literal.esp_system_abort .text.esp_system_abort)
    *libesp_system.a:ubsan.*(.literal .literal.* .text .text.*)
    *libfreertos.a:(EXCLUDE_FILE(*libfreertos.a:app_startup.* *libfreertos.a:event_groups.* *libfreertos.a:heap_idf.* *libfreertos.a:idf_additions.* *libfreertos.a:list.* *libfreertos.a:port.* *libfreertos.a:port_common.* *libfreertos.a:queue.* *libfreertos.a:stream_buffer.* *libfreertos.a:tasks.* *libfreertos.a:timers.*) .literal EXCLUDE_FILE(*libfreertos.a:app_startup.* *libfreertos.a:event_groups.* *libfreertos.a:heap_idf.* *libfreertos.a:idf_additions.* *libfreertos.a:list.* *libfreertos.a:port.* *libfreertos.a:port_common.* *libfreertos.a:queue.* *libfreertos.a:stream_buffer.* *libfreertos.a:tasks.* *libfreertos.a:timers.*) .literal.* EXCLUDE_FILE(*libfreertos.a:app_startup.* *libfreertos.a:event_groups.* *libfreertos.a:heap_idf.* *libfreertos.a:idf_additions.* *libfreertos.a:list.* *libfreertos.a:port.* *libfreertos.a:port_common.* *libfreertos.a:queue.* *libfreertos.a:stream_buffer.* *libfreertos.a:tasks.* *libfreertos.a:timers.*) .text EXCLUDE_FILE(*libfreertos.a:app_startup.* *libfreertos.a:event_groups.* *libfreertos.a:heap_idf.* *libfreertos.a:idf_additions.* *libfreertos.a:list.* *libfreertos.a:port.* *libfreertos.a:port_common.* *libfreertos.a:queue.* *libfreertos.a:stream_buffer.* *libfreertos.a:tasks.* *libfreertos.a:timers.*) .text.*)
    *libfreertos.a:event_groups.*(.literal.xEventGroupGetBitsFromISR .text .text.xEventGroupGetBitsFromISR .text.xEventGroupGetStaticBuffer)
    *libfreertos.a:heap_idf.*(.text)
    *libfreertos.a:list.*(.text .text.uxListRemove .text.vListInsert .text.vListInsertEnd)
    *libfreertos.a:port.*(.literal.unlikely.vPortEndScheduler .literal.vApplicationStackOverflowHook .literal.vPortAssertIfInISR .literal.vPortExitCritical .literal.vPortExitCriticalCompliance .literal.vPortSetStackWatchpoint .literal.vPortYieldOtherCore .literal.xPortEnterCriticalTimeout .literal.xPortEnterCriticalTimeoutCompliance .literal.xPortInIsrContext .text .text.unlikely.vPortEndScheduler .text.vApplicationStackOverflowHook .text.vPortAssertIfInISR .text.vPortExitCritical .text.vPortExitCriticalCompliance .text.vPortSetStackWatchpoint .text.vPortYieldOtherCore .text.xPortEnterCriticalTimeout .text.xPortEnterCriticalTimeoutCompliance .text.xPortInIsrContext)
    *libfreertos.a:port_common.*(.text)
    *libfreertos.a:queue.*(.literal.prvCopyDataFromQueue .literal.prvCopyDataToQueue .literal.prvNotifyQueueSetContainer .literal.xQueueGenericSendFromISR .literal.xQueueGiveFromISR .literal.xQueuePeekFromISR .literal.xQueueReceiveFromISR .literal.xQueueSelectFromSetFromISR .text .text.prvCopyDataFromQueue .text.prvCopyDataToQueue .text.prvNotifyQueueSetContainer .text.uxQueueMessagesWaitingFromISR .text.xQueueGenericGetStaticBuffers .text.xQueueGenericSendFromISR .text.xQueueGetMutexHolderFromISR .text.xQueueGiveFromISR .text.xQueueIsQueueEmptyFromISR .text.xQueueIsQueueFullFromISR .text.xQueuePeekFromISR .text.xQueueReceiveFromISR .text.xQueueSelectFromSetFromISR)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferReceiveCompletedFromISR .literal.xStreamBufferReceiveFromISR .literal.xStreamBufferSendCompletedFromISR .literal.xStreamBufferSendFromISR .text .text.prvBytesInBuffer .text.xStreamBufferGetStaticBuffers .text.xStreamBufferReceiveCompletedFromISR .text.xStreamBufferReceiveFromISR .text.xStreamBufferSendCompletedFromISR .text.xStreamBufferSendFromISR .text.xStreamBufferSpacesAvailable)
    *libfreertos.a:tasks.*(.literal.__getreent .literal.prvCheckForYieldUsingPrioritySMP .literal.prvInitialiseNewTask$constprop$0 .literal.prvResetNextTaskUnblockTime .literal.uxTaskPriorityGetFromISR .literal.vTaskGenericNotifyGiveFromISR .literal.vTaskGetSnapshot .literal.vTaskSwitchContext .literal.xTaskCatchUpTicks .literal.xTaskGenericNotifyFromISR .literal.xTaskGetSchedulerState .literal.xTaskGetStaticBuffers .literal.xTaskGetTickCount .literal.xTaskGetTickCountFromISR .literal.xTaskIncrementTick .literal.xTaskIncrementTickOtherCores .literal.xTaskRemoveFromEventList .literal.xTaskResumeFromISR .text .text.__getreent .text.prvCheckForYieldUsingPrioritySMP .text.prvInitialiseNewTask$constprop$0 .text.prvResetNextTaskUnblockTime .text.uxTaskPriorityGetFromISR .text.vTaskGenericNotifyGiveFromISR .text.vTaskGetSnapshot .text.vTaskSwitchContext .text.xTaskCatchUpTicks .text.xTaskGenericNotifyFromISR .text.xTaskGetSchedulerState .text.xTaskGetStaticBuffers .text.xTaskGetTickCount .text.xTaskGetTickCountFromISR .text.xTaskIncrementTick .text.xTaskIncrementTickOtherCores .text.xTaskRemoveFromEventList .text.xTaskResumeFromISR .text.xTimerCreateTimerTask)
    *libfreertos.a:timers.*(.literal.xTimerPendFunctionCallFromISR .text .text.xTimerGetStaticBuffer .text.xTimerPendFunctionCallFromISR)
    *libgcc.a:lib2funcs.*(.literal .literal.* .text .text.*)
    *libgcov.a:(.literal .literal.* .text .text.*)
    *libhal.a:cache_hal_esp32.*(.literal .literal.* .text .text.*)
    *libhal.a:i2c_hal_iram.*(.literal .literal.* .text .text.*)
    *libhal.a:ledc_hal_iram.*(.literal .literal.* .text .text.*)
    *libhal.a:mmu_hal.*(.literal .literal.* .text .text.*)
    *libhal.a:spi_flash_encrypt_hal_iram.*(.literal .literal.* .text .text.*)
    *libhal.a:spi_flash_hal_iram.*(.literal .literal.* .text .text.*)
    *libhal.a:timer_hal.*(.literal.timer_hal_capture_and_get_counter_value .text.timer_hal_capture_and_get_counter_value)
    *libhal.a:wdt_hal_iram.*(.literal .literal.* .text .text.*)
    *libheap.a:multi_heap.*(.literal.assert_valid_block .text.assert_valid_block)
    *libheap.a:multi_heap.*(.literal.multi_heap_aligned_alloc_impl .text.multi_heap_aligned_alloc_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_aligned_alloc_impl_offs .text.multi_heap_aligned_alloc_impl_offs)
    *libheap.a:multi_heap.*(.literal.multi_heap_free_impl .text.multi_heap_free_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_get_allocated_size_impl .text.multi_heap_get_allocated_size_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_get_block_address_impl .text.multi_heap_get_block_address_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_get_first_block .text.multi_heap_get_first_block)
    *libheap.a:multi_heap.*(.literal.multi_heap_get_next_block .text.multi_heap_get_next_block)
    *libheap.a:multi_heap.*(.literal.multi_heap_internal_lock .text.multi_heap_internal_lock)
    *libheap.a:multi_heap.*(.literal.multi_heap_internal_unlock .text.multi_heap_internal_unlock)
    *libheap.a:multi_heap.*(.literal.multi_heap_is_free .text.multi_heap_is_free)
    *libheap.a:multi_heap.*(.literal.multi_heap_malloc_impl .text.multi_heap_malloc_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_realloc_impl .text.multi_heap_realloc_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_set_lock .text.multi_heap_set_lock)
    *libheap.a:multi_heap_poisoning.*(.literal.multi_heap_aligned_alloc .text.multi_heap_aligned_alloc)
    *libheap.a:multi_heap_poisoning.*(.literal.multi_heap_aligned_free .text.multi_heap_aligned_free)
    *libheap.a:multi_heap_poisoning.*(.literal.multi_heap_free .text.multi_heap_free)
    *libheap.a:multi_heap_poisoning.*(.literal.multi_heap_get_allocated_size .text.multi_heap_get_allocated_size)
    *libheap.a:multi_heap_poisoning.*(.literal.multi_heap_get_block_address .text.multi_heap_get_block_address)
    *libheap.a:multi_heap_poisoning.*(.literal.multi_heap_get_block_owner .text.multi_heap_get_block_owner)
    *libheap.a:multi_heap_poisoning.*(.literal.multi_heap_internal_check_block_poisoning .text.multi_heap_internal_check_block_poisoning)
    *libheap.a:multi_heap_poisoning.*(.literal.multi_heap_internal_poison_fill_region .text.multi_heap_internal_poison_fill_region)
    *libheap.a:multi_heap_poisoning.*(.literal.multi_heap_malloc .text.multi_heap_malloc)
    *libheap.a:multi_heap_poisoning.*(.literal.multi_heap_realloc .text.multi_heap_realloc)
    *libheap.a:multi_heap_poisoning.*(.literal.poison_allocated_region .text.poison_allocated_region)
    *libheap.a:multi_heap_poisoning.*(.literal.verify_allocated_region .text.verify_allocated_region)
    *libheap.a:tlsf.*(.literal.tlsf_align_size .text.tlsf_align_size)
    *libheap.a:tlsf.*(.literal.tlsf_alloc_overhead .text.tlsf_alloc_overhead)
    *libheap.a:tlsf.*(.literal.tlsf_block_size .text.tlsf_block_size)
    *libheap.a:tlsf.*(.literal.tlsf_block_size_max .text.tlsf_block_size_max)
    *libheap.a:tlsf.*(.literal.tlsf_block_size_min .text.tlsf_block_size_min)
    *libheap.a:tlsf.*(.literal.tlsf_free .text.tlsf_free)
    *libheap.a:tlsf.*(.literal.tlsf_get_pool .text.tlsf_get_pool)
    *libheap.a:tlsf.*(.literal.tlsf_malloc .text.tlsf_malloc)
    *libheap.a:tlsf.*(.literal.tlsf_memalign .text.tlsf_memalign)
    *libheap.a:tlsf.*(.literal.tlsf_memalign_offs .text.tlsf_memalign_offs)
    *libheap.a:tlsf.*(.literal.tlsf_realloc .text.tlsf_realloc)
    *libheap.a:tlsf.*(.literal.tlsf_size .text.tlsf_size)
    *liblog.a:log.*(.literal.esp_log_write .text.esp_log_write)
    *liblog.a:log_freertos.*(.literal.esp_log_early_timestamp .text.esp_log_early_timestamp)
    *liblog.a:log_freertos.*(.literal.esp_log_impl_lock .text.esp_log_impl_lock)
    *liblog.a:log_freertos.*(.literal.esp_log_impl_lock_timeout .text.esp_log_impl_lock_timeout)
    *liblog.a:log_freertos.*(.literal.esp_log_impl_unlock .text.esp_log_impl_unlock)
    *liblog.a:log_freertos.*(.literal.esp_log_timestamp .text.esp_log_timestamp)
    *libnewlib.a:abort.*(.literal .literal.* .text .text.*)
    *libnewlib.a:assert.*(.literal .literal.* .text .text.*)
    *libnewlib.a:heap.*(.literal .literal.* .text .text.*)
    *libnewlib.a:stdatomic.*(.literal .literal.* .text .text.*)
    *librtc.a:(.literal .literal.* .text .text.*)
    *libsoc.a:lldesc.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:flash_brownout_hook.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:memspi_host_driver.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_boya.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_gd.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_generic.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_issi.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_mxic.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_th.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_winbond.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_wrap.*(.literal .literal.* .text .text.*)
    *libxt_hal.a:(.literal .literal.* .text .text.*)
    *libxtensa.a:eri.*(.literal .literal.* .text .text.*)
    *libxtensa.a:xtensa_intr_asm.*(.literal .literal.* .text .text.*)

  } > iram0_0_seg

  .dram0.data :
  {
    _data_start = ABSOLUTE(.);
    *(.gnu.linkonce.d.*)
    *(.data1)
    *(.sdata)
    *(.sdata.*)
    *(.gnu.linkonce.s.*)
    *(.gnu.linkonce.s2.*)
    *(.jcr)

    *(EXCLUDE_FILE(*libbt.a *libbtdm_app.a) .data EXCLUDE_FILE(*libbt.a *libbtdm_app.a) .data.*)
    *(.dram1 .dram1.*)
    _coredump_dram_start = ABSOLUTE(.);
    *(.dram2.coredump .dram2.coredump.*)
    _coredump_dram_end = ABSOLUTE(.);
    *libapp_trace.a:app_trace.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libapp_trace.a:app_trace_util.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libapp_trace.a:port_uart.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    . = ALIGN(4);
    _bt_data_start = ABSOLUTE(.);
    *libbt.a:(.data .data.*)
    . = ALIGN(4);
    _bt_data_end = ABSOLUTE(.);
    . = ALIGN(4);
    _bt_controller_data_start = ABSOLUTE(.);
    *libbtdm_app.a:(.data .data.*)
    . = ALIGN(4);
    _bt_controller_data_end = ABSOLUTE(.);
    *libc_nano.a:creat.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:isatty.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-abs.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-asctime.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-asctime_r.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-atoi.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-atol.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-bzero.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-close.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-creat.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-ctime.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-ctime_r.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-ctype_.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-div.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-environ.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-envlock.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-fclose.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-fflush.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-findfp.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-fputwc.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-fvwrite.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-fwalk.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-getenv_r.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-gettzinfo.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-gmtime.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-gmtime_r.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-impure.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-isalnum.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-isalpha.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-isascii.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-isblank.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-iscntrl.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-isdigit.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-isgraph.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-islower.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-isprint.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-ispunct.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-isspace.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-isupper.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-itoa.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-labs.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-lcltime.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-lcltime_r.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-ldiv.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-longjmp.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-makebuf.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-memccpy.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-memchr.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-memcmp.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-memcpy.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-memmove.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-memrchr.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-memset.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-mktime.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-month_lengths.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-open.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-quorem.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-raise.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-rand.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-rand_r.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-read.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-refill.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-rshift.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-s_fpclassify.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-sbrk.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-sccl.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-setjmp.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-sf_nan.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-srand.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-stdio.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strcasecmp.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strcasestr.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strcat.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strchr.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strcmp.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strcoll.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strcpy.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strcspn.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strdup.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strdup_r.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strftime.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strlcat.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strlcpy.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strlen.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strlwr.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strncasecmp.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strncat.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strncmp.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strncpy.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strndup.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strndup_r.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strnlen.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strptime.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strrchr.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strsep.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strspn.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strstr.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strtok_r.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strtol.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strtoul.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-strupr.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-sysclose.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-sysopen.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-sysread.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-syssbrk.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-system.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-systimes.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-syswrite.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-time.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-timelocal.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-toascii.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-tolower.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-toupper.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-tzcalc_limits.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-tzlock.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-tzset.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-tzset_r.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-tzvars.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-ungetc.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-utoa.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-wbuf.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-wcrtomb.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-wctomb_r.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lib_a-wsetup.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libc_nano.a:lock.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_hw_support.a:esp_memory_utils.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_hw_support.a:rtc_clk.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_mm.a:cache_esp32.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_mm.a:esp_cache.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_rom.a:esp_rom_spiflash.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_system.a:esp_err.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_system.a:ubsan.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libfreertos.a:FreeRTOS-openocd.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libgcov.a:(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:cache_hal_esp32.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:i2c_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:ledc_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:mmu_hal.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:spi_flash_encrypt_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:spi_flash_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:wdt_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libnewlib.a:abort.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libnewlib.a:assert.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libnewlib.a:heap.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libnewlib.a:stdatomic.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libphy.a:(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libsoc.a:lldesc.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:flash_brownout_hook.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:memspi_host_driver.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_boya.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_gd.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_generic.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_issi.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_mxic.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_th.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_winbond.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_wrap.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)

    _data_end = ABSOLUTE(.);
    . = ALIGN(4);
  } > dram0_0_seg

  /**
   * This section holds data that won't be initialised when startup.
   * This section locates in External RAM region.
   */
  .ext_ram_noinit (NOLOAD) :
  {
    _ext_ram_noinit_start = ABSOLUTE(.);
    *(.ext_ram_noinit*)
    . = ALIGN(4);
    _ext_ram_noinit_end = ABSOLUTE(.);
  } > extern_ram_seg

  /*This section holds data that should not be initialized at power up.
    The section located in Internal SRAM memory region. The macro _NOINIT
    can be used as attribute to place data into this section.
    See the esp_attr.h file for more information.
  */
  .noinit (NOLOAD):
  {
    . = ALIGN(4);
    _noinit_start = ABSOLUTE(.);
    *(.noinit .noinit.*)
    . = ALIGN(4) ;
    _noinit_end = ABSOLUTE(.);
  } > dram0_0_seg

   /* external memory bss, from any global variable with EXT_RAM_BSS_ATTR attribute*/
  .ext_ram.bss (NOLOAD) :
  {
    _ext_ram_bss_start = ABSOLUTE(.);


    . = ALIGN(4);
    _ext_ram_bss_end = ABSOLUTE(.);
  } > extern_ram_seg

  /* Shared RAM */
  .dram0.bss (NOLOAD) :
  {
    . = ALIGN (8);
    _bss_start = ABSOLUTE(.);

    *(EXCLUDE_FILE(*libbt.a *libbtdm_app.a) .bss EXCLUDE_FILE(*libbt.a *libbtdm_app.a) .bss.*)
    *(.dynbss .dynsbss .gnu.linkonce.b .gnu.linkonce.b.* .gnu.linkonce.sb .gnu.linkonce.sb.* .gnu.linkonce.sb2 .gnu.linkonce.sb2.* .sbss .sbss.* .sbss2 .sbss2.* .scommon .share.mem)
    *(.ext_ram.bss .ext_ram.bss.*)
    *(EXCLUDE_FILE(*libbt.a *libbtdm_app.a) COMMON)
    . = ALIGN(4);
    _bt_bss_start = ABSOLUTE(.);
    *libbt.a:(.bss .bss.*)
    . = ALIGN(4);
    _bt_bss_end = ABSOLUTE(.);
    . = ALIGN(4);
    _bt_common_start = ABSOLUTE(.);
    *libbt.a:(COMMON)
    . = ALIGN(4);
    _bt_common_end = ABSOLUTE(.);
    . = ALIGN(4);
    _bt_controller_bss_start = ABSOLUTE(.);
    *libbtdm_app.a:(.bss .bss.*)
    . = ALIGN(4);
    _bt_controller_bss_end = ABSOLUTE(.);
    . = ALIGN(4);
    _bt_controller_common_start = ABSOLUTE(.);
    *libbtdm_app.a:(COMMON)
    . = ALIGN(4);
    _bt_controller_common_end = ABSOLUTE(.);

    . = ALIGN (8);
    _bss_end = ABSOLUTE(.);
  } > dram0_0_seg

  ASSERT(((_bss_end - ORIGIN(dram0_0_seg)) <= LENGTH(dram0_0_seg)),
          "DRAM segment data does not fit.")

  .flash.appdesc : ALIGN(0x10)
  {
    _rodata_reserved_start = ABSOLUTE(.);  /* This is a symbol marking the flash.rodata start, this can be used for mmu driver to maintain virtual address */
    _rodata_start = ABSOLUTE(.);

    *(.rodata_desc .rodata_desc.*)               /* Should be the first.  App version info.        DO NOT PUT ANYTHING BEFORE IT! */
    *(.rodata_custom_desc .rodata_custom_desc.*) /* Should be the second. Custom app version info. DO NOT PUT ANYTHING BEFORE IT! */

    /* Create an empty gap within this section. Thanks to this, the end of this
     * section will match .flah.rodata's begin address. Thus, both sections
     * will be merged when creating the final bin image. */
    . = ALIGN(ALIGNOF(.flash.rodata));
  } >default_rodata_seg

  .flash.rodata : ALIGN(0x10)
  {
    _flash_rodata_start = ABSOLUTE(.);

    *(EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libc_nano.a:creat.* *libc_nano.a:isatty.* *libc_nano.a:lib_a-abs.* *libc_nano.a:lib_a-asctime.* *libc_nano.a:lib_a-asctime_r.* *libc_nano.a:lib_a-atoi.* *libc_nano.a:lib_a-atol.* *libc_nano.a:lib_a-bzero.* *libc_nano.a:lib_a-close.* *libc_nano.a:lib_a-creat.* *libc_nano.a:lib_a-ctime.* *libc_nano.a:lib_a-ctime_r.* *libc_nano.a:lib_a-ctype_.* *libc_nano.a:lib_a-div.* *libc_nano.a:lib_a-environ.* *libc_nano.a:lib_a-envlock.* *libc_nano.a:lib_a-fclose.* *libc_nano.a:lib_a-fflush.* *libc_nano.a:lib_a-findfp.* *libc_nano.a:lib_a-fputwc.* *libc_nano.a:lib_a-fvwrite.* *libc_nano.a:lib_a-fwalk.* *libc_nano.a:lib_a-getenv_r.* *libc_nano.a:lib_a-gettzinfo.* *libc_nano.a:lib_a-gmtime.* *libc_nano.a:lib_a-gmtime_r.* *libc_nano.a:lib_a-impure.* *libc_nano.a:lib_a-isalnum.* *libc_nano.a:lib_a-isalpha.* *libc_nano.a:lib_a-isascii.* *libc_nano.a:lib_a-isblank.* *libc_nano.a:lib_a-iscntrl.* *libc_nano.a:lib_a-isdigit.* *libc_nano.a:lib_a-isgraph.* *libc_nano.a:lib_a-islower.* *libc_nano.a:lib_a-isprint.* *libc_nano.a:lib_a-ispunct.* *libc_nano.a:lib_a-isspace.* *libc_nano.a:lib_a-isupper.* *libc_nano.a:lib_a-itoa.* *libc_nano.a:lib_a-labs.* *libc_nano.a:lib_a-lcltime.* *libc_nano.a:lib_a-lcltime_r.* *libc_nano.a:lib_a-ldiv.* *libc_nano.a:lib_a-longjmp.* *libc_nano.a:lib_a-makebuf.* *libc_nano.a:lib_a-memccpy.* *libc_nano.a:lib_a-memchr.* *libc_nano.a:lib_a-memcmp.* *libc_nano.a:lib_a-memcpy.* *libc_nano.a:lib_a-memmove.* *libc_nano.a:lib_a-memrchr.* *libc_nano.a:lib_a-memset.* *libc_nano.a:lib_a-mktime.* *libc_nano.a:lib_a-month_lengths.* *libc_nano.a:lib_a-open.* *libc_nano.a:lib_a-quorem.* *libc_nano.a:lib_a-raise.* *libc_nano.a:lib_a-rand.* *libc_nano.a:lib_a-rand_r.* *libc_nano.a:lib_a-read.* *libc_nano.a:lib_a-refill.* *libc_nano.a:lib_a-rshift.* *libc_nano.a:lib_a-s_fpclassify.* *libc_nano.a:lib_a-sbrk.* *libc_nano.a:lib_a-sccl.* *libc_nano.a:lib_a-setjmp.* *libc_nano.a:lib_a-sf_nan.* *libc_nano.a:lib_a-srand.* *libc_nano.a:lib_a-stdio.* *libc_nano.a:lib_a-strcasecmp.* *libc_nano.a:lib_a-strcasestr.* *libc_nano.a:lib_a-strcat.* *libc_nano.a:lib_a-strchr.* *libc_nano.a:lib_a-strcmp.* *libc_nano.a:lib_a-strcoll.* *libc_nano.a:lib_a-strcpy.* *libc_nano.a:lib_a-strcspn.* *libc_nano.a:lib_a-strdup.* *libc_nano.a:lib_a-strdup_r.* *libc_nano.a:lib_a-strftime.* *libc_nano.a:lib_a-strlcat.* *libc_nano.a:lib_a-strlcpy.* *libc_nano.a:lib_a-strlen.* *libc_nano.a:lib_a-strlwr.* *libc_nano.a:lib_a-strncasecmp.* *libc_nano.a:lib_a-strncat.* *libc_nano.a:lib_a-strncmp.* *libc_nano.a:lib_a-strncpy.* *libc_nano.a:lib_a-strndup.* *libc_nano.a:lib_a-strndup_r.* *libc_nano.a:lib_a-strnlen.* *libc_nano.a:lib_a-strptime.* *libc_nano.a:lib_a-strrchr.* *libc_nano.a:lib_a-strsep.* *libc_nano.a:lib_a-strspn.* *libc_nano.a:lib_a-strstr.* *libc_nano.a:lib_a-strtok_r.* *libc_nano.a:lib_a-strtol.* *libc_nano.a:lib_a-strtoul.* *libc_nano.a:lib_a-strupr.* *libc_nano.a:lib_a-sysclose.* *libc_nano.a:lib_a-sysopen.* *libc_nano.a:lib_a-sysread.* *libc_nano.a:lib_a-syssbrk.* *libc_nano.a:lib_a-system.* *libc_nano.a:lib_a-systimes.* *libc_nano.a:lib_a-syswrite.* *libc_nano.a:lib_a-time.* *libc_nano.a:lib_a-timelocal.* *libc_nano.a:lib_a-toascii.* *libc_nano.a:lib_a-tolower.* *libc_nano.a:lib_a-toupper.* *libc_nano.a:lib_a-tzcalc_limits.* *libc_nano.a:lib_a-tzlock.* *libc_nano.a:lib_a-tzset.* *libc_nano.a:lib_a-tzset_r.* *libc_nano.a:lib_a-tzvars.* *libc_nano.a:lib_a-ungetc.* *libc_nano.a:lib_a-utoa.* *libc_nano.a:lib_a-wbuf.* *libc_nano.a:lib_a-wcrtomb.* *libc_nano.a:lib_a-wctomb_r.* *libc_nano.a:lib_a-wsetup.* *libc_nano.a:lock.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:rtc_clk.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_spiflash.* *libesp_system.a:esp_err.* *libesp_system.a:ubsan.* *libfreertos.a:FreeRTOS-openocd.* *libhal.a:cache_hal_esp32.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:wdt_hal_iram.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.*) .rodata EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libc_nano.a:creat.* *libc_nano.a:isatty.* *libc_nano.a:lib_a-abs.* *libc_nano.a:lib_a-asctime.* *libc_nano.a:lib_a-asctime_r.* *libc_nano.a:lib_a-atoi.* *libc_nano.a:lib_a-atol.* *libc_nano.a:lib_a-bzero.* *libc_nano.a:lib_a-close.* *libc_nano.a:lib_a-creat.* *libc_nano.a:lib_a-ctime.* *libc_nano.a:lib_a-ctime_r.* *libc_nano.a:lib_a-ctype_.* *libc_nano.a:lib_a-div.* *libc_nano.a:lib_a-environ.* *libc_nano.a:lib_a-envlock.* *libc_nano.a:lib_a-fclose.* *libc_nano.a:lib_a-fflush.* *libc_nano.a:lib_a-findfp.* *libc_nano.a:lib_a-fputwc.* *libc_nano.a:lib_a-fvwrite.* *libc_nano.a:lib_a-fwalk.* *libc_nano.a:lib_a-getenv_r.* *libc_nano.a:lib_a-gettzinfo.* *libc_nano.a:lib_a-gmtime.* *libc_nano.a:lib_a-gmtime_r.* *libc_nano.a:lib_a-impure.* *libc_nano.a:lib_a-isalnum.* *libc_nano.a:lib_a-isalpha.* *libc_nano.a:lib_a-isascii.* *libc_nano.a:lib_a-isblank.* *libc_nano.a:lib_a-iscntrl.* *libc_nano.a:lib_a-isdigit.* *libc_nano.a:lib_a-isgraph.* *libc_nano.a:lib_a-islower.* *libc_nano.a:lib_a-isprint.* *libc_nano.a:lib_a-ispunct.* *libc_nano.a:lib_a-isspace.* *libc_nano.a:lib_a-isupper.* *libc_nano.a:lib_a-itoa.* *libc_nano.a:lib_a-labs.* *libc_nano.a:lib_a-lcltime.* *libc_nano.a:lib_a-lcltime_r.* *libc_nano.a:lib_a-ldiv.* *libc_nano.a:lib_a-longjmp.* *libc_nano.a:lib_a-makebuf.* *libc_nano.a:lib_a-memccpy.* *libc_nano.a:lib_a-memchr.* *libc_nano.a:lib_a-memcmp.* *libc_nano.a:lib_a-memcpy.* *libc_nano.a:lib_a-memmove.* *libc_nano.a:lib_a-memrchr.* *libc_nano.a:lib_a-memset.* *libc_nano.a:lib_a-mktime.* *libc_nano.a:lib_a-month_lengths.* *libc_nano.a:lib_a-open.* *libc_nano.a:lib_a-quorem.* *libc_nano.a:lib_a-raise.* *libc_nano.a:lib_a-rand.* *libc_nano.a:lib_a-rand_r.* *libc_nano.a:lib_a-read.* *libc_nano.a:lib_a-refill.* *libc_nano.a:lib_a-rshift.* *libc_nano.a:lib_a-s_fpclassify.* *libc_nano.a:lib_a-sbrk.* *libc_nano.a:lib_a-sccl.* *libc_nano.a:lib_a-setjmp.* *libc_nano.a:lib_a-sf_nan.* *libc_nano.a:lib_a-srand.* *libc_nano.a:lib_a-stdio.* *libc_nano.a:lib_a-strcasecmp.* *libc_nano.a:lib_a-strcasestr.* *libc_nano.a:lib_a-strcat.* *libc_nano.a:lib_a-strchr.* *libc_nano.a:lib_a-strcmp.* *libc_nano.a:lib_a-strcoll.* *libc_nano.a:lib_a-strcpy.* *libc_nano.a:lib_a-strcspn.* *libc_nano.a:lib_a-strdup.* *libc_nano.a:lib_a-strdup_r.* *libc_nano.a:lib_a-strftime.* *libc_nano.a:lib_a-strlcat.* *libc_nano.a:lib_a-strlcpy.* *libc_nano.a:lib_a-strlen.* *libc_nano.a:lib_a-strlwr.* *libc_nano.a:lib_a-strncasecmp.* *libc_nano.a:lib_a-strncat.* *libc_nano.a:lib_a-strncmp.* *libc_nano.a:lib_a-strncpy.* *libc_nano.a:lib_a-strndup.* *libc_nano.a:lib_a-strndup_r.* *libc_nano.a:lib_a-strnlen.* *libc_nano.a:lib_a-strptime.* *libc_nano.a:lib_a-strrchr.* *libc_nano.a:lib_a-strsep.* *libc_nano.a:lib_a-strspn.* *libc_nano.a:lib_a-strstr.* *libc_nano.a:lib_a-strtok_r.* *libc_nano.a:lib_a-strtol.* *libc_nano.a:lib_a-strtoul.* *libc_nano.a:lib_a-strupr.* *libc_nano.a:lib_a-sysclose.* *libc_nano.a:lib_a-sysopen.* *libc_nano.a:lib_a-sysread.* *libc_nano.a:lib_a-syssbrk.* *libc_nano.a:lib_a-system.* *libc_nano.a:lib_a-systimes.* *libc_nano.a:lib_a-syswrite.* *libc_nano.a:lib_a-time.* *libc_nano.a:lib_a-timelocal.* *libc_nano.a:lib_a-toascii.* *libc_nano.a:lib_a-tolower.* *libc_nano.a:lib_a-toupper.* *libc_nano.a:lib_a-tzcalc_limits.* *libc_nano.a:lib_a-tzlock.* *libc_nano.a:lib_a-tzset.* *libc_nano.a:lib_a-tzset_r.* *libc_nano.a:lib_a-tzvars.* *libc_nano.a:lib_a-ungetc.* *libc_nano.a:lib_a-utoa.* *libc_nano.a:lib_a-wbuf.* *libc_nano.a:lib_a-wcrtomb.* *libc_nano.a:lib_a-wctomb_r.* *libc_nano.a:lib_a-wsetup.* *libc_nano.a:lock.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:rtc_clk.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_spiflash.* *libesp_system.a:esp_err.* *libesp_system.a:ubsan.* *libfreertos.a:FreeRTOS-openocd.* *libhal.a:cache_hal_esp32.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:wdt_hal_iram.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.*) .rodata.* EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libc_nano.a:creat.* *libc_nano.a:isatty.* *libc_nano.a:lib_a-abs.* *libc_nano.a:lib_a-asctime.* *libc_nano.a:lib_a-asctime_r.* *libc_nano.a:lib_a-atoi.* *libc_nano.a:lib_a-atol.* *libc_nano.a:lib_a-bzero.* *libc_nano.a:lib_a-close.* *libc_nano.a:lib_a-creat.* *libc_nano.a:lib_a-ctime.* *libc_nano.a:lib_a-ctime_r.* *libc_nano.a:lib_a-ctype_.* *libc_nano.a:lib_a-div.* *libc_nano.a:lib_a-environ.* *libc_nano.a:lib_a-envlock.* *libc_nano.a:lib_a-fclose.* *libc_nano.a:lib_a-fflush.* *libc_nano.a:lib_a-findfp.* *libc_nano.a:lib_a-fputwc.* *libc_nano.a:lib_a-fvwrite.* *libc_nano.a:lib_a-fwalk.* *libc_nano.a:lib_a-getenv_r.* *libc_nano.a:lib_a-gettzinfo.* *libc_nano.a:lib_a-gmtime.* *libc_nano.a:lib_a-gmtime_r.* *libc_nano.a:lib_a-impure.* *libc_nano.a:lib_a-isalnum.* *libc_nano.a:lib_a-isalpha.* *libc_nano.a:lib_a-isascii.* *libc_nano.a:lib_a-isblank.* *libc_nano.a:lib_a-iscntrl.* *libc_nano.a:lib_a-isdigit.* *libc_nano.a:lib_a-isgraph.* *libc_nano.a:lib_a-islower.* *libc_nano.a:lib_a-isprint.* *libc_nano.a:lib_a-ispunct.* *libc_nano.a:lib_a-isspace.* *libc_nano.a:lib_a-isupper.* *libc_nano.a:lib_a-itoa.* *libc_nano.a:lib_a-labs.* *libc_nano.a:lib_a-lcltime.* *libc_nano.a:lib_a-lcltime_r.* *libc_nano.a:lib_a-ldiv.* *libc_nano.a:lib_a-longjmp.* *libc_nano.a:lib_a-makebuf.* *libc_nano.a:lib_a-memccpy.* *libc_nano.a:lib_a-memchr.* *libc_nano.a:lib_a-memcmp.* *libc_nano.a:lib_a-memcpy.* *libc_nano.a:lib_a-memmove.* *libc_nano.a:lib_a-memrchr.* *libc_nano.a:lib_a-memset.* *libc_nano.a:lib_a-mktime.* *libc_nano.a:lib_a-month_lengths.* *libc_nano.a:lib_a-open.* *libc_nano.a:lib_a-quorem.* *libc_nano.a:lib_a-raise.* *libc_nano.a:lib_a-rand.* *libc_nano.a:lib_a-rand_r.* *libc_nano.a:lib_a-read.* *libc_nano.a:lib_a-refill.* *libc_nano.a:lib_a-rshift.* *libc_nano.a:lib_a-s_fpclassify.* *libc_nano.a:lib_a-sbrk.* *libc_nano.a:lib_a-sccl.* *libc_nano.a:lib_a-setjmp.* *libc_nano.a:lib_a-sf_nan.* *libc_nano.a:lib_a-srand.* *libc_nano.a:lib_a-stdio.* *libc_nano.a:lib_a-strcasecmp.* *libc_nano.a:lib_a-strcasestr.* *libc_nano.a:lib_a-strcat.* *libc_nano.a:lib_a-strchr.* *libc_nano.a:lib_a-strcmp.* *libc_nano.a:lib_a-strcoll.* *libc_nano.a:lib_a-strcpy.* *libc_nano.a:lib_a-strcspn.* *libc_nano.a:lib_a-strdup.* *libc_nano.a:lib_a-strdup_r.* *libc_nano.a:lib_a-strftime.* *libc_nano.a:lib_a-strlcat.* *libc_nano.a:lib_a-strlcpy.* *libc_nano.a:lib_a-strlen.* *libc_nano.a:lib_a-strlwr.* *libc_nano.a:lib_a-strncasecmp.* *libc_nano.a:lib_a-strncat.* *libc_nano.a:lib_a-strncmp.* *libc_nano.a:lib_a-strncpy.* *libc_nano.a:lib_a-strndup.* *libc_nano.a:lib_a-strndup_r.* *libc_nano.a:lib_a-strnlen.* *libc_nano.a:lib_a-strptime.* *libc_nano.a:lib_a-strrchr.* *libc_nano.a:lib_a-strsep.* *libc_nano.a:lib_a-strspn.* *libc_nano.a:lib_a-strstr.* *libc_nano.a:lib_a-strtok_r.* *libc_nano.a:lib_a-strtol.* *libc_nano.a:lib_a-strtoul.* *libc_nano.a:lib_a-strupr.* *libc_nano.a:lib_a-sysclose.* *libc_nano.a:lib_a-sysopen.* *libc_nano.a:lib_a-sysread.* *libc_nano.a:lib_a-syssbrk.* *libc_nano.a:lib_a-system.* *libc_nano.a:lib_a-systimes.* *libc_nano.a:lib_a-syswrite.* *libc_nano.a:lib_a-time.* *libc_nano.a:lib_a-timelocal.* *libc_nano.a:lib_a-toascii.* *libc_nano.a:lib_a-tolower.* *libc_nano.a:lib_a-toupper.* *libc_nano.a:lib_a-tzcalc_limits.* *libc_nano.a:lib_a-tzlock.* *libc_nano.a:lib_a-tzset.* *libc_nano.a:lib_a-tzset_r.* *libc_nano.a:lib_a-tzvars.* *libc_nano.a:lib_a-ungetc.* *libc_nano.a:lib_a-utoa.* *libc_nano.a:lib_a-wbuf.* *libc_nano.a:lib_a-wcrtomb.* *libc_nano.a:lib_a-wctomb_r.* *libc_nano.a:lib_a-wsetup.* *libc_nano.a:lock.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:rtc_clk.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_spiflash.* *libesp_system.a:esp_err.* *libesp_system.a:ubsan.* *libfreertos.a:FreeRTOS-openocd.* *libhal.a:cache_hal_esp32.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:wdt_hal_iram.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.*) .sdata2 EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libc_nano.a:creat.* *libc_nano.a:isatty.* *libc_nano.a:lib_a-abs.* *libc_nano.a:lib_a-asctime.* *libc_nano.a:lib_a-asctime_r.* *libc_nano.a:lib_a-atoi.* *libc_nano.a:lib_a-atol.* *libc_nano.a:lib_a-bzero.* *libc_nano.a:lib_a-close.* *libc_nano.a:lib_a-creat.* *libc_nano.a:lib_a-ctime.* *libc_nano.a:lib_a-ctime_r.* *libc_nano.a:lib_a-ctype_.* *libc_nano.a:lib_a-div.* *libc_nano.a:lib_a-environ.* *libc_nano.a:lib_a-envlock.* *libc_nano.a:lib_a-fclose.* *libc_nano.a:lib_a-fflush.* *libc_nano.a:lib_a-findfp.* *libc_nano.a:lib_a-fputwc.* *libc_nano.a:lib_a-fvwrite.* *libc_nano.a:lib_a-fwalk.* *libc_nano.a:lib_a-getenv_r.* *libc_nano.a:lib_a-gettzinfo.* *libc_nano.a:lib_a-gmtime.* *libc_nano.a:lib_a-gmtime_r.* *libc_nano.a:lib_a-impure.* *libc_nano.a:lib_a-isalnum.* *libc_nano.a:lib_a-isalpha.* *libc_nano.a:lib_a-isascii.* *libc_nano.a:lib_a-isblank.* *libc_nano.a:lib_a-iscntrl.* *libc_nano.a:lib_a-isdigit.* *libc_nano.a:lib_a-isgraph.* *libc_nano.a:lib_a-islower.* *libc_nano.a:lib_a-isprint.* *libc_nano.a:lib_a-ispunct.* *libc_nano.a:lib_a-isspace.* *libc_nano.a:lib_a-isupper.* *libc_nano.a:lib_a-itoa.* *libc_nano.a:lib_a-labs.* *libc_nano.a:lib_a-lcltime.* *libc_nano.a:lib_a-lcltime_r.* *libc_nano.a:lib_a-ldiv.* *libc_nano.a:lib_a-longjmp.* *libc_nano.a:lib_a-makebuf.* *libc_nano.a:lib_a-memccpy.* *libc_nano.a:lib_a-memchr.* *libc_nano.a:lib_a-memcmp.* *libc_nano.a:lib_a-memcpy.* *libc_nano.a:lib_a-memmove.* *libc_nano.a:lib_a-memrchr.* *libc_nano.a:lib_a-memset.* *libc_nano.a:lib_a-mktime.* *libc_nano.a:lib_a-month_lengths.* *libc_nano.a:lib_a-open.* *libc_nano.a:lib_a-quorem.* *libc_nano.a:lib_a-raise.* *libc_nano.a:lib_a-rand.* *libc_nano.a:lib_a-rand_r.* *libc_nano.a:lib_a-read.* *libc_nano.a:lib_a-refill.* *libc_nano.a:lib_a-rshift.* *libc_nano.a:lib_a-s_fpclassify.* *libc_nano.a:lib_a-sbrk.* *libc_nano.a:lib_a-sccl.* *libc_nano.a:lib_a-setjmp.* *libc_nano.a:lib_a-sf_nan.* *libc_nano.a:lib_a-srand.* *libc_nano.a:lib_a-stdio.* *libc_nano.a:lib_a-strcasecmp.* *libc_nano.a:lib_a-strcasestr.* *libc_nano.a:lib_a-strcat.* *libc_nano.a:lib_a-strchr.* *libc_nano.a:lib_a-strcmp.* *libc_nano.a:lib_a-strcoll.* *libc_nano.a:lib_a-strcpy.* *libc_nano.a:lib_a-strcspn.* *libc_nano.a:lib_a-strdup.* *libc_nano.a:lib_a-strdup_r.* *libc_nano.a:lib_a-strftime.* *libc_nano.a:lib_a-strlcat.* *libc_nano.a:lib_a-strlcpy.* *libc_nano.a:lib_a-strlen.* *libc_nano.a:lib_a-strlwr.* *libc_nano.a:lib_a-strncasecmp.* *libc_nano.a:lib_a-strncat.* *libc_nano.a:lib_a-strncmp.* *libc_nano.a:lib_a-strncpy.* *libc_nano.a:lib_a-strndup.* *libc_nano.a:lib_a-strndup_r.* *libc_nano.a:lib_a-strnlen.* *libc_nano.a:lib_a-strptime.* *libc_nano.a:lib_a-strrchr.* *libc_nano.a:lib_a-strsep.* *libc_nano.a:lib_a-strspn.* *libc_nano.a:lib_a-strstr.* *libc_nano.a:lib_a-strtok_r.* *libc_nano.a:lib_a-strtol.* *libc_nano.a:lib_a-strtoul.* *libc_nano.a:lib_a-strupr.* *libc_nano.a:lib_a-sysclose.* *libc_nano.a:lib_a-sysopen.* *libc_nano.a:lib_a-sysread.* *libc_nano.a:lib_a-syssbrk.* *libc_nano.a:lib_a-system.* *libc_nano.a:lib_a-systimes.* *libc_nano.a:lib_a-syswrite.* *libc_nano.a:lib_a-time.* *libc_nano.a:lib_a-timelocal.* *libc_nano.a:lib_a-toascii.* *libc_nano.a:lib_a-tolower.* *libc_nano.a:lib_a-toupper.* *libc_nano.a:lib_a-tzcalc_limits.* *libc_nano.a:lib_a-tzlock.* *libc_nano.a:lib_a-tzset.* *libc_nano.a:lib_a-tzset_r.* *libc_nano.a:lib_a-tzvars.* *libc_nano.a:lib_a-ungetc.* *libc_nano.a:lib_a-utoa.* *libc_nano.a:lib_a-wbuf.* *libc_nano.a:lib_a-wcrtomb.* *libc_nano.a:lib_a-wctomb_r.* *libc_nano.a:lib_a-wsetup.* *libc_nano.a:lock.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:rtc_clk.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_spiflash.* *libesp_system.a:esp_err.* *libesp_system.a:ubsan.* *libfreertos.a:FreeRTOS-openocd.* *libhal.a:cache_hal_esp32.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:wdt_hal_iram.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.*) .sdata2.* EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libc_nano.a:creat.* *libc_nano.a:isatty.* *libc_nano.a:lib_a-abs.* *libc_nano.a:lib_a-asctime.* *libc_nano.a:lib_a-asctime_r.* *libc_nano.a:lib_a-atoi.* *libc_nano.a:lib_a-atol.* *libc_nano.a:lib_a-bzero.* *libc_nano.a:lib_a-close.* *libc_nano.a:lib_a-creat.* *libc_nano.a:lib_a-ctime.* *libc_nano.a:lib_a-ctime_r.* *libc_nano.a:lib_a-ctype_.* *libc_nano.a:lib_a-div.* *libc_nano.a:lib_a-environ.* *libc_nano.a:lib_a-envlock.* *libc_nano.a:lib_a-fclose.* *libc_nano.a:lib_a-fflush.* *libc_nano.a:lib_a-findfp.* *libc_nano.a:lib_a-fputwc.* *libc_nano.a:lib_a-fvwrite.* *libc_nano.a:lib_a-fwalk.* *libc_nano.a:lib_a-getenv_r.* *libc_nano.a:lib_a-gettzinfo.* *libc_nano.a:lib_a-gmtime.* *libc_nano.a:lib_a-gmtime_r.* *libc_nano.a:lib_a-impure.* *libc_nano.a:lib_a-isalnum.* *libc_nano.a:lib_a-isalpha.* *libc_nano.a:lib_a-isascii.* *libc_nano.a:lib_a-isblank.* *libc_nano.a:lib_a-iscntrl.* *libc_nano.a:lib_a-isdigit.* *libc_nano.a:lib_a-isgraph.* *libc_nano.a:lib_a-islower.* *libc_nano.a:lib_a-isprint.* *libc_nano.a:lib_a-ispunct.* *libc_nano.a:lib_a-isspace.* *libc_nano.a:lib_a-isupper.* *libc_nano.a:lib_a-itoa.* *libc_nano.a:lib_a-labs.* *libc_nano.a:lib_a-lcltime.* *libc_nano.a:lib_a-lcltime_r.* *libc_nano.a:lib_a-ldiv.* *libc_nano.a:lib_a-longjmp.* *libc_nano.a:lib_a-makebuf.* *libc_nano.a:lib_a-memccpy.* *libc_nano.a:lib_a-memchr.* *libc_nano.a:lib_a-memcmp.* *libc_nano.a:lib_a-memcpy.* *libc_nano.a:lib_a-memmove.* *libc_nano.a:lib_a-memrchr.* *libc_nano.a:lib_a-memset.* *libc_nano.a:lib_a-mktime.* *libc_nano.a:lib_a-month_lengths.* *libc_nano.a:lib_a-open.* *libc_nano.a:lib_a-quorem.* *libc_nano.a:lib_a-raise.* *libc_nano.a:lib_a-rand.* *libc_nano.a:lib_a-rand_r.* *libc_nano.a:lib_a-read.* *libc_nano.a:lib_a-refill.* *libc_nano.a:lib_a-rshift.* *libc_nano.a:lib_a-s_fpclassify.* *libc_nano.a:lib_a-sbrk.* *libc_nano.a:lib_a-sccl.* *libc_nano.a:lib_a-setjmp.* *libc_nano.a:lib_a-sf_nan.* *libc_nano.a:lib_a-srand.* *libc_nano.a:lib_a-stdio.* *libc_nano.a:lib_a-strcasecmp.* *libc_nano.a:lib_a-strcasestr.* *libc_nano.a:lib_a-strcat.* *libc_nano.a:lib_a-strchr.* *libc_nano.a:lib_a-strcmp.* *libc_nano.a:lib_a-strcoll.* *libc_nano.a:lib_a-strcpy.* *libc_nano.a:lib_a-strcspn.* *libc_nano.a:lib_a-strdup.* *libc_nano.a:lib_a-strdup_r.* *libc_nano.a:lib_a-strftime.* *libc_nano.a:lib_a-strlcat.* *libc_nano.a:lib_a-strlcpy.* *libc_nano.a:lib_a-strlen.* *libc_nano.a:lib_a-strlwr.* *libc_nano.a:lib_a-strncasecmp.* *libc_nano.a:lib_a-strncat.* *libc_nano.a:lib_a-strncmp.* *libc_nano.a:lib_a-strncpy.* *libc_nano.a:lib_a-strndup.* *libc_nano.a:lib_a-strndup_r.* *libc_nano.a:lib_a-strnlen.* *libc_nano.a:lib_a-strptime.* *libc_nano.a:lib_a-strrchr.* *libc_nano.a:lib_a-strsep.* *libc_nano.a:lib_a-strspn.* *libc_nano.a:lib_a-strstr.* *libc_nano.a:lib_a-strtok_r.* *libc_nano.a:lib_a-strtol.* *libc_nano.a:lib_a-strtoul.* *libc_nano.a:lib_a-strupr.* *libc_nano.a:lib_a-sysclose.* *libc_nano.a:lib_a-sysopen.* *libc_nano.a:lib_a-sysread.* *libc_nano.a:lib_a-syssbrk.* *libc_nano.a:lib_a-system.* *libc_nano.a:lib_a-systimes.* *libc_nano.a:lib_a-syswrite.* *libc_nano.a:lib_a-time.* *libc_nano.a:lib_a-timelocal.* *libc_nano.a:lib_a-toascii.* *libc_nano.a:lib_a-tolower.* *libc_nano.a:lib_a-toupper.* *libc_nano.a:lib_a-tzcalc_limits.* *libc_nano.a:lib_a-tzlock.* *libc_nano.a:lib_a-tzset.* *libc_nano.a:lib_a-tzset_r.* *libc_nano.a:lib_a-tzvars.* *libc_nano.a:lib_a-ungetc.* *libc_nano.a:lib_a-utoa.* *libc_nano.a:lib_a-wbuf.* *libc_nano.a:lib_a-wcrtomb.* *libc_nano.a:lib_a-wctomb_r.* *libc_nano.a:lib_a-wsetup.* *libc_nano.a:lock.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:rtc_clk.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_spiflash.* *libesp_system.a:esp_err.* *libesp_system.a:ubsan.* *libfreertos.a:FreeRTOS-openocd.* *libhal.a:cache_hal_esp32.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:wdt_hal_iram.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.*) .srodata EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libc_nano.a:creat.* *libc_nano.a:isatty.* *libc_nano.a:lib_a-abs.* *libc_nano.a:lib_a-asctime.* *libc_nano.a:lib_a-asctime_r.* *libc_nano.a:lib_a-atoi.* *libc_nano.a:lib_a-atol.* *libc_nano.a:lib_a-bzero.* *libc_nano.a:lib_a-close.* *libc_nano.a:lib_a-creat.* *libc_nano.a:lib_a-ctime.* *libc_nano.a:lib_a-ctime_r.* *libc_nano.a:lib_a-ctype_.* *libc_nano.a:lib_a-div.* *libc_nano.a:lib_a-environ.* *libc_nano.a:lib_a-envlock.* *libc_nano.a:lib_a-fclose.* *libc_nano.a:lib_a-fflush.* *libc_nano.a:lib_a-findfp.* *libc_nano.a:lib_a-fputwc.* *libc_nano.a:lib_a-fvwrite.* *libc_nano.a:lib_a-fwalk.* *libc_nano.a:lib_a-getenv_r.* *libc_nano.a:lib_a-gettzinfo.* *libc_nano.a:lib_a-gmtime.* *libc_nano.a:lib_a-gmtime_r.* *libc_nano.a:lib_a-impure.* *libc_nano.a:lib_a-isalnum.* *libc_nano.a:lib_a-isalpha.* *libc_nano.a:lib_a-isascii.* *libc_nano.a:lib_a-isblank.* *libc_nano.a:lib_a-iscntrl.* *libc_nano.a:lib_a-isdigit.* *libc_nano.a:lib_a-isgraph.* *libc_nano.a:lib_a-islower.* *libc_nano.a:lib_a-isprint.* *libc_nano.a:lib_a-ispunct.* *libc_nano.a:lib_a-isspace.* *libc_nano.a:lib_a-isupper.* *libc_nano.a:lib_a-itoa.* *libc_nano.a:lib_a-labs.* *libc_nano.a:lib_a-lcltime.* *libc_nano.a:lib_a-lcltime_r.* *libc_nano.a:lib_a-ldiv.* *libc_nano.a:lib_a-longjmp.* *libc_nano.a:lib_a-makebuf.* *libc_nano.a:lib_a-memccpy.* *libc_nano.a:lib_a-memchr.* *libc_nano.a:lib_a-memcmp.* *libc_nano.a:lib_a-memcpy.* *libc_nano.a:lib_a-memmove.* *libc_nano.a:lib_a-memrchr.* *libc_nano.a:lib_a-memset.* *libc_nano.a:lib_a-mktime.* *libc_nano.a:lib_a-month_lengths.* *libc_nano.a:lib_a-open.* *libc_nano.a:lib_a-quorem.* *libc_nano.a:lib_a-raise.* *libc_nano.a:lib_a-rand.* *libc_nano.a:lib_a-rand_r.* *libc_nano.a:lib_a-read.* *libc_nano.a:lib_a-refill.* *libc_nano.a:lib_a-rshift.* *libc_nano.a:lib_a-s_fpclassify.* *libc_nano.a:lib_a-sbrk.* *libc_nano.a:lib_a-sccl.* *libc_nano.a:lib_a-setjmp.* *libc_nano.a:lib_a-sf_nan.* *libc_nano.a:lib_a-srand.* *libc_nano.a:lib_a-stdio.* *libc_nano.a:lib_a-strcasecmp.* *libc_nano.a:lib_a-strcasestr.* *libc_nano.a:lib_a-strcat.* *libc_nano.a:lib_a-strchr.* *libc_nano.a:lib_a-strcmp.* *libc_nano.a:lib_a-strcoll.* *libc_nano.a:lib_a-strcpy.* *libc_nano.a:lib_a-strcspn.* *libc_nano.a:lib_a-strdup.* *libc_nano.a:lib_a-strdup_r.* *libc_nano.a:lib_a-strftime.* *libc_nano.a:lib_a-strlcat.* *libc_nano.a:lib_a-strlcpy.* *libc_nano.a:lib_a-strlen.* *libc_nano.a:lib_a-strlwr.* *libc_nano.a:lib_a-strncasecmp.* *libc_nano.a:lib_a-strncat.* *libc_nano.a:lib_a-strncmp.* *libc_nano.a:lib_a-strncpy.* *libc_nano.a:lib_a-strndup.* *libc_nano.a:lib_a-strndup_r.* *libc_nano.a:lib_a-strnlen.* *libc_nano.a:lib_a-strptime.* *libc_nano.a:lib_a-strrchr.* *libc_nano.a:lib_a-strsep.* *libc_nano.a:lib_a-strspn.* *libc_nano.a:lib_a-strstr.* *libc_nano.a:lib_a-strtok_r.* *libc_nano.a:lib_a-strtol.* *libc_nano.a:lib_a-strtoul.* *libc_nano.a:lib_a-strupr.* *libc_nano.a:lib_a-sysclose.* *libc_nano.a:lib_a-sysopen.* *libc_nano.a:lib_a-sysread.* *libc_nano.a:lib_a-syssbrk.* *libc_nano.a:lib_a-system.* *libc_nano.a:lib_a-systimes.* *libc_nano.a:lib_a-syswrite.* *libc_nano.a:lib_a-time.* *libc_nano.a:lib_a-timelocal.* *libc_nano.a:lib_a-toascii.* *libc_nano.a:lib_a-tolower.* *libc_nano.a:lib_a-toupper.* *libc_nano.a:lib_a-tzcalc_limits.* *libc_nano.a:lib_a-tzlock.* *libc_nano.a:lib_a-tzset.* *libc_nano.a:lib_a-tzset_r.* *libc_nano.a:lib_a-tzvars.* *libc_nano.a:lib_a-ungetc.* *libc_nano.a:lib_a-utoa.* *libc_nano.a:lib_a-wbuf.* *libc_nano.a:lib_a-wcrtomb.* *libc_nano.a:lib_a-wctomb_r.* *libc_nano.a:lib_a-wsetup.* *libc_nano.a:lock.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:rtc_clk.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_spiflash.* *libesp_system.a:esp_err.* *libesp_system.a:ubsan.* *libfreertos.a:FreeRTOS-openocd.* *libhal.a:cache_hal_esp32.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:wdt_hal_iram.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.*) .srodata.*)


    *(.irom1.text) /* catch stray ICACHE_RODATA_ATTR */
    *(.gnu.linkonce.r.*)
    *(.rodata1)
    __XT_EXCEPTION_TABLE_ = ABSOLUTE(.);
    *(.xt_except_table)
    *(.gcc_except_table .gcc_except_table.*)
    *(.gnu.linkonce.e.*)
    *(.gnu.version_r)
    . = (. + 3) & ~ 3;
    __eh_frame = ABSOLUTE(.);
    KEEP(*(.eh_frame))
    . = (. + 7) & ~ 3;
    /*  C++ constructor and destructor tables

        Make a point of not including anything from crtbegin.o or crtend.o, as IDF doesn't use toolchain crt
      */
    __init_array_start = ABSOLUTE(.);
    KEEP (*(EXCLUDE_FILE (*crtend.* *crtbegin.*) .ctors SORT(.ctors.*)))
    __init_array_end = ABSOLUTE(.);

    KEEP (*crtbegin.*(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.*) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    /*  C++ exception handlers table:  */
    __XT_EXCEPTION_DESCS_ = ABSOLUTE(.);
    *(.xt_except_desc)
    *(.gnu.linkonce.h.*)
    __XT_EXCEPTION_DESCS_END__ = ABSOLUTE(.);
    *(.xt_except_desc_end)
    *(.dynamic)
    *(.gnu.version_d)
    /* Addresses of memory regions reserved via
       SOC_RESERVE_MEMORY_REGION() */
    soc_reserved_memory_region_start = ABSOLUTE(.);
    KEEP (*(.reserved_memory_address))
    soc_reserved_memory_region_end = ABSOLUTE(.);
    /* System init functions registered via ESP_SYSTEM_INIT_FN */
    _esp_system_init_fn_array_start = ABSOLUTE(.);
    KEEP (*(SORT_BY_INIT_PRIORITY(.esp_system_init_fn.*)))
    _esp_system_init_fn_array_end = ABSOLUTE(.);
    _rodata_end = ABSOLUTE(.);
    /* Literals are also RO data. */
    _lit4_start = ABSOLUTE(.);
    *(*.lit4)
    *(.lit4.*)
    *(.gnu.linkonce.lit4.*)
    _lit4_end = ABSOLUTE(.);
    . = ALIGN(4);
    _thread_local_start = ABSOLUTE(.);
    *(.tdata)
    *(.tdata.*)
    *(.tbss)
    *(.tbss.*)
    _thread_local_end = ABSOLUTE(.);
    . = ALIGN(4);
  } >default_rodata_seg

  _flash_rodata_align = ALIGNOF(.flash.rodata);

  /*
    This section is a place where we dump all the rodata which aren't used at runtime,
    so as to avoid binary size increase
  */
  .flash.rodata_noload (NOLOAD) :
  {
    /*
      This is a symbol marking the flash.rodata end, this can be used for mmu driver to maintain virtual address
      We don't need to include the noload rodata in this section
    */
    _rodata_reserved_end = ABSOLUTE(.);
    . = ALIGN (4);
    *(.rodata_wlog_debug .rodata_wlog_debug.*)
    *(.rodata_wlog_error .rodata_wlog_error.*)
    *(.rodata_wlog_info .rodata_wlog_info.*)
    *(.rodata_wlog_verbose .rodata_wlog_verbose.*)
    *(.rodata_wlog_warning .rodata_wlog_warning.*)
  } > default_rodata_seg

  .flash.text :
  {
    _stext = .;
    _instruction_reserved_start = ABSOLUTE(.);  /* This is a symbol marking the flash.text start, this can be used for mmu driver to maintain virtual address */
    _text_start = ABSOLUTE(.);

    *(EXCLUDE_FILE(*libesp_ringbuf.a *libfreertos.a *libgcov.a *librtc.a *libxt_hal.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libc_nano.a:creat.* *libc_nano.a:isatty.* *libc_nano.a:lib_a-abs.* *libc_nano.a:lib_a-asctime.* *libc_nano.a:lib_a-asctime_r.* *libc_nano.a:lib_a-atoi.* *libc_nano.a:lib_a-atol.* *libc_nano.a:lib_a-bzero.* *libc_nano.a:lib_a-close.* *libc_nano.a:lib_a-creat.* *libc_nano.a:lib_a-ctime.* *libc_nano.a:lib_a-ctime_r.* *libc_nano.a:lib_a-ctype_.* *libc_nano.a:lib_a-div.* *libc_nano.a:lib_a-environ.* *libc_nano.a:lib_a-envlock.* *libc_nano.a:lib_a-fclose.* *libc_nano.a:lib_a-fflush.* *libc_nano.a:lib_a-findfp.* *libc_nano.a:lib_a-fputwc.* *libc_nano.a:lib_a-fvwrite.* *libc_nano.a:lib_a-fwalk.* *libc_nano.a:lib_a-getenv_r.* *libc_nano.a:lib_a-gettzinfo.* *libc_nano.a:lib_a-gmtime.* *libc_nano.a:lib_a-gmtime_r.* *libc_nano.a:lib_a-impure.* *libc_nano.a:lib_a-isalnum.* *libc_nano.a:lib_a-isalpha.* *libc_nano.a:lib_a-isascii.* *libc_nano.a:lib_a-isblank.* *libc_nano.a:lib_a-iscntrl.* *libc_nano.a:lib_a-isdigit.* *libc_nano.a:lib_a-isgraph.* *libc_nano.a:lib_a-islower.* *libc_nano.a:lib_a-isprint.* *libc_nano.a:lib_a-ispunct.* *libc_nano.a:lib_a-isspace.* *libc_nano.a:lib_a-isupper.* *libc_nano.a:lib_a-itoa.* *libc_nano.a:lib_a-labs.* *libc_nano.a:lib_a-lcltime.* *libc_nano.a:lib_a-lcltime_r.* *libc_nano.a:lib_a-ldiv.* *libc_nano.a:lib_a-longjmp.* *libc_nano.a:lib_a-makebuf.* *libc_nano.a:lib_a-memccpy.* *libc_nano.a:lib_a-memchr.* *libc_nano.a:lib_a-memcmp.* *libc_nano.a:lib_a-memcpy.* *libc_nano.a:lib_a-memmove.* *libc_nano.a:lib_a-memrchr.* *libc_nano.a:lib_a-memset.* *libc_nano.a:lib_a-mktime.* *libc_nano.a:lib_a-month_lengths.* *libc_nano.a:lib_a-open.* *libc_nano.a:lib_a-quorem.* *libc_nano.a:lib_a-raise.* *libc_nano.a:lib_a-rand.* *libc_nano.a:lib_a-rand_r.* *libc_nano.a:lib_a-read.* *libc_nano.a:lib_a-refill.* *libc_nano.a:lib_a-rshift.* *libc_nano.a:lib_a-s_fpclassify.* *libc_nano.a:lib_a-sbrk.* *libc_nano.a:lib_a-sccl.* *libc_nano.a:lib_a-setjmp.* *libc_nano.a:lib_a-sf_nan.* *libc_nano.a:lib_a-srand.* *libc_nano.a:lib_a-stdio.* *libc_nano.a:lib_a-strcasecmp.* *libc_nano.a:lib_a-strcasestr.* *libc_nano.a:lib_a-strcat.* *libc_nano.a:lib_a-strchr.* *libc_nano.a:lib_a-strcmp.* *libc_nano.a:lib_a-strcoll.* *libc_nano.a:lib_a-strcpy.* *libc_nano.a:lib_a-strcspn.* *libc_nano.a:lib_a-strdup.* *libc_nano.a:lib_a-strdup_r.* *libc_nano.a:lib_a-strftime.* *libc_nano.a:lib_a-strlcat.* *libc_nano.a:lib_a-strlcpy.* *libc_nano.a:lib_a-strlen.* *libc_nano.a:lib_a-strlwr.* *libc_nano.a:lib_a-strncasecmp.* *libc_nano.a:lib_a-strncat.* *libc_nano.a:lib_a-strncmp.* *libc_nano.a:lib_a-strncpy.* *libc_nano.a:lib_a-strndup.* *libc_nano.a:lib_a-strndup_r.* *libc_nano.a:lib_a-strnlen.* *libc_nano.a:lib_a-strptime.* *libc_nano.a:lib_a-strrchr.* *libc_nano.a:lib_a-strsep.* *libc_nano.a:lib_a-strspn.* *libc_nano.a:lib_a-strstr.* *libc_nano.a:lib_a-strtok_r.* *libc_nano.a:lib_a-strtol.* *libc_nano.a:lib_a-strtoul.* *libc_nano.a:lib_a-strupr.* *libc_nano.a:lib_a-sysclose.* *libc_nano.a:lib_a-sysopen.* *libc_nano.a:lib_a-sysread.* *libc_nano.a:lib_a-syssbrk.* *libc_nano.a:lib_a-system.* *libc_nano.a:lib_a-systimes.* *libc_nano.a:lib_a-syswrite.* *libc_nano.a:lib_a-time.* *libc_nano.a:lib_a-timelocal.* *libc_nano.a:lib_a-toascii.* *libc_nano.a:lib_a-tolower.* *libc_nano.a:lib_a-toupper.* *libc_nano.a:lib_a-tzcalc_limits.* *libc_nano.a:lib_a-tzlock.* *libc_nano.a:lib_a-tzset.* *libc_nano.a:lib_a-tzset_r.* *libc_nano.a:lib_a-tzvars.* *libc_nano.a:lib_a-ungetc.* *libc_nano.a:lib_a-utoa.* *libc_nano.a:lib_a-wbuf.* *libc_nano.a:lib_a-wcrtomb.* *libc_nano.a:lib_a-wctomb_r.* *libc_nano.a:lib_a-wsetup.* *libc_nano.a:lock.* *libdriver.a:gptimer.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_gpio_reserve.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:rtc_sleep.* *libesp_hw_support.a:rtc_time.* *libesp_hw_support.a:rtc_wdt.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_spiflash.* *libesp_system.a:esp_err.* *libesp_system.a:esp_system_chip.* *libesp_system.a:ubsan.* *libgcc.a:lib2funcs.* *libhal.a:cache_hal_esp32.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:timer_hal.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:multi_heap_poisoning.* *libheap.a:tlsf.* *liblog.a:log.* *liblog.a:log_freertos.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.* *libxtensa.a:eri.* *libxtensa.a:xtensa_intr_asm.*) .literal EXCLUDE_FILE(*libesp_ringbuf.a *libfreertos.a *libgcov.a *librtc.a *libxt_hal.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libc_nano.a:creat.* *libc_nano.a:isatty.* *libc_nano.a:lib_a-abs.* *libc_nano.a:lib_a-asctime.* *libc_nano.a:lib_a-asctime_r.* *libc_nano.a:lib_a-atoi.* *libc_nano.a:lib_a-atol.* *libc_nano.a:lib_a-bzero.* *libc_nano.a:lib_a-close.* *libc_nano.a:lib_a-creat.* *libc_nano.a:lib_a-ctime.* *libc_nano.a:lib_a-ctime_r.* *libc_nano.a:lib_a-ctype_.* *libc_nano.a:lib_a-div.* *libc_nano.a:lib_a-environ.* *libc_nano.a:lib_a-envlock.* *libc_nano.a:lib_a-fclose.* *libc_nano.a:lib_a-fflush.* *libc_nano.a:lib_a-findfp.* *libc_nano.a:lib_a-fputwc.* *libc_nano.a:lib_a-fvwrite.* *libc_nano.a:lib_a-fwalk.* *libc_nano.a:lib_a-getenv_r.* *libc_nano.a:lib_a-gettzinfo.* *libc_nano.a:lib_a-gmtime.* *libc_nano.a:lib_a-gmtime_r.* *libc_nano.a:lib_a-impure.* *libc_nano.a:lib_a-isalnum.* *libc_nano.a:lib_a-isalpha.* *libc_nano.a:lib_a-isascii.* *libc_nano.a:lib_a-isblank.* *libc_nano.a:lib_a-iscntrl.* *libc_nano.a:lib_a-isdigit.* *libc_nano.a:lib_a-isgraph.* *libc_nano.a:lib_a-islower.* *libc_nano.a:lib_a-isprint.* *libc_nano.a:lib_a-ispunct.* *libc_nano.a:lib_a-isspace.* *libc_nano.a:lib_a-isupper.* *libc_nano.a:lib_a-itoa.* *libc_nano.a:lib_a-labs.* *libc_nano.a:lib_a-lcltime.* *libc_nano.a:lib_a-lcltime_r.* *libc_nano.a:lib_a-ldiv.* *libc_nano.a:lib_a-longjmp.* *libc_nano.a:lib_a-makebuf.* *libc_nano.a:lib_a-memccpy.* *libc_nano.a:lib_a-memchr.* *libc_nano.a:lib_a-memcmp.* *libc_nano.a:lib_a-memcpy.* *libc_nano.a:lib_a-memmove.* *libc_nano.a:lib_a-memrchr.* *libc_nano.a:lib_a-memset.* *libc_nano.a:lib_a-mktime.* *libc_nano.a:lib_a-month_lengths.* *libc_nano.a:lib_a-open.* *libc_nano.a:lib_a-quorem.* *libc_nano.a:lib_a-raise.* *libc_nano.a:lib_a-rand.* *libc_nano.a:lib_a-rand_r.* *libc_nano.a:lib_a-read.* *libc_nano.a:lib_a-refill.* *libc_nano.a:lib_a-rshift.* *libc_nano.a:lib_a-s_fpclassify.* *libc_nano.a:lib_a-sbrk.* *libc_nano.a:lib_a-sccl.* *libc_nano.a:lib_a-setjmp.* *libc_nano.a:lib_a-sf_nan.* *libc_nano.a:lib_a-srand.* *libc_nano.a:lib_a-stdio.* *libc_nano.a:lib_a-strcasecmp.* *libc_nano.a:lib_a-strcasestr.* *libc_nano.a:lib_a-strcat.* *libc_nano.a:lib_a-strchr.* *libc_nano.a:lib_a-strcmp.* *libc_nano.a:lib_a-strcoll.* *libc_nano.a:lib_a-strcpy.* *libc_nano.a:lib_a-strcspn.* *libc_nano.a:lib_a-strdup.* *libc_nano.a:lib_a-strdup_r.* *libc_nano.a:lib_a-strftime.* *libc_nano.a:lib_a-strlcat.* *libc_nano.a:lib_a-strlcpy.* *libc_nano.a:lib_a-strlen.* *libc_nano.a:lib_a-strlwr.* *libc_nano.a:lib_a-strncasecmp.* *libc_nano.a:lib_a-strncat.* *libc_nano.a:lib_a-strncmp.* *libc_nano.a:lib_a-strncpy.* *libc_nano.a:lib_a-strndup.* *libc_nano.a:lib_a-strndup_r.* *libc_nano.a:lib_a-strnlen.* *libc_nano.a:lib_a-strptime.* *libc_nano.a:lib_a-strrchr.* *libc_nano.a:lib_a-strsep.* *libc_nano.a:lib_a-strspn.* *libc_nano.a:lib_a-strstr.* *libc_nano.a:lib_a-strtok_r.* *libc_nano.a:lib_a-strtol.* *libc_nano.a:lib_a-strtoul.* *libc_nano.a:lib_a-strupr.* *libc_nano.a:lib_a-sysclose.* *libc_nano.a:lib_a-sysopen.* *libc_nano.a:lib_a-sysread.* *libc_nano.a:lib_a-syssbrk.* *libc_nano.a:lib_a-system.* *libc_nano.a:lib_a-systimes.* *libc_nano.a:lib_a-syswrite.* *libc_nano.a:lib_a-time.* *libc_nano.a:lib_a-timelocal.* *libc_nano.a:lib_a-toascii.* *libc_nano.a:lib_a-tolower.* *libc_nano.a:lib_a-toupper.* *libc_nano.a:lib_a-tzcalc_limits.* *libc_nano.a:lib_a-tzlock.* *libc_nano.a:lib_a-tzset.* *libc_nano.a:lib_a-tzset_r.* *libc_nano.a:lib_a-tzvars.* *libc_nano.a:lib_a-ungetc.* *libc_nano.a:lib_a-utoa.* *libc_nano.a:lib_a-wbuf.* *libc_nano.a:lib_a-wcrtomb.* *libc_nano.a:lib_a-wctomb_r.* *libc_nano.a:lib_a-wsetup.* *libc_nano.a:lock.* *libdriver.a:gptimer.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_gpio_reserve.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:rtc_sleep.* *libesp_hw_support.a:rtc_time.* *libesp_hw_support.a:rtc_wdt.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_spiflash.* *libesp_system.a:esp_err.* *libesp_system.a:esp_system_chip.* *libesp_system.a:ubsan.* *libgcc.a:lib2funcs.* *libhal.a:cache_hal_esp32.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:timer_hal.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:multi_heap_poisoning.* *libheap.a:tlsf.* *liblog.a:log.* *liblog.a:log_freertos.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.* *libxtensa.a:eri.* *libxtensa.a:xtensa_intr_asm.*) .literal.* EXCLUDE_FILE(*libesp_ringbuf.a *libfreertos.a *libgcov.a *librtc.a *libxt_hal.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libc_nano.a:creat.* *libc_nano.a:isatty.* *libc_nano.a:lib_a-abs.* *libc_nano.a:lib_a-asctime.* *libc_nano.a:lib_a-asctime_r.* *libc_nano.a:lib_a-atoi.* *libc_nano.a:lib_a-atol.* *libc_nano.a:lib_a-bzero.* *libc_nano.a:lib_a-close.* *libc_nano.a:lib_a-creat.* *libc_nano.a:lib_a-ctime.* *libc_nano.a:lib_a-ctime_r.* *libc_nano.a:lib_a-ctype_.* *libc_nano.a:lib_a-div.* *libc_nano.a:lib_a-environ.* *libc_nano.a:lib_a-envlock.* *libc_nano.a:lib_a-fclose.* *libc_nano.a:lib_a-fflush.* *libc_nano.a:lib_a-findfp.* *libc_nano.a:lib_a-fputwc.* *libc_nano.a:lib_a-fvwrite.* *libc_nano.a:lib_a-fwalk.* *libc_nano.a:lib_a-getenv_r.* *libc_nano.a:lib_a-gettzinfo.* *libc_nano.a:lib_a-gmtime.* *libc_nano.a:lib_a-gmtime_r.* *libc_nano.a:lib_a-impure.* *libc_nano.a:lib_a-isalnum.* *libc_nano.a:lib_a-isalpha.* *libc_nano.a:lib_a-isascii.* *libc_nano.a:lib_a-isblank.* *libc_nano.a:lib_a-iscntrl.* *libc_nano.a:lib_a-isdigit.* *libc_nano.a:lib_a-isgraph.* *libc_nano.a:lib_a-islower.* *libc_nano.a:lib_a-isprint.* *libc_nano.a:lib_a-ispunct.* *libc_nano.a:lib_a-isspace.* *libc_nano.a:lib_a-isupper.* *libc_nano.a:lib_a-itoa.* *libc_nano.a:lib_a-labs.* *libc_nano.a:lib_a-lcltime.* *libc_nano.a:lib_a-lcltime_r.* *libc_nano.a:lib_a-ldiv.* *libc_nano.a:lib_a-longjmp.* *libc_nano.a:lib_a-makebuf.* *libc_nano.a:lib_a-memccpy.* *libc_nano.a:lib_a-memchr.* *libc_nano.a:lib_a-memcmp.* *libc_nano.a:lib_a-memcpy.* *libc_nano.a:lib_a-memmove.* *libc_nano.a:lib_a-memrchr.* *libc_nano.a:lib_a-memset.* *libc_nano.a:lib_a-mktime.* *libc_nano.a:lib_a-month_lengths.* *libc_nano.a:lib_a-open.* *libc_nano.a:lib_a-quorem.* *libc_nano.a:lib_a-raise.* *libc_nano.a:lib_a-rand.* *libc_nano.a:lib_a-rand_r.* *libc_nano.a:lib_a-read.* *libc_nano.a:lib_a-refill.* *libc_nano.a:lib_a-rshift.* *libc_nano.a:lib_a-s_fpclassify.* *libc_nano.a:lib_a-sbrk.* *libc_nano.a:lib_a-sccl.* *libc_nano.a:lib_a-setjmp.* *libc_nano.a:lib_a-sf_nan.* *libc_nano.a:lib_a-srand.* *libc_nano.a:lib_a-stdio.* *libc_nano.a:lib_a-strcasecmp.* *libc_nano.a:lib_a-strcasestr.* *libc_nano.a:lib_a-strcat.* *libc_nano.a:lib_a-strchr.* *libc_nano.a:lib_a-strcmp.* *libc_nano.a:lib_a-strcoll.* *libc_nano.a:lib_a-strcpy.* *libc_nano.a:lib_a-strcspn.* *libc_nano.a:lib_a-strdup.* *libc_nano.a:lib_a-strdup_r.* *libc_nano.a:lib_a-strftime.* *libc_nano.a:lib_a-strlcat.* *libc_nano.a:lib_a-strlcpy.* *libc_nano.a:lib_a-strlen.* *libc_nano.a:lib_a-strlwr.* *libc_nano.a:lib_a-strncasecmp.* *libc_nano.a:lib_a-strncat.* *libc_nano.a:lib_a-strncmp.* *libc_nano.a:lib_a-strncpy.* *libc_nano.a:lib_a-strndup.* *libc_nano.a:lib_a-strndup_r.* *libc_nano.a:lib_a-strnlen.* *libc_nano.a:lib_a-strptime.* *libc_nano.a:lib_a-strrchr.* *libc_nano.a:lib_a-strsep.* *libc_nano.a:lib_a-strspn.* *libc_nano.a:lib_a-strstr.* *libc_nano.a:lib_a-strtok_r.* *libc_nano.a:lib_a-strtol.* *libc_nano.a:lib_a-strtoul.* *libc_nano.a:lib_a-strupr.* *libc_nano.a:lib_a-sysclose.* *libc_nano.a:lib_a-sysopen.* *libc_nano.a:lib_a-sysread.* *libc_nano.a:lib_a-syssbrk.* *libc_nano.a:lib_a-system.* *libc_nano.a:lib_a-systimes.* *libc_nano.a:lib_a-syswrite.* *libc_nano.a:lib_a-time.* *libc_nano.a:lib_a-timelocal.* *libc_nano.a:lib_a-toascii.* *libc_nano.a:lib_a-tolower.* *libc_nano.a:lib_a-toupper.* *libc_nano.a:lib_a-tzcalc_limits.* *libc_nano.a:lib_a-tzlock.* *libc_nano.a:lib_a-tzset.* *libc_nano.a:lib_a-tzset_r.* *libc_nano.a:lib_a-tzvars.* *libc_nano.a:lib_a-ungetc.* *libc_nano.a:lib_a-utoa.* *libc_nano.a:lib_a-wbuf.* *libc_nano.a:lib_a-wcrtomb.* *libc_nano.a:lib_a-wctomb_r.* *libc_nano.a:lib_a-wsetup.* *libc_nano.a:lock.* *libdriver.a:gptimer.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_gpio_reserve.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:rtc_sleep.* *libesp_hw_support.a:rtc_time.* *libesp_hw_support.a:rtc_wdt.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_spiflash.* *libesp_system.a:esp_err.* *libesp_system.a:esp_system_chip.* *libesp_system.a:ubsan.* *libgcc.a:lib2funcs.* *libhal.a:cache_hal_esp32.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:timer_hal.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:multi_heap_poisoning.* *libheap.a:tlsf.* *liblog.a:log.* *liblog.a:log_freertos.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.* *libxtensa.a:eri.* *libxtensa.a:xtensa_intr_asm.*) .text EXCLUDE_FILE(*libesp_ringbuf.a *libfreertos.a *libgcov.a *librtc.a *libxt_hal.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libc_nano.a:creat.* *libc_nano.a:isatty.* *libc_nano.a:lib_a-abs.* *libc_nano.a:lib_a-asctime.* *libc_nano.a:lib_a-asctime_r.* *libc_nano.a:lib_a-atoi.* *libc_nano.a:lib_a-atol.* *libc_nano.a:lib_a-bzero.* *libc_nano.a:lib_a-close.* *libc_nano.a:lib_a-creat.* *libc_nano.a:lib_a-ctime.* *libc_nano.a:lib_a-ctime_r.* *libc_nano.a:lib_a-ctype_.* *libc_nano.a:lib_a-div.* *libc_nano.a:lib_a-environ.* *libc_nano.a:lib_a-envlock.* *libc_nano.a:lib_a-fclose.* *libc_nano.a:lib_a-fflush.* *libc_nano.a:lib_a-findfp.* *libc_nano.a:lib_a-fputwc.* *libc_nano.a:lib_a-fvwrite.* *libc_nano.a:lib_a-fwalk.* *libc_nano.a:lib_a-getenv_r.* *libc_nano.a:lib_a-gettzinfo.* *libc_nano.a:lib_a-gmtime.* *libc_nano.a:lib_a-gmtime_r.* *libc_nano.a:lib_a-impure.* *libc_nano.a:lib_a-isalnum.* *libc_nano.a:lib_a-isalpha.* *libc_nano.a:lib_a-isascii.* *libc_nano.a:lib_a-isblank.* *libc_nano.a:lib_a-iscntrl.* *libc_nano.a:lib_a-isdigit.* *libc_nano.a:lib_a-isgraph.* *libc_nano.a:lib_a-islower.* *libc_nano.a:lib_a-isprint.* *libc_nano.a:lib_a-ispunct.* *libc_nano.a:lib_a-isspace.* *libc_nano.a:lib_a-isupper.* *libc_nano.a:lib_a-itoa.* *libc_nano.a:lib_a-labs.* *libc_nano.a:lib_a-lcltime.* *libc_nano.a:lib_a-lcltime_r.* *libc_nano.a:lib_a-ldiv.* *libc_nano.a:lib_a-longjmp.* *libc_nano.a:lib_a-makebuf.* *libc_nano.a:lib_a-memccpy.* *libc_nano.a:lib_a-memchr.* *libc_nano.a:lib_a-memcmp.* *libc_nano.a:lib_a-memcpy.* *libc_nano.a:lib_a-memmove.* *libc_nano.a:lib_a-memrchr.* *libc_nano.a:lib_a-memset.* *libc_nano.a:lib_a-mktime.* *libc_nano.a:lib_a-month_lengths.* *libc_nano.a:lib_a-open.* *libc_nano.a:lib_a-quorem.* *libc_nano.a:lib_a-raise.* *libc_nano.a:lib_a-rand.* *libc_nano.a:lib_a-rand_r.* *libc_nano.a:lib_a-read.* *libc_nano.a:lib_a-refill.* *libc_nano.a:lib_a-rshift.* *libc_nano.a:lib_a-s_fpclassify.* *libc_nano.a:lib_a-sbrk.* *libc_nano.a:lib_a-sccl.* *libc_nano.a:lib_a-setjmp.* *libc_nano.a:lib_a-sf_nan.* *libc_nano.a:lib_a-srand.* *libc_nano.a:lib_a-stdio.* *libc_nano.a:lib_a-strcasecmp.* *libc_nano.a:lib_a-strcasestr.* *libc_nano.a:lib_a-strcat.* *libc_nano.a:lib_a-strchr.* *libc_nano.a:lib_a-strcmp.* *libc_nano.a:lib_a-strcoll.* *libc_nano.a:lib_a-strcpy.* *libc_nano.a:lib_a-strcspn.* *libc_nano.a:lib_a-strdup.* *libc_nano.a:lib_a-strdup_r.* *libc_nano.a:lib_a-strftime.* *libc_nano.a:lib_a-strlcat.* *libc_nano.a:lib_a-strlcpy.* *libc_nano.a:lib_a-strlen.* *libc_nano.a:lib_a-strlwr.* *libc_nano.a:lib_a-strncasecmp.* *libc_nano.a:lib_a-strncat.* *libc_nano.a:lib_a-strncmp.* *libc_nano.a:lib_a-strncpy.* *libc_nano.a:lib_a-strndup.* *libc_nano.a:lib_a-strndup_r.* *libc_nano.a:lib_a-strnlen.* *libc_nano.a:lib_a-strptime.* *libc_nano.a:lib_a-strrchr.* *libc_nano.a:lib_a-strsep.* *libc_nano.a:lib_a-strspn.* *libc_nano.a:lib_a-strstr.* *libc_nano.a:lib_a-strtok_r.* *libc_nano.a:lib_a-strtol.* *libc_nano.a:lib_a-strtoul.* *libc_nano.a:lib_a-strupr.* *libc_nano.a:lib_a-sysclose.* *libc_nano.a:lib_a-sysopen.* *libc_nano.a:lib_a-sysread.* *libc_nano.a:lib_a-syssbrk.* *libc_nano.a:lib_a-system.* *libc_nano.a:lib_a-systimes.* *libc_nano.a:lib_a-syswrite.* *libc_nano.a:lib_a-time.* *libc_nano.a:lib_a-timelocal.* *libc_nano.a:lib_a-toascii.* *libc_nano.a:lib_a-tolower.* *libc_nano.a:lib_a-toupper.* *libc_nano.a:lib_a-tzcalc_limits.* *libc_nano.a:lib_a-tzlock.* *libc_nano.a:lib_a-tzset.* *libc_nano.a:lib_a-tzset_r.* *libc_nano.a:lib_a-tzvars.* *libc_nano.a:lib_a-ungetc.* *libc_nano.a:lib_a-utoa.* *libc_nano.a:lib_a-wbuf.* *libc_nano.a:lib_a-wcrtomb.* *libc_nano.a:lib_a-wctomb_r.* *libc_nano.a:lib_a-wsetup.* *libc_nano.a:lock.* *libdriver.a:gptimer.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_gpio_reserve.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:rtc_sleep.* *libesp_hw_support.a:rtc_time.* *libesp_hw_support.a:rtc_wdt.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_mm.a:cache_esp32.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_spiflash.* *libesp_system.a:esp_err.* *libesp_system.a:esp_system_chip.* *libesp_system.a:ubsan.* *libgcc.a:lib2funcs.* *libhal.a:cache_hal_esp32.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_iram.* *libhal.a:timer_hal.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:multi_heap_poisoning.* *libheap.a:tlsf.* *liblog.a:log.* *liblog.a:log_freertos.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_wrap.* *libxtensa.a:eri.* *libxtensa.a:xtensa_intr_asm.*) .text.*)
    *(.wifi0iram .wifi0iram.*)
    *(.wifiextrairam .wifiextrairam.*)
    *(.wifiorslpiram .wifiorslpiram.*)
    *(.wifirxiram .wifirxiram.*)
    *(.wifislpiram .wifislpiram.*)
    *(.wifislprxiram .wifislprxiram.*)
    *libcoexist.a:(.coexiram .coexiram.*)
    *libcoexist.a:(.coexsleepiram .coexsleepiram.*)
    *libdriver.a:gptimer.*(.literal.gptimer_del_timer .literal.gptimer_destroy .literal.gptimer_disable .literal.gptimer_enable .literal.gptimer_get_captured_count .literal.gptimer_get_raw_count .literal.gptimer_new_timer .literal.gptimer_register_event_callbacks .literal.gptimer_release_group_handle .literal.gptimer_set_alarm_action .literal.gptimer_set_raw_count .literal.gptimer_start .literal.gptimer_stop .text .text.gptimer_del_timer .text.gptimer_destroy .text.gptimer_disable .text.gptimer_enable .text.gptimer_get_captured_count .text.gptimer_get_raw_count .text.gptimer_get_resolution .text.gptimer_new_timer .text.gptimer_register_event_callbacks .text.gptimer_release_group_handle .text.gptimer_set_alarm_action .text.gptimer_set_raw_count .text.gptimer_start .text.gptimer_stop)
    *libesp_hw_support.a:cpu.*(.literal.esp_cpu_set_watchpoint .text .text.esp_cpu_clear_breakpoint .text.esp_cpu_clear_watchpoint .text.esp_cpu_set_breakpoint .text.esp_cpu_set_watchpoint)
    *libesp_hw_support.a:esp_gpio_reserve.*(.text)
    *libesp_hw_support.a:sar_periph_ctrl.*(.literal.s_sar_power_acquire .literal.s_sar_power_release .literal.sar_periph_ctrl_adc_continuous_power_acquire .literal.sar_periph_ctrl_adc_continuous_power_release .literal.sar_periph_ctrl_adc_oneshot_power_acquire .literal.sar_periph_ctrl_adc_oneshot_power_release .literal.sar_periph_ctrl_init .literal.sar_periph_ctrl_power_disable .literal.sar_periph_ctrl_pwdet_power_acquire .literal.sar_periph_ctrl_pwdet_power_release .text .text.s_sar_power_acquire .text.s_sar_power_release .text.sar_periph_ctrl_adc_continuous_power_acquire .text.sar_periph_ctrl_adc_continuous_power_release .text.sar_periph_ctrl_adc_oneshot_power_acquire .text.sar_periph_ctrl_adc_oneshot_power_release .text.sar_periph_ctrl_init .text.sar_periph_ctrl_power_disable .text.sar_periph_ctrl_pwdet_power_acquire .text.sar_periph_ctrl_pwdet_power_release)
    *libesp_ringbuf.a:ringbuf.*(.literal.prvGetCurMaxSizeAllowSplit .text.prvGetCurMaxSizeAllowSplit)
    *libesp_ringbuf.a:ringbuf.*(.literal.prvGetCurMaxSizeByteBuf .text.prvGetCurMaxSizeByteBuf)
    *libesp_ringbuf.a:ringbuf.*(.literal.prvGetCurMaxSizeNoSplit .text.prvGetCurMaxSizeNoSplit)
    *libesp_ringbuf.a:ringbuf.*(.literal.prvInitializeNewRingbuffer .text.prvInitializeNewRingbuffer)
    *libesp_ringbuf.a:ringbuf.*(.literal.prvReceiveGeneric .text.prvReceiveGeneric)
    *libesp_ringbuf.a:ringbuf.*(.literal.prvSendAcquireGeneric .text.prvSendAcquireGeneric)
    *libesp_ringbuf.a:ringbuf.*(.literal.vRingbufferDelete .text.vRingbufferDelete)
    *libesp_ringbuf.a:ringbuf.*(.literal.vRingbufferGetInfo .text.vRingbufferGetInfo)
    *libesp_ringbuf.a:ringbuf.*(.literal.vRingbufferReturnItem .text.vRingbufferReturnItem)
    *libesp_ringbuf.a:ringbuf.*(.literal.xRingbufferAddToQueueSetRead .text.xRingbufferAddToQueueSetRead)
    *libesp_ringbuf.a:ringbuf.*(.literal.xRingbufferCreate .text.xRingbufferCreate)
    *libesp_ringbuf.a:ringbuf.*(.literal.xRingbufferCreateNoSplit .text.xRingbufferCreateNoSplit)
    *libesp_ringbuf.a:ringbuf.*(.literal.xRingbufferCreateStatic .text.xRingbufferCreateStatic)
    *libesp_ringbuf.a:ringbuf.*(.literal.xRingbufferGetCurFreeSize .text.xRingbufferGetCurFreeSize)
    *libesp_ringbuf.a:ringbuf.*(.literal.xRingbufferGetMaxItemSize .text.xRingbufferGetMaxItemSize)
    *libesp_ringbuf.a:ringbuf.*(.literal.xRingbufferPrintInfo .text.xRingbufferPrintInfo)
    *libesp_ringbuf.a:ringbuf.*(.literal.xRingbufferReceive .text.xRingbufferReceive)
    *libesp_ringbuf.a:ringbuf.*(.literal.xRingbufferReceiveSplit .text.xRingbufferReceiveSplit)
    *libesp_ringbuf.a:ringbuf.*(.literal.xRingbufferReceiveUpTo .text.xRingbufferReceiveUpTo)
    *libesp_ringbuf.a:ringbuf.*(.literal.xRingbufferRemoveFromQueueSetRead .text.xRingbufferRemoveFromQueueSetRead)
    *libesp_ringbuf.a:ringbuf.*(.literal.xRingbufferSend .text.xRingbufferSend)
    *libesp_ringbuf.a:ringbuf.*(.literal.xRingbufferSendAcquire .text.xRingbufferSendAcquire)
    *libesp_ringbuf.a:ringbuf.*(.literal.xRingbufferSendComplete .text.xRingbufferSendComplete)
    *libesp_system.a:esp_system_chip.*(.literal.esp_get_free_heap_size .literal.esp_get_free_internal_heap_size .literal.esp_get_idf_version .literal.esp_get_minimum_free_heap_size .text .text.esp_get_free_heap_size .text.esp_get_free_internal_heap_size .text.esp_get_idf_version .text.esp_get_minimum_free_heap_size)
    *libfreertos.a:app_startup.*(.literal .literal.* .text .text.*)
    *libfreertos.a:event_groups.*(.literal.vEventGroupClearBitsCallback .text.vEventGroupClearBitsCallback)
    *libfreertos.a:event_groups.*(.literal.vEventGroupDelete .text.vEventGroupDelete)
    *libfreertos.a:event_groups.*(.literal.vEventGroupSetBitsCallback .text.vEventGroupSetBitsCallback)
    *libfreertos.a:event_groups.*(.literal.xEventGroupClearBits .text.xEventGroupClearBits)
    *libfreertos.a:event_groups.*(.literal.xEventGroupCreate .text.xEventGroupCreate)
    *libfreertos.a:event_groups.*(.literal.xEventGroupCreateStatic .text.xEventGroupCreateStatic)
    *libfreertos.a:event_groups.*(.literal.xEventGroupSetBits .text.xEventGroupSetBits)
    *libfreertos.a:event_groups.*(.literal.xEventGroupSync .text.xEventGroupSync)
    *libfreertos.a:event_groups.*(.literal.xEventGroupWaitBits .text.xEventGroupWaitBits)
    *libfreertos.a:heap_idf.*(.literal.pvPortMalloc .text.pvPortMalloc)
    *libfreertos.a:heap_idf.*(.literal.vPortFree .text.vPortFree)
    *libfreertos.a:heap_idf.*(.literal.xPortCheckValidTCBMem .text.xPortCheckValidTCBMem)
    *libfreertos.a:heap_idf.*(.literal.xPortGetFreeHeapSize .text.xPortGetFreeHeapSize)
    *libfreertos.a:heap_idf.*(.literal.xPortGetMinimumEverFreeHeapSize .text.xPortGetMinimumEverFreeHeapSize)
    *libfreertos.a:heap_idf.*(.literal.xPortcheckValidStackMem .text.xPortcheckValidStackMem)
    *libfreertos.a:idf_additions.*(.literal .literal.* .text .text.*)
    *libfreertos.a:list.*(.literal.vListInitialise .text.vListInitialise)
    *libfreertos.a:list.*(.literal.vListInitialiseItem .text.vListInitialiseItem)
    *libfreertos.a:port.*(.literal.pxPortInitialiseStack .text.pxPortInitialiseStack)
    *libfreertos.a:port.*(.literal.vPortCleanUpCoprocArea .text.vPortCleanUpCoprocArea)
    *libfreertos.a:port.*(.literal.xPortGetTickRateHz .text.xPortGetTickRateHz)
    *libfreertos.a:port.*(.literal.xPortStartScheduler .text.xPortStartScheduler)
    *libfreertos.a:port_common.*(.literal.vApplicationGetIdleTaskMemory .text.vApplicationGetIdleTaskMemory)
    *libfreertos.a:port_common.*(.literal.vApplicationGetTimerTaskMemory .text.vApplicationGetTimerTaskMemory)
    *libfreertos.a:queue.*(.literal.uxQueueMessagesWaiting .text.uxQueueMessagesWaiting)
    *libfreertos.a:queue.*(.literal.uxQueueSpacesAvailable .text.uxQueueSpacesAvailable)
    *libfreertos.a:queue.*(.literal.vQueueDelete .text.vQueueDelete)
    *libfreertos.a:queue.*(.literal.vQueueWaitForMessageRestricted .text.vQueueWaitForMessageRestricted)
    *libfreertos.a:queue.*(.literal.xQueueAddToSet .text.xQueueAddToSet)
    *libfreertos.a:queue.*(.literal.xQueueCreateCountingSemaphore .text.xQueueCreateCountingSemaphore)
    *libfreertos.a:queue.*(.literal.xQueueCreateCountingSemaphoreStatic .text.xQueueCreateCountingSemaphoreStatic)
    *libfreertos.a:queue.*(.literal.xQueueCreateMutex .text.xQueueCreateMutex)
    *libfreertos.a:queue.*(.literal.xQueueCreateMutexStatic .text.xQueueCreateMutexStatic)
    *libfreertos.a:queue.*(.literal.xQueueCreateSet .text.xQueueCreateSet)
    *libfreertos.a:queue.*(.literal.xQueueGenericCreate .text.xQueueGenericCreate)
    *libfreertos.a:queue.*(.literal.xQueueGenericCreateStatic .text.xQueueGenericCreateStatic)
    *libfreertos.a:queue.*(.literal.xQueueGenericReset .text.xQueueGenericReset)
    *libfreertos.a:queue.*(.literal.xQueueGenericSend .text.xQueueGenericSend)
    *libfreertos.a:queue.*(.literal.xQueueGetMutexHolder .text.xQueueGetMutexHolder)
    *libfreertos.a:queue.*(.literal.xQueueGiveMutexRecursive .text.xQueueGiveMutexRecursive)
    *libfreertos.a:queue.*(.literal.xQueuePeek .text.xQueuePeek)
    *libfreertos.a:queue.*(.literal.xQueueReceive .text.xQueueReceive)
    *libfreertos.a:queue.*(.literal.xQueueRemoveFromSet .text.xQueueRemoveFromSet)
    *libfreertos.a:queue.*(.literal.xQueueSelectFromSet .text.xQueueSelectFromSet)
    *libfreertos.a:queue.*(.literal.xQueueSemaphoreTake .text.xQueueSemaphoreTake)
    *libfreertos.a:queue.*(.literal.xQueueTakeMutexRecursive .text.xQueueTakeMutexRecursive)
    *libfreertos.a:stream_buffer.*(.literal.prvReadBytesFromBuffer .text.prvReadBytesFromBuffer)
    *libfreertos.a:stream_buffer.*(.literal.prvWriteBytesToBuffer .text.prvWriteBytesToBuffer)
    *libfreertos.a:stream_buffer.*(.literal.vStreamBufferDelete .text.vStreamBufferDelete)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferBytesAvailable .text.xStreamBufferBytesAvailable)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferGenericCreate .text.xStreamBufferGenericCreate)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferGenericCreateStatic .text.xStreamBufferGenericCreateStatic)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferIsEmpty .text.xStreamBufferIsEmpty)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferIsFull .text.xStreamBufferIsFull)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferNextMessageLengthBytes .text.xStreamBufferNextMessageLengthBytes)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferReceive .text.xStreamBufferReceive)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferReset .text.xStreamBufferReset)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferSend .text.xStreamBufferSend)
    *libfreertos.a:stream_buffer.*(.literal.xStreamBufferSetTriggerLevel .text.xStreamBufferSetTriggerLevel)
    *libfreertos.a:tasks.*(.literal.eTaskGetState .text.eTaskGetState)
    *libfreertos.a:tasks.*(.literal.pcTaskGetName .text.pcTaskGetName)
    *libfreertos.a:tasks.*(.literal.prvAddCurrentTaskToDelayedList .text.prvAddCurrentTaskToDelayedList)
    *libfreertos.a:tasks.*(.literal.prvAddNewTaskToReadyList .text.prvAddNewTaskToReadyList)
    *libfreertos.a:tasks.*(.literal.prvDeleteTCB .text.prvDeleteTCB)
    *libfreertos.a:tasks.*(.literal.prvDeleteTLS .text.prvDeleteTLS)
    *libfreertos.a:tasks.*(.literal.prvIdleTask .text.prvIdleTask)
    *libfreertos.a:tasks.*(.literal.prvSearchForNameWithinSingleList .text.prvSearchForNameWithinSingleList)
    *libfreertos.a:tasks.*(.literal.prvTaskPriorityRaise .text.prvTaskPriorityRaise)
    *libfreertos.a:tasks.*(.literal.prvTaskPriorityRestore .text.prvTaskPriorityRestore)
    *libfreertos.a:tasks.*(.literal.pvTaskGetThreadLocalStoragePointer .text.pvTaskGetThreadLocalStoragePointer)
    *libfreertos.a:tasks.*(.literal.pvTaskIncrementMutexHeldCount .text.pvTaskIncrementMutexHeldCount)
    *libfreertos.a:tasks.*(.literal.pxGetNextTaskList .text.pxGetNextTaskList)
    *libfreertos.a:tasks.*(.literal.pxTaskGetNext .text.pxTaskGetNext)
    *libfreertos.a:tasks.*(.literal.pxTaskGetStackStart .text.pxTaskGetStackStart)
    *libfreertos.a:tasks.*(.literal.ulTaskGenericNotifyTake .text.ulTaskGenericNotifyTake)
    *libfreertos.a:tasks.*(.literal.ulTaskGenericNotifyValueClear .text.ulTaskGenericNotifyValueClear)
    *libfreertos.a:tasks.*(.literal.ulTaskNotifyTake .text.ulTaskNotifyTake)
    *libfreertos.a:tasks.*(.literal.uxTaskGetNumberOfTasks .text.uxTaskGetNumberOfTasks)
    *libfreertos.a:tasks.*(.literal.uxTaskGetSnapshotAll .text.uxTaskGetSnapshotAll)
    *libfreertos.a:tasks.*(.literal.uxTaskGetStackHighWaterMark .text.uxTaskGetStackHighWaterMark)
    *libfreertos.a:tasks.*(.literal.uxTaskGetStackHighWaterMark2 .text.uxTaskGetStackHighWaterMark2)
    *libfreertos.a:tasks.*(.literal.uxTaskPriorityGet .text.uxTaskPriorityGet)
    *libfreertos.a:tasks.*(.literal.uxTaskResetEventItemValue .text.uxTaskResetEventItemValue)
    *libfreertos.a:tasks.*(.literal.vTaskDelay .text.vTaskDelay)
    *libfreertos.a:tasks.*(.literal.vTaskDelayUntil .text.vTaskDelayUntil)
    *libfreertos.a:tasks.*(.literal.vTaskDelete .text.vTaskDelete)
    *libfreertos.a:tasks.*(.literal.vTaskEndScheduler .text.vTaskEndScheduler)
    *libfreertos.a:tasks.*(.literal.vTaskInternalSetTimeOutState .text.vTaskInternalSetTimeOutState)
    *libfreertos.a:tasks.*(.literal.vTaskMissedYield .text.vTaskMissedYield)
    *libfreertos.a:tasks.*(.literal.vTaskPlaceOnEventList .text.vTaskPlaceOnEventList)
    *libfreertos.a:tasks.*(.literal.vTaskPlaceOnEventListRestricted .text.vTaskPlaceOnEventListRestricted)
    *libfreertos.a:tasks.*(.literal.vTaskPlaceOnUnorderedEventList .text.vTaskPlaceOnUnorderedEventList)
    *libfreertos.a:tasks.*(.literal.vTaskPriorityDisinheritAfterTimeout .text.vTaskPriorityDisinheritAfterTimeout)
    *libfreertos.a:tasks.*(.literal.vTaskPrioritySet .text.vTaskPrioritySet)
    *libfreertos.a:tasks.*(.literal.vTaskReleaseKernelLock .text.vTaskReleaseKernelLock)
    *libfreertos.a:tasks.*(.literal.vTaskRemoveFromUnorderedEventList .text.vTaskRemoveFromUnorderedEventList)
    *libfreertos.a:tasks.*(.literal.vTaskResume .text.vTaskResume)
    *libfreertos.a:tasks.*(.literal.vTaskSetThreadLocalStoragePointer .text.vTaskSetThreadLocalStoragePointer)
    *libfreertos.a:tasks.*(.literal.vTaskSetThreadLocalStoragePointerAndDelCallback .text.vTaskSetThreadLocalStoragePointerAndDelCallback)
    *libfreertos.a:tasks.*(.literal.vTaskSetTimeOutState .text.vTaskSetTimeOutState)
    *libfreertos.a:tasks.*(.literal.vTaskStartScheduler .text.vTaskStartScheduler)
    *libfreertos.a:tasks.*(.literal.vTaskSuspend .text.vTaskSuspend)
    *libfreertos.a:tasks.*(.literal.vTaskSuspendAll .text.vTaskSuspendAll)
    *libfreertos.a:tasks.*(.literal.vTaskTakeKernelLock .text.vTaskTakeKernelLock)
    *libfreertos.a:tasks.*(.literal.xTaskAbortDelay .text.xTaskAbortDelay)
    *libfreertos.a:tasks.*(.literal.xTaskCheckForTimeOut .text.xTaskCheckForTimeOut)
    *libfreertos.a:tasks.*(.literal.xTaskCreatePinnedToCore .text.xTaskCreatePinnedToCore)
    *libfreertos.a:tasks.*(.literal.xTaskCreateStaticPinnedToCore .text.xTaskCreateStaticPinnedToCore)
    *libfreertos.a:tasks.*(.literal.xTaskDelayUntil .text.xTaskDelayUntil)
    *libfreertos.a:tasks.*(.literal.xTaskGenericNotify .text.xTaskGenericNotify)
    *libfreertos.a:tasks.*(.literal.xTaskGenericNotifyStateClear .text.xTaskGenericNotifyStateClear)
    *libfreertos.a:tasks.*(.literal.xTaskGenericNotifyWait .text.xTaskGenericNotifyWait)
    *libfreertos.a:tasks.*(.literal.xTaskGetAffinity .text.xTaskGetAffinity)
    *libfreertos.a:tasks.*(.literal.xTaskGetCurrentTaskHandle .text.xTaskGetCurrentTaskHandle)
    *libfreertos.a:tasks.*(.literal.xTaskGetCurrentTaskHandleForCPU .text.xTaskGetCurrentTaskHandleForCPU)
    *libfreertos.a:tasks.*(.literal.xTaskGetHandle .text.xTaskGetHandle)
    *libfreertos.a:tasks.*(.literal.xTaskGetIdleTaskHandle .text.xTaskGetIdleTaskHandle)
    *libfreertos.a:tasks.*(.literal.xTaskGetIdleTaskHandleForCPU .text.xTaskGetIdleTaskHandleForCPU)
    *libfreertos.a:tasks.*(.literal.xTaskNotifyWait .text.xTaskNotifyWait)
    *libfreertos.a:tasks.*(.literal.xTaskPriorityDisinherit .text.xTaskPriorityDisinherit)
    *libfreertos.a:tasks.*(.literal.xTaskPriorityInherit .text.xTaskPriorityInherit)
    *libfreertos.a:tasks.*(.literal.xTaskResumeAll .text.xTaskResumeAll)
    *libfreertos.a:timers.*(.literal.pcTimerGetName .text.pcTimerGetName)
    *libfreertos.a:timers.*(.literal.prvCheckForValidListAndQueue .text.prvCheckForValidListAndQueue)
    *libfreertos.a:timers.*(.literal.prvInsertTimerInActiveList .text.prvInsertTimerInActiveList)
    *libfreertos.a:timers.*(.literal.prvSampleTimeNow .text.prvSampleTimeNow)
    *libfreertos.a:timers.*(.literal.prvTimerTask .text.prvTimerTask)
    *libfreertos.a:timers.*(.literal.pvTimerGetTimerID .text.pvTimerGetTimerID)
    *libfreertos.a:timers.*(.literal.uxTimerGetReloadMode .text.uxTimerGetReloadMode)
    *libfreertos.a:timers.*(.literal.vTimerSetReloadMode .text.vTimerSetReloadMode)
    *libfreertos.a:timers.*(.literal.vTimerSetTimerID .text.vTimerSetTimerID)
    *libfreertos.a:timers.*(.literal.xTimerCreate .text.xTimerCreate)
    *libfreertos.a:timers.*(.literal.xTimerCreateStatic .text.xTimerCreateStatic)
    *libfreertos.a:timers.*(.literal.xTimerCreateTimerTask .text.xTimerCreateTimerTask)
    *libfreertos.a:timers.*(.literal.xTimerGenericCommand .text.xTimerGenericCommand)
    *libfreertos.a:timers.*(.literal.xTimerGetExpiryTime .text.xTimerGetExpiryTime)
    *libfreertos.a:timers.*(.literal.xTimerGetPeriod .text.xTimerGetPeriod)
    *libfreertos.a:timers.*(.literal.xTimerGetTimerDaemonTaskHandle .text.xTimerGetTimerDaemonTaskHandle)
    *libfreertos.a:timers.*(.literal.xTimerIsTimerActive .text.xTimerIsTimerActive)
    *libfreertos.a:timers.*(.literal.xTimerPendFunctionCall .text.xTimerPendFunctionCall)
    *libhal.a:timer_hal.*(.literal.timer_hal_deinit .literal.timer_hal_init .text .text.timer_hal_deinit .text.timer_hal_init .text.timer_hal_set_counter_value)
    *libheap.a:multi_heap.*(.literal.multi_heap_check .literal.multi_heap_dump .literal.multi_heap_dump_tlsf .literal.multi_heap_get_info_impl .literal.multi_heap_register_impl .literal.tlsf_check_hook .text .text.multi_heap_check .text.multi_heap_dump .text.multi_heap_dump_tlsf .text.multi_heap_free_size_impl .text.multi_heap_get_info_impl .text.multi_heap_get_info_tlsf .text.multi_heap_minimum_free_size_impl .text.multi_heap_register_impl .text.tlsf_check_hook)
    *libheap.a:multi_heap_poisoning.*(.literal.multi_heap_free_size .literal.multi_heap_get_info .literal.multi_heap_minimum_free_size .literal.multi_heap_register .text .text.multi_heap_free_size .text.multi_heap_get_info .text.multi_heap_minimum_free_size .text.multi_heap_register)
    *libheap.a:tlsf.*(.literal.default_walker .literal.integrity_walker .literal.tlsf_add_pool .literal.tlsf_check_pool .literal.tlsf_create .literal.tlsf_create_with_pool .literal.tlsf_walk_pool .text .text.default_walker .text.integrity_walker .text.tlsf_add_pool .text.tlsf_check .text.tlsf_check_pool .text.tlsf_create .text.tlsf_create_with_pool .text.tlsf_destroy .text.tlsf_fit_size .text.tlsf_pool_overhead .text.tlsf_remove_pool .text.tlsf_walk_pool)
    *liblog.a:log.*(.literal.esp_log_level_get .literal.esp_log_level_set .literal.esp_log_set_vprintf .literal.esp_log_writev .literal.fix_cache_generation_overflow .literal.heap_bubble_down .literal.s_log_level_get_and_unlock .text .text.esp_log_level_get .text.esp_log_level_set .text.esp_log_set_vprintf .text.esp_log_writev .text.fix_cache_generation_overflow .text.heap_bubble_down .text.s_log_level_get_and_unlock)
    *liblog.a:log_freertos.*(.literal.esp_log_system_timestamp .text .text.esp_log_system_timestamp)

    *(.stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.irom0.text) /* catch stray ICACHE_RODATA_ATTR */
    *(.fini.literal)
    *(.fini)
    *(.gnu.version)

    /** CPU will try to prefetch up to 16 bytes of
      * of instructions. This means that any configuration (e.g. MMU, PMS) must allow
      * safe access to up to 16 bytes after the last real instruction, add
      * dummy bytes to ensure this
      */
    . += _esp_flash_mmap_prefetch_pad_size;

    _text_end = ABSOLUTE(.);
    _instruction_reserved_end = ABSOLUTE(.);  /* This is a symbol marking the flash.text end, this can be used for mmu driver to maintain virtual address */
    _etext = .;

    /* Similar to _iram_start, this symbol goes here so it is
       resolved by addr2line in preference to the first symbol in
       the flash.text segment.
    */
    _flash_cache_start = ABSOLUTE(0);
  } >default_code_seg

  /* Marks the end of IRAM code segment */
  .iram0.text_end (NOLOAD) :
  {
    . = ALIGN (4);
    _iram_text_end = ABSOLUTE(.);
  } > iram0_0_seg

  .iram0.data :
  {
    . = ALIGN(4);
    _iram_data_start = ABSOLUTE(.);

    *(.iram.data .iram.data.*)
    _coredump_iram_start = ABSOLUTE(.);
    *(.iram2.coredump .iram2.coredump.*)
    _coredump_iram_end = ABSOLUTE(.);

    _iram_data_end = ABSOLUTE(.);
  } > iram0_0_seg

  .iram0.bss (NOLOAD) :
  {
    . = ALIGN(4);
    _iram_bss_start = ABSOLUTE(.);

    *(.iram.bss .iram.bss.*)

    _iram_bss_end = ABSOLUTE(.);
    . = ALIGN(4);
    _iram_end = ABSOLUTE(.);
   } > iram0_0_seg

  /* Marks the end of data, bss and possibly rodata  */
  .dram0.heap_start (NOLOAD) :
  {
    . = ALIGN (8);
    /* Lowest possible start address for the heap */
    _heap_low_start = ABSOLUTE(.);
  } > dram0_0_seg

  /** This section will be used by the debugger and disassembler to get more information
    * about raw data present in the code.
    * Indeed, it may be required to add some padding at some points in the code
    * in order to align a branch/jump destination on a particular bound.
    * Padding these instructions will generate null bytes that shall be
    * interpreted as data, and not code by the debugger or disassembler.
    * This section will only be present in the ELF file, not in the final binary
    * For more details, check GCC-212
    */
  .xt.prop 0 :
  {
    KEEP (*(.xt.prop .gnu.linkonce.prop.*))
  }

  .xt.lit 0 :
  {
    KEEP (*(.xt.lit .gnu.linkonce.p.*))
  }
}

ASSERT(((_iram_end - ORIGIN(iram0_0_seg)) <= LENGTH(iram0_0_seg)),
          "IRAM0 segment data does not fit.")

ASSERT(((_heap_low_start - ORIGIN(dram0_0_seg)) <= LENGTH(dram0_0_seg)),
          "DRAM segment data does not fit.")
