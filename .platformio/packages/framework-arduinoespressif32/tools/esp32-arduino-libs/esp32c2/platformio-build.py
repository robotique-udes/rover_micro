# Copyright 2014-present PlatformIO <contact@platformio.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Arduino

Arduino Wiring-based Framework allows writing cross-platform software to
control devices attached to a wide range of Arduino boards to create all
kinds of creative coding, interactive objects, spaces or physical experiences.

http://arduino.cc/en/Reference/HomePage
"""

# Extends: https://github.com/platformio/platform-espressif32/blob/develop/builder/main.py

from os.path import basename, join

from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()

FRAMEWORK_DIR = env.PioPlatform().get_package_dir("framework-arduinoespressif32")
FRAMEWORK_SDK_DIR = join(FRAMEWORK_DIR, "tools", "esp32-arduino-libs")

board_config = env.BoardConfig()

flatten_cppdefines = env.Flatten(env['CPPDEFINES'])

#
# zigbee libs
#
if "ZIGBEE_MODE_ZCZR" in flatten_cppdefines:
    env.Append(
        LIBS=[
            "-lesp_zb_api_zczr",
            "-lesp_zb_cli_command",
            "-lzboss_stack.zczr",
            "-lzboss_port"
        ]
    )
if "ZIGBEE_MODE_ED" in flatten_cppdefines:
    env.Append(
        LIBS=[
            "-lesp_zb_api_ed",
            "-lesp_zb_cli_command",
            "-lzboss_stack.ed",
            "-lzboss_port"
        ]
    )
if "ZIGBEE_MODE_RCP" in flatten_cppdefines:
    env.Append(
        LIBS=[
            "-lesp_zb_api_rcp",
            "-lesp_zb_cli_command",
            "-lzboss_stack.rcp",
            "-lzboss_port"
        ]
    )

env.Append(
    ASFLAGS=[
        "-march=rv32imc"
    ],

    ASPPFLAGS=[
        "-x", "assembler-with-cpp"
    ],

    CFLAGS=[
        "-march=rv32imc_zicsr_zifencei",
        "-std=gnu17",
        "-Wno-old-style-declaration"
    ],

    CXXFLAGS=[
        "-march=rv32imc_zicsr_zifencei",
        "-std=gnu++2b",
        "-fno-exceptions",
        "-fno-rtti"
    ],

    CCFLAGS=[
        "-Os",
        "-ffunction-sections",
        "-fdata-sections",
        "-Wno-error=unused-function",
        "-Wno-error=unused-variable",
        "-Wno-error=unused-but-set-variable",
        "-Wno-error=deprecated-declarations",
        "-Wno-unused-parameter",
        "-Wno-sign-compare",
        "-Wno-enum-conversion",
        "-gdwarf-4",
        "-ggdb",
        "-nostartfiles",
        "-freorder-blocks",
        "-msave-restore",
        "-Wno-address",
        "-Wno-use-after-free",
        "-fstrict-volatile-bitfields",
        "-fno-lto",
        "-fno-jump-tables",
        "-fno-tree-switch-conversion",
        "-MMD"
    ],

    LINKFLAGS=[
        "-nostartfiles",
        "-march=rv32imc_zicsr_zifencei",
        "--specs=nosys.specs",
        "-Wl,--cref",
        "-Wl,--defsym=IDF_TARGET_ESP32C2=0",
        "-Wl,--no-warn-rwx-segments",
        "-fno-rtti",
        "-fno-lto",
        "-Wl,--gc-sections",
        "-Wl,--warn-common",
        "-Wl,--undefined=uxTopUsedPriority",
        "-Wl,--undefined=FreeRTOS_openocd_params",
        "--specs=nano.specs",
        "-Wl,--wrap=_Unwind_SetEnableExceptionFdeSorting",
        "-Wl,--wrap=__register_frame_info_bases",
        "-Wl,--wrap=__register_frame_info",
        "-Wl,--wrap=__register_frame",
        "-Wl,--wrap=__register_frame_info_table_bases",
        "-Wl,--wrap=__register_frame_info_table",
        "-Wl,--wrap=__register_frame_table",
        "-Wl,--wrap=__deregister_frame_info_bases",
        "-Wl,--wrap=__deregister_frame_info",
        "-Wl,--wrap=_Unwind_Find_FDE",
        "-Wl,--wrap=_Unwind_GetGR",
        "-Wl,--wrap=_Unwind_GetCFA",
        "-Wl,--wrap=_Unwind_GetIP",
        "-Wl,--wrap=_Unwind_GetIPInfo",
        "-Wl,--wrap=_Unwind_GetRegionStart",
        "-Wl,--wrap=_Unwind_GetDataRelBase",
        "-Wl,--wrap=_Unwind_GetTextRelBase",
        "-Wl,--wrap=_Unwind_SetIP",
        "-Wl,--wrap=_Unwind_SetGR",
        "-Wl,--wrap=_Unwind_GetLanguageSpecificData",
        "-Wl,--wrap=_Unwind_FindEnclosingFunction",
        "-Wl,--wrap=_Unwind_Resume",
        "-Wl,--wrap=_Unwind_RaiseException",
        "-Wl,--wrap=_Unwind_DeleteException",
        "-Wl,--wrap=_Unwind_ForcedUnwind",
        "-Wl,--wrap=_Unwind_Resume_or_Rethrow",
        "-Wl,--wrap=_Unwind_Backtrace",
        "-Wl,--wrap=__cxa_call_unexpected",
        "-Wl,--wrap=__gxx_personality_v0",
        "-T", "esp32c2.peripherals.ld",
        "-T", "esp32c2.rom.ld",
        "-T", "esp32c2.rom.api.ld",
        "-T", "esp32c2.rom.rvfp.ld",
        "-T", "esp32c2.rom.newlib.ld",
        "-T", "esp32c2.rom.version.ld",
        "-T", "esp32c2.rom.newlib-nano.ld",
        "-T", "esp32c2.rom.ble.ld",
        "-T", "esp32c2.rom.heap.ld",
        "-T", "memory.ld",
        "-T", "sections.ld",
        "-u", "_Z5setupv",
        "-u", "_Z4loopv",
        "-u", "esp_app_desc",
        "-u", "pthread_include_pthread_impl",
        "-u", "pthread_include_pthread_cond_var_impl",
        "-u", "pthread_include_pthread_local_storage_impl",
        "-u", "pthread_include_pthread_rwlock_impl",
        "-u", "pthread_include_pthread_semaphore_impl",
        "-u", "start_app",
        "-u", "__ubsan_include",
        "-u", "tlsf_set_rom_patches",
        "-u", "app_main",
        "-u", "newlib_include_heap_impl",
        "-u", "newlib_include_syscalls_impl",
        "-u", "newlib_include_pthread_impl",
        "-u", "newlib_include_assert_impl",
        "-u", "__cxa_guard_dummy",
        "-u", "__cxx_fatal_exception",
        "-u", "include_esp_phy_override",
        "-u", "vfs_include_syscalls_impl",
        '-Wl,-Map="%s"' % join("${BUILD_DIR}", "${PROGNAME}.map")
    ],

    CPPPATH=[
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "newlib", "platform_include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "freertos", "FreeRTOS-Kernel", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "freertos", "FreeRTOS-Kernel", "portable", "riscv", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "freertos", "esp_additions", "include", "freertos"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "freertos", "esp_additions", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "freertos", "esp_additions", "arch", "riscv", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_hw_support", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_hw_support", "include", "soc"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_hw_support", "include", "soc", "esp32c2"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "heap", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "log", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "soc", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "soc", "esp32c2"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "soc", "esp32c2", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "hal", "esp32c2", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "hal", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "hal", "platform_port", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_rom", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_rom", "include", "esp32c2"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_rom", "esp32c2"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_common", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_system", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_system", "port", "soc"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_system", "port", "include", "riscv"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_system", "port", "include", "private"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "riscv", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "lwip", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "lwip", "include", "apps"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "lwip", "include", "apps", "sntp"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "lwip", "lwip", "src", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "lwip", "port", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "lwip", "port", "freertos", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "lwip", "port", "esp32xx", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "lwip", "port", "esp32xx", "include", "arch"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_ringbuf", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "efuse", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "efuse", "esp32c2", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "deprecated"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "analog_comparator", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "dac", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "gpio", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "gptimer", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "i2c", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "i2s", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "ledc", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "mcpwm", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "parlio", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "pcnt", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "rmt", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "sdio_slave", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "sdmmc", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "sigma_delta", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "spi", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "temperature_sensor", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "touch_sensor", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "twai", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "uart", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "driver", "usb_serial_jtag", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_pm", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "mbedtls", "port", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "mbedtls", "mbedtls", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "mbedtls", "mbedtls", "library"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "mbedtls", "esp_crt_bundle", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "mbedtls", "mbedtls", "3rdparty", "everest", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "mbedtls", "mbedtls", "3rdparty", "p256-m"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "mbedtls", "mbedtls", "3rdparty", "p256-m", "p256-m"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_app_format", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bootloader_support", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bootloader_support", "bootloader_flash", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_partition", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "app_update", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_mm", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "spi_flash", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "pthread", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_timer", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "app_trace", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_event", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "nvs_flash", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_phy", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_phy", "esp32c2", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "vfs", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_netif", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "wpa_supplicant", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "wpa_supplicant", "port", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "wpa_supplicant", "esp_supplicant", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_coex", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_wifi", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_wifi", "wifi_apps", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "include", "esp32c2", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "common", "osi", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "common", "api", "include", "api"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "common", "btc", "profile", "esp", "blufi", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "common", "btc", "profile", "esp", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "common", "hci_log", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "porting", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "porting", "npl", "freertos", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "porting", "transport", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "porting", "ext", "tinycrypt", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "services", "ans", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "services", "bas", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "services", "dis", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "services", "gap", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "services", "gatt", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "services", "hr", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "services", "htp", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "services", "ias", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "services", "ipss", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "services", "lls", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "services", "prox", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "services", "cts", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "services", "tps", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "services", "hid", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "services", "sps", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "util", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "store", "ram", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "host", "store", "config", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "porting", "nimble", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "port", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "bt", "host", "nimble", "nimble", "nimble", "transport", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "unity", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "unity", "unity", "src"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "cmock", "CMock", "src"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "console"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "http_parser"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp-tls"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp-tls", "esp-tls-crypto"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_adc", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_adc", "interface"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_adc", "esp32c2", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_adc", "deprecated", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_gdbstub", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_hid", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "tcp_transport", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_http_client", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_http_server", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_https_ota", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_psram", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_lcd", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_lcd", "interface"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "protobuf-c", "protobuf-c"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "protocomm", "include", "common"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "protocomm", "include", "security"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "protocomm", "include", "transports"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "protocomm", "include", "crypto", "srp6a"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "protocomm", "proto-c"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "esp_local_ctrl", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espcoredump", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espcoredump", "include", "port", "riscv"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "wear_levelling", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "sdmmc", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "fatfs", "diskio"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "fatfs", "vfs"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "fatfs", "src"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "idf_test", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "idf_test", "include", "esp32c2"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "ieee802154", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "json", "cJSON"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__mdns", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "joltwallet__littlefs", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "dotprod", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "support", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "support", "mem", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "windows", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "windows", "hann", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "windows", "blackman", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "windows", "blackman_harris", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "windows", "blackman_nuttall", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "windows", "nuttall", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "windows", "flat_top", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "iir", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "fir", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "math", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "math", "add", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "math", "sub", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "math", "mul", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "math", "addc", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "math", "mulc", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "math", "sqrt", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "matrix", "mul", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "matrix", "add", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "matrix", "addc", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "matrix", "mulc", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "matrix", "sub", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "matrix", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "fft", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "dct", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "conv", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "common", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "matrix", "mul", "test", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "kalman", "ekf", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "espressif__esp-dsp", "modules", "kalman", "ekf_imu13states", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "include", "fb_gfx", "include"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", board_config.get("build.arduino.memory_type", (board_config.get("build.flash_mode", "dio") + "_qspi")), "include"),
        join(FRAMEWORK_DIR, "cores", board_config.get("build.core"))
    ],

    LIBPATH=[
        join(FRAMEWORK_SDK_DIR, "esp32c2", "lib"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", "ld"),
        join(FRAMEWORK_SDK_DIR, "esp32c2", board_config.get("build.arduino.memory_type", (board_config.get("build.flash_mode", "dio") + "_qspi")))
    ],

    LIBS=[
        "-lriscv", "-lesp_ringbuf", "-lefuse", "-ldriver", "-lesp_pm", "-lmbedtls", "-lesp_app_format", "-lbootloader_support", "-lesp_partition", "-lapp_update", "-lesp_mm", "-lspi_flash", "-lpthread", "-lesp_system", "-lesp_rom", "-lhal", "-llog", "-lheap", "-lsoc", "-lesp_hw_support", "-lfreertos", "-lnewlib", "-lcxx", "-lesp_common", "-lesp_timer", "-lapp_trace", "-lesp_event", "-lnvs_flash", "-lesp_phy", "-lvfs", "-llwip", "-lesp_netif", "-lwpa_supplicant", "-lesp_coex", "-lesp_wifi", "-lbt", "-lunity", "-lcmock", "-lconsole", "-lhttp_parser", "-lesp-tls", "-lesp_adc", "-lesp_gdbstub", "-lesp_hid", "-ltcp_transport", "-lesp_http_client", "-lesp_http_server", "-lesp_https_ota", "-lesp_lcd", "-lprotobuf-c", "-lprotocomm", "-lesp_local_ctrl", "-lespcoredump", "-lwear_levelling", "-lsdmmc", "-lfatfs", "-ljson", "-lespressif__mdns", "-ljoltwallet__littlefs", "-lespressif__esp-dsp", "-lfb_gfx", "-lapp_trace", "-lapp_trace", "-lcmock", "-lunity", "-lesp_lcd", "-lesp_local_ctrl", "-lprotocomm", "-lprotobuf-c", "-lespcoredump", "-ljson", "-lesp_hid", "-lbt", "-lble_app", "-lfatfs", "-lwear_levelling", "-lespressif__mdns", "-lconsole", "-ljoltwallet__littlefs", "-lsdmmc", "-lespressif__esp-dsp", "-lriscv", "-lesp_ringbuf", "-lefuse", "-ldriver", "-lesp_pm", "-lmbedtls", "-lesp_app_format", "-lbootloader_support", "-lesp_partition", "-lapp_update", "-lesp_mm", "-lspi_flash", "-lpthread", "-lesp_system", "-lesp_rom", "-lhal", "-llog", "-lheap", "-lsoc", "-lesp_hw_support", "-lfreertos", "-lnewlib", "-lcxx", "-lesp_common", "-lesp_timer", "-lesp_event", "-lnvs_flash", "-lesp_phy", "-lvfs", "-llwip", "-lesp_netif", "-lwpa_supplicant", "-lesp_coex", "-lesp_wifi", "-lhttp_parser", "-lesp-tls", "-lesp_adc", "-lesp_gdbstub", "-ltcp_transport", "-lesp_http_client", "-lesp_http_server", "-lesp_https_ota", "-lmbedtls_2", "-lmbedcrypto", "-lmbedx509", "-leverest", "-lp256m", "-lcoexist", "-lcore", "-lespnow", "-lnet80211", "-lpp", "-lsmartconfig", "-lriscv", "-lesp_ringbuf", "-lefuse", "-ldriver", "-lesp_pm", "-lmbedtls", "-lesp_app_format", "-lbootloader_support", "-lesp_partition", "-lapp_update", "-lesp_mm", "-lspi_flash", "-lpthread", "-lesp_system", "-lesp_rom", "-lhal", "-llog", "-lheap", "-lsoc", "-lesp_hw_support", "-lfreertos", "-lnewlib", "-lcxx", "-lesp_common", "-lesp_timer", "-lesp_event", "-lnvs_flash", "-lesp_phy", "-lvfs", "-llwip", "-lesp_netif", "-lwpa_supplicant", "-lesp_coex", "-lesp_wifi", "-lhttp_parser", "-lesp-tls", "-lesp_adc", "-lesp_gdbstub", "-ltcp_transport", "-lesp_http_client", "-lesp_http_server", "-lesp_https_ota", "-lmbedtls_2", "-lmbedcrypto", "-lmbedx509", "-leverest", "-lp256m", "-lcoexist", "-lcore", "-lespnow", "-lnet80211", "-lpp", "-lsmartconfig", "-lriscv", "-lesp_ringbuf", "-lefuse", "-ldriver", "-lesp_pm", "-lmbedtls", "-lesp_app_format", "-lbootloader_support", "-lesp_partition", "-lapp_update", "-lesp_mm", "-lspi_flash", "-lpthread", "-lesp_system", "-lesp_rom", "-lhal", "-llog", "-lheap", "-lsoc", "-lesp_hw_support", "-lfreertos", "-lnewlib", "-lcxx", "-lesp_common", "-lesp_timer", "-lesp_event", "-lnvs_flash", "-lesp_phy", "-lvfs", "-llwip", "-lesp_netif", "-lwpa_supplicant", "-lesp_coex", "-lesp_wifi", "-lhttp_parser", "-lesp-tls", "-lesp_adc", "-lesp_gdbstub", "-ltcp_transport", "-lesp_http_client", "-lesp_http_server", "-lesp_https_ota", "-lmbedtls_2", "-lmbedcrypto", "-lmbedx509", "-leverest", "-lp256m", "-lcoexist", "-lcore", "-lespnow", "-lnet80211", "-lpp", "-lsmartconfig", "-lriscv", "-lesp_ringbuf", "-lefuse", "-ldriver", "-lesp_pm", "-lmbedtls", "-lesp_app_format", "-lbootloader_support", "-lesp_partition", "-lapp_update", "-lesp_mm", "-lspi_flash", "-lpthread", "-lesp_system", "-lesp_rom", "-lhal", "-llog", "-lheap", "-lsoc", "-lesp_hw_support", "-lfreertos", "-lnewlib", "-lcxx", "-lesp_common", "-lesp_timer", "-lesp_event", "-lnvs_flash", "-lesp_phy", "-lvfs", "-llwip", "-lesp_netif", "-lwpa_supplicant", "-lesp_coex", "-lesp_wifi", "-lhttp_parser", "-lesp-tls", "-lesp_adc", "-lesp_gdbstub", "-ltcp_transport", "-lesp_http_client", "-lesp_http_server", "-lesp_https_ota", "-lmbedtls_2", "-lmbedcrypto", "-lmbedx509", "-leverest", "-lp256m", "-lcoexist", "-lcore", "-lespnow", "-lnet80211", "-lpp", "-lsmartconfig", "-lriscv", "-lesp_ringbuf", "-lefuse", "-ldriver", "-lesp_pm", "-lmbedtls", "-lesp_app_format", "-lbootloader_support", "-lesp_partition", "-lapp_update", "-lesp_mm", "-lspi_flash", "-lpthread", "-lesp_system", "-lesp_rom", "-lhal", "-llog", "-lheap", "-lsoc", "-lesp_hw_support", "-lfreertos", "-lnewlib", "-lcxx", "-lesp_common", "-lesp_timer", "-lesp_event", "-lnvs_flash", "-lesp_phy", "-lvfs", "-llwip", "-lesp_netif", "-lwpa_supplicant", "-lesp_coex", "-lesp_wifi", "-lhttp_parser", "-lesp-tls", "-lesp_adc", "-lesp_gdbstub", "-ltcp_transport", "-lesp_http_client", "-lesp_http_server", "-lesp_https_ota", "-lmbedtls_2", "-lmbedcrypto", "-lmbedx509", "-leverest", "-lp256m", "-lcoexist", "-lcore", "-lespnow", "-lnet80211", "-lpp", "-lsmartconfig", "-lc", "-lm", "-lnewlib", "-lstdc++", "-lpthread", "-lgcc", "-lcxx", "-lphy", "-lbtbb", "-lesp_phy", "-lphy", "-lbtbb", "-lesp_phy", "-lphy", "-lbtbb"
    ],

    CPPDEFINES=[
        "ESP32_ARDUINO_LIB_BUILDER",
        "ESP_PLATFORM",
        ("IDF_VER", '\\"5.1.5.250117\\"'),
        "LFS_MULTIVERSION",
        ("MBEDTLS_CONFIG_FILE", '\\"mbedtls/esp_config.h\\"'),
        "NDEBUG",
        ("SOC_MMU_PAGE_SIZE", 'CONFIG_MMU_PAGE_SIZE'),
        "UNITY_INCLUDE_CONFIG_H",
        "_GNU_SOURCE",
        "_POSIX_READER_WRITER_LOCKS",
        ("configENABLE_FREERTOS_DEBUG_OCDAWARE", 1),
        "ARDUINO_ARCH_ESP32",
        ("ESP32", "ESP32"),
        ("F_CPU", "$BOARD_F_CPU"),
        ("ARDUINO", 10812),
        ("ARDUINO_VARIANT", '\\"%s\\"' % board_config.get("build.variant").replace('"', "")),
        ("ARDUINO_BOARD", '\\"%s\\"' % board_config.get("name").replace('"', "")),
        "ARDUINO_PARTITION_%s" % basename(board_config.get(
            "build.partitions", "default.csv")).replace(".csv", "").replace("-", "_")
    ]
)
