idf_component_register(SRCS "main.cc"
                            "logger.cc"
                            "util.cc"
                            "task.cc"
                            "i2c_util.cc"
                            "si5351a.cc"
                            "st7032.cc"
                            "morse.cc"
                            "tone_generator.cc"
                            "cw_keyer.cc"
                            "voice_dsp.cc"
                            "modulator.cc"
                            "adc_sampler.cc"
                            "gpio_input_watch_task.cc"
                            "transmitter_settings.cc"
                            "tx_controller.cc"
                            "ardf_scheduler.cc"
                            "ui_controller_task.cc"
                            "transmitter_system.cc"
                    INCLUDE_DIRS ""
                    LDFRAGMENTS "linker.lf"
                    PRIV_REQUIRES driver esp_adc esp_driver_gpio esp_driver_ledc esp_driver_gptimer
                                  esp_driver_i2c nvs_flash esp_timer esp_hal_ledc)

# Git version -> version.h, generated into the build tree (not the source
# tree) so a stale copy can never be picked up. Skipped during the
# requirements expansion pass, where no build directory exists yet.
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    execute_process(COMMAND git describe --dirty --always --tags
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                    OUTPUT_VARIABLE GIT_VERSION
                    OUTPUT_STRIP_TRAILING_WHITESPACE)
    configure_file(version.h.in ${CMAKE_CURRENT_BINARY_DIR}/version.h)
    target_include_directories(${COMPONENT_LIB} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
endif()
