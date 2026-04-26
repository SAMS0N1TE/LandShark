set(SOURCES
    core/app_main.c
    core/settings.c
    core/app_registry.c
    core/event_bus.c
    core/perf.c

    radio/usb_host.c
    radio/rtlsdr_dev.c
    radio/esp_libusb.c
    radio/stream.c
    radio/librtlsdr.c
    radio/tuner_e4k.c
    radio/tuner_fc0012.c
    radio/tuner_fc0013.c
    radio/tuner_fc2580.c
    radio/tuner_r82xx.c

    audio/audio_out.c
    audio/audio_events.c
    audio/tone.c
    audio/sam_tts.c
    audio/plane_audio.c
    audio/sam/sam.c
    audio/sam/render.c
    audio/sam/reciter.c
    audio/sam/processframes.c
    audio/sam/createtransitions.c
    audio/sam/debug.c
)

if(CONFIG_APP_ADSB)
    list(APPEND SOURCES
        apps/adsb/app_adsb.c
        apps/adsb/adsb_decode.c
        apps/adsb/adsb_state.c
        apps/adsb/mode-s.c
        apps/adsb/demod1090.c
    )
    if(CONFIG_ENABLE_TUI)
        list(APPEND SOURCES apps/adsb/adsb_render.c)
    endif()
endif()

if(CONFIG_APP_VHF)
    list(APPEND SOURCES
        apps/vhf/app_vhf.c
        apps/vhf/vhf_demod.c
    )
endif()

if(CONFIG_ENABLE_TUI)
    list(APPEND SOURCES
        ui/tui.c
        ui/tui_task.c
        ui/tui_log_hook.c
    )
endif()

if(CONFIG_OUTPUT_HUMAN_LOG)
    list(APPEND SOURCES output/log_out.c)
endif()

if(CONFIG_OUTPUT_EVENT_STREAM)
    list(APPEND SOURCES output/event_stream.c)
endif()

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS
        core
        radio
        audio
        audio/sam
        apps/adsb
        apps/vhf
        ui
        output
    REQUIRES
        usb
        driver
        esp_timer
        nvs_flash
        es8311
        esp_psram
)