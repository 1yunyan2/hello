idf_component_register(SRCS
"main.c"
"bsp/bsp_board.c"
"bsp/bsp_wifi.c"
"bsp/bsp_codec.c"
"wake_words/custom_wake_word.c"
"audio/audio_processor.c"
"audio/audio_encoder.c"
"audio/audio_decoder.c"
"application.c"
"protocol/protocol.c"
INCLUDE_DIRS "."
)
