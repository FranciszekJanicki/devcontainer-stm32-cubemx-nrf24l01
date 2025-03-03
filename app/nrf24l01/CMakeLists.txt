add_library(nrf24l01 STATIC)

target_sources(nrf24l01 PRIVATE 
    "nrf24l01.cpp"
)

target_include_directories(nrf24l01 PUBLIC 
    "."
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(nrf24l01 PRIVATE
    utility
    stm32cubemx
)

target_compile_options(nrf24l01 PUBLIC
    -std=c++23
    -Wall
    -Wextra
    -Wconversion
    -Wshadow
    -Wpedantic
    -Wnarrowing
    -Waddress
    -pedantic
    -Wdeprecated
    -Wsign-conversion
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wdouble-promotion
    -Wimplicit-fallthrough
    -Wcast-align
    -fconcepts
)