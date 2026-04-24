project("common")

add_library(common INTERFACE)

target_include_directories(
    common
    INTERFACE ./
)
