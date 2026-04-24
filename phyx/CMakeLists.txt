project (phyx)

add_library(phyx STATIC gravity.cpp)

target_include_directories(
    phyx
    PUBLIC ./
)

target_link_libraries(phyx PUBLIC common)
