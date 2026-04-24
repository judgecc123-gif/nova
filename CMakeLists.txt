cmake_minimum_required(VERSION 3.28)

project (nova)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include sub-projects.
add_subdirectory(common)
add_subdirectory(phyx)
add_subdirectory(render)

set(project "my_project")

add_executable(${project} ${project}.cpp)
target_link_libraries(${project} 
    PRIVATE 
    phyx    
    render 
)
