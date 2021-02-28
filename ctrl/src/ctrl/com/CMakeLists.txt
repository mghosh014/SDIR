add_library(Com
            json_handler.cpp
            jsoncpp.cpp
        )

add_subdirectory(remoteAPI)
add_dependencies(Com RemoteAPI)

target_include_directories(Com
            PUBLIC
            "${CMAKE_SOURCE_DIR}/include/json"
        )

target_link_libraries(Com RemoteAPI)