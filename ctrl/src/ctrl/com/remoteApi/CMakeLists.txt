add_library(RemoteAPI
            extApi.c
            extApiPlatform.c
        )

target_include_directories(RemoteAPI
            PUBLIC
            "${CMAKE_SOURCE_DIR}/include/vrep"
        )