# Install headers
set (slam_dataio_excluded_headers #"")
    # "${CMAKE_CURRENT_SOURCE_DIR}/serialization.h"
)

file(GLOB slam_dataio_headers "*.h")

find_package(nlohmann_json 3.2.0 REQUIRED)
target_link_libraries(FILES PRIVATE nlohmann_json::nlohmann_json)

install(FILES ${slam_dataio_headers} DESTINATION include/gtsam/slam/dataio)