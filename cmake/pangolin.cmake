#include_directories("third_party/eigen3")

find_package(Pangolin REQUIRED)

include_directories(
        ${Pangolin_INCLUDE_DIRS}
)
list(APPEND ALL_TARGET_LIBRARIES ${Pangolin_LIBRARIES})
