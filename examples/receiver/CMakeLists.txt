cmake_minimum_required(VERSION 3.16)

# ---------------------------------------------------------------------------
# Include lora_radio and its managed_components (RadioLib) from the parent
# repository so that no re-download is needed.
# ---------------------------------------------------------------------------
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components"
    "${CMAKE_CURRENT_LIST_DIR}/../../managed_components"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(lora_receiver)
