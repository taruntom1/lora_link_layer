cmake_minimum_required(VERSION 3.16)

# ---------------------------------------------------------------------------
# Include the lora_radio component and its managed_components (RadioLib etc.)
# from the parent project so we do not re-download the same packages.
# ---------------------------------------------------------------------------
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../components"
    "${CMAKE_CURRENT_LIST_DIR}/../managed_components"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(lora_radio_test)
