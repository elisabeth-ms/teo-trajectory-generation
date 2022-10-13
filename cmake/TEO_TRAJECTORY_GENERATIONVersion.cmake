include(GitInfo) # YCM

# Define current version.
set(TEO_TRAJECTORY_GENERATION_VERSION_SHORT ${TEO_TRAJECTORY_GENERATION_VERSION})

# Retrieve current revision from Git working tree.
git_wt_info(SOURCE_DIR "${CMAKE_SOURCE_DIR}"
            PREFIX TEO_TRAJECTORY_GENERATION)

if(DEFINED TEO_TRAJECTORY_GENERATION_GIT_WT_HASH)
    if(TEO_TRAJECTORY_GENERATION_GIT_WT_TAG_REVISION GREATER 0)
        set(TEO_TRAJECTORY_GENERATION_VERSION_REVISION ${TEO_TRAJECTORY_GENERATION_GIT_WT_TAG_REVISION})
        string(REPLACE "-" "" _date ${TEO_TRAJECTORY_GENERATION_GIT_WT_AUTHOR_DATE})
        set(TEO_TRAJECTORY_GENERATION_VERSION_SOURCE
            "${_date}.${TEO_TRAJECTORY_GENERATION_GIT_WT_DATE_REVISION}+git${TEO_TRAJECTORY_GENERATION_GIT_WT_HASH_SHORT}")
    endif()

    if(TEO_TRAJECTORY_GENERATION_GIT_WT_DIRTY)
        set(TEO_TRAJECTORY_GENERATION_VERSION_DIRTY "dirty")
    endif()
endif()

if(DEFINED TEO_TRAJECTORY_GENERATION_VERSION_SOURCE)
    if(NOT "${TEO_TRAJECTORY_GENERATION_GIT_WT_TAG}" STREQUAL "v${TEO_TRAJECTORY_GENERATION_VERSION_SHORT}")
        set(TEO_TRAJECTORY_GENERATION_VERSION
            "${TEO_TRAJECTORY_GENERATION_VERSION_SHORT}+${TEO_TRAJECTORY_GENERATION_VERSION_SOURCE}")
    else()
        set(TEO_TRAJECTORY_GENERATION_VERSION
           "${TEO_TRAJECTORY_GENERATION_VERSION_SHORT}+${TEO_TRAJECTORY_GENERATION_VERSION_REVISION}-${TEO_TRAJECTORY_GENERATION_VERSION_SOURCE}")
    endif()
elseif(NOT "${TEO_TRAJECTORY_GENERATION_GIT_WT_TAG}" STREQUAL "v${TEO_TRAJECTORY_GENERATION_VERSION_SHORT}")
    set(TEO_TRAJECTORY_GENERATION_VERSION "${TEO_TRAJECTORY_GENERATION_VERSION_SHORT}~dev")
else()
    set(TEO_TRAJECTORY_GENERATION_VERSION "${TEO_TRAJECTORY_GENERATION_VERSION_SHORT}")
endif()

if(DEFINED TEO_TRAJECTORY_GENERATION_VERSION_DIRTY)
    set(TEO_TRAJECTORY_GENERATION_VERSION "${TEO_TRAJECTORY_GENERATION_VERSION}+${TEO_TRAJECTORY_GENERATION_VERSION_DIRTY}")
endif()

# Print version.
message(STATUS "TEO_TRAJECTORY_GENERATION version: ${TEO_TRAJECTORY_GENERATION_VERSION_SHORT} (${TEO_TRAJECTORY_GENERATION_VERSION})")