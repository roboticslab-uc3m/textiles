option(MODULE_common "Compile module COMMON" TRUE)

if (MODULE_common)
    add_subdirectory(pcl)
    add_subdirectory(perception)
endif()
