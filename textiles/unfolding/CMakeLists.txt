if (MODULE_common)
    option(MODULE_unfolding "Compile module UNFOLDING" TRUE)

    if (MODULE_unfolding)
        add_subdirectory(perception)
    endif()
endif()
