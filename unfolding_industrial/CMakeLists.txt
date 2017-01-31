if (MODULE_common)
    option(MODULE_unfolding_industrial "Compile module UNFOLDING-INDUSTRIAL" TRUE)

    if (MODULE_unfolding_industrial)
        add_subdirectory(perception)
    endif()
endif()
