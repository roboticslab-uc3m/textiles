if (MODULE_common)
    option(MODULE_ironing "Compile module IRONING" TRUE)

    if (MODULE_ironing)
        add_subdirectory(perception)
    endif()
endif()
