add_subdirectory(libs)

# Add programs
include_directories(${TEXTILES_INCLUDE_DIRS})

add_executable(foldingClothes foldingClothes.cpp)
target_link_libraries (foldingClothes ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

add_executable(foldingClothesMesh foldingClothesMesh.cpp)
target_link_libraries (foldingClothesMesh ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

add_executable(draftCode draftCode.cpp)
target_link_libraries (draftCode ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

add_executable(create_test_cube testcube.cpp)
target_link_libraries (create_test_cube ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

add_executable(clusteringCleanup clusteringCleanup.cpp)
target_link_libraries (clusteringCleanup ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

add_executable(render2png render2png.cpp)
target_link_libraries (render2png ${PCL_LIBRARIES} ${TEXTILES_LIBRARIES})

