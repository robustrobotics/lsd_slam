# Build main library.
set(DATASET_UTILS_SRCS
  ${PROJECT_NAME}/utils.cc
  ${PROJECT_NAME}/utils.h

  ${PROJECT_NAME}/asl/dataset.h
  ${PROJECT_NAME}/asl/types.h

  ${PROJECT_NAME}/assert.h)

add_library(${PROJECT_NAME} SHARED ${DATASET_UTILS_SRCS})

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY ${PROJECT_NAME} DESTINATION include
  FILES_MATCHING PATTERN "*.h")

# Build ply_to_depthmap_asl.
add_executable(ply_to_depthmap_asl ./ply_to_depthmap_asl.cc)
target_link_libraries(ply_to_depthmap_asl
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES})

# For some reason libboost_program_options.so isn't added to Boost_LIBRARIES, so
# link with it manually here.
link_directories(${Boost_LIBRARY_DIRS})
target_link_libraries(ply_to_depthmap_asl
  -lboost_program_options)

install(TARGETS ply_to_depthmap_asl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
