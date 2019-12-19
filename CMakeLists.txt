add_library(control_aws1 SHARED f_control_aws1.cpp ${CHANS})

target_include_directories(control_aws1 PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(control_aws1 PUBLIC ${PROJECT_SOURCE_DIR}/channels)
install(TARGETS control_aws1 DESTINATION lib)
