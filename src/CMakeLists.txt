include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(
    ${PC_OCRA_FRAMEWORK_INCLUDE_DIRS}
    ${PC_QUADPROG_INCLUDE_DIRS}
    ${PC_EIGEN_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
    ${PC_ORCA_FRAMEWORK_LIBRARY_DIRS}
    ${PC_QUADPROG_LIBRARY_DIRS}
)

file(
	GLOB_RECURSE
	source_files
	
	*.cpp
)

add_library(
	wLQP-Control${suffix_type}
	${LIB_TYPE}
	${source_files}
)


target_link_libraries(
    wLQP-Control
    ${PC_QUADPROG_LIBRARIES}
    ${PC_OCRA_FRAMEWORK_LIBRARIES}
)

install(
    TARGETS wLQP-Control
    ${install_type} DESTINATION lib
)

