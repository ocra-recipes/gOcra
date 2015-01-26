include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(
    ${PC_ORC_FRAMEWORK_INCLUDE_DIRS}
    ${PC_QUADPROG_INCLUDE_DIRS}
    ${PC_EIGEN_LGSM_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
    ${PC_ORC_FRAMEWORK_LIBRARY_DIRS}
    ${PC_QUADPROG_LIBRARY_DIRS}
)


##### Make library
add_library(
    orcisir_NewController
    ISIRController.cpp
    GHCJTController.cpp
    ISIRDebug.cpp
    OrthonormalFamily.cpp
    Solvers/ISIRSolver.cpp
    Solvers/OneLevelSolver.cpp
    Tasks/ISIRTask.cpp
    Tasks/GHCJTTask.cpp
    Constraints/ISIRConstraint.cpp
    Constraints/TorqueLimitConstraint.cpp
    Constraints/JointLimitConstraint.cpp
    Constraints/ContactAvoidanceConstraint.cpp
    Constraints/GSHCAccelerationConstraint.cpp
    Features/ISIRFeature.cpp
    Features/ISIRPartialState.cpp
)


target_link_libraries(
    orcisir_NewController
    ${PC_QUADPROG_LIBRARIES}
    ${PC_ORC_FRAMEWORK_LIBRARIES}
)

install(
    TARGETS orcisir_NewController
    ${install_type} DESTINATION lib
)

