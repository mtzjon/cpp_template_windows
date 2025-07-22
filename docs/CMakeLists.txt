# Documentation with Doxygen
if(DOXYGEN_FOUND)
    # Set Doxygen input and output directories
    set(DOXYGEN_INPUT_DIR ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/apps)
    set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
    set(DOXYGEN_INDEX_FILE ${DOXYGEN_OUTPUT_DIR}/html/index.html)

    # Configure Doxygen configuration file
    set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # Replace variables in Doxyfile.in
    configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

    # Create a custom command to run Doxygen
    add_custom_command(
        OUTPUT ${DOXYGEN_INDEX_FILE}
        DEPENDS 
            ${CMAKE_SOURCE_DIR}/include/cpp_project_template/logger.hpp
            ${CMAKE_SOURCE_DIR}/src/logger.cpp
            ${CMAKE_SOURCE_DIR}/apps/main.cpp
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        MAIN_DEPENDENCY ${DOXYFILE_OUT} ${DOXYFILE_IN}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )

    # Create a custom target for documentation
    add_custom_target(docs ALL DEPENDS ${DOXYGEN_INDEX_FILE})

    # Install documentation
    install(DIRECTORY ${DOXYGEN_OUTPUT_DIR}/html 
            DESTINATION ${CMAKE_INSTALL_DOCDIR}
            OPTIONAL)

    message(STATUS "Doxygen documentation will be generated")
else()
    message(WARNING "Doxygen not found - documentation will not be generated")
endif()