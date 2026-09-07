# This program source code file is part of KiCad, a free EDA application.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or (at your
# option) any later version.

add_executable( qa_autorouter_parity
    autorouter_parity.cpp

    # Global mock objects pulled in by pcbnew_kiface_objects.
    ${CMAKE_SOURCE_DIR}/qa/mocks/kicad/common_mocks.cpp
)

target_compile_definitions( qa_autorouter_parity PRIVATE PCBNEW )
add_dependencies( qa_autorouter_parity pcbnew )

include_directories( BEFORE ${INC_BEFORE} )
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/3d-viewer
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}/pcbnew
    ${CMAKE_SOURCE_DIR}/pcbnew/router
    ${CMAKE_SOURCE_DIR}/pcbnew/tools
    ${CMAKE_SOURCE_DIR}/pcbnew/dialogs
    ${CMAKE_SOURCE_DIR}/polygon
    ${CMAKE_SOURCE_DIR}/common/geometry
    ${CMAKE_SOURCE_DIR}/qa/qa_utils
    ${CMAKE_SOURCE_DIR}/qa/mocks/include
    ${INC_AFTER}
)

target_link_libraries( qa_autorouter_parity
    qa_pcbnew_utils
    pcbnew_kiface_objects
    3d-viewer
    connectivity
    pcbcommon
    pnsrouter
    gal
    common
    dxflib_qcad
    tinyspline_lib
    nanosvg
    planegcs
    idf3
    markdown_lib
    nlohmann_json
    ${PCBNEW_IO_LIBRARIES}
    ${wxWidgets_LIBRARIES}
    ${GDI_PLUS_LIBRARIES}
    Boost::headers
    ${PCBNEW_EXTRA_LIBS}    # -lrt must follow Boost
)

if( NOT KICAD_BUILD_QA_TESTS )
    set_target_properties( qa_autorouter_parity PROPERTIES EXCLUDE_FROM_ALL TRUE )
endif()
