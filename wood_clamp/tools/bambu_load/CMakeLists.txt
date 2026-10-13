# bbs_load harness, added to the BambuStudio build tree via -DBBS_LOAD_HARNESS_DIR=<this dir>
# Ubuntu's cereal CMake config only exports cereal::cereal, while libslic3r links a plain 'cereal' target
# (which then degrades to -lcereal). Provide a global 'cereal' interface target so the link resolves.
if (NOT TARGET cereal)
    add_library(cereal INTERFACE)
    if (TARGET cereal::cereal)
        target_link_libraries(cereal INTERFACE cereal::cereal)
    endif ()
endif ()
# libslic3r (LogSink.cpp, utils.cpp, NSVGUtils.cpp) references a few symbols that the real app gets from
# libslic3r_gui: Slic3r::Http (src/slic3r/Utils/Http.cpp), BBL_Encrypt (src/slic3r/Utils/BBLUtil.cpp),
# nanosvg implementation (src/slic3r/GUI/BitmapCache.cpp) and OpenSSL MD5. Compile the unmodified
# Bambu sources here instead of building the whole GUI library.
add_executable(bbs_load
    bbs_load.cpp
    nanosvg_impl.cpp
    ${LIBDIR}/slic3r/Utils/Http.cpp
    ${LIBDIR}/slic3r/Utils/BBLUtil.cpp
)
target_include_directories(bbs_load PRIVATE ${LIBDIR})
target_link_libraries(bbs_load libslic3r cereal libcurl OpenSSL::SSL OpenSSL::Crypto ${CMAKE_DL_LIBS} Threads::Threads)
target_compile_definitions(bbs_load PRIVATE BBS_RESOURCES_DIR="${SLIC3R_RESOURCES_DIR}")
if (BBS_LOAD_OUTPUT_DIR)
    set_target_properties(bbs_load PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${BBS_LOAD_OUTPUT_DIR})
endif ()
