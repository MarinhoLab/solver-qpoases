# Compute the rolling version (YY.MM.NN) at configure time.
# Falls back to a hardcoded version when tools/version.sh is unavailable
# (e.g. when building from a release tarball without the tools/ directory).
set(QPOALES_FALLBACK_VERSION "26.09.00")

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../tools/version.sh")
    execute_process(
        COMMAND bash "${CMAKE_CURRENT_LIST_DIR}/../tools/version.sh"
        OUTPUT_VARIABLE _qpoales_version
        RESULT_VARIABLE _qpoales_version_result
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(NOT _qpoales_version_result EQUAL 0 OR NOT _qpoales_version MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+$")
        set(_qpoales_version "${QPOALES_FALLBACK_VERSION}")
    endif()
else()
    set(_qpoales_version "${QPOALES_FALLBACK_VERSION}")
endif()

set(QPOALES_VERSION "${_qpoales_version}")
message(STATUS "marinholab_solver_qpoales version: ${QPOALES_VERSION}")
