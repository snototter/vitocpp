
# Dump all currently known variables.
# Useful, if a libraries documentation is not really helpful in telling you
# what variables they define...
# Taken from https://stackoverflow.com/a/9328525/400948
#
# Usage:
# include(<path/to/CmakeUtils.cmake>)
# dump_cmake_variables(), or pass a regexp to match variable names
function(dump_cmake_variables)
    get_cmake_property(_variableNames VARIABLES)
    list (SORT _variableNames)
    foreach (_variableName ${_variableNames})
        if (ARGV0)
            unset(MATCHED)
            string(REGEX MATCH ${ARGV0} MATCHED ${_variableName})
            if (NOT MATCHED)
                continue()
            endif()
        endif()
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
endfunction()
