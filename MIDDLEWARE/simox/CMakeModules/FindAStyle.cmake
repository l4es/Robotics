# This module defines the following variables:
# AStyle_FOUND      : 1 if astyle was found, 0 otherwise
# AStyle_EXECUTABLE : astyle executable location

include(FindPackageHandleStandardArgs)

find_program(AStyle_EXECUTABLE astyle)

find_package_handle_standard_args(AStyle DEFAULT_MSG AStyle_EXECUTABLE)
# Hack: since the macro makes the package name uppercase
set(AStyle_FOUND ${ASTYLE_FOUND})

mark_as_advanced(AStyle_EXECUTABLE)
