include(TestCXXAcceptsFlag)

check_cxx_accepts_flag("-std=c++11" CXX_FLAG_CXX11)
check_cxx_accepts_flag("-std=c++0x" CXX_FLAG_CXX0x)
IF (CXX_FLAG_CXX11)
  SET (CXX11_FLAG "-std=c++11" CACHE STRING "Flag for enabling c++11")
ELSEIF(CXX_FLAG_CXX0x)
  SET (CXX11_FLAG "-std=c++0x" CACHE STRING "Flag for enabling c++11")
ELSEIF (MSVC)
  # on by default
ELSE ()
  message (WARNING "Your compiler does not seem to support C++11!")
ENDIF()


