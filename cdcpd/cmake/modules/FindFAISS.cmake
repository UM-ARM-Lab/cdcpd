# Modified from https://github.com/LLNL/hdtopology/blob/master/CMake/FindFAISS.cmake

# - Try to find FAISS
# Once done this will define
#
#  FAISS_FOUND        - system has FAISS
#  GPU_FAISS_FOUND    - system has GPU-FAISS
#  FAISS_INCLUDE_DIR  - the FAISS include directory
#  FAISS_LIBRARY      - Link these to use FAISS
#  GPU_FAISS_LIBRARY      - Link these to use GPU-FAISS
#

IF (FAISS_INCLUDE_DIRS)
  # Already in cache, be silent
  SET(FAISS_FIND_QUIETLY TRUE)
ENDIF (FAISS_INCLUDE_DIRS)

FIND_PATH( FAISS_INCLUDE_DIRS faiss/IndexFlat.h
           PATHS "external/" "/usr/include" "../" "~/.local/include")

if( WIN32 )

 message(status "FAISS library finder not tested for windows yet!")

else (WIN32)

FIND_LIBRARY( FAISS_LIBRARY
               NAMES faiss
               PATHS external/faiss ../faiss /lib /usr/lib /usr/lib64 /usr/local/lib ~/.local/lib)

FIND_LIBRARY( GPU_FAISS_LIBRARY
               NAMES gpufaiss
               PATHS external/faiss/gpu ../faiss /lib /usr/lib /usr/lib64 /usr/local/lib ~/.local/lib)

endif( WIN32)


IF (FAISS_INCLUDE_DIRS AND FAISS_LIBRARY)
  SET(FAISS_FOUND TRUE)
ELSE (FAISS_INCLUDE_DIRS AND FAISS_LIBRARY)
  SET( FAISS_FOUND FALSE )
ENDIF (FAISS_INCLUDE_DIRS AND FAISS_LIBRARY)

IF (FAISS_INCLUDE_DIRS AND GPU_FAISS_LIBRARY)
  SET(GPU_FAISS_FOUND TRUE)
ELSE (FAISS_INCLUDE_DIRS AND GPU_FAISS_LIBRARY)
  SET( GPU_FAISS_FOUND FALSE )
ENDIF (FAISS_INCLUDE_DIRS AND GPU_FAISS_LIBRARY)

