find_path(SVM_INCLUDE_DIR svm/svm.h)
set(SVM_LIBRARY ${SVM_INCLUDE_DIR}/../lib/libsvm.so)

set(SVM_INCLUDE_DIRS ${SVM_INCLUDE_DIR})
set(SVM_LIBRARIES ${SVM_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SVM DEFAULT_MSG SVM_LIBRARY SVM_INCLUDE_DIR)

mark_as_advanced(SVM_LIBRARY SVM_INCLUDE_DIR)
