#####################################################
#
#  FILE: ax_sick_pls_src_dir.m4
#  AUTH: Jason C. Derenick
#  CONT: jasonder(at)seas(dot)upenn(dot)edu
#  DATE: 10 August 2009
#
#  DESC: A macro for setting the pls source dir
#
#  NOTE: Assumes there is always an input arg
#
#####################################################
AC_DEFUN([AX_SICK_PLS_SRC_DIR], [
SICK_PLS_SRC_DIR=$1
AC_SUBST(SICK_PLS_SRC_DIR)
])dnl AX_SICK_PLS_SRC_DIR
