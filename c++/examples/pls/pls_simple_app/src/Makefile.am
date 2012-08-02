noinst_PROGRAMS=pls_simple_app
pls_simple_app_SOURCES=main.cc
pls_simple_app_LDADD=-lsickpls $(PTHREAD_CFLAGS) $(PTHREAD_LIBS)
pls_simple_app_LDFLAGS=-L$(top_srcdir)/c++/drivers/pls/$(SICK_LMS_2XX_SRC_DIR)
AM_CPPFLAGS=-I$(top_srcdir)/c++/drivers/pls -I$(top_srcdir)/c++/drivers/base/src $(PTHREAD_CFLAGS) $(PTHREAD_LIBS) $(all_includes)
