lib_include_hh= SickPLS.hh \
                SickPLSMessage.hh \
                SickPLSBufferMonitor.hh \
	        $(top_srcdir)/c++/drivers/base/src/SickLIDAR.hh \
	        $(top_srcdir)/c++/drivers/base/src/SickMessage.hh \
	        $(top_srcdir)/c++/drivers/base/src/SickBufferMonitor.hh \
	        $(top_srcdir)/c++/drivers/base/src/SickException.hh

hh_sources= $(lib_include_hh) \
	    $(top_srcdir)/c++/drivers/base/src/SickConfig.hh \
            SickPLSUtility.hh 

cc_sources= SickPLS.cc \
            SickPLSMessage.cc \
            SickPLSBufferMonitor.cc

library_includedir=$(includedir)/sicklms2xx/
library_include_HEADERS=$(lib_include_hh)

lib_LTLIBRARIES=libsicklms2xx.la
libsicklms2xx_la_SOURCES=$(hh_sources) $(cc_sources)
AM_CPPFLAGS=-I$(top_srcdir)/c++/drivers/base/src