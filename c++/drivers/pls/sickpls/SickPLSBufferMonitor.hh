/*!
 * \file SickPLSBufferMonitor.hh
 * \brief Defines a class for monitoring the receive
 *        buffer when interfacing w/ a Sick PLS
 *        laser range finder.
 *
 * Code by Jason C. Derenick and Thomas H. Miller.
 * Contact derenick(at)lehigh(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2008, Jason C. Derenick and Thomas H. Miller
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#ifndef SICK_PLS_BUFFER_MONITOR_HH
#define SICK_PLS_BUFFER_MONITOR_HH

#define DEFAULT_SICK_PLS_SICK_BYTE_TIMEOUT      (35000)  ///< Max allowable time between consecutive bytes

/* Definition dependencies */
#include "SickPLSMessage.hh"
#include "SickBufferMonitor.hh"
#include "SickException.hh"

/* Associate the namespace */
namespace SickToolbox {

  /*!
   * \brief A class for monitoring the receive buffer when interfacing with a Sick PLS LIDAR
   */
  class SickPLSBufferMonitor : public SickBufferMonitor< SickPLSBufferMonitor, SickPLSMessage > {

  public:

    /** A standard constructor */
    SickPLSBufferMonitor( );

    /** A method for extracting a single message from the stream */
    void GetNextMessageFromDataStream( SickPLSMessage &sick_message ) throw( SickIOException );

    /** A standard destructor */
    ~SickPLSBufferMonitor( );

  };
    
} /* namespace SickToolbox */

#endif /* SICK_PLS_BUFFER_MONITOR_HH */
