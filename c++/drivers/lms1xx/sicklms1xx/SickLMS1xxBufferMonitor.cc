/*!
 * \file SickLMS1xxBufferMonitor.cc
 * \brief Implements a class for monitoring the receive
 *        buffer when interfacing w/ a Sick LMS 1xx LIDAR.
 *
 * Code by Jason C. Derenick and Christopher R. Mansley.
 * Contact jasonder(at)seas(dot)upenn(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2009, Jason C. Derenick and Christopher R. Mansley
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

/* Auto-generated header */
#include "SickConfig.hh"

/* Implementation dependencies */
#include <iostream>
#include <sys/ioctl.h>

#include "SickLMS1xxBufferMonitor.hh"
#include "SickLMS1xxMessage.hh"
#include "SickLMS1xxUtility.hh"
#include "SickException.hh"

/* Associate the namespace */
namespace SickToolbox {

  /**
   * \brief A standard constructor
   */
  SickLMS1xxBufferMonitor::SickLMS1xxBufferMonitor( ) : SickBufferMonitor< SickLMS1xxBufferMonitor, SickLMS1xxMessage >(this) { }

  /**
   * \brief Acquires the next message from the SickLMS1xx byte stream
   * \param &sick_message The returned message object
   */
  void SickLMS1xxBufferMonitor::GetNextMessageFromDataStream( SickLMS1xxMessage &sick_message ) throw( SickIOException ) {

//     /* Flush the input buffer */
//     uint8_t byte_buffer;  

//     /* A buffer to hold the current byte out of the stream */
//     const uint8_t sick_response_header[4] = {0x02,'U','S','P'};
    
//     uint8_t checksum = 0;  
//     uint8_t message_buffer[SickLMS1xxMessage::MESSAGE_MAX_LENGTH] = {0};
//     uint32_t payload_length = 0;

//     try {

//       /* Search for the header in the byte stream */
//       for (unsigned int i = 0; i < sizeof(sick_response_header);) {
	
// 	/* Acquire the next byte from the stream */
// 	_readBytes(&byte_buffer,1,DEFAULT_SICK_BYTE_TIMEOUT);
	
// 	/* Check if the current byte matches the expected header byte */
// 	if (byte_buffer == sick_response_header[i]) {
// 	  i++;      
// 	}
// 	else {
// 	  i = 0;
// 	}
	
//       }  
      
//       /* Populate message buffer w/ response header */
//       memcpy(message_buffer,sick_response_header,4);

//       /* Acquire the payload length! */
//       _readBytes(&message_buffer[4],4,DEFAULT_SICK_BYTE_TIMEOUT);
      
//       /* Extract the payload size and adjust the byte order */
//       memcpy(&payload_length,&message_buffer[4],4);
//       payload_length = sick_ld_to_host_byte_order(payload_length);
      
//       /* Read the packet payload */
//       _readBytes(&message_buffer[8],payload_length,DEFAULT_SICK_BYTE_TIMEOUT);
      
//       /* Read the checksum */
//       _readBytes(&checksum,1,DEFAULT_SICK_BYTE_TIMEOUT);
      
//       /* Build the return message object based upon the received payload
//        * and compute the associated checksum.
//        *
//        * NOTE: In constructing this message we ignore the header bytes
//        *       buffered since the BuildMessage routine will insert the
//        *       correct header automatically and compute the payload's
//        *       checksum for us. We could probably get away with using
//        *       just ParseMessage here and not computing the checksum as
//        *       we are using TCP.  However, its safer this way.
//        */
//       sick_message.BuildMessage(&message_buffer[SickLMS1xxMessage::MESSAGE_HEADER_LENGTH],payload_length);
      
//       /* Verify the checksum is correct (this is probably unnecessary since we are using TCP/IP) */
//       if (sick_message.GetChecksum() != checksum) {
// 	throw SickBadChecksumException("SickLMS1xx::GetNextMessageFromDataStream: BAD CHECKSUM!!!");
//       }
      
//       /* Success */

//     }

//     catch(SickTimeoutException &sick_timeout) { /* This is ok! */ }
    
//     /* Catch a bad checksum! */
//     catch(SickBadChecksumException &sick_checksum_exception) {
//       sick_message.Clear(); // Clear the message container
//     }
    
//     /* Catch any serious IO buffer exceptions */
//     catch(SickIOException &sick_io_exception) {
//       throw;
//     }

//     /* A sanity check */
//     catch (...) {
//       throw;
//     }
    
  }
  
  /**
   * \brief A standard destructor
   */
  SickLMS1xxBufferMonitor::~SickLMS1xxBufferMonitor( ) { }
    
} /* namespace SickToolbox */