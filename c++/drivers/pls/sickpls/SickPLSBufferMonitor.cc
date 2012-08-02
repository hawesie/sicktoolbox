/*!
 * \file SickPLSBufferMonitor.cc
 * \brief Implements a class for monitoring the receive
 *        buffer when interfacing w/ a Sick PLS LIDAR.
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

/* Auto-generated header */
#include "SickConfig.hh"

/* Implementation dependencies */
#include <iostream>
#include <termios.h>

#include "SickPLS.hh"
#include "SickPLSBufferMonitor.hh"
#include "SickPLSMessage.hh"
#include "SickPLSUtility.hh"
#include "SickException.hh"

/* Associate the namespace */
namespace SickToolbox {

  /**
   * \brief A standard constructor
   */
  SickPLSBufferMonitor::SickPLSBufferMonitor( ) : SickBufferMonitor<SickPLSBufferMonitor,SickPLSMessage>(this) { }

  /**
   * \brief Acquires the next message from the SickPLS byte stream
   * \param &sick_message The returned message object
   */
  void SickPLSBufferMonitor::GetNextMessageFromDataStream( SickPLSMessage &sick_message ) throw( SickIOException ) {
    
    uint8_t search_buffer[2] = {0};
    uint8_t payload_length_buffer[2] = {0};
    uint8_t payload_buffer[SickPLSMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
    uint8_t checksum_buffer[2] = {0};  
    uint16_t payload_length, checksum;
    
    try {

      /* Drain the I/O buffers! */
      if (tcdrain(_sick_fd) != 0) {
     	throw SickIOException("SickPLSBufferMonitor::GetNextMessageFromDataStream: tcdrain failed!");
      }

      /* Read until we get a valid message header */
      unsigned int bytes_searched = 0;
      while(search_buffer[0] != 0x02 || search_buffer[1] != DEFAULT_SICK_PLS_HOST_ADDRESS) {
	
 	/* Slide the search window */
 	search_buffer[0] = search_buffer[1];
	
 	/* Attempt to read in another byte */
 	_readBytes(&search_buffer[1],1,DEFAULT_SICK_PLS_SICK_BYTE_TIMEOUT);

	/* Header should be no more than max message length + header length bytes away */
	if (bytes_searched > SickPLSMessage::MESSAGE_MAX_LENGTH + SickPLSMessage::MESSAGE_HEADER_LENGTH) {
	  throw SickTimeoutException("SickPLSBufferMonitor::GetNextMessageFromDataStream: header timeout!");
	}
	
	/* Increment the number of bytes searched */
	bytes_searched++;
	
      }
      
      /* Read until we receive the payload length or we timeout */
      _readBytes(payload_length_buffer,2,DEFAULT_SICK_PLS_SICK_BYTE_TIMEOUT);

      /* Extract the payload length */
      memcpy(&payload_length,payload_length_buffer,2);
      payload_length = sick_pls_to_host_byte_order(payload_length);

      /* Make sure the payload length is legitimate, otherwise disregard */
      if (payload_length <= SickPLSMessage::MESSAGE_MAX_LENGTH) {

	/* Read until we receive the payload or we timeout */
	_readBytes(payload_buffer,payload_length,DEFAULT_SICK_PLS_SICK_BYTE_TIMEOUT);
	
	/* Read until we receive the checksum or we timeout */
	_readBytes(checksum_buffer,2,DEFAULT_SICK_PLS_SICK_BYTE_TIMEOUT);
	
	/* Copy into uint16_t so it can be used */
	memcpy(&checksum,checksum_buffer,2);
	checksum = sick_pls_to_host_byte_order(checksum);
	
	/* Build a frame and compute the crc */
	sick_message.BuildMessage(DEFAULT_SICK_PLS_HOST_ADDRESS,payload_buffer,payload_length);
	
	/* See if the checksums match */
	if(sick_message.GetChecksum() != checksum) {
	  throw SickBadChecksumException("SickPLS::GetNextMessageFromDataStream: CRC16 failed!");
	}

      }
      
    }
    
    catch(SickTimeoutException &sick_timeout_exception) { /* This is ok! */ }
    
    /* Handle a bad checksum! */
    catch(SickBadChecksumException &sick_checksum_exception) {
      sick_message.Clear(); // Clear the message container
    }
    
    /* Handle any serious IO exceptions */
    catch(SickIOException &sick_io_exception) {
      throw; 
    }

    /* A sanity check */
    catch (...) { 
      throw; 
    }
    
  }
  
  /**
   * \brief A standard destructor
   */
  SickPLSBufferMonitor::~SickPLSBufferMonitor( ) { }
    
} /* namespace SickToolbox */
