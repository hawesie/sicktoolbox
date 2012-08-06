/*!
 * \file SickPLS.cc
 * \brief Definition of the class SickPLS
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
#include <sstream>
#include <iostream>
#include <termios.h>
#include <sys/ioctl.h>
#include <signal.h>

#include "SickPLS.hh"
#include "SickPLSMessage.hh"
#include "SickPLSBufferMonitor.hh"
#include "SickPLSUtility.hh"
#include "SickException.hh"

#ifdef HAVE_LINUX_SERIAL_H
#include <linux/serial.h>
#else
#define B500000 0010005
#endif

/* Associate the namespace */
namespace SickToolbox {

  /*!
   * \brief Primary constructor
   * \param sick_device_path The path of the device
   */
  SickPLS::SickPLS( const std::string sick_device_path ): SickLIDAR< SickPLSBufferMonitor, SickPLSMessage >( ),
								_sick_device_path(sick_device_path),
								_curr_session_baud(SICK_BAUD_UNKNOWN),
								_desired_session_baud(SICK_BAUD_UNKNOWN)  {
    
    /* Initialize the protected/private structs */
    memset(&_sick_operating_status,0,sizeof(sick_pls_operating_status_t));
    memset(&_sick_baud_status,0,sizeof(sick_pls_baud_status_t));
    memset(&_old_term,0,sizeof(struct termios));

    //start in an unknown mode
    _sick_operating_status.sick_operating_mode  = SICK_OP_MODE_UNKNOWN;   
    
    //fixed parameters
    _sick_operating_status.sick_measuring_units = SICK_MEASURING_UNITS_CM;
    _sick_operating_status.sick_scan_resolution = SICK_SCAN_RESOLUTION_50;
    _sick_operating_status.sick_scan_angle = SICK_SCAN_ANGLE_180;
  }

  /**
   * \brief Destructor
   */
  SickPLS::~SickPLS() {

    try {

      /* Attempt to uninitialize the device */
      _teardownConnection();
      
    }

    /* Catch an I/O exception */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
    }
    
    /* Catch anything else */
    catch(...) {
      std::cerr << "SickPLS::~SickPLS: Unknown exception!" << std::endl;
    }
    
  }
  
  /**
   * \brief Attempts to initialize the Sick PLS and then sets communication at
   *        at the given baud rate.
   * \param desired_baud_rate Desired session baud rate
   */
  void SickPLS::Initialize( const sick_pls_baud_t desired_baud_rate )
    throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException ) {

    /* Buffer the desired baud rate in case we have to reset */
    _desired_session_baud = desired_baud_rate;
    
    try {
    
      std::cout << std::endl << "\t*** Attempting to initialize the Sick PLS..." << std::endl << std::flush;
      
      /* Initialize the serial term for communication */
      std::cout << "\tAttempting to open device @ " << _sick_device_path << std::endl << std::flush;
      _setupConnection();
      std::cout << "\t\tDevice opened!" << std::endl << std::flush;

      /* Start/reset the buffer monitor */
      if (!_sick_monitor_running) {
	std::cout << "\tAttempting to start buffer monitor..." << std::endl;       
	_startListening();
	std::cout << "\t\tBuffer monitor started!" << std::endl;
      }
      else {
	std::cout << "\tAttempting to reset buffer monitor..." << std::endl;       
	_sick_buffer_monitor->SetDataStream(_sick_fd);
	std::cout << "\t\tBuffer monitor reset!" << std::endl;       
      }

      try {

	std::cout << "\tAttempting to set session baud rate to "
		  << SickPLS::SickBaudToString(_desired_session_baud) 
		  << " as requested..." << std::endl;
	_setSessionBaud(_desired_session_baud);
	
      }

      /* Assume a timeout is due to a misconfigured terminal baud */
      catch(SickTimeoutException &sick_timeout) {
      
	/* Check whether to do an autodetect */
	sick_pls_baud_t default_baud = _baudToSickBaud(DEFAULT_SICK_PLS_SICK_BAUD);
	std::cout << "\tFailed to set requested baud rate..." << std::endl << std::flush;
	std::cout << "\tAttempting to detect PLS baud rate..." << std::endl << std::flush;
	if((default_baud != SICK_BAUD_9600) && _testSickBaud(SICK_BAUD_9600)) {
	  std::cout << "\t\tDetected PLS baud @ " << SickBaudToString(SICK_BAUD_9600) << "!" << std::endl;
	} else if((default_baud != SICK_BAUD_19200) && _testSickBaud(SICK_BAUD_19200)) {
	  std::cout << "\t\tDetected PLS baud @ " << SickBaudToString(SICK_BAUD_19200) << "!" << std::endl;
	} else if((default_baud != SICK_BAUD_38400) && _testSickBaud(SICK_BAUD_38400)) {
	  std::cout << "\t\tDetected PLS baud @ " << SickBaudToString(SICK_BAUD_38400) << "!" << std::endl;
	} else if((default_baud) != SICK_BAUD_500K && _testSickBaud(SICK_BAUD_500K)) {
	  std::cout << "\t\tDetected PLS baud @ " << SickBaudToString(SICK_BAUD_500K) << "!" << std::endl;
	} else {
          _stopListening();
	  throw SickIOException("SickPLS::Initialize: failed to detect baud rate!");	
	}
	std::cout << std::flush;
	
	/* Try again! */
	if (_curr_session_baud != _desired_session_baud) {
	  std::cout << "\tAttempting to setup desired baud (again)..." << std::endl << std::flush;
	  _setSessionBaud(_desired_session_baud);	  
	}
	
      }

      /* Catch anything else */
      catch(...) {
	std::cerr << "SickPLS::Initialize: Unknown exception!" << std::endl;
	throw;
      }

      std::cout << "\t\tOperating @ " << SickBaudToString(_curr_session_baud) << std::endl;     
      
      /* Set the device to request range mode */
      _setSickOpModeMonitorRequestValues();
      

      /* Set the flag */
      _sick_initialized = true;
      
    }

    /* Handle a config exception */
    catch(SickConfigException &sick_config_exception) {
      std::cerr << sick_config_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a timeout exception */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Handle any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }

    /* Handle anything else */
    catch(...) {
      std::cerr << "SickPLS::Initialize: Unknown exception!" << std::endl;
      throw;
    }

    /* Initialization was successful! */
    std::cout << "\t*** Init. complete: Sick PLS is online and ready!" << std::endl; 
    std::cout << "\tScan Angle: " << GetSickScanAngle() << " (deg)" << std::endl;  
    std::cout << "\tScan Resolution: " << GetSickScanResolution() << " (deg)" << std::endl;
    std::cout << "\tMeasuring Units: " << SickMeasuringUnitsToString(GetSickMeasuringUnits()) << std::endl;
    std::cout << std::endl << std::flush;
    
  }

  /**
   * \brief Uninitializes the PLS by putting it in a mode where it stops streaming data,
   *        and returns it to the default baud rate (specified in the header).
   */
  void SickPLS::Uninitialize( ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException ) {

    if (_sick_initialized) {

      std::cout << std::endl << "\t*** Attempting to uninitialize the Sick PLS..." << std::endl;       

      try {
	
	/* Restore original operating mode */
	_setSickOpModeMonitorRequestValues();
	
	/* Restore original baud rate settings */
	_setSessionBaud(_baudToSickBaud(DEFAULT_SICK_PLS_SICK_BAUD));

	/* Attempt to cancel the buffer monitor */
	if (_sick_monitor_running) {
	  std::cout << "\tAttempting to stop buffer monitor..." << std::endl;
	  _stopListening();
	  std::cout << "\t\tBuffer monitor stopped!" << std::endl;
	}
	
	std::cout << "\t*** Uninit. complete - Sick PLS is now offline!" << std::endl << std::flush;
	
      }
    
      /* Handle any config exceptions */
      catch(SickConfigException &sick_config_exception) {
	std::cerr << sick_config_exception.what() << " (attempting to kill connection anyways)" << std::endl;
	throw;
      }
      
      /* Handle a timeout exception */
      catch(SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << " (attempting to kill connection anyways)" << std::endl;
	throw;
      }
      
      /* Handle any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << " (attempting to kill connection anyways)" << std::endl;
	throw;
      }
      
      /* Handle any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
	std::cerr << sick_thread_exception.what() << " (attempting to kill connection anyways)" << std::endl;
	throw;
      }
      
      /* Handle anything else */
      catch(...) {
	std::cerr << "SickPLS::Unintialize: Unknown exception!!!" << std::endl;
	throw;
      }

      /* Reset the flag */
      _sick_initialized = false;
      
    }
      
  }

  /**
   * \brief Gets the Sick PLS device path
   * \return The device path as a std::string
   */
  std::string SickPLS::GetSickDevicePath( ) const {
    return _sick_device_path;
  }
  

  /**
   * \brief Gets the current scan angle of the device
   * \return Scan angle of the device (sick_pls_scan_angle_t) 
   */
  double SickPLS::GetSickScanAngle( ) const throw( SickConfigException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickPLS::GetSickScanAngle: Sick PLS is not initialized!");
    }

    /* Return the Sick scan angle */
    return (double)_sick_operating_status.sick_scan_angle;

  }

  /**
   * \brief Gets the current angular resolution
   * \return Angular resolution of the Sick PLS (sick_pls_scan_resolution_t) 
   */
  double SickPLS::GetSickScanResolution( ) const throw( SickConfigException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickPLS::GetSickScanResolution: Sick PLS is not initialized!");
    }

    /* Return the scan resolution */
    return _sick_operating_status.sick_scan_resolution*(0.01);

  }
    
  

  /**
   * \brief Gets the current Sick PLS measuring units
   * \return Measuring units (sick_pls_measuring_units_t)
   */
  sick_pls_measuring_units_t SickPLS::GetSickMeasuringUnits( ) const throw( SickConfigException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickPLS::GetSickMeasuringUnits: Sick PLS is not initialized!");
    }

    return (sick_pls_measuring_units_t)_sick_operating_status.sick_measuring_units;
  }
  
  /**
   * \brief Gets the current Sick PLS operating mode
   * \return Operating mode (sick_pls_operating_mode_t)
   */
  sick_pls_operating_mode_t SickPLS::GetSickOperatingMode( ) const throw( SickConfigException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickPLS::GetSickScanAngle: Sick PLS is not initialized!");
    }

    /* Return the current operating mode of the device */
    return (sick_pls_operating_mode_t)_sick_operating_status.sick_operating_mode;

  }
  
  
 
  /**
   * \brief Returns the most recent measured values obtained by the Sick PLS
   * \param *measurement_values Destination buffer for holding the current round of measured values
   * \param &num_measurement_values Number of values stored in measurement_values
   * \param *sick_field_a_values Stores the Field A values associated with the given scan (Default: NULL => Not wanted)
   * \param *sick_field_b_values Stores the Field B values associated with the given scan (Default: NULL => Not wanted)
   * \param *sick_field_c_values Stores the Field C values associated with the given scan (Default: NULL => Not wanted)
   * \param *sick_telegram_index The telegram index assigned to the message (modulo: 256) (Default: NULL => Not wanted)
   * \param *sick_real_time_scan_index The real time scan index for the latest message (module 256) (Default: NULL => Not wanted)
   *
   * NOTE: Calling this function will return either range or reflectivity measurements
   *       depending upon the current measuring mode of the device.
   *
   * NOTE: Real-time scan indices must be enabled by setting the corresponding availability
   *       of the Sick PLS for this value to be populated.
   */
  void SickPLS::GetSickScan( unsigned int * const measurement_values,
			     unsigned int & num_measurement_values) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickPLS::GetSickScan: Sick PLS is not initialized!");
    }
    
    /* Declare message objects */
    SickPLSMessage response;

    /* Declare some useful variables and a buffer */
    uint8_t payload_buffer[SickPLSMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
    
    try {
    
      /* Restore original operating mode */
      _setSickOpModeMonitorStreamValues();
      
      /* Receive a data frame from the stream. */
      _recvMessage(response,DEFAULT_SICK_PLS_SICK_MESSAGE_TIMEOUT);
      
      /* Check that our payload has the proper command byte of 0xB0 */
      if(response.GetCommandCode() != 0xB0) {
	throw SickIOException("SickPLS::GetSickScan: Unexpected message!");
      }

      /* Acquire the payload buffer and length*/
      response.GetPayload(payload_buffer);

      /* Define a local scan profile object */
      sick_pls_scan_profile_b0_t sick_scan_profile;

      /* Initialize the profile */
      memset(&sick_scan_profile,0,sizeof(sick_pls_scan_profile_b0_t));

      /* Parse the message payload */
      _parseSickScanProfileB0(&payload_buffer[1],sick_scan_profile);

      /* Return the request values! */
      num_measurement_values = sick_scan_profile.sick_num_measurements;

      for (unsigned int i = 0; i < num_measurement_values; i++) {

	/* Copy the measurement value */
	measurement_values[i] = sick_scan_profile.sick_measurements[i];

      }

    }

    /* Handle any config exceptions */
    catch(SickConfigException &sick_config_exception) {
      std::cerr << sick_config_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a timeout exception */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Handle any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Handle anything else */
    catch(...) {
      std::cerr << "SickPLS::GetSickScan: Unknown exception!!!" << std::endl;
      throw;
    }

  }




   
  /**
   * \brief Reset the Sick PLS active field values
   * NOTE: Considered successful if the PLS ready message is received.
   */
  void SickPLS::ResetSick( ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickPLS::ResetSick: Sick PLS is not initialized!");
    }
    
    SickPLSMessage message,response;
    uint8_t payload[SickPLSMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Construct the reset command */
    payload[0] = 0x10; // Request field reset
    message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload,1);
    
    std::cout << "\tResetting the device..." << std::endl;
    std::cout << "\tWaiting for Power on message..." << std::endl;

    try {

      /* Send the reset command and wait for the reply */
      _sendMessageAndGetReply(message,response,0x91,(unsigned int)60e6,DEFAULT_SICK_PLS_NUM_TRIES);

      std::cout << "\t\tPower on message received!" << std::endl;
      std::cout << "\tWaiting for PLS Ready message..." << std::endl;

      /* Set terminal baud to the detected rate to get the PLS ready message */
      _setTerminalBaud(_baudToSickBaud(DEFAULT_SICK_PLS_SICK_BAUD));

      /* Receive the PLS ready message after power on */
      _recvMessage(response,(unsigned int)30e6);
      
      /* Verify the response */
      if(response.GetCommandCode() != 0x90) {
 	std::cerr << "SickPLS::ResetSick: Unexpected reply! (assuming device has been reset!)"  << std::endl;
      } else {
 	std::cout << "\t\tPLS Ready message received!" << std::endl;
      }
      std::cout << std::endl;

      /* Reinitialize and sync the device */
      Initialize(_desired_session_baud);

    }
    
    /* Catch any timeout exceptions */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Catch any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Catch any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Catch anything else */
    catch(...) {
      std::cerr << "SickPLS::ResetSick: Unknown exception!!!" << std::endl;
      throw;
    }
    
    std::cout << "\tRe-initialization sucessful. PLS is ready to go!" << std::endl;
    
  }

  /**
   * \brief Acquire the Sick PLS's status as a printable string
   * \return The Sick PLS status as a well-formatted string
   */
  std::string SickPLS::GetSickStatusAsString( ) const {

    std::stringstream str_stream;

    str_stream << "\t=============== Sick PLS Status ===============" << std::endl;

    /* If Sick is initialized then print the status! */
    if (_sick_initialized) {

      str_stream << "\tScan Angle: " << GetSickScanAngle() << " (deg)" << std::endl;
      str_stream << "\tScan Resolution: " << GetSickScanResolution() << " (deg)" << std::endl;
      str_stream << "\tOperating Mode: " << SickOperatingModeToString(GetSickOperatingMode()) << std::endl;
      str_stream << "\tMeasuring Units: " << SickMeasuringUnitsToString(GetSickMeasuringUnits()) << std::endl;

    }
    else {
      
      str_stream << "\t Unknown (Device is not initialized)" << std::endl;

    }      

    str_stream << "\t===============================================" << std::endl;
    
    return str_stream.str();
  }


  
  

  /**
   * \brief Converts integer to corresponding Sick PLS scan angle
   * \param scan_angle_int Scan angle (FOV) as an integer (e.g. 90,100,180)
   */
  sick_pls_scan_angle_t SickPLS::IntToSickScanAngle( const int scan_angle_int ) {

    switch(scan_angle_int) {
    case 180:
      return SICK_SCAN_ANGLE_180;
    default:
      return SICK_SCAN_ANGLE_UNKNOWN;      
    }

  }
  
  /**
   * \brief Converts integer to corresponding Sick PLS scan resolution
   * \param scan_resolution_int Scan resolution as an integer (e.g. 25,50,100)
   */
  sick_pls_scan_resolution_t SickPLS::IntToSickScanResolution( const int scan_resolution_int ) {

    switch(scan_resolution_int) {
    case 50:
      return SICK_SCAN_RESOLUTION_50;
    default:
      return SICK_SCAN_RESOLUTION_UNKNOWN;      
    }

  }
  
  /**
   * \brief Converts double to corresponding Sick PLS scan resolution
   * \param scan_resolution_double Scan resolution as a double (e.g. 0.25,0.5,1.0)
   */
  sick_pls_scan_resolution_t SickPLS::DoubleToSickScanResolution( const double scan_resolution_double ) {
    return IntToSickScanResolution((const int)(scan_resolution_double*100));
  }
  
  /**
   * \brief Converts Sick PLS baud to a corresponding string
   * \param baud_rate The baud rate to be represented as a string
   * \return The string representation of the baud rate
   */
  std::string SickPLS::SickBaudToString( const sick_pls_baud_t baud_rate ) {

    switch(baud_rate) {
    case SICK_BAUD_9600:
      return "9600bps";
    case SICK_BAUD_19200:
      return "19200bps";
    case SICK_BAUD_38400:
      return "38400bps";
    case SICK_BAUD_500K:
      return "500Kbps";
    default:
      return "Unknown!";
    }
    
  }

  /**
   * \brief Converts integer to corresponding Sick PLS baud
   * \param baud_str Baud rate as integer (e.g. 9600,19200,38400,500000)
   */
  sick_pls_baud_t SickPLS::IntToSickBaud( const int baud_int ) {

    switch(baud_int) {
    case 9600:
      return SICK_BAUD_9600;
    case 19200:
      return SICK_BAUD_19200;
    case 38400:
      return SICK_BAUD_38400;
    case 500000:
      return SICK_BAUD_500K;
    default:
      return SICK_BAUD_UNKNOWN;      
    }

  }
  
  /**
   * \brief Converts string to corresponding Sick PLS baud
   * \param baud_str Baud rate as string (e.g. "9600","19200","38400","500000")
   */
  sick_pls_baud_t SickPLS::StringToSickBaud( const std::string baud_str ) {

    int baud_int;
    std::istringstream input_stream(baud_str);
    input_stream >> baud_int;
    
    return IntToSickBaud(baud_int);

  }
  
  /**
   * \brief Converts the Sick PLS status code to a string
   * \param sick_status The device status
   * \return A string corresponding to the given status code
   */
  std::string SickPLS::SickStatusToString( const sick_pls_status_t sick_status ) {

    /* Return a string */
    if(sick_status != SICK_STATUS_OK) {
      return "Error (possibly fatal)";
    }  
    return "OK!";
    
  }


  /**
   * \brief Converts the Sick operating mode to a corresponding string
   * \param sick_operating_mode The Sick operating mode
   * \return The corresponding string
   */
  std::string SickPLS::SickOperatingModeToString( const sick_pls_operating_mode_t sick_operating_mode ) {

    switch(sick_operating_mode) {
    case SICK_OP_MODE_INSTALLATION:
      return "Installation Mode";
    case SICK_OP_MODE_DIAGNOSTIC:
      return "Diagnostic Mode";
    case SICK_OP_MODE_MONITOR_STREAM_MIN_VALUE_FOR_EACH_SEGMENT:
      return "Stream mim measured values for each segment";
    case SICK_OP_MODE_MONITOR_TRIGGER_MIN_VALUE_ON_OBJECT:
      return "Min measured value for each segment when object detected";
    case SICK_OP_MODE_MONITOR_STREAM_MIN_VERT_DIST_TO_OBJECT:
      return "Min vertical distance";
    case SICK_OP_MODE_MONITOR_TRIGGER_MIN_VERT_DIST_TO_OBJECT:
      return "Min vertical distance when object detected";
    case SICK_OP_MODE_MONITOR_STREAM_VALUES:
      return "Stream all measured values";
    case SICK_OP_MODE_MONITOR_REQUEST_VALUES:
      return "Request measured values";
    case SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES:
      return "Stream mean measured values";
    case SICK_OP_MODE_MONITOR_STREAM_VALUES_SUBRANGE:
      return "Stream measured value subrange";
    case SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES_SUBRANGE:
      return "Stream mean measured value subrange";
    case SICK_OP_MODE_MONITOR_STREAM_VALUES_WITH_FIELDS:
      return "Stream measured and field values";
    case SICK_OP_MODE_MONITOR_STREAM_VALUES_FROM_PARTIAL_SCAN:
      return "Stream measured values from partial scan";
    case SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT_FROM_PARTIAL_SCAN:
      return "Stream range w/ reflectivity from partial scan";
    case SICK_OP_MODE_MONITOR_STREAM_MIN_VALUES_FOR_EACH_SEGMENT_SUBRANGE:
      return "Stream min measured values for each segment over a subrange";
    case SICK_OP_MODE_MONITOR_NAVIGATION:
      return "Output navigation data records";
    case SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT:
      return "Stream range w/ reflectivity values";
    default:
      return "Unknown!";
    };
    
  }
  
 
  
  /**
   * \brief Converts the Sick PLS measurement units to a corresponding string
   * \param sick_units The measuring units
   * \return The corresponding string
   */
  std::string SickPLS::SickMeasuringUnitsToString( const sick_pls_measuring_units_t sick_units ) {

    /* Return the proper string */
    switch(sick_units) {
    case SICK_MEASURING_UNITS_CM:
      return "Centimeters (cm)";
    default:
      return "Unknown!";
    }
    
  }
  
  /**
   * \brief Attempts to open a I/O stream using the device path given at object instantiation
   */
  void SickPLS::_setupConnection( ) throw ( SickIOException, SickThreadException ) {

    try {
    
      /* Open the device */
      if((_sick_fd = open(_sick_device_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
	throw SickIOException("SickPLS::_setupConnection: - Unable to open serial port");
      }
      
      /* Backup the original term settings */
      if(tcgetattr(_sick_fd,&_old_term) < 0) {
	throw SickIOException("SickPLS::_setupConnection: tcgetattr() failed!");
      }

      /* Set the host terminal baud rate to the new speed */
      _setTerminalBaud(_baudToSickBaud(DEFAULT_SICK_PLS_SICK_BAUD));
      
    }

    /* Handle any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Handle any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }

    /* Handle unknown exceptions */
    catch(...) {
      std::cerr << "SickPLS::_setupConnection: Unknown exception!" << std::endl;
      throw;
    }
    
  }

  /**
   * \brief Closes the data connection associated with the device
   */
  void SickPLS::_teardownConnection( ) throw( SickIOException ) {

    /* Check whether device was initialized */
    if(!_sick_initialized) {
      return;
    }
    
    /* Restore old terminal settings */
    if (tcsetattr(_sick_fd,TCSANOW,&_old_term) < 0) {
      throw SickIOException("SickPLS::_teardownConnection: tcsetattr() failed!");
    }

    /* Actually close the device */
    if(close(_sick_fd) != 0) {
      throw SickIOException("SickPLS::_teardownConnection: close() failed!");
    }

  }

  /**
   * \brief Flushes terminal I/O buffers
   */
  void SickPLS::_flushTerminalBuffer( ) throw ( SickThreadException ) {

    try {
    
      /* Acquire access to the data stream */    
      _sick_buffer_monitor->AcquireDataStream();

      /* Nobody is reading a message, so safely flush! */
      if (tcflush(_sick_fd,TCIOFLUSH) != 0) {
      	throw SickThreadException("SickPLS::_flushTerminalBuffer: tcflush() failed!");
      }
      
      /* Attempt to release the data stream */
      _sick_buffer_monitor->ReleaseDataStream();

    }

    /* Handle thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }

    /* A sanity check */
    catch(...) {
      std::cerr << "SickPLS::_flushTerminalBuffer: Unknown exception!" << std::endl;
      throw;
    }

  }

  /**
   * \brief Sends a message and searches for the corresponding reply
   * \param &send_message The message to be sent to the Sick PLS unit
   * \param &recv_message The expected message reply from the Sick PLS
   * \param timeout_value The epoch to wait before considering a sent frame lost (in usecs)
   * \param num_tries The number of times to try and transmit the message
   *                  before quitting
   *
   * NOTE: Uses the 0x80 response code rule for looking for the response message
   */
  void SickPLS::_sendMessageAndGetReply( const SickPLSMessage &send_message,
					    SickPLSMessage &recv_message,
					    const unsigned int timeout_value,
					    const unsigned int num_tries ) throw( SickIOException, SickThreadException, SickTimeoutException ) {

    uint8_t sick_reply_code = send_message.GetCommandCode() + 0x80;

    // std::cout.setf(std::ios::hex,std::ios::basefield);
    // std::cout << "Waiting for reply code: " << (unsigned int) sick_reply_code << std::endl;
    // std::cout.setf(std::ios::dec,std::ios::basefield);
    

    try {

      /* Send a message and get reply using a reply code */
      _sendMessageAndGetReply(send_message,recv_message,sick_reply_code,timeout_value,num_tries);

    }
    
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout) {
      /* For now just rethrow it */
      throw;
    }
    
    /* Handle a thread exception */
    catch (SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Handle write buffer exceptions */
    catch (SickIOException &sick_io_error) {
      std::cerr << sick_io_error.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickPLS::_sendMessageAndGetReply: Unknown exception!!!" << std::endl;
      throw;
    }
    
  }
  
  /**
   * \brief Sends a message and searches for the reply with given reply code
   * \param &send_message The message to be sent to the Sick PLS unit
   * \param &recv_message The expected message reply from the Sick PLS
   * \param reply_code The reply code associated with the expected messgage
   * \param timeout_value The epoch to wait before considering a sent frame lost (in usecs)
   * \param num_tries The number of times to send the message in the event the PLS fails to reply
   */
  void SickPLS::_sendMessageAndGetReply( const SickPLSMessage &send_message,
					    SickPLSMessage &recv_message,
					    const uint8_t reply_code,
					    const unsigned int timeout_value,
					    const unsigned int num_tries ) throw( SickIOException, SickThreadException, SickTimeoutException ) {

    try {

      /* Attempt to flush the terminal buffer */
      _flushTerminalBuffer();
      
      /* Send a message and get reply using parent's method */
      SickLIDAR< SickPLSBufferMonitor, SickPLSMessage >::_sendMessageAndGetReply(send_message,
										 recv_message,
										 &reply_code, 1, 
										 DEFAULT_SICK_PLS_BYTE_INTERVAL,
										 timeout_value, num_tries);

    }
    
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout) {
      throw;
    }
    
    /* Handle a thread exception */
    catch (SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Handle write buffer exceptions */
    catch (SickIOException &sick_io_error) {
      std::cerr << sick_io_error.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickPLS::_sendMessageAndGetReply: Unknown exception!!!" << std::endl;
      throw;
    }
    
  }
  
  /**
   * \brief Sets the baud rate for the current communication session
   * \param baud_rate The desired baud rate
   */
  void SickPLS::_setSessionBaud(const sick_pls_baud_t baud_rate) throw ( SickIOException, SickThreadException, SickTimeoutException ){
    
    //PLS-specific before session baud can be set, we need to enter setup mode (setup in PLS software, called installation in SickToolbox)

    _setSickOpModeInstallation();
    
    std::cout<< "Setting session baud from operating mode: "
	     << _sick_operating_status.sick_operating_mode
	     << std::endl;

    SickPLSMessage message, response;
    
    uint8_t payload[SickPLSMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
    
    /* Another sanity check */
    if(baud_rate == SICK_BAUD_UNKNOWN) {
      throw SickIOException("SickPLS::_setSessionBaud: Undefined baud rate!");
    }    
    
    /* Construct the command telegram */
    payload[0] = 0x20;
    payload[1] = baud_rate;
    
    message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload,2);
    
    // message.Print();

    try {

      /* Send the status request and get a reply */
      _sendMessageAndGetReply(message,response,DEFAULT_SICK_PLS_SICK_MESSAGE_TIMEOUT,DEFAULT_SICK_PLS_NUM_TRIES);

      /* Set the host terminal baud rate to the new speed */
      _setTerminalBaud(baud_rate);

      /* Sick likes a sleep here */
      usleep(250000);
      
    }
    
    /* Catch a timeout */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Catch any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Catch any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Catch anything else */
    catch(...) {
      std::cerr << "SickPLS::_getSickErrors: Unknown exception!!!" << std::endl;
      throw;
    }

  }
  
  /**
   * \brief Attempts to detect whether the PLS is operating at the given baud rate
   * \param baud_rate The baud rate to use when "pinging" the Sick PLS
   */
  bool SickPLS::_testSickBaud(const sick_pls_baud_t baud_rate) throw( SickIOException, SickThreadException ) {

    try {
    
      /* Another sanity check */
      if(baud_rate == SICK_BAUD_UNKNOWN) {
	throw SickIOException("SickPLS::_testBaudRate: Undefined baud rate!");
      }
      
      /* Attempt to get status information at the current baud */
      std::cout << "\t\tChecking " << SickBaudToString(baud_rate) << "..." << std::endl;
      
      /* Set the host terminal baud rate to the test speed */
      _setTerminalBaud(baud_rate);
      
      try {

	/* Check to see if the Sick replies! */
	_getSickErrors();

      }

      /* Catch a timeout exception */
      catch(SickTimeoutException &sick_timeout_exception) {
	/* This means that the current baud rate timed out! */
	return false;
      }

      /* Catch anything else and throw it away */
      catch(...) {
	std::cerr << "SickPLS::_testBaudRate: Unknown exception!" << std::endl;
	throw;
      }
      
    }

    /* Handle any IO exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Handle thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw; 
    }

    /* A safety net */
    catch(...) {
      std::cerr << "SickPLS::_testBaudRate: Unknown exception!!!" << std::endl;
      throw; 
    }

    /* Success! */
    return true;

  }

  /**
   * \brief Sets the local terminal baud rate
   * \param baud_rate The desired terminal baud rate
   */
  void SickPLS::_setTerminalBaud( const sick_pls_baud_t baud_rate ) throw( SickIOException, SickThreadException ) {

    struct termios term;

    term.c_iflag |= INPCK;
    term.c_iflag &= ~IXOFF;
    term.c_cflag |= PARENB;


#ifdef HAVE_LINUX_SERIAL_H
    struct serial_struct serial;
#endif
    
    try {
    
      /* If seeting baud to 500k */
      if (baud_rate == SICK_BAUD_500K) {

#ifdef HAVE_LINUX_SERIAL_H
	
	/* Get serial attributes */
	if(ioctl(_sick_fd,TIOCGSERIAL,&serial) < 0) {
	  throw SickIOException("SickPLS::_setTerminalBaud: ioctl() failed!");
	}
	
	/* Set the custom devisor */
	serial.flags |= ASYNC_SPD_CUST;
	serial.custom_divisor = 48; // for FTDI USB/serial converter divisor is 240/5
	
	/* Set the new attibute values */
	if(ioctl(_sick_fd,TIOCSSERIAL,&serial) < 0) {
	  throw SickIOException("SickPLS::_setTerminalBaud: ioctl() failed!");
	}

#else
	throw SickIOException("SickPLS::_setTerminalBaud - 500K baud is only supported under Linux!");
#endif
	
      }

#ifdef HAVE_LINUX_SERIAL_H
      
      else { /* Using a standard baud rate */

	/* We let the next few errors slide in case USB adapter is being used */
	if(ioctl(_sick_fd,TIOCGSERIAL,&serial) < 0) {
	  std::cerr << "SickPLS::_setTermSpeed: ioctl() failed while trying to get serial port info!" << std::endl;
	  std::cerr << "\tNOTE: This is normal when connected via USB!" <<std::endl;
	}
	
	serial.custom_divisor = 0;
        serial.flags &= ~ASYNC_SPD_CUST;
	
	if(ioctl(_sick_fd,TIOCSSERIAL,&serial) < 0) {
	  std::cerr << "SickPLS::_setTerminalBaud: ioctl() failed while trying to set serial port info!" << std::endl;
	  std::cerr << "\tNOTE: This is normal when connected via USB!" <<std::endl;
	}
	
      }
      
#endif
      
      /* Attempt to acquire device attributes */
      if(tcgetattr(_sick_fd,&term) < 0) {
	throw SickIOException("SickPLS::_setTerminalBaud: Unable to get device attributes!");
      }
      
      /* Switch on the baud rate */
      switch(baud_rate) {      
      case SICK_BAUD_9600: {
	cfmakeraw(&term);

      term.c_iflag |= INPCK;
      term.c_iflag &= ~IXOFF;
      term.c_cflag |= PARENB;

	cfsetispeed(&term,B9600);
	cfsetospeed(&term,B9600);
	break;
      }
      case SICK_BAUD_19200: {
	cfmakeraw(&term);

      term.c_iflag |= INPCK;
      term.c_iflag &= ~IXOFF;
      term.c_cflag |= PARENB;

	cfsetispeed(&term,B19200);
	cfsetospeed(&term,B19200);
	break;
      }
      case SICK_BAUD_38400: {
	cfmakeraw(&term);

      term.c_iflag |= INPCK;
      term.c_iflag &= ~IXOFF;
      term.c_cflag |= PARENB;

	cfsetispeed(&term,B38400);
	cfsetospeed(&term,B38400);            
	break;
      }
      case SICK_BAUD_500K: {      
	cfmakeraw(&term);

      term.c_iflag |= INPCK;
      term.c_iflag &= ~IXOFF;
      term.c_cflag |= PARENB;

	cfsetispeed(&term,B38400);
	cfsetospeed(&term,B38400);
	break;
      }
      default:
	throw SickIOException("SickPLS::_setTerminalBaud: Unknown baud rate!");
      }
      
      /* Attempt to set the device attributes */
      if(tcsetattr(_sick_fd,TCSAFLUSH,&term) < 0 ) {
	throw SickIOException("SickPLS::_setTerminalBaud: Unable to set device attributes!");
      }
      
      /* Buffer the rate locally */
      _curr_session_baud = baud_rate;
      
      /* Attempt to flush the I/O buffers */
      _flushTerminalBuffer();
      
    } // try

    /* Catch an IO exception */
    catch(SickIOException sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Catch an IO exception */
    catch(SickThreadException sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* A sanity check */
    catch(...) {
      std::cerr << "SickPLS::_setTerminalBaud: Unknown exception!!!" << std::endl;
      throw;
    }

  }


 
  /**
   * \brief Obtains any error codes from the Sick PLS
   */
  void SickPLS::_getSickErrors( unsigned int * const num_sick_errors, uint8_t * const error_type_buffer,
				uint8_t * const error_num_buffer ) throw( SickTimeoutException, SickIOException, SickThreadException ) {

     SickPLSMessage message, response;

     int payload_length;
     uint8_t payload_buffer[SickPLSMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
  
     /* The command to request PLS status */
     payload_buffer[0] = 0x32;
     
     /* Build the request message */
     message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,1);
     
     try {
       
       /* Send the status request and get a reply */
       _sendMessageAndGetReply(message,response,DEFAULT_SICK_PLS_SICK_MESSAGE_TIMEOUT,DEFAULT_SICK_PLS_NUM_TRIES);
       
     }

     /* Catch any timeout exceptions */
     catch(SickTimeoutException &sick_timeout_exception) {
       std::cerr << sick_timeout_exception.what() << std::endl;
       throw;
     }
     
     /* Catch any I/O exceptions */
     catch(SickIOException &sick_io_exception) {
       std::cerr << sick_io_exception.what() << std::endl;
       throw;
     }
     
     /* Catch any thread exceptions */
     catch(SickThreadException &sick_thread_exception) {
       std::cerr << sick_thread_exception.what() << std::endl;
       throw;
     }
     
     /* Catch anything else */
     catch(...) {
       std::cerr << "SickPLS::_getSickErrors: Unknown exception!!!" << std::endl;
       throw;
     }
     
     /* Extract the payload_length */
     payload_length = response.GetPayloadLength();
     
     /* Compute the number of errors */
     double num_errors = ((payload_length-2)/((double)2));
     
     /* Assign the number of errors if necessary */
     if (num_sick_errors) {
       *num_sick_errors = (unsigned int)num_errors;
     }
     
     /* Populate the return buffers with the error data */
     for (unsigned int i = 0, k = 1; i < (unsigned int)num_errors && (error_type_buffer || error_num_buffer); i++) {
       
       /* Check if the error type has been requested */
       if (error_type_buffer) {
	 error_type_buffer[i] = payload_buffer[k];
       }
       k++;
       
       /* Check if the error number has been requested */
       if (error_num_buffer) {
	 error_num_buffer[i] = payload_buffer[k];
       }
       k++;
       
     }

  }
  


  /**
   * \brief Sets the device to installation mode
   */
  void SickPLS::_setSickOpModeInstallation( )
    throw( SickConfigException, SickIOException, SickThreadException, SickTimeoutException) {
    
    /* Assign the password for entering installation mode */
    uint8_t sick_password[9] = DEFAULT_SICK_PLS_SICK_PASSWORD;

    /* Check if mode should be changed */
    if (_sick_operating_status.sick_operating_mode != SICK_OP_MODE_INSTALLATION) {

      try {

	/* Attempt to switch modes! */	
	_switchSickOperatingMode(SICK_OP_MODE_INSTALLATION,sick_password);

      }

      /* Catch any config exceptions */
      catch(SickConfigException &sick_config_exception) {
	std::cerr << sick_config_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any timeout exceptions */
      catch(SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Catch anything else */
      catch(...) {
	std::cerr << "SickPLS::_setSickOpModeInstallation: Unknown exception!!!" << std::endl;
	throw;
      } 
      
      /* Assign the new operating mode */
      _sick_operating_status.sick_operating_mode = SICK_OP_MODE_INSTALLATION;
    }

  }

  /**
   * \brief Sets the device to diagnostic mode
   */
  void SickPLS::_setSickOpModeDiagnostic( )
    throw( SickConfigException, SickIOException, SickThreadException, SickTimeoutException) {

    /* Check if mode should be changed */
    if (_sick_operating_status.sick_operating_mode != SICK_OP_MODE_DIAGNOSTIC) {

      std::cout << "\tAttempting to enter diagnostic mode..." << std::endl;
      
      try {

	/* Attempt to switch modes! */	
	_switchSickOperatingMode(SICK_OP_MODE_DIAGNOSTIC);

      }

      /* Catch any config exceptions */
      catch(SickConfigException &sick_config_exception) {
	std::cerr << sick_config_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any timeout exceptions */
      catch(SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Catch anything else */
      catch(...) {
	std::cerr << "SickPLS::_setSickOpModeInstallation: Unknown exception!!!" << std::endl;
	throw;
      } 
      
      /* Assign the new operating mode */
      _sick_operating_status.sick_operating_mode = SICK_OP_MODE_DIAGNOSTIC;

      std::cout << "Success!" << std::endl;
      
    }

  }

  /**
   * \brief Sets the device to monitor mode and tells it to send values only upon request
   */
  void SickPLS::_setSickOpModeMonitorRequestValues( )
    throw( SickConfigException, SickIOException, SickThreadException, SickTimeoutException) {

    /* Check if mode should be changed */
    if (_sick_operating_status.sick_operating_mode != SICK_OP_MODE_MONITOR_REQUEST_VALUES) {

      try {

	/* Attempt to switch operating mode */
	_switchSickOperatingMode(SICK_OP_MODE_MONITOR_REQUEST_VALUES);

      }

      /* Catch any config exceptions */
      catch(SickConfigException &sick_config_exception) {
	std::cerr << sick_config_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any timeout exceptions */
      catch(SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Catch anything else */
      catch(...) {
	std::cerr << "SickPLS::_setSickOpModeMonitorRequestValues: Unknown exception!!!" << std::endl;
	throw;
      } 
      
      /* Assign the new operating mode */
      _sick_operating_status.sick_operating_mode = SICK_OP_MODE_MONITOR_REQUEST_VALUES;

    }

  }

  /**
   * \brief Sets the device to monitor mode and tells it to stream measured values
   */
  void SickPLS::_setSickOpModeMonitorStreamValues( )
    throw( SickConfigException, SickIOException, SickThreadException, SickTimeoutException) {

    /* Check if mode should be changed */
    if (_sick_operating_status.sick_operating_mode != SICK_OP_MODE_MONITOR_STREAM_VALUES) {

      std::cout << "\tRequesting measured value data stream..." << std::endl;
      
      try {

	/* Attempt to switch modes */
	_switchSickOperatingMode(SICK_OP_MODE_MONITOR_STREAM_VALUES);

      }

      /* Catch any config exceptions */
      catch(SickConfigException &sick_config_exception) {
	std::cerr << sick_config_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any timeout exceptions */
      catch(SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Catch anything else */
      catch(...) {
	std::cerr << "SickPLS::_setSickOpModeMonitorStreamValues: Unknown exception!!!" << std::endl;
	throw;
      } 
      
      /* Assign the new operating mode */
      _sick_operating_status.sick_operating_mode = SICK_OP_MODE_MONITOR_STREAM_VALUES;

      std::cout << "\t\tData stream started!" << std::endl;
      
    }

  }

   
  /**
   * \brief Attempts to switch the operating mode of the Sick PLS
   * \param sick_mode The desired operating mode
   * \param mode_params Additional parameters required to set the new operating mode
   */
  void SickPLS::_switchSickOperatingMode( const uint8_t sick_mode, const uint8_t * const mode_params )
    throw( SickConfigException, SickIOException, SickThreadException, SickTimeoutException) {

    SickPLSMessage message,response;

    uint8_t payload_buffer[SickPLSMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};    
    uint16_t num_partial_scans = 0;

    /* Construct the correct switch mode packet */
    payload_buffer[0] = 0x20;
    payload_buffer[1] = sick_mode;

    switch(sick_mode) {
   
    case SICK_OP_MODE_INSTALLATION:

      /* Make sure the params are defined */
      if(mode_params == NULL) {
	throw SickConfigException("SickPLS::_switchSickOperatingMode - Requested mode requires parameters!");
      }

      memcpy(&payload_buffer[2],mode_params,8); //Copy password
      message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,10);
      break;

    case SICK_OP_MODE_DIAGNOSTIC:
      message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_MIN_VALUE_FOR_EACH_SEGMENT:
      message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_TRIGGER_MIN_VALUE_ON_OBJECT:
      message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_MIN_VERT_DIST_TO_OBJECT:
      message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_TRIGGER_MIN_VERT_DIST_TO_OBJECT:
      message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_VALUES:
      message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_REQUEST_VALUES:
      message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES:

      /* Make sure the params are defined */
      if(mode_params == NULL) {
	throw SickConfigException("SickPLS::_switchSickOperatingMode - Requested mode requires parameters!");
      }

      payload_buffer[2] = *mode_params;
      message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,3);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_VALUES_SUBRANGE:

      /* Make sure the params are defined */
      if(mode_params == NULL) {
	throw SickConfigException("SickPLS::_switchSickOperatingMode - Requested mode requires parameters!");
      }

      memcpy(&payload_buffer[2],mode_params,2);       //Begin range
      memcpy(&payload_buffer[4],&mode_params[2],2);   //End range
      message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,6);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES_SUBRANGE:

      /* Make sure the params are defined */
      if(mode_params == NULL) {
	throw SickConfigException("SickPLS::_switchSickOperatingMode - Requested mode requires parameters!");
      }

      payload_buffer[2] = mode_params[0];             //Sample size 
      memcpy(&payload_buffer[3],&mode_params[1],2);   //Begin mean range
      memcpy(&payload_buffer[5],&mode_params[3],2);   //End mean range
      message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,7);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_VALUES_WITH_FIELDS:

      /* Make sure the params are defined */
      if(mode_params == NULL) {
	throw SickConfigException("SickPLS::_switchSickOperatingMode - Requested mode requires parameters!");
      }

      memcpy(&payload_buffer[2],mode_params,2);       //Start
      memcpy(&payload_buffer[4],&mode_params[2],2);   //End
      message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,6);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_VALUES_FROM_PARTIAL_SCAN:
      message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT_FROM_PARTIAL_SCAN:

      /* Make sure the params are defined */
      if(mode_params == NULL) {
	throw SickConfigException("SickPLS::_switchSickOperatingMode - Requested mode requires parameters!");
      }

      /* Get the number of partial scans (between 1 and 5) */
      memcpy(&num_partial_scans,mode_params,2);

      /* Setup the command packet */
      memcpy(&payload_buffer[2],mode_params,num_partial_scans*4+2);
      message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,num_partial_scans*4+4);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_MIN_VALUES_FOR_EACH_SEGMENT_SUBRANGE:

      /* Make sure the params are defined */
      if(mode_params == NULL) {
	throw SickConfigException("SickPLS::_switchSickOperatingMode - Requested mode requires parameters!");
      }
    
      /* Get the number of partial scans (between 1 and 5) */
      memcpy(&num_partial_scans,mode_params,2);
    
      /* Setup the command packet */
      memcpy(&payload_buffer[2],mode_params,num_partial_scans*4+2);    
      message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,num_partial_scans*4+4);
      break;

    case SICK_OP_MODE_MONITOR_NAVIGATION:
      message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT:

      /* Make sure the params are defined */
      if(mode_params == NULL) {
	throw SickConfigException("SickPLS::_switchSickOperatingMode - Requested mode requires parameters!");
      }
      
      memcpy(&payload_buffer[2],mode_params,2);       //Start
      memcpy(&payload_buffer[4],&mode_params[2],2);   //End
      message.BuildMessage(DEFAULT_SICK_PLS_SICK_ADDRESS,payload_buffer,6);
      break;

    case SICK_OP_MODE_UNKNOWN:
      //Let this case go straight to default
    
    default:
      throw SickConfigException("SickPLS::_switchSickOperatingMode: Unrecognized operating mode!");
    }

    //    message.Print();

    try {
      
      /* Attempt to send the message and get the reply */
      _sendMessageAndGetReply(message,
			      response,
			      DEFAULT_SICK_PLS_SICK_SWITCH_MODE_TIMEOUT,
			      DEFAULT_SICK_PLS_NUM_TRIES);
      
    }

    /* Catch any timeout exceptions */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Catch any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Catch any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }

    /* Catch anything else */
    catch(...) {
      std::cerr << "SickPLS::_switchSickOperatingMode: Unknown exception!!!" << std::endl;
      throw;
    }
    
    /* Reset the buffer */
    memset(payload_buffer,0,sizeof(payload_buffer));
  
    /* Obtain the response payload */
    response.GetPayload(payload_buffer);

    /* Make sure the reply was expected */
    if(payload_buffer[1] != 0x00) {
      throw SickConfigException("SickPLS::_switchSickOperatingMode: configuration request failed!");
    }

  }

  /**
   * \brief Parses a byte sequence into a scan profile corresponding to message B0
   * \param *src_buffer The byte sequence to be parsed
   * \param &sick_scan_profile The returned scan profile for the current round of measurements
   */
  void SickPLS::_parseSickScanProfileB0( const uint8_t * const src_buffer, sick_pls_scan_profile_b0_t &sick_scan_profile ) const {

    /* Read block A, the number of measurments */
    sick_scan_profile.sick_num_measurements = src_buffer[0] + 256*(src_buffer[1] & 0x03);

    /* Check whether this is a partial scan */
    sick_scan_profile.sick_partial_scan_index = ((src_buffer[1] & 0x18) >> 3);

    /* Extract the measurements and Field values (if there are any) */
    _extractSickMeasurementValues(&src_buffer[2],
				  sick_scan_profile.sick_num_measurements,
				  sick_scan_profile.sick_measurements);
    
    /* If the Sick is pulling real-time indices then pull them too */
    unsigned int data_offset = 2 + 2*sick_scan_profile.sick_num_measurements;

    // /* Buffer the Sick telegram index */
    // sick_scan_profile.sick_telegram_index = src_buffer[data_offset];
    
  }

  

	  /**
	   * \brief Extracts the measured values (w/ flags) that were returned by the device.
	   * \param *byte_sequence The byte sequence holding the current measured values
	   * \param num_measurements The number of measurements given in the byte sequence
	   * \param *measured_values A buffer to hold the extracted measured values
	   * \param *field_a_values Stores the Field A values associated with the given measurements (Default: NULL => Not wanted)
	   * \param *field_b_values Stores the Field B values associated with the given measurements (Default: NULL => Not wanted)
	   * \param *field_c_values Stores the Field C values associated with the given measurements (Default: NULL => Not wanted)
	   */
  void SickPLS::_extractSickMeasurementValues( const uint8_t * const byte_sequence, 
					       const uint16_t num_measurements, 
					       uint16_t * const measured_values) const {
    /* Extract the range and Field values */
    for(unsigned int i = 0; i < num_measurements; i++) {
      measured_values[i] = byte_sequence[i*2] + 256*(byte_sequence[i*2+1] & 0x1F);                 
    }    
  }
  
  /**
   * \brief Indicates whether the given scan angle is defined
   * \param sick_scan_angle The scan angle in question
   */ 
  bool SickPLS::_validSickScanAngle( const sick_pls_scan_angle_t sick_scan_angle ) const {

    /* Check the given Sick scan angle */
    if (sick_scan_angle != SICK_SCAN_ANGLE_180 ) {
      
      return false;
    }

    /* Valid */
    return true;
  }

  /**
   * \brief Indicates whether the given scan resolution is defined
   * \param sick_scan_resolution The scan resolution in question
   */ 
  bool SickPLS::_validSickScanResolution( const sick_pls_scan_resolution_t sick_scan_resolution ) const {

    /* Check the given Sick scan resolution value */
    if (sick_scan_resolution != SICK_SCAN_RESOLUTION_50) {     
      return false;
    }

    /* Valid */
    return true;
  }
  

  

  /**
   * \brief Converts a termios baud to an equivalent Sick baud
   * \param baud_rate The baud rate to be converted to a Sick PLS baud
   * \return The Sick PLS equivalent of the given baud rate
   */
  sick_pls_baud_t SickPLS::_baudToSickBaud( const int baud_rate ) const {
  
    switch(baud_rate) {
    case B9600:
      return SICK_BAUD_9600;
    case B19200:
      return SICK_BAUD_19200;
    case B38400:
      return SICK_BAUD_38400;
    case B500000:
      return SICK_BAUD_500K;
    default:
      std::cerr << "Unexpected baud rate!" << std::endl;
      return SICK_BAUD_9600;
    }
    
  }

    
  
} //namespace SickToolbox
