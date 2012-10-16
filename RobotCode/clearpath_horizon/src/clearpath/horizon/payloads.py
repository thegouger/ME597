#! /usr/bin/env python -m
# -*- coding: utf-8 -*-
#     _____
#    /  _  \
#   / _/ \  \
#  / / \_/   \
# /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
# \  / \_/ \  / /  _\| |  | __| / _ \ | ┌┐ \ | ┌┐ \ / _ \ |_   _|| | | |
#  \ \_/ \_/ /  | |  | |  | └─┐| |_| || └┘ / | └┘_/| |_| |  | |  | └─┘ |
#   \  \_/  /   | |_ | |_ | ┌─┘|  _  || |\ \ | |   |  _  |  | |  | ┌─┐ |
#    \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
#            ROBOTICS™
#
#  File: payloads.py
#  Desc: Horizon Protocol Message Definitions
#  Auth: Malcolm Robert
#  
#  Copyright © 2010 Clearpath Robotics, Inc. 
#  All Rights Reserved
# 
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in the
#        documentation and/or other materials provided with the distribution.
#      * Neither the name of Clearpath Robotics, Inc. nor the
#        names of its contributors may be used to endorse or promote products
#        derived from this software without specific prior written permission.
# 
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
#  ARE DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
#  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#  Please send comments, questions, or patches to code@clearpathrobotics.com
#




################################################################################
# Script



# Check if run as a script
if __name__ == "__main__":
    
    # Warn of Module ONLY status
    print ("ERROR: clearpath.horizon.payloads is a module and can NOT be run"\
           " as a script!\nFor a command-line interface demo of Horizon, run:"\
           "\n  python -m clearpath.horizon.demo\n"\
           "For Horizon message forwarding, run:\n"\
           "  python -m clearpath.horizon.forward")

    # Exit Error
    import sys
    sys.exit(1)




################################################################################
# Module



## @package clearpath.horizon.payloads
#  Horizon Protocol Message Payloads Python Module
# 
#  Horizon Protocol Message Payload Definitions                               \n
#  Abstracted from knowing message codes and message header.                  \n
#  Supported Horizon version(s): 0.1 - 1.0
#
#  @author     Ryan Gariepy
#  @author     Malcolm Robert
#  @date       25/01/10
#  @req        clearpath.utils                                                \n
#              clearpath.horizon.versioning
#  @version    1.0
#
#  @section representation Data Representations
#  @copydoc representation
#
#  @section USE
#
#  The intended purpose of this module is to provide the various HorizonPayload
#  definitions for the various messages within Horizon. The HorizonPayload class
#  abstracts a payload and can represent a payload within a message without
#  having any knowledge of the contained format whereas subclasses should know
#  how payload data is formatted for one or more messages that they represent.
#  These classes have no knowledge of message fields that do not fall within
#  the payload field.
#
#  Whether creating or parsing a payload, instantiation requires the parameters
#  store_error, timestamp, and version. Parsing only requires the additional
#  parameter raw, which is a list of integers representing the data of the 
#  payload. Note that the superclass HorizonPayload can only be instantiated
#  in the parsing method. The parameters required for creation is dependent on
#  the payload type being created (ex. HorizonPayload_Request - subscription,
#  HorizonPayload_PlatformName - name). Regardless of instantiation method,
#  the additional parameters required for creation are readable as properties.
#  As with HorizonMessage, once created, data in a HorizonPayload class
#  should not be modified. If modification is desired, create a new 
#  HorizonPayload object with the modified data.
#
#  Various logging messages (debug, warning,...) are logged with the standard
#  Python logger to the log named 'clearpath.horizon.payloads'.               \n
#  Messages are ignored by default; remove the only handler from it and turn
#  on propagation to get messages to propagate.
#
#  @section HISTORY
#  Version 0.1 - 0.3 {Ryan Gariepy}
#  - Initial Creation as protocol.py
#
#  Version 0.4 {Malcolm Robert}
#  - Move to horizon_messages.py
#  - Added manipulator payloads
#  - Added safety system payloads
#  - Added payload abstraction
#  - Added logging
#  - Added version support
#  - Added Doxygen documentation
#  - Changed version scheme to match Horizon doc
#  - Horizon support for v0.4
#
#  Version 0.5
#  - Added GPADC payload
#  - Horizon support for v0.5
#
#  Version 0.6
#  - Added content to platform info payload
#  - Move to messages.py
#
#  Version 0.7
#  - Added reset payload
#  - Fixed number scales
#  - Horizon support for v0.7
#
#  Version 0.8
#  - Added power status payload
#  - Added raw encoders payload
#  - Horizon support for v0.8
#
#  Version 1.0
#  - Move to payloads.py
#  - Horizon support for v 0.1 - 1.0
#  - Python 2.6+ & 3.x compatible
#
#  @section License
#  @copydoc public_license
#
## @defgroup representation Data Representations
#  @ingroup doc
#  All decimal quantities are represented by fixed-point signed numbers with 
#  prespecified scales. For example, a two byte number with a scale of \b 1000 
#  will represent numbers in the range \b [-32.768, \b 32.767], while the same 
#  number with a scale of \b 1 represents \b [-32768, \b 32767].
#
#  @htmlonly <center><img src="form_0.png" ></center> @endhtmlonly
#  @manonly 
#             REAL_DATA
#  RAW_DATA = ---------
#               SCALE
#  @endmanonly
#
#  A \b char/byte is an 8 bit field, a \b short is a 16 bit field, and an \b int 
#  is a 32 bit field. Two's complement is used for signed fields. \b byte 
#  implies an unsigned char.
#
#  @defgroup formula Data Representation Formula
#  @ingroup representation
#  \f[ RAW DATA = \frac{REAL DATA}{SCALE} \f] 
#
"""Horizon Protocol Message Payload Definitions 

   Copyright © 2010 Clearpath Robotics, Inc.
   All rights reserved
   
   Created: 25/01/10
   Authors: Ryan Gariepy & Malcolm Robert
   Version: 1.0
   """


# A note on comments:
# To take advantage of the superior Doxygen documentation generator,
# Doxygen comments (first line ##, following lines #) are used.
# It is still important to provide Python __doc__ strings for use with help() 
# and pydoc, however, they (except for module) are also grabbed by Doxygen.
# To fix this, the custom Doxygen tag @pydoc has been added to separate
# __doc__ strings from being included in the last Doxygen tag.
# @pydoc must be the last Doxygen tag before """doc string"""
# Additionally, the custom Doxygen tag @req has been added to specify
# required modules that do not come with the standard Python distribution.
# Both custom tags are aliases to @par.


# Required Clearpath Modules
from .. import utils            # Clearpath Utilities
from .  import versioning       # Horizon Protocol Versions 

# Required Python Modules
import datetime                 # Date & Time Manipulation
import logging                  # Logging Utilities
import math                     # Math Constants and Functions


# Module Support
## Module Version
__version__  = "1.0"
"""Module Version"""
## SVN Code Revision
__revision__ = "$Revision: 493 $"
""" SVN Code Revision"""


## Message Log
logger = logging.getLogger('clearpath.horizon.payloads')
"""Horizon Message Payloads Module Log"""
logger.setLevel(logging.NOTSET)
logger.addHandler(utils.NullLoggingHandler())
logger.propagate = False
logger.debug("Loading clearpath.horizon.payloads ...")  




################################################################################
# Horizon Payload Superclass
    
    
    
## Horizon Payload
#
#  Represents the basic payload of a Horizon message.                         \n
#  To be inherited for specific message payloads.
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#  
#  @pydoc
class HorizonPayload():
    """Horizon Protocol Message Payload"""
    
    ## Create A Horizon Message Payload
    #
    #  Constructor for the Horizon Payload class.                             \n
    #  Creates a basic message payload which is simply a byte list.           \n
    #  Does NOT support version auto-detection.                               \n
    #                                                                         \n
    #  Subclass overrides should throw ValueError if invalid format and 
    #  LookupError if it has version detection problems (if supported).
    #
    #  @param  raw            Raw data buffer to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, raw = [], timestamp = 0, version = None):
        """Create A Horizon Message Payload"""
        
        # Class Variables
        self.data = []
        self.error = None
        self.timestamp = 0
        self.version = version
        
        # verify timestamp
        if timestamp < 0 or timestamp > 4294967295: # 4294967295 for int
            logger.warning("%s: Invalid timestamp %d!" \
                         % (self.__class__.__name__, timestamp))
            self.error = ValueError("Invalid timestamp!")
            if not store_error: raise self.error  
            else: return
        self.timestamp = timestamp
        logger.debug("%s: Timestamp: %d" % (self.__class__.__name__, 
                     self.timestamp))
        
        # Doesn't actually do anything, just copies raw
        self.data = raw
        logger.debug("%s: Raw payload data: %s" % (self.__class__.__name__, 
                     str(self)))
        
        
    ## Hex String Representation
    #
    #  Return the entire payload in a string of hex characters.
    # 
    #  @return String of hex info
    #
    #  @pydoc
    def __str__(self):
        """Return the entire payload in a string of hex characters."""
        
        return ' '.join(map(utils.hex,self.data))
    
    
    ## Copy Instance
    #
    #  @return A deep copy of this object
    #
    #  @pydoc
    def copy(self):
        """Copy Instance"""
        
        # determine required data
        store_error = False
        if self.error != None: store_error = True
        raw = None
        if self.data != None: raw = self.data[:]
        
        # create new copy
        logger.debug("%s: Creating copy." % self.__class__.__name__)
        return self.__class__(store_error = store_error, raw = raw, 
                              timestamp = self.timestamp)
    
    
    ## Human Readable Payload String
    #
    #  @return Human readable string representation of the payload
    #
    #  @pydoc
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Payload: %s\n" % str(self)
    
    
    ## Raw Bytes Payload Representation
    #
    #  Convert the payload into raw bytes (character string for Python 2.x)
    #  useful for writing to devices.
    #
    #  @return raw bytes
    #
    #  @pydoc 
    def raw_string(self):
        """Returns the data converted into raw bytes."""
        
        return utils.to_bytes(self.data)
    



################################################################################
# Horizon Payloads
    
    
   
## Horizon Message Payload - Null
#
#  Represents the null payload (payload of a message without a payload).
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @pydoc
class HorizonPayload_Null(HorizonPayload):
    """Horizon Message Payload - Null"""
    
    
    ## Create A Horizon Message Payload - Null 
    #
    #  Constructor for the Horizon Message Payload - Null  Class.             \n
    #  Version auto-detection is unsupported.
    #
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, raw = None, store_error = False, timestamp = 0, 
                 version = tuple([-1,0])):
        """Create A Horizon Message Payload - Null"""
        
        # Verify Length
        if raw == None: raw = []
        if len(raw) != 0:
            logger.warning("%s: Bad length!" % self.__class__.__name__)
            self.error = ValueError("Invalid length!")
            if not store_error: raise self.error  
            else: return
                
        # Pass on to super-class
        HorizonPayload.__init__(self, raw = raw, version = version, 
                                store_error = store_error, 
                                timestamp = timestamp)
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Payload: NULL"

    
   
## @defgroup acknowledgments Acknowledgments
#  @ingroup format
#
#  The platform will acknowledge each command or data request it receives 
#  immediately upon message processing, unless the header has the flag set to 
#  suppress acknowledgements. An acknowledgement from the platform has the same 
#  format as the incoming message. The timestamp is the same as the message 
#  being acknowledged, and the payload is a two byte result code. If no issues 
#  were found in the command or request, the result code will be two empty bytes 
#  (no bits set). Otherwise, the result code will be as described below.    \n\n
#
#  The second result code byte is currently unused. Length & checksum are 
#  recalculated to match the new two byte payload.
#
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'>
#  <td><b>7</b></td><td><b>6</b></td><td><b>5</b></td><td><b>4</b></td>
#  <td><b>3</b></td><td><b>2</b></td><td><b>1</b></td><td><b>0</b></td></tr>
#  <tr><td>-</td><td>Too Many Message Types</td><td>Frequency too High</td>
#  <td>No Bandwidth</td><td>Out of Range</td><td>Bad Format</td>
#  <td>Type Not Supported</td><td>Bad Checksum</td></tr>
#  <tr style='background-color: #B3B3B3'>
#  <td><b>15</b></td><td><b>14</b></td><td><b>13</b></td><td><b>12</b></td>
#  <td><b>11</b></td><td><b>10</b></td><td><b>9</b></td><td><b>8</b></td></tr>
#  <tr><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td>
#  <td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#  ---------------------------------
#  | - | Q | H | B | R | F | T | C |
#  |---+---+---+---+---+---+---+---|
#  | - | - | - | - | - | - | - | - |
#  ---------------------------------
#  @endmanonly                                                                \n
#
#  <i>[0] Bad checksum (C)</i>                                                \n
#
#  The command or request was received, but the checksum is incorrect.      \n\n
#
#  <i>[1] Type not supported (T)</i>                                          \n
#
#  The platform configuration being used does not support this command or 
#  request.                                                                 \n\n
#
#  <i>[2] Bad format (F)</i>                                                  \n
#
#  (commands only) The command or request is in an incorrect format.        \n\n
#
#  <i>[3] Out of range (R)</i>                                                \n
#
#  One or more of the parameters in the payload are out of range of the 
#  acceptable values for the command.                                       \n\n
#
#  <i>[4] No bandwidth (B)</i>                                                \n
#
#  (requests only) There is not enough bandwidth remaining on the communication 
#  line to add this subscription.                                           \n\n
#
#  <i>[5] Frequency too high (H)</i>                                          \n
#
#  (requests only) The desired subscription frequency is too high.          \n\n
#
#  <i>[6] Too many message types (Q)</i>                                      \n
#
#  (requests only) Too many different message types are being subscribed to.
#


## Horizon Message Payload - Acknowledgment 
#
#  Represents the payload of a standard acknowledgment message.
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section Acknowledgments
#  @copydoc acknowledgments
#
#  @pydoc
class HorizonPayload_Ack(HorizonPayload):
    """Horizon Message Payload - Acknowledgment"""
    
    
    
    ## Create A Horizon Message Payload - Acknowledgment 
    #
    #  Constructor for the Horizon Message Payload - Acknowledgment  Class.   \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_Ack(bad_checksum,raw=None,...)                        \n
    #    Create an acknowledgment message payload to send.                    \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_Ack(raw,timestamp)                                    \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  bad_bandwidth  Not enough bandwidth
    #  @param  bad_checksum   Message had a bad checksum
    #  @param  bad_code       Message type is unsupported
    #  @param  bad_code_count Too many subscription types
    #  @param  bad_format     Message format is bad
    #  @param  bad_frequency  Subscription frequency is too high
    #  @param  bad_values     Message value(s) are out of range
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, bad_bandwidth = False, bad_checksum = False, 
                 bad_code = False, bad_code_count = False, bad_format = False, 
                 bad_frequency = False, bad_values = False, 
                 store_error = False, raw = None, timestamp = 0, 
                 version = tuple([-1,0])):
        """Create A Horizon Acknowledgment Message Payload"""
        
        # Class Variables
        ## Not enough Bandwidth
        self._band = False
        ## Bad Checksum
        self._checksum = False
        ## Unsupported Type
        self._code = False
        ## Too Many Subscription Types
        self._code_count = False
        ## Invalid Format
        self._format = False
        ## Frequency too High
        self._frequency = False
        ## Value(s) are out of Range
        self._range = False
        
        # Create Constructor
        if raw == None:
            data = 0x0000
            self._checksum = bad_checksum
            if bad_checksum: data |= 0x0001
            self._code = bad_code
            if bad_code: data |= 0x0002
            self._format = bad_format
            if bad_format: data |= 0x0004
            self._range = bad_values
            if bad_values: data |= 0x0008
            self._band = bad_bandwidth
            if bad_bandwidth: data |= 0x0010
            # @deprecated
            if version[0] == 0 and version[1] < 5:
                if bad_frequency or bad_code_count:
                    logger.warning("%s: bad_frequency and bad_code_count "\
                                   "requires Horizon v0.5+!"\
                                     % self.__class__.__name__)
            # current
            else:
                self._frequency = bad_frequency
                if bad_frequency: data |= 0x0020
                self._code_count = bad_code_count
                if bad_code_count: data |= 0x0040
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, 
                                    raw = utils.from_unsigned_short(data), 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 2:
                logger.warning("%s: Bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Invalid length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp) 
            
            # Verify Unused Space
            if utils.to_unsigned_short(raw) & 0xFF80 != 0: 
                logger.warning("%s: Bad format!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Invalid format!")
                if not store_error: raise self.error  
                else: return
            
            # Extract checksum
            if utils.to_unsigned_short(raw) & 0x0001 > 0:
                self._checksum = True
            logger.debug("%s: bad checksum: %d" % (self.__class__.__name__, 
                     self._checksum))
            
            # Extract type
            if utils.to_unsigned_short(raw) & 0x0002 > 0:
                self._code = True
            logger.debug("%s: bad code: %d" % (self.__class__.__name__, 
                     self._code))
            
            # Extract format
            if utils.to_unsigned_short(raw) & 0x0004 > 0:
                self._format = True
            logger.debug("%s: bad format: %d" % (self.__class__.__name__, 
                     self._format))
            
            # Extract Range
            if utils.to_unsigned_short(raw) & 0x0008 > 0:
                self._range = True
            logger.debug("%s: bad value(s): %d" % (self.__class__.__name__, 
                     self._range))
            
            # Extract bandwidth
            if utils.to_unsigned_short(raw) & 0x0010 > 0:
                self._band = True
            logger.debug("%s: bad bandwidth: %d" % (self.__class__.__name__, 
                     self._band))
            
            # Extract frequency
            if utils.to_unsigned_short(raw) & 0x0020 > 0:
                self._frequency = True
            logger.debug("%s: bad frequency: %d" % (self.__class__.__name__, 
                     self._frequency))
            
            # Extract code count
            if utils.to_unsigned_short(raw) & 0x0040 > 0:
                self._code_count = True
            logger.debug("%s: bad code count: %d" % (self.__class__.__name__, 
                     self._code_count))
    
   
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Bad Checksum: %d\nBad Code: %d\nBad Format: %d\n"\
               "Bad Values: %d\nBad Bandwidth: %d\nBad Frequency: %d\n"\
               "Bad Code Count: %d" % (self._checksum, self._code, 
                self._format, self._range, self._band, self._frequency, 
                self._code_count)
                
                
    ## Has Checksum Error?
    #
    #  @return has checksum error flag set?
    #
    #  @pydoc
    def has_checksumerror(self):
        """Has Checksum Error?"""
        
        return self._checksum
    
    
    ## Has Message Type Error?
    #
    #  @return has unssuported type error flag set?
    #
    #  @pydoc
    def has_codeerror(self):
        """Has Message Type Error?"""
        
        return self._code
    
    
    ## Has Format Error?
    #
    #  @return has format error flag set?
    #
    #  @pydoc
    def has_formaterror(self):
        """Has Format Error?"""
        
        return self._format
    
    
    ## Has Value Error?
    #
    #  @return has value error flag set?
    #
    #  @pydoc
    def has_valueerror(self):
        """Has Value Error?"""
        
        return self._range
    
    
    ## Has Bandwidth Error?
    #
    #  @return has bandwidth error flag set?
    #
    #  @pydoc
    def has_bandwidtherror(self):
        """Has Bandwidth Error?"""
        
        return self._band
    
    
    ## Has Frequency Error?
    #
    #  @return has frequency error flag set?
    #
    #  @pydoc
    def has_frequencyerror(self):
        """Has Frequency Error?"""
        
        return self._frequency
    
    
    ## Has Code Count Error?
    #
    #  @return has code count error flag set?
    #
    #  @pydoc
    def has_code_counterror(self):
        """Has Code Count Error?"""
        
        return self._code_count
    
    
    # Class Properties
    ## Bad Checksum
    bad_checksum = property(fget=has_checksumerror, doc="Bad Checksum")
    ## Unsupported Message Type
    bad_code = property(fget=has_codeerror, doc="Unsupported Message Type")
    ## Bad Message Format
    bad_format = property(fget=has_formaterror, doc="Bad Message Format")
    ## Bad Values
    bad_values = property(fget=has_valueerror, doc="Bad Values")
    ## Not Enough Bandwidth
    bad_bandwidth = property(fget=has_bandwidtherror, 
                             doc="Not Enough Bandwidth")
    ## Frequency Too High
    bad_frequency = property(fget=has_frequencyerror, 
                             doc="Frequency Too High")
    ## Too Many Subscription Types
    bad_code_count = property(fget=has_code_counterror, 
                             doc="Too Many Subscription Types")



## Horizon Message Payload - Request
#
#  Represents the payload of a common request message
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section Subscriptions
#  @copydoc subscriptions
#
#  @section data Request Data
#  @copydoc request
#
#  @pydoc
class HorizonPayload_Request(HorizonPayload):
    """Horizon Message Payload - Request"""
    
    
    ## Create A Horizon Message Payload - Request
    #
    #  Constructor for the Horizon Message Payload - Request Class.           \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_Request(subscription, raw=None, version, timestamp)   \n
    #    Create a request message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_Request(raw, version, timestamp)                      \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  subscription   Subscription Frequency in Hz
    #                         0 - immediate, 0xFFFF - off
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  verify         Verify the length? (useful for subclasses)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, raw = None, subscription = 0, 
                 timestamp = 0, version = tuple([-1,0]), verify = True):
        """Create A Horizon Message Payload - Request"""
        
        # Class Variables
        ## Subscription Frequency
        self.subscription = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test subscription
            if subscription < 0 or subscription > 0x0FFFF or \
                    (version[0] == 0 and version[1]< 4 and subscription > 0xFF):
                logger.warning("Tried to create %s with invalid subscription!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Invalid subscription!")
                if not store_error: raise self.error  
                else: return
            self.subscription = subscription
            if version[0] == 0 and version[1]< 4:
                data = utils.from_byte(subscription)
            else:
                data = utils.from_unsigned_short(subscription)
            
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if verify and ((version[0] == 0 and version[1]< 4 and len(raw) != 1)
                or (not(version[0] == 0 and version[1]< 4) and len(raw) != 2)):
                logger.warning("Tried to create %s of bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Invalid payload length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Subscription
            if version[0] == 0 and version[1]< 4:
                self.subscription = utils.to_byte(raw)
            else:
                self.subscription = utils.to_unsigned_short(raw)
            logger.debug("%s subscription: %s" % (self.__class__.__name__, 
                     self.subscription))
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Subscription: %s" % self.subscription
                
    
## @defgroup echo Echo
#  @ingroup data_echo
#  \b Description: Blank echo message.                                      \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field | Type | Size | Scale | Range | Units
#  ------+------+------+-------+-------+------
#  -     | -    | -    | -     | -     | -
#  @endmanonly

## Horizon Message Payload - Echo
#
#  Represents the payload of the echo message.
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Echo Data
#  @copydoc echo
#
#  @pydoc
class HorizonPayload_Echo(HorizonPayload_Null):
    """Horizon Message Payload - Echo"""
    
    
    ## Create A Horizon Message Payload - Echo 
    #
    #  Constructor for the Horizon Message Payload - Echo  Class.             \n
    #  Version auto-detection is unsupported.
    #
    #  @see HorizonPayload_Null.__init__
    #
    #  @pydoc
    def __init__(self, raw = None, store_error = False, timestamp= 0, 
                 version = tuple([-1,0])):
        """Create A Horizon Message Payload - Null"""
        
        # Pass on to super-class
        HorizonPayload_Null.__init__(self, raw = raw, version = version, 
                                     store_error = store_error,
                                     timestamp = timestamp)
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Echo"


    
## @defgroup platform_information Platform Information
#  @ingroup data_platform_information
#  \b Description: Information about the specific platform configuration.   \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Model Length</td><td>byte</td><td>1 byte</td><td>-</td><td>-</td>
#  <td>-</td></tr>
#  <tr><td>Model</td><td>ASCII</td><td>n bytes</td><td>-</td><td>-</td>
#  <td>-</td></tr>
#  <tr><td>Revision</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Serial</td><td>unsigned int</td><td>4 bytes</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field        | Type         | Size    | Scale | Range | Units
#  -------------+--------------+---------+-------+-------+------
#  Model Length | byte         | 1 byte  | -     | -     | -
#  Model        | ASCII        | n bytes | -     | -     | -
#  Revision     | byte         | 1 byte  | -     | -     | -
#  Serial       | unsigned int | 4 bytes | -     | -     | -
#  @endmanonly                                                                \n
#
#  \b Model \b Length: Total number of characters in the \b model string, 
#  including the null-terminator.                                           \n\n
#
#  \b Model: Null-terminated ASCII string representing the platform model, 
#  where the number of bytes is equal to the preceding \b length field.     \n\n
#
#  \b Revision: The platform model's revision number.                       \n\n
#
#  \b Serial: The platform specific, unique serial number. Obtained at the 
#  time of purchase.

## Horizon Message Payload - Platform Information
#
#  Represents the payload of the data message 'platform information'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Platform Information Data
#  @copydoc platform_information
#
#  @pydoc
class HorizonPayload_PlatformInfo(HorizonPayload):
    """Horizon Message Payload - Platform Information"""
    
    
    ## Create A Horizon Message Payload - Platform Information
    #
    #  Constructor for the Horizon Message Payload - Platform Information
    #  Class.                                                                 \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_PlatformInformation(model, raw=None, version, ...)    \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_PlatformInformation(raw, version, timestamp)          \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  model          The Platform Model
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  revision       The Platform Model Revision Number
    #  @param  serial         The Platform Serial Number
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, passcode = 0, model = '', revision = 0, serial = 0, 
                 store_error = False, raw = None, timestamp = 0, 
                 version = tuple([-1,0])):
        """Create A Horizon Message Payload - Platform Information"""
        
        # Class Variables
        self.passcode = passcode
        self.model = ''
        self.revision = 0
        self.serial = 0x00000000;
        
        # Create Constructor - assume this is a set_platform_info payload
        if raw == None:
            data = utils.from_unsigned_short(self.passcode)
            
            # test model
            if not all(ord(c) < 256 for c in model):
                logger.warning("Tried to create %s with non-ASCII model!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Invalid ASCII model!")
                if not store_error: raise self.error  
                else: return
            if not (len(model) > 0 and ord(model[len(model)-1]) == 0):
                model += '\0'
            if (not (version[0] == 0 and version[1] < 6)) and \
                    (len(model) > 64 or len(model) < 2):
                logger.warning("Tried to create %s with model of bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Model must be 1-63 characters!")
                if not store_error: raise self.error  
                else: return
            self.model = model
            data += utils.from_byte(len(model))
            data += utils.from_ascii(model)
            
            # test revision
            if revision < 0 or revision > 255:
                logger.warning("Tried to create %s with bad revision!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Revision must be 0-255!")
                if not store_error: raise self.error  
                else: return
            self.revision = revision
            data += utils.from_byte(revision)
            
            # test serial
            if serial < 0 or serial > 0xFFFFFFFF:
                logger.warning("Tried to create %s with bad serial!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Serial must be 0-4294967295!")
                if not store_error: raise self.error  
                else: return
            self.serial = serial
            data += utils.from_unsigned_int(serial)
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                         store_error = store_error, 
                                         timestamp = timestamp)
        
        # Parse Constructor
        else:
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version,
                                         store_error = store_error,
                                         timestamp = timestamp)            
            # Extract Model
            self.model = utils.to_ascii(raw[1:-5])
            logger.debug("%s model: %s" % (self.__class__.__name__, self.model))
                
            # Extract Revision
            self.revision = utils.to_byte(raw[-5:-4])
            logger.debug("%s revision: %d" % (self.__class__.__name__, self.revision))
            
            # Extract Serial
            self.serial = utils.to_unsigned_int(raw[-4:])
            logger.debug("%s serial: %d" % (self.__class__.__name__, self.serial))
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Platform Model: %s\nPlatform Model Revision: %d\n"\
               "Platform Serial Number: %08X" % (self.model,
                                                 self.revision,
                                                 self.serial)
    
    
## @defgroup platform_name Platform Name
#  @ingroup set_platform_name data_platform_name
#  \b Description: A human readable name of the platform. 
#  Default is "Clearpath1".                                                 \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Length</td><td>byte</td><td>1 byte</td><td>-</td><td>-</td>
#  <td>-</td></tr>
#  <tr><td>Name</td><td>ASCII</td><td>n bytes</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field  | Type  | Size    | Scale | Range | Units
#  -------+-------+---------+-------+-------+------
#  Length | byte  | 1 byte  | -     | -     | -
#  Name   | ASCII | n bytes | -     | -     | -
#  @endmanonly                                                                \n
#
#  \b Length: Total number of characters in the name, including the 
#  null-terminaor.                                                          \n\n
#
#  \b Name: Null-terminated ASCII string representing the platform name, where
#  the number of bytes is equal to the preceding \b length field.

## Horizon Message Payload - Platform Name
#
#  Represents the payload of the command and data messages 'platform name'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Platform Name Data
#  @copydoc platform_name
#
#  @pydoc
class HorizonPayload_PlatformName(HorizonPayload):
    """Horizon Message Payload - Platform Name"""
    
    
    ## Create A Horizon Message Payload - Platform Name
    #
    #  Constructor for the Horizon Message Payload - Platform Name Class.     \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_PlatformName(name, raw=None, version, timestamp)      \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_PlatformName(raw, version, timestamp)                 \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  name           Platform Name
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, name = 'Clearpath1', raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Platform Name"""
        
        # Class Variables
        ## Platform Name
        self.name = 'Clearpath1'
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test name
            if not all(ord(c) < 256 for c in name):
                logger.warning("Tried to create %s with non-ASCII name!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Invalid ASCII name!")
                if not store_error: raise self.error  
                else: return
            if not (len(name) > 0 and ord(name[len(name)-1]) == 0):
                name += '\0'
            if len(name) > 64 or len(name) < 2:
                logger.warning("Tried to create %s with name of bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Name must be 1-63 characters!")
                if not store_error: raise self.error  
                else: return
            self.name = name
            data += utils.from_byte(len(name))
            data += utils.from_ascii(name)
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw)-1 != raw[:1][0]:
                logger.warning("Tried to create %s with name of bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Name must be 1-63 characters!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Name
            self.name = utils.to_ascii(raw[1:])
            logger.debug("%s name: %s" % (self.__class__.__name__, 
                     self.name))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Name: %s" % self.name
    
    
    
## @defgroup platform_time Platform Time
#  @ingroup set_platform_time
#  \b Description: The platform's internal clock.                           \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Time</td><td>unsigned int</td><td>4 bytes</td><td>-</td><td>-</td>
#  <td>ms</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field | Type         | Size    | Scale | Range | Units
#  ------+--------------+---------+-------+-------+------
#  Time  | unsigned int | 4 bytes | -     | -     | ms
#  @endmanonly                                                                \n
#
#  \b Time: The value of the platform internal clock (little-endian byte order).

## Horizon Message Payload - Platform Time
#
#  Represents the payload of the command message 'platform time'
#  @warning Data should not be modified once created
#
#  @since 0.3
#
#  @section data Platform Time Data
#  @copydoc platform_time
#
#  @pydoc
class HorizonPayload_PlatformTime(HorizonPayload):
    """Horizon Message Payload - Platform Time"""

    
    ## Create A Horizon Message Payload - Platform Time
    #
    #  Constructor for the Horizon Message Payload - Platform Time Class.     \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_PlatformTime(time, raw=None, version, timestamp)      \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_PlatformTime(raw, version, timestamp)                 \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  time           Platform Time (0-4294967295)
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, time = 0, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Platform Time"""
        
        # Class Variables
        ## Platform Time
        self.time = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test time
            if time < 0 or time > 4294967295:
                logger.warning("Tried to create %s with bad time!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Time must be within 50 days!")
                if not store_error: raise self.error  
                else: return
            self.time = time
            data += utils.from_unsigned_int(time)
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 4:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Time
            self.time = utils.to_unsigned_int(raw)
            logger.debug("%s time: %d" % (self.__class__.__name__, 
                     self.time))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Time: %d" % self.time
 

    
## @defgroup firmware_information Firmware Information
#  @ingroup data_firmware_information
#  \b Description: Information about the version of firmware the platform is 
#  running.                                                                 \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Major firmware version</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Minor firmware version</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Major protocol version</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Minor protocol version</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Firmware write time</td><td>unsigned int</td><td>4 bytes</td>
#  <td>-</td><td>-</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field               | Type         | Size    | Scale | Range | Units
#  --------------------+--------------+---------+-------+-------+------
#  Major firmware      | byte         | 1 byte  | -     | -     | -
#  Minor firmware      | byte         | 1 byte  | -     | -     | -
#  Major protocol      | byte         | 1 byte  | -     | -     | -
#  Minor protocol      | byte         | 1 byte  | -     | -     | -
#  Firmware write time | unsigned int | 4 bytes | -     | -     | -
#  @endmanonly                                                                \n
#
#  \b Major \b firmware: The major version number in the platform's 
#  control firmware.                                                        \n\n
#
#  \b Minor \b firmware: The minor version number in the platform's 
#  control firmware.                                                        \n\n
#
#  \b Major \b protocol: The major version number of the protocol 
#  library in use.                                                          \n\n
#
#  \b Minor \b protocol: The minor version number of the protocol 
#  library in use.                                                          \n\n
#
#  \b Write \b time: 4 bytes representing the last time the firmware was 
#  programmed. Little-endian byte order.
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'>
#  <td><b>7</b></td><td><b>6</b></td><td><b>5</b></td><td><b>4</b></td>
#  <td><b>3</b></td><td><b>2</b></td><td><b>1</b></td><td><b>0</b></td></tr>
#  <tr><td>H</td><td>H</td><td>M</td><td>M</td><td>M</td><td>M</td><td>M</td>
#  <td>M</td></tr>
#  <tr style='background-color: #B3B3B3'>
#  <td><b>15</b></td><td><b>14</b></td><td><b>13</b></td><td><b>12</b></td>
#  <td><b>11</b></td><td><b>10</b></td><td><b>9</b></td><td><b>8</b></td></tr>
#  <tr><td>D</td><td>D</td><td>D</td><td>D</td><td>D</td><td>H</td><td>H</td>
#  <td>H</td></tr>
#  <tr style='background-color: #B3B3B3'>
#  <td><b>23</b></td><td><b>22</b></td><td><b>21</b></td><td><b>20</b></td>
#  <td><b>19</b></td><td><b>18</b></td><td><b>17</b></td><td><b>16</b></td></tr>
#  <tr><td>Y</td><td>Y</td><td>Y</td><td>O</td><td>O</td><td>O</td><td>O</td>
#  <td>D</td></tr>
#  <tr style='background-color: #B3B3B3'>
#  <td><b>31</b></td><td><b>30</b></td><td><b>29</b></td><td><b>28</b></td>
#  <td><b>27</b></td><td><b>26</b></td><td><b>25</b></td><td><b>24</b></td></tr>
#  <tr><td>-</td><td>-</td><td>-</td><td>-</td><td>Y</td><td>Y</td><td>Y</td>
#  <td>Y</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#  ---------------------------------
#  | H | H | M | M | M | M | M | M |
#  |---+---+---+---+---+---+---+---|
#  | D | D | D | D | D | H | H | H |
#  |---+---+---+---+---+---+---+---|
#  | Y | Y | Y | O | O | O | O | D |
#  |---+---+---+---+---+---+---+---|
#  | - | - | - | - | Y | Y | Y | Y |
#  ---------------------------------
#  @endmanonly                                                                \n
#
#  <i>[0-5] Minute (M)</i> (0-59)                                             \n
#
#  <i>[6-10] Hour (H)</i> (0-23)                                              \n
#
#  <i>[11-16] Day (D)</i> (1-31)                                              \n
#
#  <i>[17-20] Month (O)</i> (1-12)                                            \n
#
#  <i>[21-27] Year (Y)</i> (2000-2127)


## Horizon Message Payload - Firmware Information
#
#  Represents the payload of the data message 'firmware information'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Firmware Information Data
#  @copydoc firmware_information
#
#  @pydoc
class HorizonPayload_FirmwareInfo(HorizonPayload):
    """Horizon Message Payload - Firmare Information"""
    
    ## Create A Horizon Message Payload - Firmware Information
    #
    #  Constructor for the Horizon Message Payload - Firmware Information
    #  Class.                                                                 \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_FirmwareInformation(firmware, raw=None, version,...)  \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_FirmwareInformation(raw, version, timestamp)          \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is supported.
    #
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  firmware       Firmware version (major,minor)
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @param  written        Time & Date written (year: 2000-2127)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, firmware = tuple([1,0]), raw = None, 
                 timestamp = 0, version = tuple([-1,0]), 
                 written = datetime.datetime(2000,1,1,0,0)):
        """Create A Horizon Message Payload - Firmware Information"""
        
        # Class Variables
        ## Firmware Version
        self._firmware = tuple([1,0])
        ## Date Written
        self._written = datetime.datetime(2000,1,1,0,0)
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test firmware
            if firmware[0] > 255 or firmware[0] < 0 or firmware[1] > 255 or \
                    firmware[1] < 0:
                logger.warning("Tried to create %s with bad firmware version!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Invalid firmware version!")
                if not store_error: raise self.error  
                else: return
            self._firmware = firmware
            data += utils.from_byte(firmware[0])
            data += utils.from_byte(firmware[1])
            
            # test date written
            if written.year > 2127 or written.year < 2000:
                logger.warning("Tried to create %s with bad written year!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Invalid write date!")
                if not store_error: raise self.error  
                else: return
            self._written = written
            time = 0xFFFFFFFF
            time &= ((written.year - 2000) & 0x7F) << 21
            time |= ((written.month - 1) & 0x0F) << 17
            time |= ((written.day - 1) & 0x3F) << 11
            time |= (written.hour & 0x1F) << 6
            time |= (written.minute & 0x3F)
            data += utils.from_unsigned_int(time)
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Insert version
            data.insert(2, utils.from_byte(version[0])[0])
            data.insert(2, utils.from_byte(version[1])[0])
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 8:
                logger.warning("Tried to create %s from bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Extract / Verify Version
            v = tuple([utils.to_byte(raw[2:3]),
                                utils.to_byte(raw[3:4])])
            self._version = v
            logger.debug("%s version: %d.%d" % (self.__class__.__name__, 
                     self._version[0], self._version[1]))
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Firmware Version
            self._firmware = tuple([utils.to_byte(raw[0:1]),
                                utils.to_byte(raw[1:2])])
            logger.debug("%s firmware: %d.%d" % (self.__class__.__name__, 
                     self._firmware[0], self._firmware[1]))
            
            # Extract Write Time
            time = utils.to_unsigned_int(raw[4:8])
            year = 2000 + ((time >> 21) & 0x7F)
            month = 1 + ((time >> 17) & 0x0F)
            day = 1 + ((time >> 11) & 0x3F)
            hour = ((time >> 6) & 0x1F)
            minute = ((time) & 0x3F)
            try:
                self._written = datetime.datetime(year,month,day,hour,minute)
            except ValueError as ex:
                self.error = ValueError(ex)
                if not store_error: raise self.error
                else: return
            logger.debug("%s write: %s" % (self.__class__.__name__, 
                     self._written))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Firmware: %d.%d\nProtocol: %d.%d\nWrite: %s" % (
                self._firmware[0], self._firmware[1], self._version[0], 
                self._version[1], self._written)
                
                
    ## Get Firmware
    #
    #  @return the firmware version
    #
    #  @pydoc
    def get_firmware(self):
        """Get Firmware"""
        
        return self._firmware
                
                
    ## Get Write Date
    #
    #  @return the date written
    #
    #  @pydoc
    def get_written(self):
        """Get Write Date"""
        
        return self._written
    
    
    # Class Properties
    ## Firmware Version
    firmware = property(fget=get_firmware, doc="Firmware Version")
    ## Write Date
    written = property(fget=get_written, doc="Write Date")



    
## @defgroup system_status System Status
#  @ingroup data_system_status
#  \b Description: General system status data – voltage, current, and 
#  temperature at varying points within the platform. The location and number 
#  of each internal sensor is platform dependent.                           \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Uptime</td><td>unsigned int</td><td>4 bytes</td><td>-</td>
#  <td>-</td><td>ms</td></tr>
#  <tr><td>Voltage Count</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Voltages</td><td>signed short</td><td>2n bytes</td><td>100</td>
#  <td>[-320,320]</td><td>V</td></tr>
#  <tr><td>Current Count</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Currents</td><td>signed short</td><td>2n bytes</td>
#  <td>100</td><td>[-320,320]</td><td>A</td></tr>
#  <tr><td>Temperature Count</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Temperatures</td><td>signed short</td><td>2n bytes</td>
#  <td>100</td><td>[-320,320]</td><td>°C</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field         | Type         | Size     | Scale | Range      | Units
#  --------------+--------------+----------+-------+------------+------
#  Uptime        | unsigned int | 4 bytes  | -     | -          | ms
#  Voltage Count | byte         | 1 byte   | -     | -          | -
#  Voltages      | signed short | 2n bytes | 100   | [-320,320] | V
#  Current Count | byte         | 1 byte   | -     | -          | -
#  Currents      | signed short | 2n bytes | 100   | [-320,320] | A
#  Temperature # | byte         | 1 byte   | -     | -          | -
#  Temperatures  | signed short | 2n bytes | 100   | [-320,320] | °C
#  @endmanonly                                                                \n
#
#  \b Uptime: The amount of time since the controller was turned on (in ms) \n\n
#
#  \b Voltage \b Count: The number of voltage measurements                  \n\n
#
#  \b Voltages: The voltage measurements. Each measurement is two bytes 
#  (little-endian). The list is also little-endian (first voltage received is 
#  for sensor #0, and so on). There are 2n bytes returned for n integrated
#  voltage sensors.                                                         \n\n
#
#  \b Current \b Count: The number of current measurements                  \n\n
#
#  \b Currents: The current measurements. Each measurement is two bytes 
#  (little-endian). The list is also little-endian (first current received is 
#  for sensor #0, and so on). There are 2n bytes returned for n integrated
#  current sensors.                                                         \n\n
#
#  \b Temperature \b Count: The number of temperature measurements          \n\n
#
#  \b Temperatures: The temperature measurements. Each measurement is two 
#  bytes (little-endian). The list is also little-endian (first temperature 
#  received is for sensor #0, and so on). There are 2n bytes returned
#  for n integrated temperature sensors.


## Horizon Message Payload - System Status
#
#  Represents the payload of the data message 'system status'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data System Status Data
#  @copydoc system_status
#
#  @pydoc
class HorizonPayload_SystemStatus(HorizonPayload):
    """Horizon Message Payload - System Status"""
    
    
    ## Create A Horizon Message Payload - System Status
    #
    #  Constructor for the Horizon Message Payload - System Status Class.     \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_SystemStatus(uptime, voltage, raw=None, version,...)  \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_SystemStatus(raw, version, timestamp)                 \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  current        A list of currents [-320A,320A]
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  temperature    A list of temperatures [-320 degC, 320 degC]
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  uptime         System uptime ([0,4294967295] milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @param  voltage        A list of voltages [-320V,320V]
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If raw is invalid
    #
    #  @pydoc
    def __init__(self, uptime = 0, voltage = [], current = [], temperature = [],
                 raw = None, store_error = False, timestamp = 0,
                 version = tuple([-1,0])):
        """Create A Horizon Message Payload - System Status"""
        
        # Class Variables
        self.uptime = 0
        self.currents = []
        self.temperatures = []
        self.voltages = []
        
 
        # Parse Constructor
        if raw != None:
            # Verify Length
            length = 4;
            voltc = 0
            ampc = 0
            tempc = 0
            if len(raw) > length:
                voltc = raw[4]
                length += 1 + voltc*2;
                if len(raw) > length:
                    ampc = raw[length]
                    length += 1 + ampc*2
                    if len(raw) > length:
                        tempc = raw[length]
                        length += 1 + tempc*2
                        if length != len(raw):
                            length = -1
                    else: length = -1
                else: length = -1
            else: length = -1
            if length == -1:
                logger.warning("Tried to create %s with bad length!"\
                                     % self.__class__.__name__)
                self.error = ValueError( 
                            "Measurement counts do not match raw data length!")
                if not store_error: raise self.error  
                else: return
                    
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
                
            # Extract Uptime
            self.uptime = utils.to_unsigned_int(raw[:4])
            logger.debug("%s uptime: %d" % (self.__class__.__name__, 
                                            self.uptime))
                
            # Extract Voltages
            voltc = raw[4]
            for i in range(0,voltc):
                self.voltages.append(
                            utils.to_short(raw[5+i*2:i*2+7])/100.0)
            logger.debug("%s voltages: %s" % (self.__class__.__name__, 
                                              ' '.join(map(str,self.voltages))))
                
            # Extract Currents
            ampc = raw[voltc*2+5]
            for i in range(0,ampc):
                self.currents.append(utils.to_short(
                                        raw[voltc*2+i*2+6:i*2+8+voltc*2])/100.0)
            logger.debug("%s currents: %s" % (self.__class__.__name__, 
                                              ' '.join(map(str,self.currents))))
                
            # Extract Temperatures
            tempc = raw[voltc*2+ampc*2+6]
            for i in range(0,tempc):
                self.temperatures.append(utils.to_short(
                          raw[voltc*2+ampc*2+i*2+7:i*2+9+voltc*2+ampc*2])/100.0)
            logger.debug("%s temperatures: %s" % (self.__class__.__name__, 
                                        ' '.join(map(str,self.temperatures))))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Uptime: %d\nVoltages: %s\nCurrents: %s\nTemperatures: %s" % (
                self.uptime, 'V '.join(map(str,self.voltages)) + 'V', 
                'A '.join(map(str,self.currents)) + 'A',
                '℃ '.join(map(str,self.temperatures)) + ' degC')
    
    
## @defgroup power_status Power Status
#  @ingroup data_power_status
#  \b Description: Returns estimates of each power source's current charge 
#  state, indications of which power sources are in use, and the chemistry and 
#  capacity of each attached power source (if applicable). In some cases, units 
#  will have multiple power sources, of which only a few will be used (in the 
#  case of platforms with hot-swap capability). In other cases, multiple power 
#  sources may be in use simultaneously (in the case of platforms with dedicated
#  drive and control power). Power source locations are platform-dependent. 
#  The total size of the payload is equal to number-of-sources * 5 + 1      \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Uptime</td><td>unsigned int</td><td>4 bytes</td><td>-</td>
#  <td>-</td><td>ms</td></tr>
#  <tr><td>Voltage Count</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Voltages</td><td>signed short</td><td>2n bytes</td><td>100</td>
#  <td>[-320,320]</td><td>V</td></tr>
#  <tr><td>Current Count</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Currents</td><td>signed short</td><td>2n bytes</td>
#  <td>100</td><td>[-320,320]</td><td>A</td></tr>
#  <tr><td>Temperature Count</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Temperatures</td><td>signed short</td><td>2n bytes</td>
#  <td>100</td><td>[-320,320]</td><td>°C</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field        | Type         | Size     | Scale | Range     | Units
#  -------------+--------------+----------+-------+-----------+------
#  Count        | byte         | 1 byte   | -     | -         | -
#  Charges      | signed short | 2n bytes | 100   | [0,100]   | %
#  Capacities   | signed short | 2n bytes | 1     | [0,32000] | W-Hr
#  Descriptions | byte         | n bytes  | -     | -         | -
#  @endmanonly                                                                \n
#
#  \b Count: The number of batteries the platform supports                  \n\n
#
#  \b Charges: An estimate of the battery's state of charge                 \n\n
#
#  \b Capacities: An estimate of the battery's total energy capacity.       \n\n
#
#  \b Description: 
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>7</i></td><td><i>6</i></td><td><i>5</i></td><td><i>4</i></td>
#  <td><i>3</i></td><td><i>2</i></td><td><i>1</i></td><td><i>0</i></td></tr>
#  <tr><td>PRESENT</td><td>IN_USE</td><td>-</td><td>-</td><td>TYPE_3</td>
#  <td>TYPE_2</td><td>TYPE_1</td><td>TYPE_0</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  ---------------------------------
#  | P | U | - | - | T | T | T | T |
#  ---------------------------------
#  @endmanonly                                                                \n
#
#  <i>[0-3] Type (T):</i>                                                     \n
#
#  Power system type.
#  - 0x0: External supply
#  - 0x1: Lead-acid battery
#  - 0x2: Ni-Mh battery
#  - 0x8: Gas engine (for this case, the capacity estimate is units of liters)
#
#  <i>[6] In Use (U):</i>                                                     \n
#
#  The power source is in use                                                 \n
#
#  <i>[7] Present (P):</i>                                                    \n
#
#  The battery in this position is attached. If a platform is not capable of 
#  detecting this, it will return 1.
#


## Horizon Message Payload - Power Status
#
#  Represents the payload of the data message 'power status'
#  @warning Data should not be modified once created
#
#  @since 0.8
#
#  @section data Power Status Data
#  @copydoc power_status
#
#  @pydoc
class HorizonPayload_PowerStatus(HorizonPayload):
    """Horizon Message Payload - Power Status"""
 
    ## Create A Horizon Message Payload - Power Status
    #
    #  Constructor for the Horizon Message Payload - Power Status Class.      \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_SystemStatus(charges, raw=None, version,...)          \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_SystemStatus(raw, version, timestamp)                 \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  charges        List of battery percentages
    #  @param  capacities     List of battery capacities
    #  @param  descriptions   List of tuple([present,in_use,type])
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If raw is invalid
    #
    #  @pydoc
    def __init__(self, charges = [], capacities = [], descriptions = [],
                 raw = None, store_error = False, timestamp = 0,
                 version = tuple([-1,0])):
        """Create A Horizon Message Payload - Power Status"""
        
        # Class Variables
        ## Charge Measurements
        self.charges = []
        ## Capacity Measurements
        self.capacities = []
        ## Battery Descriptions
        self.descriptions = []
        
        # Create Constructor
        if raw == None:
            data = []
            
            # Verify Lengths
            if len(charges) != len(capacities) or \
                    len(capacities) != len(descriptions) or \
                    len(charges) < 1 or len(charges) > 255:
                logger.warning("Tried to create %s with number of batteries!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Number of batteries must be [0,255]!")
                if not store_error: raise self.error  
                else: return
            data = utils.from_byte([len(charges)])
            
            # Verify Charges
            for c in charges:
                if c < 0 or c > 100:
                    logger.warning("Tried to create %s with bad charges!"\
                                 % self.__class__.__name__)
                    self.error = ValueError("Charges must be [0,100]!")
                    if not store_error: raise self.error  
                    else: return
                data += utils.from_short(int(c*100))
            
            # Verify Capacities
            for c in capacities:
                if c < 0 or c > 32000:
                    logger.warning("Tried to create %s with bad capacities!"\
                                 % self.__class__.__name__)
                    self.error = ValueError("Capacities must be [0,32000]!")
                    if not store_error: raise self.error  
                    else: return
                data += utils.from_short(int(c))
            
            # Verify Descriptions
            for d in descriptions:
                if d[2] < 0 or (d[2] > 2 and d[2] != 8):
                    logger.warning("Tried to create %s with bad descriptions!"\
                                 % self.__class__.__name__)
                    self.error = ValueError(
                                    "Description types must be [0|1|2|8]!")
                    if not store_error: raise self.error  
                    else: return
                desc = 0xCF
                if d[0] == False:
                    desc = desc & 0x7F
                if d[1] == False:
                    desc = desc & 0xBF
                desc = desc & (0xFF | (0xFF & d[2]))
                data += utils.from_byte([desc])
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != raw[0]*5 + 1:
                logger.warning("Tried to create %s with bad length!"\
                                     % self.__class__.__name__)
                self.error = ValueError( 
                            "Measurement counts do not match raw data length!")
                if not store_error: raise self.error  
                else: return
                    
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
                
            # Extract Charges
            for i in range(0,raw[0]):
                self.charges.append(
                            utils.to_short(raw[1+i*2:i*2+3])/100.0)
            logger.debug("%s charges: %s" % (self.__class__.__name__, 
                                            '% '.join(map(str,self.charges))))
                
            # Extract Capacities
            for i in range(0,raw[0]):
                self.capacities.append(utils.to_short(
                                    raw[raw[0]*2+i*2+1:i*2+3+raw[0]*2]))
            logger.debug("%s capacities: %s" % (self.__class__.__name__, 
                                        ' '.join(map(str,self.capacities))))
                
            # Extract Descriptions
            for i in range(0,raw[0]):
                desc = utils.to_byte(raw[raw[0]*4+i+1:i+2+raw[0]*4])
                self.descriptions.append(tuple([desc & 0x80 > 0, 
                                                 desc & 0x40 > 0,
                                                 desc & 0x0F]))
            logger.debug("%s descriptions: %s" % (self.__class__.__name__, 
                                        ' '.join(map(str,self.descriptions))))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Charges: %s\nCapacities: %s\nDescriptions: %s" % (
                '% '.join(map(str,self.charges)) + '%', 
                'W-Hr '.join(map(str,self.capacities)) + 'W-Hr',
                ' '.join(map(str,self.descriptions)))
              


## Horizon Message Payload - Processor Status
#
class HorizonPayload_ProcessorStatus(HorizonPayload):
    """Horizon Message Payload - Power Status"""
 
    def __init__(self, raw = None, store_error = False, timestamp = 0,
                 version = tuple([-1,0])):
        """Create A Horizon Message Payload - Processor Status"""
        
        # Class Variables
        ## Charge Measurements
        self.errors = []

        # Create Constructor
        if raw != None:

            # Verify Length
            if len(raw) != raw[0] * 2 + 1:
                logger.warning("Tried to create %s with bad length!"\
                                     % self.__class__.__name__)
                if not store_error: raise self.error  
                else: return
                    
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
                
            # Extract Errors
            for i in range(0,raw[0]):
                self.errors.append(
                    utils.to_short(raw[1+i*2:i*2+3]))
            logger.debug("%s errors: %s" % (self.__class__.__name__, 
                                            '  '.join(map(str, self.errors))))
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Errors: %s\n" % ' '.join(map(str, self.errors))
              

    
## @defgroup safety_status Safety System
#  @ingroup set_safety_status data_safety_status
#  \b Description: The status of the platform safety system.                \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Flags</td><td>unsigned short</td><td>2 bytes</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field | Type           | Size    | Scale | Range | Units
#  ------+----------------+---------+-------+-------+------
#  Flags | unsigned short | 2 bytes | -     | -     | -
#  @endmanonly                                                                \n
#
#  \b Flags: The status for onboard safety system. Refer to the platform 
#  documentation for platform specific flags.
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>7</i></td><td><i>6</i></td><td><i>5</i></td><td><i>4</i></td>
#  <td><i>3</i></td><td><i>2</i></td><td><i>1</i></td><td><i>0</i></td></tr>
#  <tr><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td>
#  <td>-</td><td>E</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  ---------------------------------
#  | - | - | - | - | - | - | - | E |
#  ---------------------------------
#  @endmanonly                                                                \n
#
#  <i>[0] Emergency Stop (E):</i>                                             \n
#
#  Is the platformed in emergency stop mode?


## Horizon Message Payload - Safety System
#
#  Represents the payload of the command and data messages 'safety system'
#  @warning Data should not be modified once created
#
#  @since 0.4
#
#  @section data Safety System Data
#  @copydoc safety_status
#
#  @pydoc
class HorizonPayload_SafetyStatus(HorizonPayload):
    """Horizon Message Payload - Safety System"""
    
    # Class Constants
    ## Emergency Stop flag mask
    EMERGENCY_STOP = 0x0001
    
    
    ## Create A Horizon Message Payload - Safety System
    #
    #  Constructor for the Horizon Message Payload - Safety System Class.     \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_SafetySystem(flags, raw=None, version, timestamp)     \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_SafetySystem(raw, version, timestamp)                 \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  flags          Platform Safety System Flags
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, flags = 0x0000, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Platform Name"""
        
        # Class Variables
        ## Platform Safety System Flags
        self._flags = 0x0000
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test flags
            if flags < 0 or flags > 65535:
                logger.warning("Tried to create %s with invalid flags!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Invalid flags!")
                if not store_error: raise self.error  
                else: return
            self._flags = flags
            data = utils.from_unsigned_short(flags)
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 2:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Bad length!")
                if not store_error: raise self.error
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Flags
            self._flags = utils.to_unsigned_short(raw)
            logger.debug("%s flags: 0x%04X" % (self.__class__.__name__, 
                     self._flags))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Flags: 0x%04X" % self._flags
                
                
    ## Has Emergency Stop Set?
    #
    #  @return is the emergency stop platform safety system flag set
    #
    #  @pydoc
    def has_emergency_stop(self):
        """Has Emergency Stop Set?"""
        
        return (self._flags & self.EMERGENCY_STOP) == self.EMERGENCY_STOP
                
                
    ## Get Flags
    #
    #  @return the platform safety system flags
    #
    #  @pydoc
    def get_flags(self):
        """Get Flags"""
        
        return self._flags
    
    
    # Class Properties
    ## Platform Safety System Flags
    flags = property(fget=get_flags, doc="Platform Safety System Flags")



## Horizon Message Payload - Differential Speed
#
#  Represents the payload of the command and data messages 'differential speed'
#  @warning Data should not be modified once created
#
#  @pydoc
class HorizonPayload_DifferentialSpeed(HorizonPayload):
    """Horizon Message Payload - Differential Speed"""
    
    
    ## Create A Horizon Message Payload - Differential Speed
    #
    #  Constructor for the Horizon Message Payload - Differential Speed.      \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_DifferentialSpeed(l_speed, r_accel, raw=None,...)     \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_DifferentialSpeed(raw, version, timestamp)            \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  left_accel     Left Acceleration (m/s^2)
    #  @param  left_speed     Left Speed (m/s)
    #  @param  right_accel    Right Acceleration (m/s^2)
    #  @param  right_speed    Right Speed (m/s)
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, left_speed = 0, right_speed = 0, left_accel = 0, right_accel = 0, 
                 store_error = False, raw = None, timestamp = 0, 
                 version = tuple([-1,0])):
        """Create A Horizon Message Payload - Differential Speed"""
        
        # Class Variables
        self.left_accel = 0
        self.left_speed = 0
        self.right_accel = 0
        self.right_speed = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test left speed
            if left_speed < -320 or left_speed > 320:
                logger.warning(
                        "Tried to create %s with bad left speed!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Left Speed must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self.left_speed = left_speed
            data = utils.from_short(int(left_speed * 100))
            
            # test right speed
            if right_speed < -320 or right_speed > 320:
                logger.warning(
                        "Tried to create %s with bad right speed!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Right Speed must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self.right_speed = right_speed
            data += utils.from_short(int(right_speed * 100))
            
            # test left acceleration
            if left_accel < 0 or left_accel > 320:
                logger.warning(
                    "Tried to create %s with bad left acceleration!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Left Acceleration must be [0,320]!")
                if not store_error: raise self.error  
                else: return
            self.left_accel = left_accel
            data += utils.from_short(int(left_accel * 100))
            
            # test right acceleration
            if right_accel < 0 or right_accel > 320:
                logger.warning(
                    "Tried to create %s with bad right acceleration!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Right Acceleration must be [0,320]!")
                if not store_error: raise self.error  
                else: return
            self.right_accel = right_accel
            data += utils.from_short(int(right_accel * 100))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 8:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Left Speed
            self.left_speed = utils.to_short(raw[0:2]) / 100.0
            logger.debug("%s left speed: %fm/s" % \
                         (self.__class__.__name__, self.left_speed))
            
            # Extract Right Speed
            self.right_speed = utils.to_short(raw[2:4]) / 100.0
            logger.debug("%s right speed: %fm/s" % \
                         (self.__class__.__name__, self.right_speed))
            
            # Extract Left Acceleration
            self.left_accel = utils.to_short(raw[4:6]) / 100.0
            logger.debug("%s left acceleration: %fm/s^2" % \
                         (self.__class__.__name__, self.left_accel))
            
            # Extract Right Acceleration
            self.right_accel = utils.to_short(raw[6:8]) / 100.0
            logger.debug("%s right acceleration: %fm/s^2" % \
                         (self.__class__.__name__, self.right_accel))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Left Speed: %fm/s\nRight Speed: %fm/s\n"\
               "Left Acceleration: %fm/s^2\nRight Acceleration: %fm/s^2" % (
                    self.left_speed, self.right_speed, self.left_accel, self.right_accel)



    
    

## Horizon Message Payload - Differential Control
#
#  Represents the payload of the command and data messages 'differential 
#  control'
#  @warning Data should not be modified once created
#
#
#  @pydoc
class HorizonPayload_DifferentialControl(HorizonPayload):
    """Horizon Message Payload - Differential Control"""
    
    ## Create A Horizon Message Payload - Differential Control
    #
    #  Constructor for the Horizon Message Payload - Differential Control.    \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_DifferentialControl(l_p, l_d, r_i, raw=None,...)      \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_DifferentialControl(raw, version, timestamp)          \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  l_d            Left derivative constant
    #  @param  l_feed         Left feed-forward constant
    #  @param  l_i            Left integral constant
    #  @param  l_limit        Left integral limit
    #  @param  l_p            Left proportional constant
    #  @param  l_stiction     Left stiction compenstation
    #  @param  r_d            Right derivative constant
    #  @param  r_feed         Right feed-forward constant
    #  @param  r_i            Right integral constant
    #  @param  r_limit        Right integral limit
    #  @param  r_p            Right proportional constant
    #  @param  r_stiction     Right stiction compenstation
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, l_d = 0, l_feed = 0, l_i = 0, l_limit = 0, l_p = 0,
                 l_stiction = 0, r_d = 0, r_feed = 0, r_i = 0, r_limit = 0,
                 r_p = 0, r_stiction = 0, store_error = False, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Differential Control"""
        
        # Class Variables
        ## Left Derivative constant
        self._l_d = 0
        ## Left feed-forward constant
        self._l_f = 0
        ## Left Integral constant
        self._l_i = 0
        ## Left integral Limit
        self._l_l = 0
        ## Left proportional Constant
        self._l_p = 0
        ## Left stiction compensation
        self._l_s = 0
        ## Right Derivative constant
        self._r_d = 0
        ## Right feed-forward constant
        self._r_f = 0
        ## Right Integral constant
        self._r_i = 0
        ## Right integral Limit
        self._r_l = 0
        ## Right proportional Constant
        self._r_p = 0
        ## Right stiction compensation
        self._r_s = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test left P
            if l_p < -320 or l_p > 320:
                logger.warning(
                        "Tried to create %s with bad left P!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Left proportional constant must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self._l_p = l_p
            data = utils.from_short(int(l_p*100))
            
            # test left I
            if l_i < -320 or l_i > 320:
                logger.warning(
                        "Tried to create %s with bad left I!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Left integral constant must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self._l_i = l_i
            data += utils.from_short(int(l_i*100))
            
            # test left D
            if l_d < -320 or l_d > 320:
                logger.warning(
                        "Tried to create %s with bad left D!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Left derivative constant must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self._l_d = l_d
            data += utils.from_short(int(l_d*100))
            
            # test left feed
            if l_feed < -320 or l_feed > 320:
                logger.warning(
                        "Tried to create %s with bad left feed!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Left feed-forward constant must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self._l_f = l_feed
            data += utils.from_short(int(l_feed*100))
            
            # test left Stiction
            if l_stiction < 0 or l_stiction > 100:
                logger.warning(
                        "Tried to create %s with bad left stiction!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Left stiction compensation must be [0,100]!")
                if not store_error: raise self.error  
                else: return
            self._l_s = l_stiction
            data += utils.from_short(int(l_stiction*100))
            
            # test left limit
            if l_limit < 0 or l_limit > 100:
                logger.warning(
                        "Tried to create %s with bad left limit!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Left integral limit must be [0,100]!")
                if not store_error: raise self.error  
                else: return
            self._l_l = l_limit
            data += utils.from_short(int(l_limit*100))
            
            # test right P
            if r_p < -320 or r_p > 320:
                logger.warning(
                        "Tried to create %s with bad right P!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                            "Right proportional constant must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self._r_p = r_p
            data += utils.from_short(int(r_p*100))
            
            # test right I
            if r_i < -320 or r_i > 320:
                logger.warning(
                        "Tried to create %s with bad right I!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Right integral constant must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self._r_i = r_i
            data += utils.from_short(int(r_i*100))
            
            # test right D
            if r_d < -320 or r_d > 320:
                logger.warning(
                        "Tried to create %s with bad right D!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Right derivative constant must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self._r_d = r_d
            data += utils.from_short(int(r_d*100))
            
            # test right feed
            if r_feed < -320 or r_feed > 320:
                logger.warning(
                        "Tried to create %s with bad right feed!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                             "Right feed-forward constant must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self._r_f = r_feed
            data += utils.from_short(int(r_feed*100))
            
            # test right Stiction
            if r_stiction < 0 or r_stiction > 100:
                logger.warning(
                        "Tried to create %s with bad left stiction!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Right stiction compensation must be [0,100]!")
                if not store_error: raise self.error  
                else: return
            self._r_s = r_stiction
            data += utils.from_short(int(r_stiction*100))
            
            # test right limit
            if r_limit < 0 or r_limit > 100:
                logger.warning(
                        "Tried to create %s with bad right limit!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Right integral limit must be [0,100]!")
                if not store_error: raise self.error  
                else: return
            self._r_l = r_limit
            data += utils.from_short(int(r_limit*100))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 24:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Left P
            self._l_p = utils.to_short(raw[0:2])/100.0
            logger.debug("%s left P: %f" % \
                         (self.__class__.__name__, self._l_p))
            
            # Extract Left I
            self._l_i = utils.to_short(raw[2:4])/100.0
            logger.debug("%s left I: %f" % \
                         (self.__class__.__name__, self._l_i))
            
            # Extract Left D
            self._l_d = utils.to_short(raw[4:6])/100.0
            logger.debug("%s left D: %f" % \
                         (self.__class__.__name__, self._l_d))
            
            # Extract Left feed
            self._l_f = utils.to_short(raw[6:8])/100.0
            logger.debug("%s left feed: %f" % \
                         (self.__class__.__name__, self._l_f))
            
            # Extract Left stiction
            self._l_s = utils.to_short(raw[8:10])/100.0
            logger.debug("%s left stiction: %f" % \
                         (self.__class__.__name__, self._l_s))
            
            # Extract Left limit
            self._l_l = utils.to_short(raw[10:12])/100.0
            logger.debug("%s left limit: %f" % \
                         (self.__class__.__name__, self._l_l))
            
            # Extract Right P
            self._r_p = utils.to_short(raw[12:14])/100.0
            logger.debug("%s right P: %f" % \
                         (self.__class__.__name__, self._r_p))
            
            # Extract Right I
            self._r_i = utils.to_short(raw[14:16])/100.0
            logger.debug("%s right I: %f" % \
                         (self.__class__.__name__, self._r_i))
            
            # Extract Right D
            self._r_d = utils.to_short(raw[16:18])/100.0
            logger.debug("%s right D: %f" % \
                         (self.__class__.__name__, self._r_d))
            
            # Extract Right feed
            self._r_f = utils.to_short(raw[18:20])/100.0
            logger.debug("%s right feed: %f" % \
                         (self.__class__.__name__, self._r_f))
            
            # Extract Right stiction
            self._r_s = utils.to_short(raw[20:22])/100.0
            logger.debug("%s right stiction: %fm/s" % \
                         (self.__class__.__name__, self._r_s))
            
            # Extract Right limit
            self._r_l = utils.to_short(raw[22:24])/100.0
            logger.debug("%s right limit: %f" % \
                         (self.__class__.__name__, self._r_l))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Left P: %f\nLeft I: %f\nLeft D: %f\nLeft feed-forward: %f\n"\
               "Left stiction compensation: %f\nLeft integral limit: %f\n"\
               "Right P: %f\nRight I: %f\nRight D: %f\nRight feed-forward: %f"\
               "\nRight stiction compensation: %f\nRight integral limit: %f\n"\
                % (
            self._l_p, self._l_i, self._l_d, self._l_f, self._l_s, self._l_l,
            self._r_p, self._r_i, self._r_d, self._r_f, self._r_s, self._r_l)
                
                
    ## Get Left Proportional Constant
    #
    #  @return the left proportional constant
    #
    #  @pydoc
    def get_left_proportion(self):
        """Get Left Proportional Constant"""
        
        return self._l_p
                
                
    ## Get Left Integral Constant
    #
    #  @return the left integral constant
    #
    #  @pydoc
    def get_left_integral(self):
        """Get Left Integral Constant"""
        
        return self._l_i
                
                
    ## Get Left Derivative Constant
    #
    #  @return the left derivative constant
    #
    #  @pydoc
    def get_left_derivative(self):
        """Get Left Derivative Constant"""
        
        return self._l_d
                
                
    ## Get Left Feed-Forward Constant
    #
    #  @return the left feed-forward constant
    #
    #  @pydoc
    def get_left_feed_forward(self):
        """Get Left Feed-Forward Constant"""
        
        return self._l_f
                
                
    ## Get Left Stiction Compensation
    #
    #  @return the left stiction compensation
    #
    #  @pydoc
    def get_left_stiction(self):
        """Get Left Stiction Compensation"""
        
        return self._l_s
                
                
    ## Get Left Integral Limit
    #
    #  @return the left integral limit
    #
    #  @pydoc
    def get_left_limit(self):
        """Get Left Integral Limit"""
        
        return self._l_l
                
                
    ## Get Right Proportional Constant
    #
    #  @return the right proportional constant
    #
    #  @pydoc
    def get_right_proportion(self):
        """Get Right Proportional Constant"""
        
        return self._r_p
                
                
    ## Get Right Integral Constant
    #
    #  @return the right integral constant
    #
    #  @pydoc
    def get_right_integral(self):
        """Get Right Integral Constant"""
        
        return self._r_i
                
                
    ## Get Right Derivative Constant
    #
    #  @return the right derivative constant
    #
    #  @pydoc
    def get_right_derivative(self):
        """Get Right Derivative Constant"""
        
        return self._r_d
                
                
    ## Get Right Feed-Forward Constant
    #
    #  @return the right feed-forward constant
    #
    #  @pydoc
    def get_right_feed_forward(self):
        """Get Right Feed-Forward Constant"""
        
        return self._r_f
                
                
    ## Get Right Stiction Compensation
    #
    #  @return the right stiction compensation
    #
    #  @pydoc
    def get_right_stiction(self):
        """Get Right Stiction Compensation"""
        
        return self._r_s
                
                
    ## Get Right Integral Limit
    #
    #  @return the right integral limit
    #
    #  @pydoc
    def get_right_limit(self):
        """Get Right Integral Limit"""
        
        return self._r_l
    
    
    # Class Properties
    ## Left Proportional Constant
    left_proportion = property(fget=get_left_proportion, 
                               doc="Left Proportional Constant")
    ## Left Integral Constant
    left_integral = property(fget=get_left_integral, 
                               doc="Left Integral Constant")
    ## Left Derivative Constant
    left_derivative = property(fget=get_left_derivative, 
                               doc="Left Derivative Constant")
    ## Left Feed-Forward Constant
    left_feed_forward = property(fget=get_left_feed_forward, 
                               doc="Left Feed-Forward Constant")
    ## Left Stiction Compensation
    left_stiction = property(fget=get_left_stiction, 
                               doc="Left Stiction Compensation")
    ## Left Integral Limit
    left_limit = property(fget=get_left_limit, 
                               doc="Left Integral Limit")
    ## Right Proportional Constant
    right_proportion = property(fget=get_right_proportion, 
                               doc="Right Proportional Constant")
    ## Right Integral Constant
    right_integral = property(fget=get_right_integral, 
                               doc="Right Integral Constant")
    ## Right Derivative Constant
    right_derivative = property(fget=get_right_derivative, 
                               doc="Right Derivative Constant")
    ## Right Feed-Forward Constant
    right_feed_forward = property(fget=get_right_feed_forward, 
                               doc="Right Feed-Forward Constant")
    ## Right Stiction Compensation
    right_stiction = property(fget=get_right_stiction, 
                               doc="Right Stiction Compensation")
    ## Right Integral Limit
    right_limit = property(fget=get_right_limit, 
                               doc="Right Integral Limit")


    

## Horizon Message Payload - Differential Output
#
#  Represents the payload of the command and data messages 'differential motors'
#  @warning Data should not be modified once created
#
#  @section data Differential Motors Data
#  @copydoc differential_motors
#
#  @pydoc
class HorizonPayload_DifferentialOutput(HorizonPayload):
    """Horizon Message Payload - Differential Motors"""
    
    
    ## Create A Horizon Message Payload - Differential Motors
    #
    #  Constructor for the Horizon Message Payload - Differential Speed.      \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_DifferentialMotors(left, right, raw=None,...)         \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_DifferentialMotors(raw, version, timestamp)           \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  left           Left Motor Output
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  right          Right Motor Output
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, left = 0, right = 0, store_error = False, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Differential Motors"""
        
        # Class Variables
        self.left = 0
        self.right = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test left
            if left < -100 or left > 100:
                logger.warning(
                        "Tried to create %s with bad left motor!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Left Motor must be [-100,100]!")
                if not store_error: raise self.error  
                else: return
            self.left = left
            data = utils.from_short(int(left*100))
            
            # test right
            if right < -100 or right > 100:
                logger.warning(
                        "Tried to create %s with bad right motor!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Right Motor must be [-100,100]!")
                if not store_error: raise self.error  
                else: return
            self.right = right
            data += utils.from_short(int(right*100))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 4:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Left
            self.left = utils.to_short(raw[0:2])/100.0
            logger.debug("%s left motor: %f%%" % \
                         (self.__class__.__name__, self.left))
            
            # Extract Right
            self.right = utils.to_short(raw[2:4])/100.0
            logger.debug("%s right motor: %f%%" % \
                         (self.__class__.__name__, self.right))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Left Motor: %f%%\nRight Motor: %f%%" % (self.left, self.right)



## @defgroup ackermann_servos Ackermann Servos
#  @ingroup set_ackermann_servos data_ackermann_servos 
#  @ingroup data_ackermann_servos_position
#  \b Description: The servo positioning setpoints for an Ackermann-based 
#  platform.                                                                \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Steering</td><td>signed short</td><td>2 bytes</td><td>100</td>
#  <td>[-100,100]</td><td>%</td></tr>
#  <tr><td>Throttle</td><td>signed short</td><td>2 bytes</td><td>100</td>
#  <td>[-100,100]</td><td>%</td></tr>
#  <tr><td>Brake</td><td>signed short</td><td>2 bytes</td><td>100</td>
#  <td>[0,100]</td><td>%</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field    | Type         | Size    | Scale | Range      | Units
#  ---------+--------------+---------+-------+------------+------
#  Steering | signed short | 2 bytes | 100   | [-100,100] | %
#  Throttle | signed short | 2 bytes | 100   | [-100,100] | %
#  Brake    | signed short | 2 bytes | 100   | [0,100]    | %
#  @endmanonly                                                                \n
#
#  \b Steering: The current steering servo position setpoint as a percentage of
#  the maximum travel, where 0 is straight ahead. A positive value corresponds
#  to a left turn.                                                          \n\n
#
#  \b Throttle: The current throttle servo position setpoint as a percentage of
#  the maximum travel, where 0 is closed, 100 is full forward, and -100 is full
#  reverse.                                                                 \n\n
#
#  \b Brake: The current brake servo position setpoint as a percentage of the
#  maximum travel, where 0 is completely released and 100 is completely engaged.


## Horizon Message Payload - Ackermann Servos
#
#  Represents the payload of the command and data messages 'ackermann servos'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Ackermann Servos Data
#  @copydoc ackermann_servos
#
#  @pydoc
class HorizonPayload_AckermannOutput(HorizonPayload):
    """Horizon Message Payload - Ackermann Servos"""
    
    
    ## Create A Horizon Message Payload - Ackermann Servos
    #
    #  Constructor for the Horizon Message Payload - Ackermann Servos.        \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_AckermannServos(steering, throttle, raw=None,...)     \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_AckermannServos(raw, version, timestamp)              \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  brake          The brake position
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  steering       The steering position
    #  @param  throttle       The throttle position
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, steering = 0, throttle = 0, brake = 0, 
                 store_error = False, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Ackermann Servos"""
        
        # Class Variables
        ## Brake
        self._brake = 0
        ## Steering
        self._steering = 0
        ## Throttle
        self._throttle = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test steering
            if steering < -100 or steering > 100:
                logger.warning(
                        "Tried to create %s with bad steering!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Steering must be [-100,100]!")
                if not store_error: raise self.error  
                else: return
            self._steering = steering
            data = utils.from_short(int(steering*100))
            
            # test throttle
            if throttle < -100 or throttle > 100:
                logger.warning(
                        "Tried to create %s with bad throttle!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Throttle must be [-100,100]!")
                if not store_error: raise self.error  
                else: return
            self._throttle = throttle
            data += utils.from_short(int(throttle*100))
            
            # test brake
            if brake < 0 or brake > 100:
                logger.warning(
                        "Tried to create %s with bad brake!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Brake must be [0,100]!")
                if not store_error: raise self.error  
                else: return
            self._brake = brake
            data += utils.from_short(int(brake*100))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 6:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Steering
            self._steering = utils.to_short(raw[0:2])/100.0
            logger.debug("%s steering: %f%%" % \
                         (self.__class__.__name__, self._steering))
            
            # Extract Throttle
            self._throttle = utils.to_short(raw[2:4])/100.0
            logger.debug("%s throttle: %f%%" % \
                         (self.__class__.__name__, self._throttle))
            
            # Extract Brake
            self._brake = utils.to_short(raw[4:6])/100.0
            logger.debug("%s brake: %f%%" % \
                         (self.__class__.__name__, self._brake))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Steering: %f%%\nThrottle: %f%%\nBrake: %f%%" % (
                                    self._steering, self._throttle, self._brake)
                
                
    ## Get Steering
    #
    #  @return the steering position
    #
    #  @pydoc
    def get_steering(self):
        """Get Steering"""
        
        return self._steering
                
                
    ## Get Throttle
    #
    #  @return the throttle
    #
    #  @pydoc
    def get_throttle(self):
        """Get Throttle"""
        
        return self._throttle
                
                
    ## Get Brake
    #
    #  @return the brake
    #
    #  @pydoc
    def get_brake(self):
        """Get Brake"""
        
        return self._brake
    
    
    # Class Properties
    ## Steering
    steering = property(fget=get_steering, doc="Steering")
    ## Throttle
    throttle = property(fget=get_throttle, doc="Throttle")
    ## Brake
    brake = property(fget=get_brake, doc="Brake")



## @defgroup velocity Velocity
#  @ingroup set_velocity data_velocity
#  \b Description: The desired vehicle velocity.                            \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Translational velocity</td><td>signed short</td><td>2 bytes</td>
#  <td>100</td><td>[-320,320]</td><td>m/s</td></tr>
#  <tr><td>Rotational velocity</td><td>signed short</td><td>2 bytes</td>
#  <td>100</td><td>[-320,320]</td><td>rad/s</td></tr>
#  <tr><td>Translational acceleration</td><td>signed short</td><td>2 bytes</td>
#  <td>100</td><td>[0,320]</td><td>m/s²</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field             | Type         | Size    | Scale | Range      | Units
#  ------------------+--------------+---------+-------+------------+------
#  Velocity          | signed short | 2 bytes | 100   | [-320,320] | m/s
#  Rotational Vel... | signed short | 2 bytes | 100   | [-320,320] | rad/s
#  Acceleration      | signed short | 2 bytes | 100   | [0,320]    | m/s²
#  @endmanonly                                                                \n
#
#  \b Translational \b Velocity: The current desired translational velocity of 
#  the vehicle.                                                             \n\n
#
#  \b Rotational \b Velocity: The current desired rotational velocity of the 
#  vehicle. A positive value corresponds to a left turn.                    \n\n
#
#  \b Translational \b Acceleration: The current desired magnitude of the 
#  translational acceleration of the vehicle.


## Horizon Message Payload - Velocity
#
#  Represents the payload of the command and data messages 'velocity'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Velocity Data
#  @copydoc velocity
#
#  @pydoc
class HorizonPayload_Velocity(HorizonPayload):
    """Horizon Message Payload - Velocity"""
    
    
    ## Create A Horizon Message Payload - Velocity
    #
    #  Constructor for the Horizon Message Payload - Velocity.                \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_Velocity(trans, rot, accel, raw=None, ...)            \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_Velocity(raw, version, timestamp)                     \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  accel          The desired translational acceleration (m/s^2)
    #  @param  flags          0x02 - Automatic Transmission, 0x01 - Dynamic
    #                         Compensation, 0x03 - Both, 0x00 - None.
    #                         DEPRECATED: not in Horizon as of v0.4.
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  rot            The desired rotational speed (rad/s)
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  trans          The desired translational speed (m/s)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, trans = 0, rot = 0, accel = 0, store_error = False, 
                 raw = None, timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Velocity"""
        
        # Class Variables
        ## Translational Acceleration
        self.accel = 0
        ## Rotational speed
        self.rot = 0
        ## Translational speed
        self.trans = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test translation
            if trans < -320 or trans > 320:
                logger.warning(
                        "Tried to create %s with bad translational speed!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Translational Speed must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self.trans = trans
            data += utils.from_short(int(trans*100))
            
            # test rotation
            if rot < -320 or rot > 320:
                logger.warning(
                            "Tried to create %s with bad rotational speed!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Rotational Speed must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self.rot = rot
            data += utils.from_short(int(rot*100))
            
            # test acceleration
            if accel < 0 or accel > 320:
                logger.warning(
                    "Tried to create %s with bad translational acceleration!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Translational Acceleration must be [0,320]!")
                if not store_error: raise self.error  
                else: return
            self.accel = accel
            data += utils.from_short(int(accel*100))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error, 
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if (len(raw) != 6):
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Translational
            self.trans = utils.to_short(raw[0:2]) / 100.0
            logger.debug("%s: translational speed: %fm/s" % \
                         (self.__class__.__name__, self.trans))
            
            # Extract Rotational
            self.rot = utils.to_short(raw[2:4]) / 100.0
            logger.debug("%s: rotational speed: %frad/s" % \
                         (self.__class__.__name__, self.rot))
            
            # Extract Acceleration
            self.accel = utils.to_short(raw[4:6]) / 100.0
            logger.debug("%s: translational acceleration: %fm/s^2" % \
                         (self.__class__.__name__, self.accel))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Translational Speed: %fm/s\nRotational Speed: %frad/s\n"\
               "Translational Acceleration: %fm/s^2" % (self.trans, 
                                                        self.rot, self.accel)


    
## @defgroup turn Turn
#  @ingroup set_turn data_turn
#  \b Description: The desired vehicle turn.                                \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Translational velocity</td><td>signed short</td><td>2 bytes</td>
#  <td>100</td><td>[-32,32]</td><td>m/s</td></tr>
#  <tr><td>Turn radius</td><td>signed short</td><td>2 bytes</td>
#  <td>100</td><td>[-320,320]</td><td>m</td></tr>
#  <tr><td>Translational acceleration</td><td>signed short</td><td>2 bytes</td>
#  <td>100</td><td>[0,320]</td><td>m/s²</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field        | Type         | Size    | Scale | Range      | Units
#  -------------+--------------+---------+-------+------------+------
#  Velocity     | signed short | 2 bytes | 100   | [-320,320] | m/s
#  Turn Radius  | signed short | 2 bytes | 100   | [-320,320] | m
#  Acceleration | signed short | 2 bytes | 100   | [0,320]    | m/s²
#  @endmanonly                                                                \n
#
#  \b Translational \b Velocity: The current desired translational velocity of 
#  the vehicle.                                                             \n\n
#
#  \b Turn \b Radius: The current desired turn radius of the vehicle. A 
#  positive value corresponds to a left turn.                               \n\n
#
#  \b Translational \b Acceleration: The current desired magnitude of the 
#  translational acceleration of the vehicle.


## Horizon Message Payload - Turn
#
#  Represents the payload of the command and data messages 'turn'
#  @warning Data should not be modified once created
#
#  @since 0.3
#
#  @section data Turn Data
#  @copydoc turn
#
#  @pydoc
class HorizonPayload_Turn(HorizonPayload):
    """Horizon Message Payload - Turn"""
    
    
    ## Create A Horizon Message Payload - Turn
    #
    #  Constructor for the Horizon Message Payload - Turn.                    \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_Turn(trans, turn, accel, raw=None, ...)               \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_Turn(raw, version, timestamp)                         \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  accel          The desired translational acceleration (m/s^2)
    #  @param  flags          0x02 - Automatic Transmission, 0x01 - Dynamic
    #                         Compensation, 0x03 - Both, 0x00 - None.
    #                         DEPRECATED: not in Horizon as of v0.4.
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  trans          The desired translational speed (m/s)
    #  @param  turn           The desired turn radius
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, trans = 0, turn = 0, accel = 0, store_error = False, 
                 raw = None, timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Turn"""
        
        # Class Variables
        self.accel = 0
        self.turn = 0
        self.trans = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test translation
            if trans < -320 or trans > 320:
                logger.warning(
                        "Tried to create %s with bad translational speed!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Translational Speed must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self.trans = trans
            data += utils.from_short(int(trans*100))
            
            # test turn
            if turn < -320 or turn > 320:
                logger.warning(
                            "Tried to create %s with bad turn radius!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Turn Radius must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self.turn = turn
            data += utils.from_short(int(turn*100))
            
            # test acceleration
            if accel < 0 or accel > 320:
                logger.warning(
                    "Tried to create %s with bad translational acceleration!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Translational Acceleration must be [0,320]!")
                if not store_error: raise self.error  
                else: return
            self.accel = accel
            data += utils.from_short(int(accel*100))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw=data, 
                                    store_error=store_error,
                                    timestamp=timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if (len(raw) != 6):
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Translational
            self.trans = utils.to_short(raw[0:2]) / 100.0
            logger.debug("%s translational speed: %fm/s" % \
                         (self.__class__.__name__, self.trans))
            
            # Extract Turn Radius
            self.turn = utils.to_short(raw[2:4]) / 100.0
            logger.debug("%s turn radius: %fm" % \
                         (self.__class__.__name__, self.turn))
            
            # Extract Acceleration
            self.accel = utils.to_short(raw[4:6]) / 100.0
            logger.debug("%s translational acceleration: %fm/s^2" % \
                         (self.__class__.__name__, self.accel))
    
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Translational Speed: %fm/s\nTurn Radius: %fm\n"\
               "Translational Acceleration: %fm/s^2" % (self.trans, 
                                                        self.turn, self.accel)


    
## @defgroup max_speed Max Speed
#  @ingroup set_max_speed data_max_speed
#  \b Description: The maximum translational speed for the platform.        \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Max Forward Speed</td><td>signed short</td><td>2 bytes</td>
#  <td>100</td><td>[0,320]</td><td>m/s</td></tr>
#  <tr><td>Max Reverse Speed</td><td>signed short</td><td>2 bytes</td>
#  <td>100</td><td>[0,320]</td><td>m/s</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field             | Type         | Size    | Scale | Range   | Units
#  ------------------+--------------+---------+-------+---------+------
#  Max Forward Speed | signed short | 2 bytes | 100   | [0,320] | m/s
#  Max Reverse Speed | signed short | 2 bytes | 100   | [0,320] | m/s
#  @endmanonly                                                                \n
#
#  \b Max \b Forward \b Speed: The maximum forward translational speed.     \n\n
#
#  \b Max \b Reverse \b Speed: The maximum reverse translational speed.


## Horizon Message Payload - Max Speed
#
#  Represents the payload of the command and data messages 'max speed'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Max Speed Data
#  @copydoc max_speed
#
#  @pydoc
class HorizonPayload_MaxSpeed(HorizonPayload):
    """Horizon Message Payload - Max Speed"""
    
    ## Create A Horizon Message Payload - Max Speed
    #
    #  Constructor for the Horizon Message Payload - Max Speed.               \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_MaxSpeed(forward, reverse, raw=None, ...)             \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_MaxSpeed(raw, version, timestamp)                     \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  forward        The maximum forward speed
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  reverse        The maximum reverse speed
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, forward = 0, reverse = 0, store_error = False, raw= None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Max Speed"""
        
        # Class Variables
        ## Max Forward Speed
        self._forward = 0
        ## Max Reverse Speed
        self._reverse = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test forward
            if forward < 0 or forward > 320:
                logger.warning(
                        "Tried to create %s with bad forward speed!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Forward Speed must be [0,320]!")
                if not store_error: raise self.error  
                else: return
            self._forward = forward
            if version[0] == 0 and version[1]< 7:
                data = utils.from_short(int(forward*1000))
            else:
                data = utils.from_short(int(forward*100))
            
            # test reverse
            if reverse < 0 or reverse > 320:
                logger.warning(
                            "Tried to create %s with bad reverse speed!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Reverse Speed must be [0,320]!")
                if not store_error: raise self.error  
                else: return
            self._reverse = reverse
            if version[0] == 0 and version[1]< 7:
                data += utils.from_short(int(reverse*1000))
            else:
                data += utils.from_short(int(reverse*100))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 4:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Forward
            if version[0] == 0 and version[1]< 7:
                self._forward = utils.to_short(raw[0:2])/1000.0
            else:
                self._forward = utils.to_short(raw[0:2])/100.0
            logger.debug("%s forward speed: %fm/s" % \
                         (self.__class__.__name__, self._forward))
            
            # Extract Reverse
            if version[0] == 0 and version[1]< 7:
                self._reverse = utils.to_short(raw[2:4])/1000.0
            else:
                self._reverse = utils.to_short(raw[2:4])/100.0
            logger.debug("%s reverse speed: %fm/s" % \
                         (self.__class__.__name__, self._reverse))
    
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Max Forward Speed: %fm/s\nMax Reverse Speed: %fm/s"\
               % (self._forward, self._reverse)
                
                
    ## Get Max Forward Speed
    #
    #  @return the forward speed (m/s)
    #
    #  @pydoc
    def get_forward(self):
        """Get Max Forward Speed"""
        
        return self._forward
                
                
    ## Get Max Reverse Speed
    #
    #  @return the reverse speed (m/s)
    #
    #  @pydoc
    def get_reverse(self):
        """Get Max Reverse Speed"""
        
        return self._reverse
    
    
    # Class Properties
    ## Max Forward Speed
    forward = property(fget=get_forward, doc="Max Forward Speed")
    ## Max Reverse Speed
    reverse = property(fget=get_reverse, doc="Max Reverse Speed")



    
    
## @defgroup max_acceleration Max Acceleration
#  @ingroup set_max_acceleration data_max_acceleration
#  \b Description: The maximum translational acceleration for the platform. \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Max Forward Acceleration</td><td>signed short</td><td>2 bytes</td>
#  <td>100</td><td>[0,320]</td><td>m/s</td></tr>
#  <tr><td>Max Reverse Acceleration</td><td>signed short</td><td>2 bytes</td>
#  <td>100</td><td>[0,320]</td><td>m/s</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field               | Type         | Size    | Scale | Range   | Units
#  --------------------+--------------+---------+-------+---------+------
#  Max Forward Acce... | signed short | 2 bytes | 100   | [0,320] | m/s
#  Max Reverse Acce... | signed short | 2 bytes | 100   | [0,320] | m/s
#  @endmanonly                                                                \n
#
#  \b Max \b Forward \b Acceleration: The maximum forward translational 
#  acceleration.                                                            \n\n
#
#  \b Max \b Reverse \b Acceleration: The maximum reverse translational 
#  acceleration.


## Horizon Message Payload - Max Acceleration
#
#  Represents the payload of the command and data messages 'max acceleration'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Max Acceleration Data
#  @copydoc max_acceleration
#
#  @pydoc
class HorizonPayload_MaxAcceleration(HorizonPayload):
    """Horizon Message Payload - Max Acceleration"""
    
    
    ## Create A Horizon Message Payload - Max Acceleration
    #
    #  Constructor for the Horizon Message Payload - Max Acceleration.        \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_MaxAcceleration(forward, reverse, raw=None, ...)      \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_MaxAcceleration(raw, version, timestamp)              \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  forward        The maximum forward acceleration
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  reverse        The maximum reverse acceleration
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, forward = 0, reverse = 0, store_error = False, raw= None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Max Acceleration"""
        
        # Class Variables
        ## Max Forward Acceleration
        self._forward = 0
        ## Max Reverse Acceleration
        self._reverse = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test forward
            if forward < 0 or forward > 320:
                logger.warning(
                        "Tried to create %s with bad forward acceleration!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Forward Acceleration must be [0,32]!")
                if not store_error: raise self.error  
                else: return
            self._forward = forward
            if version[0] == 0 and version[1]< 7:
                data = utils.from_short(int(forward*1000))
            else:
                data = utils.from_short(int(forward*100))
            
            # test reverse
            if reverse < 0 or reverse > 320:
                logger.warning(
                            "Tried to create %s with bad reverse acceleration!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Reverse Acceleration must be [0,32]!")
                if not store_error: raise self.error  
                else: return
            self._reverse = reverse
            if version[0] == 0 and version[1]< 7:
                data += utils.from_short(int(reverse*1000))
            else:
                data += utils.from_short(int(reverse*100))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 4:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Forward
            if version[0] == 0 and version[1]< 7:
                self._forward = utils.to_short(raw[0:2])/1000.0
            else:
                self._forward = utils.to_short(raw[0:2])/100.0
            logger.debug("%s forward acceleration: %fm/s^2" % \
                         (self.__class__.__name__, self._forward))
            
            # Extract Reverse
            if version[0] == 0 and version[1]< 7:
                self._reverse = utils.to_short(raw[2:4])/1000.0
            else:
                self._reverse = utils.to_short(raw[2:4])/100.0
            logger.debug("%s reverse acceleration: %fm/s^2" % \
                         (self.__class__.__name__, self._reverse))
    
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Max Forward Acceleration: %fm/s^2\n"\
                "Max Reverse Acceleration: %fm/s^2"\
               % (self._forward, self._reverse)
                
                
    ## Get Max Forward Acceleration
    #
    #  @return the forward acceleration (m/s)
    #
    #  @pydoc
    def get_forward(self):
        """Get Max Forward Acceleration"""
        
        return self._forward
                
                
    ## Get Max Reverse Acceleration
    #
    #  @return the reverse acceleration (m/s)
    #
    #  @pydoc
    def get_reverse(self):
        """Get Max Reverse Acceleration"""
        
        return self._reverse
    
    
    # Class Properties
    ## Max Forward Acceleration
    forward = property(fget=get_forward, doc="Max Forward Acceleration")
    ## Max Reverse Acceleration
    reverse = property(fget=get_reverse, doc="Max Reverse Acceleration")




## @defgroup gear Gear
#  @ingroup set_gear
#  \b Description: The current desired gear.                                \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Gear</td><td>signed char</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field | Type        | Size   | Scale | Range | Units
#  ------+-------------+--------+-------+-------+------
#  Gear  | signed char | 1 byte | -     | -     | -
#  @endmanonly                                                                \n
#
#  \b Gear: The desired gear for the vehicle's transmission. 0 corresponds to 
#  neutral, -1 to parked. Gears > 0 are forward gears in order of their gear
#  ratio. Gears < -1 are reverse gears in order of their gear ratio.


## Horizon Message Payload - Gear
#
#  Represents the payload of the command message 'gear'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Gear Data
#  @copydoc gear
#
#  @pydoc
class HorizonPayload_Gear(HorizonPayload):
    """Horizon Message Payload - Gear"""
    
    
    
    ## Create A Horizon Message Payload - Gear
    #
    #  Constructor for the Horizon Message Payload - Gear.                    \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_Gear(gear, raw=None,...)                              \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_Gear(raw, version, timestamp)                         \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  gear           The desired gear [-128,127]
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, gear = -1,
                 store_error = False, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Gear"""
        
        # Class Variables
        ## Gear
        self._gear = -1
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test Gear
            if gear < -128 or gear > 127:
                logger.warning(
                        "Tried to create %s with bad gear!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Gear must be [-128,127]!")
                if not store_error: raise self.error  
                else: return
            self._gear = gear
            data += utils.from_char(gear)
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 1:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Gear
            self._gear = utils.to_char(raw[0:1])
            logger.debug("%s gear: %d" % \
                         (self.__class__.__name__, self._gear))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Gear: %d" % (self._gear)
                
                
    ## Get Gear
    #
    #  @return the gear
    #
    #  @pydoc
    def get_gear(self):
        """Get Gear"""
        
        return self._gear
    
    
    # Class Properties
    ## Gear
    gear = property(fget=get_gear, doc="Gear")




## @defgroup gear_status Gear Status
#  @ingroup data_gear_status
#  \b Description: The current desired gear.                                \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Flags</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Gear</td><td>signed char</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field | Type        | Size   | Scale | Range | Units
#  ------+-------------+--------+-------+-------+------
#  Flags | byte        | 1 byte | -     | -     | -
#  Gear  | signed char | 1 byte | -     | -     | -
#  @endmanonly                                                                \n
#
#  \b Flags:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>7</i></td><td><i>6</i></td><td><i>5</i></td><td><i>4</i></td>
#  <td><i>3</i></td><td><i>2</i></td><td><i>1</i></td><td><i>0</i></td></tr>
#  <tr><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td>
#  <td>U</td><td>D</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  ---------------------------------
#  | - | - | - | - | - | - | U | D |
#  ---------------------------------
#  @endmanonly                                                                \n
#
#  <i>[1] Upshifting (U):</i>                                                 \n
#
#  Vehicle is currently in the process of shifting up.                      \n\n
#
#  <i>[0] Downshifting (D) :</i>                                              \n
#
#  Vehicle is currently in the process of shifting down.                    \n\n
#
#  \b Gear: The desired gear for the vehicle's transmission. 0 corresponds to 
#  neutral, -1 to parked. Gears > 0 are forward gears in order of their gear
#  ratio. Gears < -1 are reverse gears in order of their gear ratio.


## Horizon Message Payload - Gear Status
#
#  Represents the payload of the data message 'gear status'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Gear Status Data
#  @copydoc gear
#
#  @pydoc
class HorizonPayload_GearStatus(HorizonPayload):
    """Horizon Message Payload - Gear Status"""
    
    
    ## Create A Horizon Message Payload - Gear Status
    #
    #  Constructor for the Horizon Message Payload - Gear.                    \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_GearStatus(upshifting, downshifting,gear,raw=None,...)\n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_GearStatus(raw, version, timestamp)                   \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  downshifting   Set the downshifting flag?
    #  @param  gear           The desired gear [-128,127]
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  upshifting     Set the upshifting flag?
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, downshifting = False, upshifting = False, gear = -1,
                 store_error = False, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Gear Status"""
        
        # Class Variables
        ## Downshifting
        self._down = False
        ## Gear
        self._gear = -1
        ## Upshifting
        self._up = False
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test flags
            flags = 0x0
            if downshifting and upshifting:
                logger.warning(
                        "Tried to create %s with both down and up shifting!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Cannot downshift and upshift!")
                if not store_error: raise self.error  
                else: return
            self._down = downshifting
            self._up = upshifting
            if self._down: flags &= 0x01
            if self._up: flags &= 0x02
            data += utils.from_byte(flags)
            
            # test Gear
            if gear < -128 or gear > 127:
                logger.warning(
                        "Tried to create %s with bad gear!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Gear must be [-128,127]!")
                if not store_error: raise self.error  
                else: return
            self._gear = gear
            data += utils.from_char(gear)
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 2:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Flags
            flags = utils.to_byte(raw[0:1])
            self._down = (flags & 0x01) == 0x01
            self._up = (flags & 0x02) == 0x02
            logger.debug("%s downshifting: %s" % \
                         (self.__class__.__name__, str(self._down)))
            logger.debug("%s upshifting: %s" % \
                         (self.__class__.__name__, str(self._up)))
            
            # Extract Gear
            self._gear = utils.to_char(raw[1:2])
            logger.debug("%s gear: %d" % \
                         (self.__class__.__name__, self._gear))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Downshifting: %s\nUpshifting: %s\nGear: %d" % (
                                    str(self._down), str(self._up), self._gear)
                
                
    ## Is Downshifting?
    #
    #  @return is the platform downshifting?
    #
    #  @pydoc
    def is_downshifting(self):
        """Is Downshifting?"""
        
        return self._down
                
                
    ## Is Upshifting?
    #
    #  @return is the platform upshifting?
    #
    #  @pydoc
    def is_upshifting(self):
        """Is Upshifting?"""
        
        return self._up
                
                
    ## Get Gear
    #
    #  @return the gear
    #
    #  @pydoc
    def get_gear(self):
        """Get Gear"""
        
        return self._gear
    
    
    # Class Properties
    ## Downshifting
    downshifting = property(fget=is_downshifting, doc="Downshifting")
    ## Upshifting
    upshifting = property(fget=is_upshifting, doc="Upshifting")
    ## Gear
    gear = property(fget=get_gear, doc="Gear")



## @defgroup gpadc GPADC
#  @ingroup set_gpadc_output
#  \b Description: The setpoints for each of the n generic analog output 
#  channels. The payload will be 2n + 1 bytes in size. The amount of analog 
#  output channels is platform dependent.                                   \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Count</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>ID 1</td><td>unsigned byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Value 1</td><td>unsigned short</td><td>2 bytes</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td colspan="6">...</td></tr>
#  <tr><td>ID n</td><td>unsigned byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Value n</td><td>unsigned short</td><td>2 bytes</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field   | Type           | Size    | Scale | Range | Units
#  --------+----------------+---------+-------+-------+------
#  Count   | byte           | 1 byte  | -     | -     | -
#  ID 1    | byte           | 1 byte  | -     | -     | -
#  Value 1 | unsigned short | 2 bytes | -     | -     | -
#  ...     | ...            | ...     | ...   | ...   | ...
#  ID n    | byte           | 1 byte  | -     | -     | -
#  Value n | unsigned short | 2 bytes | -     | -     | -
#  @endmanonly                                                                \n
#
#  \b Count: The number of channel setpoint values being returned.            \n
#
#  \b ID x: The identifier of the channel to be controlled. Identifiers 
#  are dependent on the specific platform configuration.                      \n
#
#  \b Value x: The setpoint of the channel, scaled such that the maximum 
#  (0xFFFF) corresponds to the maximum voltage that channel can output.


## Horizon Message Payload - GPADC
#
#  Represents the payload of the command message 'gpadc'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data GPADC Output Data
#  @copydoc gpadc_output
#
#  @pydoc
class HorizonPayload_GPADC(HorizonPayload):
    """Horizon Message Payload - GPADC"""
    
    
    ## Create A Horizon Message Payload - GPADC
    #
    #  Constructor for the Horizon Message Payload - GPADC.                   \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_GPADC(values, raw=None, version, timestamp)           \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_GPADC(raw, version, timestamp)                        \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  values         Dictionary of GPADC channel IDs mapping to values
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, values = {}, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - GPADC Output"""
        
        # Class Variables
        ## GPADC Values
        self._values = {}
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test channels
            if len(values) > 256 or (version[0] == 0 and version[1]< 5 and 
                                     len(values) < 1):
                logger.warning("Tried to create %s with too many channels!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Number of channels must be 0-256!")
                if not store_error: raise self.error  
                else: return
            if not (version[0] == 0 and version[1]< 5):
                data = utils.from_byte(len(values))
            
            # test values
            for id in values:
                if id < 0 or id > 255:
                    logger.warning("Tried to create %s with bad ID!"\
                                 % self.__class__.__name__)
                    self.error = ValueError( "ID must be 0-255!")
                    if not store_error: raise self.error  
                    else: return
                if values[id] < 0 or values[id] > 65535:
                    logger.warning("Tried to create %s with bad value!"\
                                 % self.__class__.__name__)
                    self.error = ValueError( "Value must be 0-65535!")
                    if not store_error: raise self.error  
                    else: return
                data += utils.from_byte(id)
                data += utils.from_unsigned_short(values[id])
            self._values = values
            
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) < 1 or (version[0] == 0 and version[1]< 5 and 
                    len(raw) != 3) or ((not(version[0] == 0 and version[1]< 5)) 
                                       and len(raw) != raw[0]*3 + 1):
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Values
            length = raw[0]
            pos = 1
            if version[0] == 0 and version[1]< 5:
                length = 3
                pos = 0
            for i in range(0,length):
                self._values[utils.to_byte(raw[i*3+pos:i*3+1+pos])] = \
                        utils.to_unsigned_short(raw[i*3+pos+1:i*3+3+pos])
            logger.debug("%s values: %s" % (self.__class__.__name__, 
                     str(self._values)))
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Count: %d\nValues: %s" % (len(self._values), str(self._values))
                
                
    ## Get Count
    #
    #  @return number of GPADC Channels
    #
    #  @pydoc
    def get_count(self):
        """Get Count"""
        
        return len(self._values)
                
                
    ## Get Value
    #
    #  @param  channel channel of value to get
    #  @return the GPADC Channel's Value, or list of all channels if channel -1
    #
    #  @pydoc
    def get_value(self, channel=-1):
        """Get Value"""
        
        # singular
        if channel in self._values:
            return self._values[channel]
        
        # multiple
        else:
            return self._values.copy()
    
    
    # Class Properties
    ## GPADC Count
    count = property(fget=get_count, doc="GPADC Count")
    ## GPADC Value
    value = property(fget=get_value, doc="GPADC Value")



## @defgroup gpadc_output GPADC Output
#  @ingroup set_gpadc_output data_gpadc_output
#  \b Description: The setpoints for each of the n generic analog output 
#  channels. The payload will be 2n + 1 bytes in size. The amount of analog 
#  output channels is platform dependent.                                   \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Count</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Value</td><td>unsigned short</td><td>2n bytes</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field | Type           | Size     | Scale | Range | Units
#  ------+----------------+----------+-------+-------+------
#  Count | byte           | 1 byte   | -     | -     | -
#  Value | unsigned short | 2n bytes | -     | -     | -
#  @endmanonly                                                                \n
#
#  \b Count: The number of channel setpoint values being returned.            \n
#
#  \b Value: The setpoint of the channel, scaled such that the maximum 
#  (0xFFFF) corresponds to the maximum voltage that channel can output.


## Horizon Message Payload - GPADC Output
#
#  Represents the payload of the data message 'gpadc output'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data GPADC Output Data
#  @copydoc gpadc_output
#
#  @pydoc
class HorizonPayload_GPADCOutput(HorizonPayload):
    """Horizon Message Payload - GPADC Output"""
    
    
    ## Create A Horizon Message Payload - GPADC Output
    #
    #  Constructor for the Horizon Message Payload - GPADC Output.            \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_GPADCOutput(values, raw=None, version, timestamp)     \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_GPADCOutput(raw, version, timestamp)                  \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  channel        The GPADC Channel.
    #                         DEPRECATED: Not in Horizon as of v0.5.
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  values         List of GPADC channel values (0-65535)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, values = [], raw = None, 
                 timestamp = 0, version = tuple([-1,0]), channel = 0):
        """Create A Horizon Message Payload - GPADC Output"""
        
        # Class Variables
        ## GPADC Values
        self._values = []
        if version[0] == 0 and version[1]< 5:
            ## GPADC Channel
            #  @deprecated Not in Horizon as of v0.5
            self._channel = channel
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test channels
            if len(values) > 255 or (version[0] == 0 and version[1]< 5 and 
                                     len(values) != 1):
                logger.warning("Tried to create %s with too many channels!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Number of channels must be 0-255!")
                if not store_error: raise self.error  
                else: return
            if not (version[0] == 0 and version[1]< 5):
                data = utils.from_byte(len(values))
            else:
                if channel > 255 or channel < 0:
                    logger.warning("Tried to create %s with bad channel!"\
                                     % self.__class__.__name__)
                    self.error = ValueError( "Channel must be 0-255!")
                    if not store_error: raise self.error  
                    else: return
                data = utils.from_byte(channel)
            
            # test values
            for value in values:
                if value < 0 or value > 65535:
                    logger.warning("Tried to create %s with bad value!"\
                                 % self.__class__.__name__)
                    self.error = ValueError( "Value must be 0-65535!")
                    if not store_error: raise self.error  
                    else: return
                data += utils.from_unsigned_short(value)
            self._values = values
            
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) < 1 or (version[0] == 0 and version[1]< 5 and 
                    len(raw) != 3) or ((not(version[0] == 0 and version[1]< 5)) 
                                       and len(raw) != raw[0]*2 + 1):
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Channel
            # @deprecated
            length = raw[0]
            if version[0] == 0 and version[1]< 5:
                length = 1
                self._channel = utils.to_byte(raw[0:1])
                logger.debug("%s: Channel: %d" % (self.__class__.__name__, 
                     self._channel))
            
            # Extract Values
            for i in range(0,length):
                self._values.append(utils.to_unsigned_short(raw[i*2+1:i*2+3]))
            logger.debug("%s: values: %s" % (self.__class__.__name__, 
                     str(self._values)))
    
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Count: %d\nValues: %s" % (len(self._values), str(self._values))
                
                
    ## Get Count
    #
    #  @return number of GPADC Channels
    #
    #  @pydoc
    def get_count(self):
        """Get Count"""
        
        return len(self._values)
                
                
    ## Get Value
    #
    #  @param  channel channel of value to get
    #  @return the GPADC Channel's Value, or list of all channels if channel -1
    #
    #  @pydoc
    def get_value(self, channel=-1):
        """Get Value"""
        
        # singular
        if channel > -1 and channel < len(self._values):
            return self._values[channel]
        
        # multiple
        else:
            return self._values[:]
    
    
    # Class Properties
    ## GPADC Count
    count = property(fget=get_count, doc="GPADC Count")
    ## GPADC Value
    value = property(fget=get_value, doc="GPADC Value")



## @defgroup gpadc_output_request Request GPADC Output
#  @ingroup request_gpadc_output
#  \b Description: Requests the values for the general-purpose ADC output 
#  channels.                                                                \n\n
#
#  @copydoc request


## Horizon Message Payload - Request GPADC Output
#
#  Represents the payload of the request message for 'GPADC output'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Request GPADC Output Data
#  @copydoc gpadc_output_request
#
#  @pydoc
class HorizonPayload_Request_GPADCOutput(HorizonPayload_Request):
    """Horizon Message Payload - Request GPADC Output"""
    
    
    ## Create A Horizon Message Payload - Request GPADC Output
    #
    #  Constructor for the Horizon Message Payload - Request GPADC Output
    #  Class.                                                                 \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_Request_GPADCOutput(subscription, raw=None, ...)      \n
    #    Create a request message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_Request_GPADCOutput(raw, version, timestamp)          \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  channel        The desired channel.
    #                         DEPRECATED: Not in Horizon as of v0.5.
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  subscription   Subscription Frequency in Hz
    #                         0 - immediate, 0xFFFF - off
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  verify         Verify the length? (useful for subclasses)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, channel = 0, store_error = False, raw = None, 
                 subscription = 0, timestamp = 0, version = tuple([-1,0]), 
                 verify = True):
        """Create A Horizon Message Payload - Request GPADC Output"""
        
        # Class Variables
        ## GPADC Channel
        self._channel = 0
        
        # Create Constructor
        if raw == None:
            
            # Pass on to super-class
            HorizonPayload_Request.__init__(self, version = version, raw = None, 
                                            store_error = store_error, 
                                            subscription = subscription,
                                            timestamp = timestamp)
            
            # Test channel
            if channel < 0 or channel > 255 or (version[0] == 0 and 
                                                version[1]< 5 and channel != 0):
                logger.warning("Tried to create %s with bad channel!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Channel must be 0-255!")
                if not store_error: raise self.error  
                else: return
            self._channel = channel
            if version[0] == 0 and version[1]< 5:
                self.data += utils.from_byte(channel)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if verify and ((version[0] == 0 and version[1]< 4 and len(raw) != 2)
                or (version[0] == 0 and version[1] == 4 and len(raw) != 3)
                or (not(version[0] == 0 and version[1]< 5) and len(raw) != 2)):
                logger.warning("Tried to create %s of bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Invalid payload length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload_Request.__init__(self, raw = raw, version = version, 
                                            store_error = store_error,
                                            timestamp = timestamp, 
                                            verify = False)
            
            # Extract Mount
            if version[0] == 0 and version[1]< 4:
                self._mount = utils.to_byte(raw[1:2])
                logger.debug("%s mount: %d" % (self.__class__.__name__, 
                     self._mount))
            elif version[0] == 0 and version[1]< 5:
                self._mount = utils.to_byte(raw[2:3])
                logger.debug("%s mount: %d" % (self.__class__.__name__, 
                     self._mount))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        if self._version[0] == 0 and self._version[1] < 5:
            return HorizonPayload_Request.print_format(self) + \
                "\nChannel: %d" % self._channel
        return HorizonPayload_Request.print_format(self)
                
                
    ## Get Channel
    #
    #  @deprecated Not in Horizon as of v0.5.
    #  @return the GPADC channel
    #
    #  @pydoc
    def get_channel(self):
        """Get Channel"""
        
        return self._channel
    
    
    # Class Properties
    ## GPADC Channel
    channel = property(fget=get_channel, doc="GPADC Channel")



    
## @defgroup gpadc_input GPADC Input
#  @ingroup data_gpadc_input
#  \b Description: The value of the n generic analog input channels. The 
#  payload will be 2n + 1 bytes in size. The amount of analog input channels is 
#  platform dependent.                                                      \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Count</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Value</td><td>unsigned short</td><td>2n bytes</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field | Type           | Size     | Scale | Range | Units
#  ------+----------------+----------+-------+-------+------
#  Count | byte           | 1 byte   | -     | -     | -
#  Value | unsigned short | 2n bytes | -     | -     | -
#  @endmanonly                                                                \n
#
#  \b Count: The number of channel values being returned. This number is 
#  platform-dependent.                                                        \n
#
#  \b Value: The value of the channel, scaled such that the maximum (0xFFFF) 
#  corresponds to that channel's reference voltage.


## Horizon Message Payload - GPADC Input
#
#  Represents the payload of the data message 'gpadc input'
#
#  @warning Data should not be modified once created
#
#  @since 0.5
#
#  @section data GPADC Input Data
#  @copydoc gpadc_input
#
#  @pydoc
class HorizonPayload_GPADCInput(HorizonPayload):
    """Horizon Message Payload - GPADC Input"""
    
    
    
    ## Create A Horizon Message Payload - GPADC Input
    #
    #  Constructor for the Horizon Message Payload - GPADC Input.             \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_GPADCInput(values, raw=None, version, timestamp)      \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_GPADCInput(raw, version, timestamp)                   \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  values         List of GPADC channel values (0-65535)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, values = [], raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - GPADC Input"""
        
        # Class Variables
        ## GPADC Values
        self._values = []
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test channels
            if len(values) > 256:
                logger.warning("Tried to create %s with too many channels!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Number of channels must be 0-256!")
                if not store_error: raise self.error  
                else: return
            data = utils.from_byte(len(values))
            
            # test values
            for value in values:
                if value < 0 or value > 65535:
                    logger.warning("Tried to create %s with bad value!"\
                                 % self.__class__.__name__)
                    self.error = ValueError( "Value must be 0-65535!")
                    if not store_error: raise self.error  
                    else: return
                data += utils.from_unsigned_short(value)
            self._values = values
            
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) < 1 or len(raw) != 2*raw[0] + 1:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Values
            for i in range(0,raw[0]):
                self._values.append(utils.to_unsigned_short(
                                                        raw[i*2+1:(i+1)*2+1]))
            logger.debug("%s values: %s" % (self.__class__.__name__, 
                     str(self._values)))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Count: %d\nValues: %s" % (len(self._values), str(self._values))
                
                
    ## Get Count
    #
    #  @return number of GPADC Channels
    #
    #  @pydoc
    def get_count(self):
        """Get Count"""
        
        return len(self._values)
                
                
    ## Get Value
    #
    #  @param  channel channel of value to get
    #  @return the GPADC Channel's Value, or list of all channels if channel -1
    #
    #  @pydoc
    def get_value(self, channel=-1):
        """Get Value"""
        
        # singular
        if channel > -1 and channel < len(self._values):
            return self._values[channel]
        
        # multiple
        else:
            return self._values[:]
    
    
    # Class Properties
    ## GPADC Count
    count = property(fget=get_count, doc="GPADC Count")
    ## GPADC Value
    value = property(fget=get_value, doc="GPADC Value")


## @defgroup gpio_direction GPIO Direction
#  @ingroup set_gpio_direction
#  \b Description: Sets the direction of the generic digital I/O channels. The 
#  number of I/O channels is platform dependent.                            \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Bitmask</td><td>unsigned int</td><td>4 bytes</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Direction</td><td>unsigned int</td><td>4 bytes</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field     | Type         | Size    | Scale | Range | Units
#  ----------+--------------+---------+-------+-------+------
#  Bitmask   | unsigned int | 4 bytes | -     | -     | -
#  Direction | unsigned int | 4 bytes | -     | -     | -
#  @endmanonly                                                                \n
#
#  \b Bitmask: Set bits high to configure the corresponding channels. 
#  A platform may not necessarily have 32 GPIO channels available.            \n
#
#  \b Direction: Write a 1 to configure channel as an output, 0 to configure 
#  as an input. Will only be written if the corresponding bit in the bitmask 
#  is high.


## Horizon Message Payload - GPIO Direction
#
#  Represents the payload of the command message 'gpio direction'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data GPIO Direction Data
#  @copydoc gpio_direction
#
#  @pydoc
class HorizonPayload_GPIODirection(HorizonPayload):
    """Horizon Message Payload - GPIO Direction"""
    
    ## Create A Horizon Message Payload - GPIO Direction
    #
    #  Constructor for the Horizon Message Payload - GPIO Direction.          \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_GPIODirection(direction, mask, raw=None, ...)         \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_GPIODirection(raw, version, timestamp)                \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  direction      The GPIO port directions (32 bits for 32 channels)
    #  @param  mask           The GPIO ports to set
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, direction = 0, mask = 0, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - GPIO Direction"""
        
        # Class Variables
        ## GPIO Directions
        self._direction = 0
        ## GPIO Mask
        self._mask = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test mask
            if mask < 0 or mask > 4294967295:
                logger.warning("Tried to create %s with bad mask!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Mask must be 0-4294967295!")
                if not store_error: raise self.error  
                else: return
            self._mask = mask
            data = utils.from_unsigned_int(mask)
            
            # test direction
            if direction < 0 or direction > 4294967295:
                logger.warning("Tried to create %s with bad directions!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Directions must be 0-4294967295!")
                if not store_error: raise self.error  
                else: return
            self._direction = direction
            data += utils.from_unsigned_int(direction)
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 8:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Mask
            self._mask = utils.to_unsigned_int(raw[0:4])
            logger.debug("%s mask: %08X" % (self.__class__.__name__, 
                     self._mask))
            
            # Extract Directions
            self._direction = utils.to_unsigned_int(raw[4:8])
            logger.debug("%s direction: %08X" % (self.__class__.__name__, 
                     self._direction))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Direction: %08X\nMask: %08X" % (self._direction, self._mask)
        
        
    ## Get the number of GPIO channels
    #
    #  @return number of GPIO channels
    #
    #  @pydoc
    def get_count(self):
        """Get the number of GPIO channels."""
        
        return 32
                
                
    ## Get Direction
    #
    #  @param  channel  The GPIO channel to get the direction of. (0-31)
    #                   If -1 then it returns a 32 bit integer of all channels
    #  @return is the GPIO channel Direction input? or all GPIO directions
    #
    #  @pydoc
    def get_direction(self, channel = -1):
        """Get Direction"""
        
        # all channels
        if channel < 0 or channel > 31:
            return self._direction
        
        # single channel
        else:
            return ((self._direction >> channel) & 0x01) == 0x01
                
                
    ## Get Mask
    #
    #  @return the GPIO Channel Mask
    #
    #  @pydoc
    def get_mask(self):
        """Get Mask"""
        
        return self._mask
    
    
    # Class Properties
    ## GPIO Direction
    direction = property(fget=get_direction, doc="GPIO Direction")
    ## GPIO Mask
    mask = property(fget=get_mask, doc="GPIO Mask")


## @defgroup gpio_output GPIO Output
#  @ingroup set_gpio_output
#  \b Description: Sets the value of the generic digital output channels. 
#  The number of I/O channels is platform dependent.                        \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Bitmask</td><td>unsigned int</td><td>4 bytes</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Output value</td><td>unsigned int</td><td>4 bytes</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field        | Type         | Size    | Scale | Range | Units
#  -------------+--------------+---------+-------+-------+------
#  Bitmask      | unsigned int | 4 bytes | -     | -     | -
#  Output Value | unsigned int | 4 bytes | -     | -     | -
#  @endmanonly                                                                \n
#
#  \b Bitmask: Set bits high to write to these channels. A platform may not 
#  necessarily have 32 GPIO channels available.                               \n
#
#  \b Output \b Value: Write a 1 for a HIGH output, 0 for a LOW output. 
#  Will only be written if the corresponding bit in the bitmask is high.


## Horizon Message Payload - GPIO Output
#
#  Represents the payload of the command message 'gpio output'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data GPIO Output Data
#  @copydoc gpio_output
#
#  @pydoc
class HorizonPayload_GPIOOutput(HorizonPayload):
    """Horizon Message Payload - GPIO Output"""
    
    ## Create A Horizon Message Payload - GPIO Output
    #
    #  Constructor for the Horizon Message Payload - GPIO Output.             \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_GPIOOutput(output, mask, raw=None, ...)               \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_GPIOOutput(raw, version, timestamp)                   \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  mask           The GPIO ports to set
    #  @param  output         The GPIO port outputs (32 bits for 32 channels)
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, output = 0, mask = 0, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - GPIO Direction"""
        
        # Class Variables
        ## GPIO Outputs
        self._output = 0
        ## GPIO Mask
        self._mask = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test mask
            if mask < 0 or mask > 4294967295:
                logger.warning("Tried to create %s with bad mask!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Mask must be 0-4294967295!")
                if not store_error: raise self.error  
                else: return
            self._mask = mask
            data = utils.from_unsigned_int(mask)
            
            # test output
            if output < 0 or output > 4294967295:
                logger.warning("Tried to create %s with bad outputs!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Outputs must be 0-4294967295!")
                if not store_error: raise self.error  
                else: return
            self._output = output
            data += utils.from_unsigned_int(output)
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 8:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Mask
            self._mask = utils.to_unsigned_int(raw[0:4])
            logger.debug("%s mask: %08X" % (self.__class__.__name__, 
                     self._mask))
            
            # Extract Outputs
            self._output = utils.to_unsigned_int(raw[4:8])
            logger.debug("%s output: %08X" % (self.__class__.__name__, 
                     self._output))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Output: %08X\nMask: %08X" % (self._output, self._mask)
        
        
    ## Get the number of GPIO channels
    #
    #  @return number of GPIO channels
    #
    #  @pydoc
    def get_count(self):
        """Get the number of GPIO channels."""
        
        return 32
                
                
    ## Get Output
    #
    #  @param  channel  The GPIO channel to get the output of. (0-31)
    #                   If -1 then it returns a 32 bit integer of all channels
    #  @return is the GPIO channel output high? or all GPIO directions
    #
    #  @pydoc
    def get_output(self, channel = -1):
        """Get Output"""
        
        # all channels
        if channel < 0 or channel > 31:
            return self._output
        
        # single channel
        else:
            return ((self._output >> channel) & 0x01) == 0x01
                
                
    ## Get Mask
    #
    #  @return the GPIO Channel Mask
    #
    #  @pydoc
    def get_mask(self):
        """Get Mask"""
        
        return self._mask
    
    
    # Class Properties
    ## GPIO Output
    output = property(fget=get_output, doc="GPIO Output")
    ## GPIO Mask
    mask = property(fget=get_mask, doc="GPIO Mask")


## @defgroup gpio GPIO
#  @ingroup data_gpio
#  \b Description: The value of the generic digital channels.               \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Direction</td><td>unsigned int</td><td>4 bytes</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Value</td><td>unsigned int</td><td>4 bytes</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field     | Type         | Size    | Scale | Range | Units
#  ----------+--------------+---------+-------+-------+------
#  Direction | unsigned int | 4 bytes | -     | -     | -
#  Value     | unsigned int | 4 bytes | -     | -     | -
#  @endmanonly                                                                \n
#
#  \b Direction: If a bit is high, the channel is currently an output. 
#  If low, is an input.                                                     \n\n
#
#  \b Value: 1 for HIGH output, 0 for LOW.


## Horizon Message Payload - GPIO
#
#  Represents the payload of the data message 'gpio'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data GPIO Data
#  @copydoc gpio
#
#  @pydoc
class HorizonPayload_GPIO(HorizonPayload):
    """Horizon Message Payload - GPIO"""
    
    ## Create A Horizon Message Payload - GPIO
    #
    #  Constructor for the Horizon Message Payload - GPIO.                    \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_GPIO(direction, value, raw=None, ...)                 \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_GPIO(raw, version, timestamp)                         \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  direction      The GPIO port directions (32 bits for 32 channels)
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  value          The GPIO port values
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, direction = 0, value = 0, raw= None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - GPIO"""
        
        # Class Variables
        ## GPIO Directions
        self._direction = 0
        ## GPIO Values
        self._value = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test direction
            if direction < 0 or direction > 4294967295:
                logger.warning("Tried to create %s with bad directions!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Directions must be 0-4294967295!")
                if not store_error: raise self.error  
                else: return
            self._direction = direction
            data = utils.from_unsigned_int(direction)
            
            # test value
            if value < 0 or value > 4294967295:
                logger.warning("Tried to create %s with bad value!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Value must be 0-4294967295!")
                if not store_error: raise self.error  
                else: return
            self._value = value
            data += utils.from_unsigned_int(value)
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 8:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Directions
            self._direction = utils.to_unsigned_int(raw[0:4])
            logger.debug("%s direction: %08X" % (self.__class__.__name__, 
                     self._direction))
            
            # Extract Value
            self._value = utils.to_unsigned_int(raw[4:8])
            logger.debug("%s value: %08X" % (self.__class__.__name__, 
                     self._value))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Direction: %08X\nValue: %08X" % (self._direction, self._value)
        
        
    ## Get the number of GPIO channels
    #
    #  @return number of GPIO channels
    #
    #  @pydoc
    def get_count(self):
        """Get the number of GPIO channels."""
        
        return 32
                
                
    ## Get Direction
    #
    #  @param  channel  The GPIO channel to get the direction of. (0-31)
    #                   If -1 then it returns a 32 bit integer of all channels
    #  @return is the GPIO channel Direction input? or all GPIO directions
    #
    #  @pydoc
    def get_direction(self, channel = -1):
        """Get Direction"""
        
        # all channels
        if channel < 0 or channel > 31:
            return self._direction
        
        # single channel
        else:
            return ((self._direction >> channel) & 0x01) == 0x01
                
                
    ## Get Value
    #
    #  @param  channel  The GPIO channel to get the value of. (0-31)
    #                   If -1 then it returns a 32 bit integer of all channels
    #  @return is the GPIO channel value 1? or all GPIO values
    #
    #  @pydoc
    def get_value(self, channel = -1):
        """Get Value"""
        
        # all channels
        if channel < 0 or channel > 31:
            return self._value
        
        # single channel
        else:
            return ((self._value >> channel) & 0x01) == 0x01
    
    
    # Class Properties
    ## GPIO Direction
    direction = property(fget=get_direction, doc="GPIO Direction")
    ## GPIO Value
    value = property(fget=get_value, doc="GPIO Value")



## @defgroup pan_tilt_zoom Pan/Tilt/Zoom
#  @ingroup set_pan_tilt_zoom data_pan_tilt_zoom
#  \b Description: The pan, tilt, and zoom settings on a specified camera 
#  mount. If mount does not have a pan, tilt, or zoom axis, 0 will be returned. 
#  Zero position and direction of rotation for the pan and tilt axes are 
#  dependent on the camera mount.                                           \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Mount</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Pan</td><td>signed short</td><td>2 bytes</td><td>100</td>
#  <td>[-180,180]</td><td>deg</td></tr>
#  <tr><td>Tilt</td><td>signed short</td><td>2 bytes</td><td>100</td>
#  <td>[-180,180]</td><td>deg</td></tr>
#  <tr><td>Zoom</td><td>signed short</td><td>2 bytes</td><td>100</td>
#  <td>[1,320]</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field | Type         | Size    | Scale | Range      | Units
#  ------+--------------+---------+-------+------------+------
#  Mount | byte         | 1 byte  | -     | -          | -
#  Pan   | signed short | 2 bytes | 100   | [-180,180] | deg
#  Tilt  | signed short | 2 bytes | 100   | [-180,180] | deg
#  Zoom  | signed short | 2 bytes | 100   | [1,320]    | -
#  @endmanonly                                                                \n
#
#  \b Mount: The camera mount corresponding to the data.                    \n\n
#
#  \b Pan: The current pan position of the camera mount.                    \n\n
#
#  \b Tilt: The current tilt position of the camera mount.                  \n\n
#
#  \b Zoom: The current zoom level of the camera.


## Horizon Message Payload - Pan/Tilt/Zoom
#
#  Represents the payload of the command and data messages 'pan/tilt/zoom'
#  @warning Data should not be modified once created
#
#  @since 0.2
#
#  @section data Pan/Tilt/Zoom Data
#  @copydoc pan_tilt_zoom
#
#  @pydoc
class HorizonPayload_PanTiltZoom(HorizonPayload):
    """Horizon Message Payload - Pan/Tilt/Zoom"""
    
    ## Create A Horizon Message Payload - Pan/Tilt/Zoom
    #
    #  Constructor for the Horizon Message Payload - Pan/Tilt/Zoom.           \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_PanTiltZoom(mount, pan, raw=None, ...)                \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_PanTiltZoom(raw, version, timestamp)                  \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  mount          The camera mount
    #  @param  pan            The pan position
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  tilt           The tilt position
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @param  zoom           The zoom level
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, mount = 0, pan = 0, tilt = 0, zoom = 1, 
                 store_error = False, raw = None, timestamp = 0, 
                 version = tuple([-1,0])):
        """Create A Horizon Message Payload - Pan/Tilt/Zoom"""
        
        # Class Variables
        ## Camera Mount
        self._mount = 0
        ## Pan Position
        self._pan = 0
        ## Tilt Position
        self._tilt = 0
        ## Zoom Position
        self._zoom = 1
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test mount
            if mount < 0 or mount > 255:
                logger.warning("Tried to create %s with bad mount!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Camera mount must be 0-255!")
                if not store_error: raise self.error  
                else: return
            self._mount = mount
            data = utils.from_byte(mount)
            
            # test pan
            if pan < -180 or pan > 180:
                logger.warning("Tried to create %s with bad pan!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Pan must be [-180,180]!")
                if not store_error: raise self.error  
                else: return
            self._pan = pan
            data += utils.from_short(int(pan*100))
            
            # test tilt
            if tilt < -180 or tilt > 180:
                logger.warning("Tried to create %s with bad tilt!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Tilt must be [-180,180]!")
                if not store_error: raise self.error  
                else: return
            self._tilt = tilt
            data += utils.from_short(int(tilt*100))
            
            # test zoom
            if zoom < 0 or zoom > 320:
                logger.warning("Tried to create %s with bad zoom!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Zoom must be [0,320]!")
                if not store_error: raise self.error  
                else: return
            self._zoom = zoom
            data += utils.from_short(int(zoom*100))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 7:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Mount
            self._mount = utils.to_byte(raw[0:1])
            logger.debug("%s mount: %d" % (self.__class__.__name__, 
                     self._mount))
            
            # Extract Pan
            self._pan = utils.to_short(raw[1:3])/100.0
            logger.debug("%s pan: %f" % (self.__class__.__name__, 
                     self._pan))
            
            # Extract Tilt
            self._tilt = utils.to_short(raw[3:5])/100.0
            logger.debug("%s tilt: %f" % (self.__class__.__name__, 
                     self._tilt))
            
            # Extract Zoom
            self._zoom = utils.to_short(raw[5:7])/100.0
            logger.debug("%s zoom: %f" % (self.__class__.__name__, 
                     self._zoom))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Camera Mount: %d\nPan: %f°\nTilt: %f°\nZoom: %f" % (
                                self._mount, self._pan, self._tilt, self._zoom)
        
        
    ## Get the camera mount
    #
    #  @return camera mount number
    #
    #  @pydoc
    def get_mount(self):
        """Get the camera mount."""
        
        return self._mount
                
                
    ## Get Pan Position
    #
    #  @return pan position
    #
    #  @pydoc
    def get_pan(self):
        """Get Pan Position"""
        
        return self._pan
                
                
    ## Get Tilt Position
    #
    #  @return tilt position
    #
    #  @pydoc
    def get_tilt(self):
        """Get Tilt Position"""
        
        return self._tilt
                
                
    ## Get Zoom Level
    #
    #  @return zoom level
    #
    #  @pydoc
    def get_zoom(self):
        """Get Zoom Level"""
        
        return self._zoom
    
    
    # Class Properties
    ## Camera Mount
    mount = property(fget=get_mount, doc="Camera Mount")
    ## Pan Position
    pan = property(fget=get_pan, doc="Pan Position")
    ## Tilt Position
    tilt = property(fget=get_tilt, doc="Tilt Position")
    ## Zoom Level
    zoom = property(fget=get_zoom, doc="Zoom Level")


## @defgroup pan_tilt_zoom_request Request Pan/Tilt/Zoom
#  @ingroup request_pan_tilt_zoom
#  \b Description: Requests the position and zoom setting for a specified 
#  camera mount.                                                            \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Subscription</td><td>unsigned short</td><td>2 bytes</td><td>-</td>
#  <td>-</td><td>Hz</td></tr>
#  <tr><td>Mount</td><td>byte</td><td>1 byte</td><td>-</td><td>-</td>
#  <td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field        | Type           | Size    | Scale | Range      | Units
#  -------------+----------------+---------+-------+------------+------
#  Subscription | unsigned short | 2 bytes | -     | [0,65535]  | Hz
#  Mount        | byte           | 1 byte  | -     | -          | -
#  @endmanonly                                                                \n
#
#  \b Subscription: The frequency to receive data. 0 represents an immediate,
#  single response and 0xFFFF turns off the current subscription.             \n
#
#  \b Mount: The mount being requested. The number of mounts available is 
#  dependent on the platform hardware.


## Horizon Message Payload - Request Pan/Tilt/Zoom
#
#  Represents the payload of the request message for 'pan/tilt/zoom'
#  @warning Data should not be modified once created
#
#  @since 0.2
#
#  @section data Request Pan/Tilt/Zoom Data
#  @copydoc pan_tilt_zoom_request
#
#  @pydoc
class HorizonPayload_Request_PanTiltZoom(HorizonPayload_Request):
    """Horizon Message Payload - Request Pan/Tilt/Zoom"""
    
    ## Create A Horizon Message Payload - Request Pan/Tilt/Zoom
    #
    #  Constructor for the Horizon Message Payload - Request Pan/Tilt/Zoom 
    #  Class.                                                                 \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_Request_PanTiltZoom(mount, raw=None, ...)             \n
    #    Create a request message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_Request_PanTiltZoom(raw, version, timestamp)          \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  mount          The desired camera mount
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  subscription   Subscription Frequency in Hz
    #                         0 - immediate, 0xFFFF - off
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  verify         Verify the length? (useful for subclasses)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, mount = 0, store_error = False, raw = None, 
                 subscription = 0, timestamp = 0, version = tuple([-1,0]), 
                 verify = True):
        """Create A Horizon Message Payload - Request Pan/Tilt/Zoom"""
        
        # Class Variables
        ## Camera Mount
        self._mount = 0
        
        # Create Constructor
        if raw == None:
            
            # Pass on to super-class
            HorizonPayload_Request.__init__(self, version = version, raw = None, 
                                            store_error = store_error, 
                                            subscription = subscription,
                                            timestamp = timestamp)
            
            # Test mount
            if mount < 0 or mount > 255:
                logger.warning("Tried to create %s with bad mount!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Camera Mount must be 0-255!")
                if not store_error: raise self.error  
                else: return
            self._mount = mount
            self.data += utils.from_byte(mount)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if verify and ((version[0] == 0 and version[1]< 4 and len(raw) != 2)
                or (not(version[0] == 0 and version[1]< 4) and len(raw) != 3)):
                logger.warning("Tried to create %s of bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Invalid payload length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload_Request.__init__(self, raw = raw, version = version, 
                                            store_error = store_error,
                                            timestamp = timestamp, 
                                            verify = False)
            
            # Extract Mount
            if version[0] == 0 and version[1]< 4:
                self._mount = utils.to_byte(raw[1:2])
            else:
                self._mount = utils.to_byte(raw[2:3])
            logger.debug("%s mount: %d" % (self.__class__.__name__, 
                     self._mount))
    
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return HorizonPayload_Request.print_format(self) + \
                "\nCamera Mount: %d" % self._mount
                
                
    ## Get Camera Mount
    #
    #  @return the camera mount number
    #
    #  @pydoc
    def get_mount(self):
        """Get Camera Mount"""
        
        return self._mount
    
    
    # Class Properties
    ## Camera Mount
    mount = property(fget=get_mount, doc="Camera Mount")


## @defgroup distance Distance
#  @ingroup data_distance
#  \b Description: The range values for the n equipped distance sensors. 
#  The position and type of each distance sensor is platform-dependent. 
#  The payload will be 2n + 1 bytes in size.                                \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Count</td><td>byte</td><td>1 byte</td><td>-</td><td>-</td>
#  <td>-</td></tr>
#  <tr><td>Distance</td><td>signed short</td><td>2n bytes</td><td>1000</td>
#  <td>[0,32]</td><td>m</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field    | Type         | Size     | Scale | Range  | Units
#  ---------+--------------+----------+-------+--------+------
#  Count    | byte         | 1 byte   | -     | -      | -
#  Distance | signed short | 2n bytes | 1000  | [0,32] | m
#  @endmanonly                                                                \n
#
#  \b Count: The number of distance measurements being returned.              \n
#
#  \b Distance: The distance value from the distance sensor, in meters.


## Horizon Message Payload - Distance
#
#  Represents the payload of the data message 'distance'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Distance Data
#  @copydoc distance
#
#  @pydoc
class HorizonPayload_Distance(HorizonPayload):
    """Horizon Message Payload - Distance"""
    
    
    ## Create A Horizon Message Payload - Distance
    #
    #  Constructor for the Horizon Message Payload - Distance.                \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_Distance(distance, raw=None, timestamp, version, ...) \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_Distance(raw, version, timestamp)                     \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  distance       List of distances scaled to two bytes
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, distance = [], store_error = False, 
                 raw = None, timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Distance"""
        
        # Class Variables
        self.distances = []
        
        # Parse Constructor
        if raw != None:
            # Verify Length
            if len(raw) < 1 or len(raw) != raw[0]*2+1:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Distances
            for i in range(0, raw[0]):
                self.distances += [utils.to_short(raw[i * 2 + 1:i * 2 + 3])]
            logger.debug("%s distances: %s" % (self.__class__.__name__, 
                     ' '.join(map(hex,self.distances))))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Distances: %s" % ' '.join(map(hex,self.distances))
        

    
## @defgroup distance_timing Distance & Timing
#  @ingroup data_distance_timing
#  \b Description: The range values for the n equipped sensors, and 
#  the time at which each was last updated. The position and type of each 
#  distance sensor is platform-dependent. The payload will be 6n + 1 bytes
#  in size.                                                                 \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Count</td><td>byte</td><td>1 byte</td><td>-</td><td>-</td>
#  <td>-</td></tr>
#  <tr><td>Distance</td><td>signed short</td><td>2n bytes</td><td>1000</td>
#  <td>[0,32]</td><td>m</td></tr>
#  <tr><td>Time</td><td>unsigned int</td><td>4n bytes</td><td>-</td>
#  <td>-</td><td>ms</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field    | Type         | Size     | Scale | Range  | Units
#  ---------+--------------+----------+-------+--------+------
#  Count    | byte         | 1 byte   | -     | -      | -
#  Distance | signed short | 2n bytes | 1000  | [0,32] | m
#  Time     | unsigned int | 4n bytes | -     | -      | ms
#  @endmanonly                                                                \n
#
#  \b Count: The number of distance measurements being returned.              \n
#
#  \b Distance: The distance value from the distance sensor, in meters.       \n
#
#  \b Time: 4 bytes representing the time the sensor acquired its data, 
#  according to the platform clock.


## Horizon Message Payload - Distance & Timing
#
#  Represents the payload of the data message 'distance timing'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Distance Timing Data
#  @copydoc distance_timing
#
#  @pydoc
class HorizonPayload_DistanceTiming(HorizonPayload):
    """Horizon Message Payload - Distance & Timing"""
    
    
    ## Create A Horizon Message Payload - Distance & Timing
    #
    #  Constructor for the Horizon Message Payload - Distance & Timing.       \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_DistanceTiming(distance, timing, raw=None, ...)       \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_DistanceTiming(raw, version, timestamp)               \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  distance       List of distances scaled to two bytes
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  timing         List of distance aquisition times (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, distance = [], timing = [], store_error = False, 
                 raw = None, timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Distance & Timing"""
        
        # Class Variables
        self.distances = []
        self.timings = []
        
        # Parse Constructor
        if raw != None:

            # Verify Length
            if len(raw) < 1 or len(raw) != raw[0] * 6 + 1:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Distances
            for i in range(0, raw[0]):
                self.distances += [utils.to_short(raw[i*2+1:i*2+3])]
            logger.debug("%s distances: %s" % (self.__class__.__name__, 
                     ' '.join(map(hex,self.distances))))
            
            # Extract Timing
            for i in range(0,raw[0]):
                self.timings += [utils.to_unsigned_int(
                                            raw[raw[0]*2+i*4+1:i*4+5+raw[0]*2])]
            logger.debug("%s times: %s" % (self.__class__.__name__, 
                     ' '.join(map(str,self.timings))))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Distances: %s\nTiming: %s" % (' '.join(map(hex,self.distances)),
                                              ' '.join(map(str,self.timings)))
        

    
## @defgroup platform_orientation Platform Orientation
#  @ingroup data_platform_orientation
#  \b Description: The vehicle's best estimates of its orientation.         \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Roll</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  <tr><td>Pitch</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  <tr><td>Yaw</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field           | Type         | Size    | Scale | Range      | Units
#  ----------------+--------------+---------+-------+------------+------
#  Roll            | signed short | 2 bytes | 1000  | [-π,π]     | rad
#  Pitch           | signed short | 2 bytes | 1000  | [-π,π]     | rad
#  Yaw             | signed short | 2 bytes | 1000  | [-π,π]     | rad
#  @endmanonly                                                                \n
#
#  \b Roll: Vehicle's angle about its roll axis, in a right-hand sense.       \n
#
#  \b Pitch: Vehicle's angle about its pitch axis, in a right-hand sense.     \n
#
#  \b Yaw: Vehicle's global heading with respect to magnetic north, 
#  about its yaw axis, in a right-hand sense. Note: This means that the 
#  vehicle's yaw increases as the vehicle turns towards the west. Compasses 
#  traditionally have heading increasing as one turns east. If a vehicle is 
#  only equipped with tilt sensors, this field will default to 0.


## Horizon Message Payload - Orientation
#
#  Represents the payload of the data message 'orientation'
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Platform Orientation Data
#  @copydoc platform_orientation
#
#  @pydoc
class HorizonPayload_Orientation(HorizonPayload):
    """Horizon Message Payload - Orientation"""
    
    ## Create A Horizon Message Payload - Orientation
    #
    #  Constructor for the Horizon Message Payload - Orientation.             \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_Orientation(roll, pitch, yaw, raw=None, ...)          \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_Orientation(raw, version, timestamp)                  \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  pitch          The vehicle's pitch angle
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  roll           The vehicle's roll angle
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @param  yaw            The vehicle's yaw (magnetic) angle
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, roll = 0, pitch = 0, yaw = 0, store_error = False, 
                 raw = None, timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Orientation"""
        
        # Class Variables
        ## Roll Angle
        self._roll = 0
        ## Pitch Angle
        self._pitch = 0
        ## Yaw Angle
        self._yaw = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test roll
            if roll < -math.pi or roll > math.pi:
                logger.warning("Tried to create %s with bad roll!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Roll must be [-π,π]!")
                if not store_error: raise self.error  
                else: return
            self._roll = roll
            data += utils.from_short(int(roll*1000))
            
            # test pitch
            if pitch < -math.pi or pitch > math.pi:
                logger.warning("Tried to create %s with bad pitch!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Pitch must be [-π,π]!")
                if not store_error: raise self.error  
                else: return
            self._pitch = pitch
            data += utils.from_short(int(pitch*1000))
            
            # test yaw
            if yaw < -math.pi or yaw > math.pi:
                logger.warning("Tried to create %s with bad yaw!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Yaw must be [-π,π]!")
                if not store_error: raise self.error  
                else: return
            self._yaw = yaw
            data += utils.from_short(int(yaw*1000))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 6:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Roll
            self._roll = utils.to_short(raw[0:2])/1000.0
            logger.debug("%s roll: %f" % (self.__class__.__name__, 
                     self._roll))
            
            # Extract Pitch
            self._pitch = utils.to_short(raw[2:4])/1000.0
            logger.debug("%s pitch: %f" % (self.__class__.__name__, 
                     self._pitch))
            
            # Extract Yaw
            self._yaw = utils.to_short(raw[4:6])/1000.0
            logger.debug("%s yaw: %f" % (self.__class__.__name__, 
                     self._yaw))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Roll: %f\nPitch: %f\nYaw: %f" % (
                                        self._roll, self._pitch, self._yaw)
        
        
    ## Get the vehicle's Roll
    #
    #  @return roll angle
    #
    #  @pydoc
    def get_roll(self):
        """Get the vehicle's Roll."""
        
        return self._roll
        
        
    ## Get the vehicle's Pitch
    #
    #  @return pitch angle
    #
    #  @pydoc
    def get_pitch(self):
        """Get the vehicle's Pitch."""
        
        return self._pitch
        
        
    ## Get the vehicle's Yaw
    #
    #  @return yaw angle
    #
    #  @pydoc
    def get_yaw(self):
        """Get the vehicle's Yaw."""
        
        return self._yaw
    
    
    # Class Properties
    ## Pitch Angle
    pitch = property(fget=get_pitch, doc="Pitch Angle")
    ## Roll Angle
    roll = property(fget=get_roll, doc="Roll Angle")
    ## Yaw Angle
    yaw = property(fget=get_yaw, doc="Yaw Angle")
    
    
## @defgroup platform_rotation Platform Rotation
#  @ingroup data_platform_rotation
#  \b Description: The vehicle's best estimates of its rotational rates.    \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Rotational Roll</td><td>signed short</td><td>2 bytes</td>
#  <td>1000</td><td>[-10π,10π]</td><td>rad/s</td></tr>
#  <tr><td>Rotational Pitch</td><td>signed short</td><td>2 bytes</td>
#  <td>1000</td><td>[-10π,10π]</td><td>rad/s</td></tr>
#  <tr><td>Rotational Yaw</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-10π,10π]</td><td>rad/s</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field           | Type         | Size    | Scale | Range      | Units
#  ----------------+--------------+---------+-------+------------+------
#  Rotational Roll | signed short | 2 bytes | 1000  | [-10π,10π] | rad/s
#  Rotational P... | signed short | 2 bytes | 1000  | [-10π,10π] | rad/s
#  Rotational Yaw  | signed short | 2 bytes | 1000  | [-10π,10π] | rad/s
#  @endmanonly                                                                \n
#
#  \b Rotational \b Roll: Vehicle's angular rate about its roll axis, in a 
#  right-hand sense.                                                          \n
#
#  \b Rotational \b Pitch: Vehicle's angular rate about its pitch axis, in a 
#  right-hand sense.                                                          \n
#
#  \b Rotational \b Yaw:              Vehicle‟s angular rate about its yaw 
#  axis, in a right-hand sense. Note: Positive yaw rate corresponds to a left 
#  turn. Compasses traditionally have heading increasing as one turns right.


## Horizon Message Payload - Rotation
#
#  Represents the payload of the data message 'rotational rate'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Platform Rotation Data
#  @copydoc platform_rotation
#
#  @pydoc
class HorizonPayload_Rotation(HorizonPayload):
    """Horizon Message Payload - Rotation"""
    
    
    ## Create A Horizon Message Payload - Rotation
    #
    #  Constructor for the Horizon Message Payload - Rotation.                \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_Rotation(roll, pitch, yaw, raw=None, ...)             \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_Rotation(raw, version, timestamp)                     \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  rot_pitch      The vehicle's pitch rotation rate
    #  @param  rot_roll       The vehicle's roll rotation rate
    #  @param  rot_yaw        The vehicle's yaw rotation rate
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, rot_roll = 0, rot_pitch = 0, rot_yaw = 0, 
                 store_error = False, raw = None, timestamp = 0, 
                 version = tuple([-1,0])):
        """Create A Horizon Message Payload - Rotation"""
        
        # Class Variables
        ## Roll Rotation
        self._rot_roll = 0
        ## Pitch Rotation
        self._rot_pitch = 0
        ## Yaw Rotation
        self._rot_yaw = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test roll
            if rot_roll < -10*math.pi or rot_roll > 10*math.pi:
                logger.warning("Tried to create %s with bad roll!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Roll must be [-10π,10π]!")
                if not store_error: raise self.error  
                else: return
            self._rot_roll = rot_roll
            data += utils.from_short(int(rot_roll*1000))
            
            # test pitch
            if rot_pitch < -10*math.pi or rot_pitch > 10*math.pi:
                logger.warning("Tried to create %s with bad pitch!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Pitch must be [-10π,10π]!")
                if not store_error: raise self.error  
                else: return
            self._rot_pitch = rot_pitch
            data += utils.from_short(int(rot_pitch*1000))
            
            # test yaw
            if rot_yaw < -10*math.pi or rot_yaw > 10*math.pi:
                logger.warning("Tried to create %s with bad yaw!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Yaw must be [-10π,10π]!")
                if not store_error: raise self.error  
                else: return
            self._rot_yaw = rot_yaw
            data += utils.from_short(int(rot_yaw*1000))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 6:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Roll
            self._rot_roll = utils.to_short(raw[0:2])/1000.0
            logger.debug("%s roll: %f" % (self.__class__.__name__, 
                     self._rot_roll))
            
            # Extract Pitch
            self._rot_pitch = utils.to_short(raw[2:4])/1000.0
            logger.debug("%s pitch: %f" % (self.__class__.__name__, 
                     self._rot_pitch))
            
            # Extract Yaw
            self._rot_yaw = utils.to_short(raw[4:6])/1000.0
            logger.debug("%s yaw: %f" % (self.__class__.__name__, 
                     self._rot_yaw))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Rotational Roll: %frad/s\nRotational Pitch: %frad/s\n"\
               "Rotational Yaw: %frad/s" % (
                          self._rot_roll, self._rot_pitch, self._rot_yaw)
        
        
    ## Get the vehicle's Roll rotation rate
    #
    #  @return roll rotation rate
    #
    #  @pydoc
    def get_rotational_roll(self):
        """Get the vehicle's Roll rotation rate."""
        
        return self._rot_roll
        
        
    ## Get the vehicle's Pitch rotation rate
    #
    #  @return pitch rotation rate
    #
    #  @pydoc
    def get_rotational_pitch(self):
        """Get the vehicle's Pitch rotation rate."""
        
        return self._rot_pitch
        
        
    ## Get the vehicle's Yaw rotation rate
    #
    #  @return yaw rotation rate
    #
    #  @pydoc
    def get_rotational_yaw(self):
        """Get the vehicle's Yaw rotation rate."""
        
        return self._rot_yaw
    
    
    # Class Properties
    ## Pitch rotation rate
    rotational_pitch = property(fget=get_rotational_pitch, 
                                doc="Pitch rotation rate")
    ## Roll rotation rate
    rotational_roll = property(fget=get_rotational_roll, 
                               doc="Roll rotation rate")
    ## Yaw rotation rate
    rotational_yaw = property(fget=get_rotational_yaw, doc="Yaw rotation rate")
    
    

class HorizonPayload_Acceleration(HorizonPayload):
    """Horizon Message Payload - Acceleration"""
    
    ## Create A Horizon Message Payload - Acceleration
    def __init__(self, x = 0, y = 0, z = 0, store_error = False, 
                 raw = None, timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Acceleration"""
        
        # Class Variables
        self.x = 0
        self.y = 0
        self.z = 0

        # Verify Length
        if raw == None or len(raw) != 6:
            logger.warning("Tried to create %s with bad length!"\
                               % self.__class__.__name__)
            self.error = ValueError( "Bad length!")
            if not store_error: raise self.error  
            else: return
                
        # Pass on to super-class
        HorizonPayload.__init__(self, raw = raw, version = version, 
                                store_error = store_error, timestamp = timestamp)
            
        # Extract X
        self.x = utils.to_short(raw[0:2]) / 1.0
        logger.debug("%s x: %f" % (self.__class__.__name__, self.x))
            
        # Extract Y
        self.y = utils.to_short(raw[2:4]) / 1.0
        logger.debug("%s y: %f" % (self.__class__.__name__, self.y))
            
        # Extract Z
        self.z = utils.to_short(raw[4:6]) / 1.0
        logger.debug("%s z: %f" % (self.__class__.__name__, self.z))
 
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "X: %f\nY: %f\nZ: %f" % (self.x, self.y, self.z)
        

 
class HorizonPayload_Magnetometer(HorizonPayload):
    """Horizon Message Payload - Magnetometer"""
    
    ## Create A Horizon Message Payload - Magnetometer
    def __init__(self, x = 0, y = 0, z = 0, store_error = False, 
                 raw = None, timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Magnetometer Data"""
        
        # Class Variables
        self.x = 0
        self.y = 0
        self.z = 0 

        # Verify Length
        if raw == None or len(raw) != 6:
            logger.warning("Tried to create %s with bad length!"\
                               % self.__class__.__name__)
            self.error = ValueError( "Bad length!")
            if not store_error: raise self.error  
            else: return
                
        # Pass on to super-class
        HorizonPayload.__init__(self, raw = raw, version = version, 
                                store_error = store_error, timestamp = timestamp)
            
        # Extract X
        self.x = utils.to_short(raw[0:2]) / 1.0
        logger.debug("%s x: %f" % (self.__class__.__name__, self.x))
            
        # Extract Y
        self.y = utils.to_short(raw[2:4]) / 1.0
        logger.debug("%s y: %f" % (self.__class__.__name__, self.y))
            
        # Extract Z
        self.z = utils.to_short(raw[4:6]) / 1.0
        logger.debug("%s z: %f" % (self.__class__.__name__, self.z))
 
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "X: %f\nY: %f\nZ: %f" % (self.x, self.y, self.z)
 
    
## @defgroup platform_6axis Platform 6-Axis
#  @ingroup data_platform_6axis
#  \b Description: The vehicle's best estimates of its translational 
#  acceleration and rotational rates.                                       \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>X</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-32,32]</td><td>m</td></tr>
#  <tr><td>Y</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-32,32]</td><td>m</td></tr>
#  <tr><td>X</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-32,32]</td><td>m</td></tr>
#  <tr><td>Rotational Roll</td><td>signed short</td><td>2 bytes</td>
#  <td>1000</td><td>[-10π,10π]</td><td>rad/s</td></tr>
#  <tr><td>Rotational Pitch</td><td>signed short</td><td>2 bytes</td>
#  <td>1000</td><td>[-10π,10π]</td><td>rad/s</td></tr>
#  <tr><td>Rotational Yaw</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-10π,10π]</td><td>rad/s</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field           | Type         | Size    | Scale | Range      | Units
#  ----------------+--------------+---------+-------+------------+------
#  x               | signed short | 2 bytes | 1000  | [-32,32]   | m
#  y               | signed short | 2 bytes | 1000  | [-32,32]   | m
#  z               | signed short | 2 bytes | 1000  | [-32,32]   | m
#  Rotational Roll | signed short | 2 bytes | 1000  | [-10π,10π] | rad/s
#  Rotational P... | signed short | 2 bytes | 1000  | [-10π,10π] | rad/s
#  Rotational Yaw  | signed short | 2 bytes | 1000  | [-10π,10π] | rad/s
#  @endmanonly                                                                \n
#
#  \b x: Vehicle's acceleration along its x axis.                             \n
#
#  \b y: Vehicle's acceleration along its y axis.                             \n
#
#  \b z: Vehicle's acceleration along its z axis.                             \n
#
#  \b Rotational \b Roll: Vehicle's angular rate about its roll axis, in a 
#  right-hand sense.                                                          \n
#
#  \b Rotational \b Pitch: Vehicle's angular rate about its pitch axis, in a 
#  right-hand sense.                                                          \n
#
#  \b Rotational \b Yaw:              Vehicle‟s angular rate about its yaw 
#  axis, in a right-hand sense. Note: Positive yaw rate corresponds to a left 
#  turn. Compasses traditionally have heading increasing as one turns right.


## Horizon Message Payload - Platform 6-Axis
#
#  Represents the payload of the data message 'platform 6axis'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Platform 6-Axis Data
#  @copydoc platform_6axis
#
#  @pydoc
class HorizonPayload_Platform6Axis(HorizonPayload):
    """Horizon Message Payload - Platform 6-Axis"""
    
    ## Create A Horizon Message Payload - Platform 6-Axis
    #
    #  Constructor for the Horizon Message Payload - Platform 6-Axis Class.   \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_Platform6Axis(x, rot_roll, rot_pitch, raw=None, ...)  \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_Platform6Axis(raw, version, timestamp)                \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  rot_pitch      The vehicle's pitch rotation rate
    #  @param  rot_roll       The vehicle's roll rotation rate
    #  @param  rot_yaw        The vehicle's yaw rotation rate
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @param  x              x-axis acceleration
    #  @param  y              y-axis acceleration
    #  @param  z              z-axis acceleration
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, x = 0, y = 0, z = 0, rot_roll = 0, rot_pitch = 0,
                 rot_yaw = 0, store_error = False, raw = None, timestamp = 0, 
                 version = tuple([-1,0])):
        """Create A Horizon Message Payload - Platform 6-Axis"""
        
        # Class Variables
        ## X-Axis Acceleration
        self._x = 0
        ## Y-Axis Acceleration
        self._y = 0
        ## Z-Axis Acceleration
        self._z = 0
        ## Roll Rotation 
        self._rot_roll = 0
        ## Pitch Rotation
        self._rot_pitch = 0
        ## Yaw Rotation
        self._rot_yaw = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test x
            if x < -32 or x > 32:
                logger.warning("Tried to create %s with bad x!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "X must be [-32,32]!")
                if not store_error: raise self.error  
                else: return
            self._x = x
            data += utils.from_short(int(x*1000))
            
            # test y
            if y < -32 or y > 32:
                logger.warning("Tried to create %s with bad y!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Y must be [-32,32]!")
                if not store_error: raise self.error  
                else: return
            self._y = y
            data += utils.from_short(int(y*1000))
            
            # test z
            if z < -32 or z > 32:
                logger.warning("Tried to create %s with bad z!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Z must be [-32,32]!")
                if not store_error: raise self.error  
                else: return
            self._z = z
            data += utils.from_short(int(z*1000))
            
            # test roll
            if rot_roll < -10*math.pi or rot_roll > 10*math.pi:
                logger.warning("Tried to create %s with bad rotational roll!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Rotational Roll must be [-10π,10π]!")
                if not store_error: raise self.error  
                else: return
            self._rot_roll = rot_roll
            data += utils.from_short(int(rot_roll*1000))
            
            # test pitch
            if rot_pitch < -10*math.pi or rot_pitch > 10*math.pi:
                logger.warning("Tried to create %s with bad rotational pitch!"\
                                 % self.__class__.__name__)
                self.error =ValueError("Rotational Pitch must be [-10π,10π]!")
                if not store_error: raise self.error  
                else: return
            self._rot_pitch = rot_pitch
            data += utils.from_short(int(rot_pitch*1000))
            
            # test yaw
            if rot_yaw < -10*math.pi or rot_yaw > 10*math.pi:
                logger.warning("Tried to create %s with bad rotational yaw!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Rotational Yaw must be [-10π,10π]!")
                if not store_error: raise self.error  
                else: return
            self._rot_yaw = rot_yaw
            data += utils.from_short(int(rot_yaw*1000))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 12:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract X
            self._x = utils.to_short(raw[0:2])/1000.0
            logger.debug("%s x: %f" % (self.__class__.__name__, 
                     self._x))
            
            # Extract Y
            self._y = utils.to_short(raw[2:4])/1000.0
            logger.debug("%s y: %f" % (self.__class__.__name__, 
                     self._y))
            
            # Extract Z
            self._z = utils.to_short(raw[4:6])/1000.0
            logger.debug("%s z: %f" % (self.__class__.__name__, 
                     self._z))
            
            # Extract rotational Roll
            self._rot_roll = utils.to_short(raw[6:8])/1000.0
            logger.debug("%s rotational roll: %f" % (self.__class__.__name__, 
                     self._rot_roll))
            
            # Extract rotational Pitch
            self._rot_pitch = utils.to_short(raw[8:10])/1000.0
            logger.debug("%s rotational pitch: %f" % (self.__class__.__name__, 
                     self._rot_pitch))
            
            # Extract rotational Yaw
            self._rot_yaw = utils.to_short(raw[10:12])/1000.0
            logger.debug("%s rotational yaw: %f" % (self.__class__.__name__, 
                     self._rot_yaw))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "X: %f\nY: %f\nZ: %f\nRotational Roll: %f\nRotational Pitch: %f"\
               "\nRotational Yaw: %f" % (self._x, self._y, self._z, 
                                self._rot_roll, self._rot_pitch, self._rot_yaw)
        
        
    ## Get the vehicle's X acceleration
    #
    #  @return x acceleration
    #
    #  @pydoc
    def get_x(self):
        """Get the vehicle's x acceleration."""
        
        return self._x
        
        
    ## Get the vehicle's y acceleration
    #
    #  @return y acceleration
    #
    #  @pydoc
    def get_y(self):
        """Get the vehicle's y acceleration."""
        
        return self._y
        
        
    ## Get the vehicle's z acceleration
    #
    #  @return z acceleration
    #
    #  @pydoc
    def get_z(self):
        """Get the vehicle's z acceleration."""
        
        return self._z
        
        
    ## Get the vehicle's Roll rotation rate
    #
    #  @return roll rotation rate
    #
    #  @pydoc
    def get_rotational_roll(self):
        """Get the vehicle's Roll rotation rate."""
        
        return self._rot_roll
        
        
    ## Get the vehicle's Pitch rotation rate
    #
    #  @return pitch rotation rate
    #
    #  @pydoc
    def get_rotational_pitch(self):
        """Get the vehicle's Pitch rotation rate."""
        
        return self._rot_pitch
        
        
    ## Get the vehicle's Yaw rotation rate
    #
    #  @return yaw rotation rate
    #
    #  @pydoc
    def get_rotational_yaw(self):
        """Get the vehicle's Yaw rotation rate."""
        
        return self._rot_yaw
    
    
    # Class Properties
    ## x acceleration
    x = property(fget=get_x, doc="x acceleration")
    ## y acceleration
    y = property(fget=get_y, doc="y acceleration")
    ## z acceleration
    z = property(fget=get_z, doc="z acceleration")
    ## Pitch rotation rate
    rotational_pitch = property(fget=get_rotational_pitch, 
                                doc="Pitch rotation rate")
    ## Roll rotation rate
    rotational_roll = property(fget=get_rotational_roll, 
                               doc="Roll rotation rate")
    ## Yaw rotation rate
    rotational_yaw = property(fget=get_rotational_yaw, doc="Yaw rotation rate")
    
    
## @defgroup platform_6axis_orientation Platform 6-Axis Orientation
#  @ingroup data_platform_6axis_orientation
#  \b Description: The vehicle's best estimates of its translational 
#  acceleration, rotational rates, and orientation.                         \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Roll</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  <tr><td>Pitch</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  <tr><td>Yaw</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  <tr><td>X</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-32,32]</td><td>m</td></tr>
#  <tr><td>Y</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-32,32]</td><td>m</td></tr>
#  <tr><td>X</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-32,32]</td><td>m</td></tr>
#  <tr><td>Rotational Roll</td><td>signed short</td><td>2 bytes</td>
#  <td>1000</td><td>[-10π,10π]</td><td>rad/s</td></tr>
#  <tr><td>Rotational Pitch</td><td>signed short</td><td>2 bytes</td>
#  <td>1000</td><td>[-10π,10π]</td><td>rad/s</td></tr>
#  <tr><td>Rotational Yaw</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-10π,10π]</td><td>rad/s</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field           | Type         | Size    | Scale | Range      | Units
#  ----------------+--------------+---------+-------+------------+------
#  Roll            | signed short | 2 bytes | 1000  | [-π,π]     | rad
#  Pitch           | signed short | 2 bytes | 1000  | [-π,π]     | rad
#  Yaw             | signed short | 2 bytes | 1000  | [-π,π]     | rad
#  x               | signed short | 2 bytes | 1000  | [-32,32]   | m
#  y               | signed short | 2 bytes | 1000  | [-32,32]   | m
#  z               | signed short | 2 bytes | 1000  | [-32,32]   | m
#  Rotational Roll | signed short | 2 bytes | 1000  | [-10π,10π] | rad/s
#  Rotational P... | signed short | 2 bytes | 1000  | [-10π,10π] | rad/s
#  Rotational Yaw  | signed short | 2 bytes | 1000  | [-10π,10π] | rad/s
#  @endmanonly                                                                \n
#
#  \b Roll: Vehicle's angle about its roll axis, in a right-hand sense.       \n
#
#  \b Pitch: Vehicle's angle about its pitch axis, in a right-hand sense.     \n
#
#  \b Yaw: Vehicle's global heading with respect to magnetic north, 
#  about its yaw axis, in a right-hand sense. Note: This means that the 
#  vehicle's yaw increases as the vehicle turns towards the west. Compasses 
#  traditionally have heading increasing as one turns east. If a vehicle is 
#  only equipped with tilt sensors, this field will default to 0.             \n
#
#  \b x: Vehicle's acceleration along its x axis.                             \n
#
#  \b y: Vehicle's acceleration along its y axis.                             \n
#
#  \b z: Vehicle's acceleration along its z axis.                             \n
#
#  \b Rotational \b Roll: Vehicle's angular rate about its roll axis, in a 
#  right-hand sense.                                                          \n
#
#  \b Rotational \b Pitch: Vehicle's angular rate about its pitch axis, in a 
#  right-hand sense.                                                          \n
#
#  \b Rotational \b Yaw: Vehicle's angular rate about its yaw 
#  axis, in a right-hand sense. Note: Positive yaw rate corresponds to a left 
#  turn. Compasses traditionally have heading increasing as one turns right.


## Horizon Message Payload - Platform 6-Axis Orientation
#
#  Represents the payload of the data message 'platform 6axis orientation'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Platform 6-Axis Orientation Data
#  @copydoc platform_6axis_orientation
#
#  @pydoc
class HorizonPayload_Platform6AxisOrientation(HorizonPayload):
    """Horizon Message Payload - Platform 6-Axis Orientation"""
    
    
    ## Create A Horizon Message Payload - Platform 6-Axis Orientation
    #
    #  Constructor for the Horizon Message Payload - Platform 6-Axis 
    #  Orientation Class.                                                     \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_Platform6AxisOrientation(x, rot_roll, raw=None, ...)  \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_Platform6AxisOrientation(raw, version, timestamp)     \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  pitch          The vehicle's pitch angle
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  roll           The vehicle's roll angle
    #  @param  rot_pitch      The vehicle's pitch rotation rate
    #  @param  rot_roll       The vehicle's roll rotation rate
    #  @param  rot_yaw        The vehicle's yaw rotation rate
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @param  x              x-axis acceleration
    #  @param  y              y-axis acceleration
    #  @param  yaw            The vehicle's yaw (magnetic) angle
    #  @param  z              z-axis acceleration
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, roll = 0, pitch = 0, yaw = 0, x = 0, y = 0, z = 0, 
                 rot_roll = 0, rot_pitch = 0, rot_yaw = 0, store_error = False, 
                 raw = None, timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Platform 6-Axis Orientation"""
        
        # Class Variables
        ## Roll Angle
        self._roll = 0
        ## Pitch Angle
        self._pitch = 0
        ## Yaw Angle
        self._yaw = 0
        ## X-Axis Acceleration
        self._x = 0
        ## Y-Axis Acceleration
        self._y = 0
        ## Z-Axis Acceleration
        self._z = 0
        ## Roll Rotation
        self._rot_roll = 0
        ## Pitch Rotation
        self._rot_pitch = 0
        ## Yaw Rotation
        self._rot_yaw = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test roll
            if roll < -math.pi or roll > math.pi:
                logger.warning("Tried to create %s with bad roll!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Roll must be [-π,π]!")
                if not store_error: raise self.error  
                else: return
            self._roll = roll
            data += utils.from_short(int(roll*1000))
            
            # test pitch
            if pitch < -math.pi or pitch > math.pi:
                logger.warning("Tried to create %s with bad pitch!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Pitch must be [-π,π]!")
                if not store_error: raise self.error  
                else: return
            self._pitch = pitch
            data += utils.from_short(int(pitch*1000))
            
            # test yaw
            if yaw < -math.pi or yaw > math.pi:
                logger.warning("Tried to create %s with bad yaw!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Yaw must be [-π,π]!")
                if not store_error: raise self.error  
                else: return
            self._yaw = yaw
            data += utils.from_short(int(yaw*1000))
            
            # test x
            if x < -32 or x > 32:
                logger.warning("Tried to create %s with bad x!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "X must be [-32,32]!")
                if not store_error: raise self.error  
                else: return
            self._x = x
            data += utils.from_short(int(x*1000))
            
            # test y
            if y < -32 or y > 32:
                logger.warning("Tried to create %s with bad y!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Y must be [-32,32]!")
                if not store_error: raise self.error  
                else: return
            self._y = y
            data += utils.from_short(int(y*1000))
            
            # test z
            if z < -32 or z > 32:
                logger.warning("Tried to create %s with bad z!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Z must be [-32,32]!")
                if not store_error: raise self.error  
                else: return
            self._z = z
            data += utils.from_short(int(z*1000))
            
            # test roll
            if rot_roll < -10*math.pi or rot_roll > 10*math.pi:
                logger.warning("Tried to create %s with bad rotational roll!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Rotational Roll must be [-10π,10π]!")
                if not store_error: raise self.error  
                else: return
            self._rot_roll = rot_roll
            data += utils.from_short(int(rot_roll*1000))
            
            # test pitch
            if rot_pitch < -10*math.pi or rot_pitch > 10*math.pi:
                logger.warning("Tried to create %s with bad rotational pitch!"\
                                 % self.__class__.__name__)
                self.error =ValueError("Rotational Pitch must be [-10π,10π]!")
                if not store_error: raise self.error  
                else: return
            self._rot_pitch = rot_pitch
            data += utils.from_short(int(rot_pitch*1000))
            
            # test yaw
            if rot_yaw < -10*math.pi or rot_yaw > 10*math.pi:
                logger.warning("Tried to create %s with bad rotational yaw!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Rotational Yaw must be [-10π,10π]!")
                if not store_error: raise self.error  
                else: return
            self._rot_yaw = rot_yaw
            data += utils.from_short(int(rot_yaw*1000))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 18:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Roll
            self._roll = utils.to_short(raw[0:2])/1000.0
            logger.debug("%s roll: %f" % (self.__class__.__name__, 
                     self._roll))
            
            # Extract Pitch
            self._pitch = utils.to_short(raw[2:4])/1000.0
            logger.debug("%s pitch: %f" % (self.__class__.__name__, 
                     self._pitch))
            
            # Extract Yaw
            self._yaw = utils.to_short(raw[4:6])/1000.0
            logger.debug("%s yaw: %f" % (self.__class__.__name__, 
                     self._yaw))
            
            # Extract X
            self._x = utils.to_short(raw[6:8])/1000.0
            logger.debug("%s x: %f" % (self.__class__.__name__, 
                     self._x))
            
            # Extract Y
            self._y = utils.to_short(raw[8:10])/1000.0
            logger.debug("%s y: %f" % (self.__class__.__name__, 
                     self._y))
            
            # Extract Z
            self._z = utils.to_short(raw[10:12])/1000.0
            logger.debug("%s z: %f" % (self.__class__.__name__, 
                     self._z))
            
            # Extract rotational Roll
            self._rot_roll = utils.to_short(raw[12:14])/1000.0
            logger.debug("%s rotational roll: %f" % (self.__class__.__name__, 
                     self._rot_roll))
            
            # Extract rotational Pitch
            self._rot_pitch = utils.to_short(raw[14:16])/1000.0
            logger.debug("%s rotational pitch: %f" % (self.__class__.__name__, 
                     self._rot_pitch))
            
            # Extract rotational Yaw
            self._rot_yaw = utils.to_short(raw[16:18])/1000.0
            logger.debug("%s rotational yaw: %f" % (self.__class__.__name__, 
                     self._rot_yaw))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Roll: %f\nPitch: %f\nYaw: %f\nX: %f\nY: %f\nZ: %f\n"\
               "Rotational Roll: %f\nRotational Pitch: %f\nRotational Yaw: %f"\
                % (self._roll, self._pitch, self._yaw, self._x, self._y, 
                   self._z, self._rot_roll, self._rot_pitch, self._rot_yaw)
        
        
    ## Get the vehicle's Roll
    #
    #  @return roll angle
    #
    #  @pydoc
    def get_roll(self):
        """Get the vehicle's Roll."""
        
        return self._roll
        
        
    ## Get the vehicle's Pitch
    #
    #  @return pitch angle
    #
    #  @pydoc
    def get_pitch(self):
        """Get the vehicle's Pitch."""
        
        return self._pitch
        
        
    ## Get the vehicle's Yaw
    #
    #  @return yaw angle
    #
    #  @pydoc
    def get_yaw(self):
        """Get the vehicle's Yaw."""
        
        return self._yaw
        
        
    ## Get the vehicle's X acceleration
    #
    #  @return x acceleration
    #
    #  @pydoc
    def get_x(self):
        """Get the vehicle's x acceleration."""
        
        return self._x
        
        
    ## Get the vehicle's y acceleration
    #
    #  @return y acceleration
    #
    #  @pydoc
    def get_y(self):
        """Get the vehicle's y acceleration."""
        
        return self._y
        
        
    ## Get the vehicle's z acceleration
    #
    #  @return z acceleration
    #
    #  @pydoc
    def get_z(self):
        """Get the vehicle's z acceleration."""
        
        return self._z
        
        
    ## Get the vehicle's Roll rotation rate
    #
    #  @return roll rotation rate
    #
    #  @pydoc
    def get_rotational_roll(self):
        """Get the vehicle's Roll rotation rate."""
        
        return self._rot_roll
        
        
    ## Get the vehicle's Pitch rotation rate
    #
    #  @return pitch rotation rate
    #
    #  @pydoc
    def get_rotational_pitch(self):
        """Get the vehicle's Pitch rotation rate."""
        
        return self._rot_pitch
        
        
    ## Get the vehicle's Yaw rotation rate
    #
    #  @return yaw rotation rate
    #
    #  @pydoc
    def get_rotational_yaw(self):
        """Get the vehicle's Yaw rotation rate."""
        
        return self._rot_yaw
    
    
    # Class Properties
    ## Pitch Angle
    pitch = property(fget=get_pitch, doc="Pitch Angle")
    ## Roll Angle
    roll = property(fget=get_roll, doc="Roll Angle")
    ## Yaw Angle
    yaw = property(fget=get_yaw, doc="Yaw Angle")
    ## x acceleration
    x = property(fget=get_x, doc="x acceleration")
    ## y acceleration
    y = property(fget=get_y, doc="y acceleration")
    ## z acceleration
    z = property(fget=get_z, doc="z acceleration")
    ## Pitch rotation rate
    rotational_pitch = property(fget=get_rotational_pitch, 
                                doc="Pitch rotation rate")
    ## Roll rotation rate
    rotational_roll = property(fget=get_rotational_roll, 
                               doc="Roll rotation rate")
    ## Yaw rotation rate
    rotational_yaw = property(fget=get_rotational_yaw, doc="Yaw rotation rate")
    
    
    
## @defgroup encoders Encoders
#  @ingroup data_encoders
#  \b Description: Data from the vehicle's n encoders. The payload will be 
#  6n + 1 bytes in size.                                                    \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Count</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Travel</td><td>signed int</td><td>4n bytes</td><td>1000</td>
#  <td>[-2*10^6,2*10^6]</td><td>m</td></tr>
#  <tr><td>Speed</td><td>signed short</td><td>2n bytes</td><td>1000</td>
#  <td>[-32,32]</td><td>m/s</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field  | Type         | Size     | Scale | Range            | Units
#  -------+--------------+----------+-------+------------------+------
#  Count  | byte         | 1 byte   | -     | -                | -
#  Travel | signed int   | 4n bytes | 1000  | [-2*10^6,2*10^6] | m
#  Speed  | signed short | 2n bytes | 1000  | [-32,32]         | m/s
#  @endmanonly                                                                \n
#
#  \b Count: The number of encoder measurements being returned. The position 
#  of each encoder is platform-dependent.                                     \n
#
#  \b Travel: The distance the encoder will have driven since startup under 
#  ideal non-slip conditions.                                                \n
#
#  \b Speed: The current instantaneous speed of the encoder.


## Horizon Message Payload - Encoders
#
#  Represents the payload of the data message 'encoders'
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Encoder Data
#  @copydoc encoders
#
#  @pydoc
class HorizonPayload_Encoders(HorizonPayload):
    """Horizon Message Payload - Encoders"""
    
    
    ## Create A Horizon Message Payload - Encoders
    #
    #  Constructor for the Horizon Message Payload - Encoders.                \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_Encoders(travel, speed, raw=None, ...)                \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_Encoders(raw, version, timestamp)                     \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  speed          A list of encoder speeds
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  travel         A list of encoder distances
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, travel = [], speed = [], raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Encoders"""
        
        # Class Variables
        ## Encoder Speeds
        self._speed = []
        ## Encoder Distances
        self._travel = []
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test length
            if len(travel) != len(speed):
                logger.warning("Tried to create %s with different number of "\
                               "distances and speeds!"\
                                     % self.__class__.__name__)
                self.error = ValueError(
                               "Must have same number of distances and speeds!")
                if not store_error: raise self.error  
                else: return
            if len(travel) > 255:
                logger.warning("Tried to create %s with too many encoders!"\
                                     % self.__class__.__name__)
                self.error = ValueError(
                               "Must have 0-255 encoders!")
                if not store_error: raise self.error  
                else: return
            data = utils.from_byte(len(speed))
            
            # test distances
            for t in travel:
                if t < -32 or t > 32:
                    logger.warning("Tried to create %s with bad distances!"\
                                     % self.__class__.__name__)
                    self.error = ValueError(
                                   "Distances must be [-2x10^6,2x10^6]!")
                    if not store_error: raise self.error  
                    else: return
                data += utils.from_int(int(t*1000))
            self._travel = travel
            
            # test speeds
            for s in speed:
                if s < -32 or s > 32:
                    logger.warning("Tried to create %s with bad speeds!"\
                                     % self.__class__.__name__)
                    self.error = ValueError( "Speeds must be [-32,32]!")
                    if not store_error: raise self.error  
                    else: return
                data += utils.from_short(int(s*1000))
            self._speed = speed
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) < 1 or len(raw) != raw[0]*6+1:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Distances
            self._travel = []
            for i in range(0,raw[0]):
                self._travel.append(utils.to_int(
                                                raw[i*4+1:(i+1)*4+1])/1000.0)
            logger.debug("%s distances: %s" % (self.__class__.__name__, 
                     self._travel))
            
            # Extract Speeds
            self._speed = []
            for i in range(0,raw[0]):
                self._speed.append(utils.to_short(\
                                raw[i*2+raw[0]*4+1:(i+1)*2+raw[0]*4+1])/1000.0)
            logger.debug("%s speeds: %s" % (self.__class__.__name__, 
                     self._speed))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Distances: %s\nSpeeds: %s" % (\
                                        'm, '.join(map(str,self._travel))+'m', 
                                       'm/s, '.join(map(str,self._speed))+'m/s')
        
        
    ## Get the number of Encoders
    #
    #  @return number of encoders
    #
    #  @pydoc
    def get_count(self):
        """Get the number of Encoders."""
        
        return len(self._speed)
                
                
    ## Get Speed
    #
    #  @param  encoder  The encoder to get the speed of.
    #                   If -1 then it returns a list of all encoders
    #  @return the encoder speed (m/s), or all encoder speeds
    #
    #  @pydoc
    def get_speed(self, encoder = -1):
        """Get Encoder"""
        
        # all encoders
        if encoder < 0 or encoder >= len(self._speed):
            return self._speed
        
        # single encoder
        else:
            return self._speed[encoder]
                
                
    ## Get Distance
    #
    #  @param  encoder  The encoder to get the distance of.
    #                   If -1 then it returns a list of all encoders
    #  @return the distance of encoder (metres), or all encoder distances
    #
    #  @pydoc
    def get_travel(self, encoder = -1):
        """Get Distance"""
        
        # all encoders
        if encoder < 0 or encoder >= len(self._speed):
            return self._travel
        
        # single encoder
        else:
            return self._travel[encoder]
    
    
    # Class Properties
    ## Encoder Speed
    speed = property(fget=get_speed, doc="Encoder Speed")
    ## Encoder Distance
    travel = property(fget=get_travel, doc="Encoder Distance")
    
    
## @defgroup raw_encoders Raw Encoders
#  @ingroup data_raw_encoders
#  \b Description: Raw data from the vehicle's n encoders. The payload will be 
#  4n + 1 bytes in size.                                                    \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Count</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Ticks</td><td>signed int</td><td>4n bytes</td><td>1</td>
#  <td>[-2^31,2^31-1]</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field  | Type       | Size     | Scale | Range          | Units
#  -------+------------+----------+-------+----------------+------
#  Count  | byte       | 1 byte   | -     | -              | -
#  Ticks  | signed int | 4n bytes | 1     | [-2^31,2^31-1] | -
#  @endmanonly                                                                \n
#
#  \b Count: The number of encoder measurements being returned. The position 
#  of each encoder is platform-dependent.                                     \n
#
#  \b Ticks: The amount of ticks accumulated by the encoder. Forward travel 
#  increments this value, backwards travel decrements this value.
#


## Horizon Message Payload - Raw Encoders
#
#  Represents the payload of the data message 'raw encoders'
#
#  @warning Data should not be modified once created
#
#  @since 0.8
#
#  @section data Raw Encoder Data
#  @copydoc raw_encoders
#
#  @pydoc
class HorizonPayload_RawEncoders(HorizonPayload):
    """Horizon Message Payload - Raw Encoders"""
    
    
    
    ## Create A Horizon Message Payload - Raw Encoders
    #
    #  Constructor for the Horizon Message Payload - Raw Encoders.            \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_Encoders(ticks, raw=None, ...)                        \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_Encoders(raw, version, timestamp)                     \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  ticks          A list of encoder ticks
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, ticks = [], raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Raw Encoders"""
        
        # Class Variables
        ## Encoder Ticks
        self._ticks = []
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test length
            if len(ticks) > 255:
                logger.warning("Tried to create %s with too many encoders!"\
                                     % self.__class__.__name__)
                self.error = ValueError(
                               "Must have 0-255 encoders!")
                if not store_error: raise self.error  
                else: return
            data = utils.from_byte([len(ticks)])
            
            # test ticks
            for t in ticks:
                if t < -math.pow(2,31) or t > math.pow(2,31)-1:
                    logger.warning("Tried to create %s with bad ticks!"\
                                     % self.__class__.__name__)
                    self.error = ValueError( "Ticks must be [-2^31,2^31-1]!")
                    if not store_error: raise self.error  
                    else: return
                data += utils.from_int(int(t))
            self._ticks = ticks
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) < 1 or len(raw) != raw[0]*4+1:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Ticks
            self._ticks = []
            for i in range(0,raw[0]):
                self._ticks.append(utils.to_int(\
                                raw[i*4+1:(i+1)*4+1]))
            logger.debug("%s ticks: %s" % (self.__class__.__name__, 
                     self._ticks))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Ticks: %s" % (' '.join(map(str,self._ticks)))
        
        
    ## Get the number of Encoders
    #
    #  @return number of encoders
    #
    #  @pydoc
    def get_count(self):
        """Get the number of Encoders."""
        
        return len(self._ticks)
                
                
    ## Get Ticks
    #
    #  @param  encoder  The encoder to get the ticks of.
    #                   If -1 then it returns a list of all encoders
    #  @return the encoder ticks, or all encoder ticks
    #
    #  @pydoc
    def get_ticks(self, encoder = -1):
        """Get Ticks"""
        
        # all encoders
        if encoder < 0 or encoder >= len(self._ticks):
            return self._ticks[:]
        
        # single encoder
        else:
            return self._ticks[encoder]
    
    
    # Class Properties
    ## Encoder Ticks
    ticks = property(fget=get_ticks, doc="Encoder Ticks")
    
    
    
## @defgroup absolute_joint_position_targets Absolute Joint Position Targets
#  @ingroup set_absolute_joint_position
#  \b Description: The current desired joint positions. Platform will use its 
#  onboard control to the best of its ability to maintain these positions.  \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Count</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>ID 1</td><td>unsigned byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Angle 1</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  <tr><td colspan="6">...</td></tr>
#  <tr><td>ID n</td><td>unsigned byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Angle n</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field   | Type         | Size    | Scale | Range  | Units
#  --------+--------------+---------+-------+--------+------
#  Count   | byte         | 1 byte  | -     | -      | -
#  ID 1    | byte         | 1 byte  | -     | -      | -
#  Angle 1 | signed short | 2 bytes | 1000  | [-π,π] | rad
#  ...     | ...          | ...     | ...   | ...    | ...
#  ID n    | byte         | 1 byte  | -     | -      | -
#  Angle n | signed short | 2 bytes | 1000  | [-π,π] | rad
#  @endmanonly                                                                \n
#
#  \b Count: The amount of joint angles to set. The position and ID of each 
#  joint is manipulator-dependent.                                            \n
#
#  \b ID x: The identifier of the joint to be controlled. Identifiers are 
#  dependent on the specific manipulator hardware.                            \n
#
#  \b Angle x: The angle the joint is attempting to attain, in radians. 
#  The definition of which direction is a positive turn is dependent on the 
#  specific manipulator hardware.



## Horizon Message Payload - Absolute Joint Position Targets
#
#  Represents the payload of the command message
#  'absolute joint position targets'
#
#  @warning Data should not be modified once created
#
#  @since 0.4
#
#  @section data Absolute Joint Position Targets Data
#  @copydoc absolute_joint_position_targets
#
#  @pydoc
class HorizonPayload_AbsoluteJointPositionTargets(HorizonPayload):
    """Horizon Message Payload - Absolute Joint Position Targets"""
    
    
    ## Create A Horizon Message Payload - Absolute Joint Position Targets
    #
    #  Constructor for the Horizon Message Payload - Absolute Joint Position 
    #  Targets Class.                                                         \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_AbsoluteJointPositionTargets(angles, raw=None, ...)   \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_AbsoluteJointPositionTargets(raw, version, timestamp) \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  angles         Dictionary of joint IDs mapping to angles [-π,π]
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, angles = {}, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Absolute Joint Position Targets"""
        
        # Class Variables
        ## Joint Angles
        self._angles = {}
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test length
            if len(angles) > 256:
                logger.warning("Tried to create %s with too many angles!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Number of angles must be 0-256!")
                if not store_error: raise self.error  
                else: return
            data = utils.from_byte(len(angles))
            
            # test values
            for id in angles:
                if id < 0 or id > 255:
                    logger.warning("Tried to create %s with bad id!"\
                                 % self.__class__.__name__)
                    self.error = ValueError( "ID must be [0,255]!")
                    if not store_error: raise self.error  
                    else: return
                if angles[id] < -math.pi or angles[id] > math.pi:
                    logger.warning("Tried to create %s with bad angle!"\
                                 % self.__class__.__name__)
                    self.error = ValueError( "Angle must be [-π,π]!")
                    if not store_error: raise self.error  
                    else: return
                data += utils.from_byte(id)
                data += utils.from_short(int(angles[id]*1000))
            self._angles = angles
            
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) < 1 or len(raw) != raw[0]*3 + 1:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Values
            for i in range(0,raw[0]):
                self._angles[utils.to_byte(raw[i*3+1:i*3+2])/1000.0] = \
                    utils.to_short(raw[i*3+2:i*3+4])/1000.0
            logger.debug("%s angles: %s" % (self.__class__.__name__, 
                                            str(self._angles)))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        angs = ''
        for id in self._angles:
            angs += '%d = %frad  ' % (id,self._angles[id])
        return "Count: %d\nAngles: %s" % (len(self._torques), angs)
                
                
    ## Get Count
    #
    #  @return number of angles
    #
    #  @pydoc
    def get_count(self):
        """Get Count"""
        
        return len(self._angles)
                
                
    ## Get Angle
    #
    #  @param  joint joint of angle to get
    #  @return the joint's angle, or list of all angles if joint is -1
    #
    #  @pydoc
    def get_angle(self, joint=-1):
        """Get Angle"""
        
        # singular
        if joint in self._angles:
            return self._angles[joint]
        
        # multiple
        else:
            return self._angles.copy()
    
    
    # Class Properties
    ## Angle Count
    count = property(fget=get_count, doc="Angle Count")
    ## Joint Angle
    angle = property(fget=get_angle, doc="Joint Angle")
    
    
    
    
## @defgroup absolute_joint_position Absolute Joint Position
#  @ingroup data_absolute_joint_position
#  \b Description: The current desired absolute joint positions. The payload 
#  will be 2n + 1 bytes in size, where n is the amount of joints on the 
#  manipulator.                                                             \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Count</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Angle</td><td>signed short</td><td>2n bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field | Type         | Size     | Scale | Range  | Units
#  ------+--------------+----------+-------+--------+------
#  Count | byte         | 1 byte   | -     | -      | -
#  Angle | signed short | 2n bytes | 1000  | [-π,π] | rad
#  @endmanonly                                                                \n
#
#  \b Count: The amount of joint angles being returned. The position of each 
#  joint is manipulator-dependent.                                            \n
#
#  \b Angle: The angle the joint is attempting to attain, in radians. 
#  The definition of which direction is a positive turn is dependent on the 
#  specific manipulator hardware.



## Horizon Message Payload - Absolute Joint Position
#
#  Represents the payload of the data message 'absolute joint position'
#  @warning Data should not be modified once created
#
#  @since 1.0
#
#  @section data Absolute Joint Position Data
#  @copydoc absolute_joint_position
#
#  @pydoc
class HorizonPayload_AbsoluteJointPosition(HorizonPayload):
    """Horizon Message Payload - Absolute Joint Position"""
    
    
    
    ## Create A Horizon Message Payload - Absolute Joint Position
    #
    #  Constructor for the Horizon Message Payload - Absolute Joint Position 
    #  Class.                                                                 \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_AbsoluteJointPosition(angles, raw=None, version, ...) \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_AbsoluteJointPosition(raw, version, timestamp)        \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  angles         List of joint angles [-π,π]
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, angles = {}, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Absolute Joint Position"""
        
        # Class Variables
        ## Joint Angles
        self._angles = {}
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test length
            if len(angles) > 256:
                logger.warning("Tried to create %s with too many angles!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Number of angles must be 0-256!")
                if not store_error: raise self.error  
                else: return
            data = utils.from_byte(len(angles))
            
            # test values
            for angle in angles:
                if angle < -math.pi or angle > math.pi:
                    logger.warning("Tried to create %s with bad angle!"\
                                 % self.__class__.__name__)
                    self.error = ValueError( "Angle must be [-π,π]!")
                    if not store_error: raise self.error  
                    else: return
                data += utils.from_short(int(angle*1000))
            self._angles = angles
            
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) < 1 or len(raw) != raw[0]*2 + 1:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Values
            for i in range(0,raw[0]):
                self._angles +=utils.to_short(raw[i*2+1:i*2+3])/1000.0
            logger.debug("%s angles: %s" % (self.__class__.__name__, 
                     'rad '.join(self._angles) + 'rad '))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Count: %d\nAngles: %s" % (len(self._torques), 
                                           'rad '.join(self._angles) + 'rad')
                
                
    ## Get Count
    #
    #  @return number of angles
    #
    #  @pydoc
    def get_count(self):
        """Get Count"""
        
        return len(self._angles)
                
                
    ## Get Angle
    #
    #  @param  joint joint of angle to get
    #  @return the joint's angle, or list of all angles if joint is -1
    #
    #  @pydoc
    def get_angle(self, joint=-1):
        """Get Angle"""
        
        # singular
        if joint > -1 and joint < len(self._angles):
            return self._angles[joint]
        
        # multiple
        else:
            return self._angles[:]
    
    
    # Class Properties
    ## Angle Count
    count = property(fget=get_count, doc="Angle Count")
    ## Joint Angle
    angle = property(fget=get_angle, doc="Joint Angle")
    
    
    
    
## @defgroup relative_joint_position_targets Relative Joint Position Targets
#  @ingroup set_relative_joint_position
#  \b Description: A set of relative joint positions. Platform will use its 
#  onboard control to the best of its ability to maintain these positions.  \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Count</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>ID 1</td><td>unsigned byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Angle 1</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  <tr><td colspan="6">...</td></tr>
#  <tr><td>ID n</td><td>unsigned byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Angle n</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field   | Type         | Size    | Scale | Range  | Units
#  --------+--------------+---------+-------+--------+------
#  Count   | byte         | 1 byte  | -     | -      | -
#  ID 1    | byte         | 1 byte  | -     | -      | -
#  Angle 1 | signed short | 2 bytes | 1000  | [-π,π] | rad
#  ...     | ...          | ...     | ...   | ...    | ...
#  ID n    | byte         | 1 byte  | -     | -      | -
#  Angle n | signed short | 2 bytes | 1000  | [-π,π] | rad
#  @endmanonly                                                                \n
#
#  \b Count: The amount of joint angles to set. The position and ID of each 
#  joint is manipulator-dependent.                                            \n
#
#  \b ID x: The identifier of the joint to be controlled. Identifiers are 
#  dependent on the specific manipulator hardware.                            \n
#
#  \b Angle x: The relative angle the joint is attempting to attain, in radians. 
#  The definition of which direction is a positive turn is dependent on the 
#  specific manipulator hardware.


## Horizon Message Payload - Relative Joint Position Targets
#
#  Represents the payload of the command message 
#  'relative joint position targets'
#
#  @warning Data should not be modified once created
#
#  @since 0.4
#
#  @section data Relative Joint Position Targets Data
#  @copydoc relative_joint_position_targets
#
#  @pydoc
class HorizonPayload_RelativeJointPositionTargets(HorizonPayload):
    """Horizon Message Payload - Relative Joint Position Targets"""
    
    
    ## Create A Horizon Message Payload - Relative Joint Position Targets
    #
    #  Constructor for the Horizon Message Payload - Relative Joint Position 
    #  Targets Class.                                                         \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_RelativeJointPositionTargets(angles, raw=None, ...)   \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_RelativeJointPositionTargets(raw, version, timestamp) \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  angles         Dictionary of joint IDs mapping to joint angles
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, angles = {}, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Relative Joint Position Targets"""
        
        # Class Variables
        ## Joint Angles
        self._angles = {}
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test length
            if len(angles) > 256:
                logger.warning("Tried to create %s with too many angles!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Number of angles must be 0-256!")
                if not store_error: raise self.error  
                else: return
            data = utils.from_byte(len(angles))
            
            # test values
            for id in angles:
                if id < 0 or id > 255:
                    logger.warning("Tried to create %s with bad id!"\
                                 % self.__class__.__name__)
                    self.error = ValueError( "ID must be [0,255]!")
                    if not store_error: raise self.error  
                    else: return
                if angles[id] < -math.pi or angles[id] > math.pi:
                    logger.warning("Tried to create %s with bad angle!"\
                                 % self.__class__.__name__)
                    self.error = ValueError( "Angle must be [-π,π]!")
                    if not store_error: raise self.error  
                    else: return
                data += utils.from_byte(id)
                data += utils.from_short(int(angles[id]*1000))
            self._angles = angles
            
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) < 1 or len(raw) != raw[0]*3 + 1:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Values
            for i in range(0,raw[0]):
                self._angles[utils.to_byte(raw[i*3+1:i*3+2])/1000.0] = \
                    utils.to_short(raw[i*3+2:i*3+4])/1000.0
            logger.debug("%s angles: %s" % (self.__class__.__name__, 
                                            str(self._angles)))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        angs = ''
        for id in self._angles:
            angs += '%d = %frad  ' % (id,self._angles[id])
        return "Count: %d\nAngles: %s" % (len(self._torques), angs)
                
                
    ## Get Count
    #
    #  @return number of angles
    #
    #  @pydoc
    def get_count(self):
        """Get Count"""
        
        return len(self._angles)
                
                
    ## Get Angle
    #
    #  @param  joint joint of angle to get
    #  @return the joint's angle, or list of all angles if joint is -1
    #
    #  @pydoc
    def get_angle(self, joint=-1):
        """Get Angle"""
        
        # singular
        if joint in self._angles:
            return self._angles[joint]
        
        # multiple
        else:
            return self._angles.copy()
    
    
    # Class Properties
    ## Angle Count
    count = property(fget=get_count, doc="Angle Count")
    ## Joint Angle
    angle = property(fget=get_angle, doc="Joint Angle")
    
    
    
## @defgroup relative_joint_position Relative Joint Position
#  @ingroup data_relative_joint_position
#  \b Description: The current desired relative joint positions. As these 
#  positions are relative to the angle the joints were at when the relative 
#  motion command was received, this information is not necessarily helpful 
#  after the movement has started. The payload will be 2n + 1 bytes in size, 
#  where n is the amount of joints on the manipulator.                      \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Count</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Angle</td><td>signed short</td><td>2n bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field | Type         | Size     | Scale | Range  | Units
#  ------+--------------+----------+-------+--------+------
#  Count | byte         | 1 byte   | -     | -      | -
#  Angle | signed short | 2n bytes | 1000  | [-π,π] | rad
#  @endmanonly                                                                \n
#
#  \b Count: The amount of joint angles being returned. The position of each 
#  joint is manipulator-dependent.                                            \n
#
#  \b Angle: The relative angle the joint is attempting to attain, in radians. 
#  The definition of which direction is a positive turn is dependent on the 
#  specific manipulator hardware.


## Horizon Message Payload - Relative Joint Position
#
#  Represents the payload of the data message 'relative joint position'
#
#  @warning Data should not be modified once created
#
#  @since 0.4
#
#  @section data Relative Joint Position Data
#  @copydoc relative_joint_position
#
#  @pydoc
class HorizonPayload_RelativeJointPosition(HorizonPayload):
    """Horizon Message Payload - Relative Joint Position"""
    
    
    ## Create A Horizon Message Payload - Relative Joint Position
    #
    #  Constructor for the Horizon Message Payload - Relative Joint Position 
    #  Class.                                                                 \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_RelativeJointPosition(angles, raw=None, version, ...) \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_RelativeJointPosition(raw, version, timestamp)        \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  angles         List of joint angles [-π,π]
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, angles = [], raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Relative Joint Position"""
        
        # Class Variables
        ## Joint Angles
        self._angles = []
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test length
            if len(angles) > 256:
                logger.warning("Tried to create %s with too many angles!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Number of angles must be 0-256!")
                if not store_error: raise self.error  
                else: return
            data = utils.from_byte(len(angles))
            
            # test values
            for angle in angles:
                if angle < -math.pi or angle > math.pi:
                    logger.warning("Tried to create %s with bad angle!"\
                                 % self.__class__.__name__)
                    self.error = ValueError( "Angle must be [-π,π]!")
                    if not store_error: raise self.error  
                    else: return
                data += utils.from_short(int(angle*1000))
            self._angles = angles
            
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) < 1 or len(raw) != raw[0]*2 + 1:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Values
            for i in range(0,raw[0]):
                self._angles +=utils.to_short(raw[i*2+1:i*2+3])/1000.0
            logger.debug("%s angles: %s" % (self.__class__.__name__, 
                     'rad '.join(self._angles) + 'rad '))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Count: %d\nAngles: %s" % (len(self._torques), 
                                           'rad '.join(self._angles) + 'rad')
                
                
    ## Get Count
    #
    #  @return number of angles
    #
    #  @pydoc
    def get_count(self):
        """Get Count"""
        
        return len(self._angles)
                
                
    ## Get Angle
    #
    #  @param  joint joint of angle to get
    #  @return the joint's angle, or list of all angles if joint is -1
    #
    #  @pydoc
    def get_angle(self, joint=-1):
        """Get Angle"""
        
        # singular
        if joint > -1 and joint < len(self._angles):
            return self._angles[joint]
        
        # multiple
        else:
            return self._angles[:]
    
    
    # Class Properties
    ## Angle Count
    count = property(fget=get_count, doc="Angle Count")
    ## Joint Angle
    angle = property(fget=get_angle, doc="Joint Angle")



## @defgroup joint_control Joint Control
#  @ingroup set_joint_control data_joint_control
#  \b Description: The control constants for the specified joint on a 
#  manipulator. Input into each controller is the position error (in radians), 
#  and the output is the torque the corresponding motor's should exert.     \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Joint ID</td><td>byte</td><td>1 byte</td><td>-</td><td>-</td>
#  <td>-</td></tr>
#  <tr><td>P</td><td>signed short</td><td>2 bytes</td><td>100</td>
#  <td>[-320,320]</td><td>-</td></tr>
#  <tr><td>I</td><td>signed short</td><td>2 bytes</td><td>100</td>
#  <td>[-320,320]</td><td>-</td></tr>
#  <tr><td>D</td><td>signed short</td><td>2 bytes</td><td>100</td>
#  <td>[-320,320]</td><td>-</td></tr>
#  <tr><td>Feed-Forward</td><td>signed short</td><td>2 bytes</td><td>100</td>
#  <td>[-320,320]</td><td>-</td></tr>
#  <tr><td>Striction Compensation</td><td>signed short</td><td>2 bytes</td>
#  <td>100</td><td>[0,100]</td><td>N-m</td></tr>
#  <tr><td>Integral Limit</td><td>signed short</td><td>2 bytes</td>
#  <td>100</td><td>[0,100]</td><td>N-m</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field          | Type         | Size    | Scale | Range      | Units
#  ---------------+--------------+---------+-------+------------+------
#  Joint ID       | byte         | 1 byte  | -     | -          | -
#  P              | signed short | 2 bytes | 100   | [-320,320] | -
#  I              | signed short | 2 bytes | 100   | [-320,320] | -
#  D              | signed short | 2 bytes | 100   | [-320,320] | -
#  Feed-Forward   | signed short | 2 bytes | 100   | [-320,320] | -
#  Striction      | signed short | 2 bytes | 100   | [0,100]    | N-m
#  Integral Limit | signed short | 2 bytes | 100   | [0,100]    | N-m
#  @endmanonly                                                                \n
#
#  \b Joint ID: The joint the data corresponds to. The position of each joint 
#  is manipulator-dependent.                                                  \n
#
#  \b P: The proportional constant for the control loop.                      \n
#
#  \b I: The integral constant for the control loop.                          \n
#
#  \b D: The derivative constant for the control loop.                        \n
#
#  \b Feed-Forward: The feed-forward constant for the control loop.           \n
#
#  \b Striction \b Compensation: An offset (in N-m) for the control output used 
#  to compensate for static friction.                                         \n
#  
#  \b Integral \b Limit: A limit (in N-m) to the amount that the integral term 
#  can contribute to the output of the controller.


## Horizon Message Payload - Joint Control
#
#  Represents the payload of the command and data messages 'joint control'
#
#  @warning Data should not be modified once created
#
#  @since 0.4
#
#  @section data Joint Control Data
#  @copydoc joint_control
#
#  @pydoc
class HorizonPayload_JointControl(HorizonPayload):
    """Horizon Message Payload - Joint Control"""
    
    ## Create A Horizon Message Payload - Joint Control
    #
    #  Constructor for the Horizon Message Payload - Joint Control.           \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_JointControl(joint, p, d, i, raw=None,...)            \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_JointControl(raw, version, timestamp)                 \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  d              Derivative constant
    #  @param  feed           Feed-forward constant
    #  @param  i              Integral constant
    #  @param  joint          The joint id
    #  @param  limit          Integral limit
    #  @param  p              Proportional constant
    #  @param  stiction       Stiction compenstation
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, d = 0.0, feed = 0.0, i = 0.0, joint = 0, limit = 0.0, 
                 p = 0.0, stiction = 0.0, store_error = False, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Joint Control"""
        
        # Class Variables
        ## Derivative constant
        self._d = 0
        ## Feed-forward constant
        self._f = 0
        ## Integral constant
        self._i = 0
        ## Integral Limit
        self._l = 0
        ## Proportional Constant
        self._p = 0
        ## Stiction compensation
        self._s = 0
        ## Joint ID
        self._id = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test id
            if joint < 0 or joint > 255:
                logger.warning(
                        "Tried to create %s with bad joint!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Joint ID must be [0,255]!")
                if not store_error: raise self.error  
                else: return
            self._id = joint
            data += utils.from_byte(joint)
            
            # test P
            if p < -320 or p > 320:
                logger.warning(
                        "Tried to create %s with bad P!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Proportional constant must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self._p = p
            data += utils.from_short(int(p*100))
            
            # test I
            if i < -320 or i > 320:
                logger.warning(
                        "Tried to create %s with bad I!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Integral constant must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self._i = i
            data += utils.from_short(int(i*100))
            
            # test D
            if d < -320 or d > 320:
                logger.warning(
                        "Tried to create %s with bad D!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Derivative constant must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self._d = d
            data += utils.from_short(int(d*100))
            
            # test feed
            if feed < -320 or feed > 320:
                logger.warning(
                        "Tried to create %s with bad feed!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Feed-forward constant must be [-320,320]!")
                if not store_error: raise self.error  
                else: return
            self._f = feed
            data += utils.from_short(int(feed*100))
            
            # test Stiction
            if stiction < 0 or stiction > 100:
                logger.warning(
                        "Tried to create %s with bad stiction!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Stiction compensation must be [0,100]!")
                if not store_error: raise self.error  
                else: return
            self._s = stiction
            data += utils.from_short(int(stiction*100))
            
            # test limit
            if limit < 0 or limit > 100:
                logger.warning(
                        "Tried to create %s with bad limit!"\
                                 % self.__class__.__name__)
                self.error = ValueError( 
                               "Integral limit must be [0,100]!")
                if not store_error: raise self.error  
                else: return
            self._l = limit
            data += utils.from_short(int(limit*100))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 13:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract ID
            self._id = utils.to_byte(raw[0:1])
            logger.debug("%s joint: %d" % \
                         (self.__class__.__name__, self._id))
            
            # Extract P
            self._p = utils.to_short(raw[1:3])/100.0
            logger.debug("%s P: %f" % \
                         (self.__class__.__name__, self._p))
            
            # Extract I
            self._i = utils.to_short(raw[3:5])/100.0
            logger.debug("%s I: %f" % \
                         (self.__class__.__name__, self._i))
            
            # Extract D
            self._d = utils.to_short(raw[5:7])/100.0
            logger.debug("%s D: %f" % \
                         (self.__class__.__name__, self._d))
            
            # Extract feed
            self._f = utils.to_short(raw[7:9])/100.0
            logger.debug("%s feed: %f" % \
                         (self.__class__.__name__, self._f))
            
            # Extract stiction
            self._s = utils.to_short(raw[9:11])/100.0
            logger.debug("%s stiction: %f" % \
                         (self.__class__.__name__, self._s))
            
            # Extract limit
            self._l = utils.to_short(raw[11:13])/100.0
            logger.debug("%s limit: %f" % \
                         (self.__class__.__name__, self._l))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Joint ID: %d\nP: %f\nI: %f\nD: %f\nFeed-forward: %f\n"\
               "Stiction compensation: %f\nIntegral limit: %f"\
                % (self._id, self._p, self._i, self._d, self._f,self._s,self._l)
                
                
    ## Get Joint ID
    #
    #  @return the joint id
    #
    #  @pydoc
    def get_joint(self):
        """Get Joint ID"""
        
        return self._id
                
                
    ## Get Proportional Constant
    #
    #  @return the proportional constant
    #
    #  @pydoc
    def get_proportion(self):
        """Get Proportional Constant"""
        
        return self._p
                
                
    ## Get Integral Constant
    #
    #  @return the integral constant
    #
    #  @pydoc
    def get_integral(self):
        """Get Integral Constant"""
        
        return self._i
                
                
    ## Get Derivative Constant
    #
    #  @return the derivative constant
    #
    #  @pydoc
    def get_derivative(self):
        """Get Derivative Constant"""
        
        return self._d
                
                
    ## Get Feed-Forward Constant
    #
    #  @return the feed-forward constant
    #
    #  @pydoc
    def get_feed_forward(self):
        """Get Feed-Forward Constant"""
        
        return self._l_f
                
                
    ## Get Stiction Compensation
    #
    #  @return the stiction compensation
    #
    #  @pydoc
    def get_stiction(self):
        """Get Stiction Compensation"""
        
        return self._s
                
                
    ## Get Integral Limit
    #
    #  @return the integral limit
    #
    #  @pydoc
    def get_limit(self):
        """Get Integral Limit"""
        
        return self._l
    
    
    # Class Properties
    ## Proportional Constant
    proportion = property(fget=get_proportion, doc="Proportional Constant")
    ## Integral Constant
    integral = property(fget=get_integral, doc="Integral Constant")
    ## Derivative Constant
    drivative = property(fget=get_derivative, doc="Derivative Constant")
    ## Feed-Forward Constant
    feed_forward = property(fget=get_feed_forward, 
                            doc="Feed-Forward Constant")
    ## Stiction Compensation
    stiction = property(fget=get_stiction, doc="Stiction Compensation")
    ## Integral Limit
    limit = property(fget=get_limit, doc="Integral Limit")
    ## Joint ID
    joint = property(fget=get_joint, doc="Joint ID")


    
    
## @defgroup joint_control_request Request Joint Control
#  @ingroup request_joint_control
#  \b Description: Requests the control constants for a specified joint.    \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Subscription</td><td>unsigned short</td><td>2 bytes</td><td>-</td>
#  <td>-</td><td>Hz</td></tr>
#  <tr><td>Joint</td><td>byte</td><td>1 byte</td><td>-</td><td>-</td>
#  <td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field        | Type           | Size    | Scale | Range      | Units
#  -------------+----------------+---------+-------+------------+------
#  Subscription | unsigned short | 2 bytes | -     | [0,65535]  | Hz
#  Joint        | byte           | 1 byte  | -     | -          | -
#  @endmanonly                                                                \n
#
#  \b Subscription: The frequency to receive data. 0 represents an immediate,
#  single response and 0xFFFF turns off the current subscription.             \n
#
#  \b Joint: The joint for which the constants are being requested. Joints are 
#  defined on a per-manipulator basis.


## Horizon Message Payload - Request Joint Control
#
#  Represents the payload of the request message for 'joint control'
#
#  @warning Data should not be modified once created
#
#  @since 0.4
#
#  @section data Request Joint Control Data
#  @copydoc joint_control_request
#
#  @pydoc
class HorizonPayload_Request_JointControl(HorizonPayload_Request):
    """Horizon Message Payload - Request Joint Control"""
    
    
    ## Create A Horizon Message Payload - Request Joint Control
    #
    #  Constructor for the Horizon Message Payload - Request Joint Control 
    #  Class.                                                                 \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_Request_JointControl(joint, raw=None, ...)            \n
    #    Create a request message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_Request_JointControl(raw, version, timestamp)         \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  joint          The desired joint
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  subscription   Subscription Frequency in Hz
    #                         0 - immediate, 0xFFFF - off
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  verify         Verify the length? (useful for subclasses)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, joint = 0, store_error = False, raw = None, 
                 subscription = 0, timestamp = 0, version = tuple([-1,0]), 
                 verify = True):
        """Create A Horizon Message Payload - Request Joint Control"""
        
        # Class Variables
        ## Joint
        self._joint = 0
        
        # Create Constructor
        if raw == None:
            
            # Pass on to super-class
            HorizonPayload_Request.__init__(self, version = version, raw = None, 
                                            store_error = store_error, 
                                            subscription = subscription,
                                            timestamp = timestamp)
            
            # Test joint
            if joint < 0 or joint > 255:
                logger.warning("Tried to create %s with bad joint!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Joint must be 0-255!")
                if not store_error: raise self.error  
                else: return
            self._joint = joint
            self.data += utils.from_byte(joint)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if verify and ((version[0] == 0 and version[1]< 4 and len(raw) != 2)
                or (not(version[0] == 0 and version[1]< 4) and len(raw) != 3)):
                logger.warning("Tried to create %s of bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Invalid payload length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload_Request.__init__(self, raw = raw, version = version, 
                                            store_error = store_error,
                                            timestamp = timestamp, 
                                            verify = False)
            
            # Extract Joint
            if version[0] == 0 and version[1]< 4:
                self._joint = utils.to_byte(raw[1:2])
            else:
                self._joint = utils.to_byte(raw[2:3])
            logger.debug("%s joint: %d" % (self.__class__.__name__, 
                     self._joint))
    

    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return HorizonPayload_Request.print_format(self) + \
                "\nJoint: %d" % self._joint
                
                
    ## Get Joint
    #
    #  @return the joint number
    #
    #  @pydoc
    def get_joint(self):
        """Get Joint"""
        
        return self._joint
    
    
    # Class Properties
    ## Joint
    joint = property(fget=get_joint, doc="Joint")


## @defgroup joint_homing Joint Homing
#  @ingroup set_joint_homing
#  \b Description: Homes a specified joint. Uses position feedback and limit 
#  sensors as available. When homing is complete, the joint will be moved to 
#  its zero angle.                                                          \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Joint ID</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field    | Type | Size   | Scale | Range | Units
#  ---------+------+--------+-------+-------+------
#  Joint ID | byte | 1 byte | -     | -     | -
#  @endmanonly                                                                \n
#
#  \b Joint ID: The joint being homed. Identifiers are dependent on the 
#  specific manipulator hardware.


## Horizon Message Payload - Joint Homing
#
#  Represents the payload of the command message 'joint homing'
#
#  @warning Data should not be modified once created
#
#  @since 0.4
#
#  @section data Joint Homing Data
#  @copydoc joint_homing
#
#  @pydoc
class HorizonPayload_JointHoming(HorizonPayload):
    """Horizon Message Payload - Joint Homing"""
    
    ## Create A Horizon Message Payload - Joint Homing
    #
    #  Constructor for the Horizon Message Payload - Joint Homing Class.      \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_JointHoming(joint, raw=None, version, ...)            \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_JointHoming(raw, version, timestamp)                  \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  joint          Joint to home
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, joint = 0, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Joint Homing"""
        
        # Class Variables
        ## Joint ID
        self._joint = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test joint
            if joint > 255 or joint < 0:
                logger.warning("Tried to create %s with bad joint!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Joint ID must be 0-255!")
                if not store_error: raise self.error  
                else: return
            data = utils.from_byte(joint)
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 1:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Joint
            self._joint = utils.to_byte(raw[0:1])
            logger.debug("%s joint: %d" % (self.__class__.__name__,self._joint))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Joint ID: %d" % self._joint
                
                
    ## Get Joint
    #
    #  @return joint id
    #
    #  @pydoc
    def get_joint(self):
        """Get Joint"""
        
        return self._joint
    
    
    # Class Properties
    ## Joint ID
    joint = property(fget=get_joint, doc="Joint ID")



## @defgroup joint_homing_status Joint Homing Status
#  @ingroup data_joint_homing_status
#  \b Description: The homing status for each joint – if they have been homed 
#  and if they have been moved from this position. The payload will be n + 1 
#  bytes in size, where n is the amount of joints on the manipulator.       \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Count</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Status</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field  | Type | Size   | Scale | Range | Units
#  -------+------+--------+-------+-------+------
#  Count  | byte | 1 byte | -     | -     | -
#  Status | byte | 1 byte | -     | -     | -
#  @endmanonly                                                                \n
#
#  \b Count: The number of torque measurements being returned. The position of 
#  each torque sensor is manipulator-dependent.                               \n
#
#  \b Status: The joint homing status.
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'>
#  <td><i>Value</i></td><td><i>Status</i></td></tr>
#  <tr><td>0x00</td><td>Unhomed</td></tr>
#  <tr><td>0x01</td><td>Homed, still at its home position</td></tr>
#  <tr><td>0x02</td><td>Homed, moved away from its home position</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Value | Status
#  ------+-----------------------------------------
#  0x00  | Unhomed
#  0x01  | Homed, still at its home position
#  0x02  | Homed, moved away from its home position
#  @endmanonly


## Horizon Message Payload - Joint Homing Status
#
#  Represents the payload of the data message 'joint homing status'
#
#  @warning Data should not be modified once created
#
#  @since 0.4
#
#  @section data Joint Homing Status Data
#  @copydoc joint_homing_status
#
#  @pydoc
class HorizonPayload_JointHomingStatus(HorizonPayload):
    """Horizon Message Payload - Joint Homing Status"""
    
    
    ## Create A Horizon Message Payload - Joint Homing Status
    #
    #  Constructor for the Horizon Message Payload - Joint Homing Status 
    #  Class.                                                                 \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_JointHomingStatus(status, raw=None, version, ...)     \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_JointHomingStatus(raw, version, timestamp)            \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  status         List of joint homing status
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, status = {}, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Joint Homing Status"""
        
        # Class Variables
        ## Joint Status
        self._status = {}
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test length
            if len(status) > 256:
                logger.warning("Tried to create %s with too many status!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Number of status must be 0-256!")
                if not store_error: raise self.error  
                else: return
            data = utils.from_byte(len(status))
            
            # test values
            for stat in status:
                if stat < 0 or stat > 2:
                    logger.warning("Tried to create %s with bad status!"\
                                 % self.__class__.__name__)
                    self.error = ValueError( "Status must be [0,2]!")
                    if not store_error: raise self.error  
                    else: return
                data += utils.from_byte(stat)
            self._status = status
            
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) < 1 or len(raw) != raw[0] + 1:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Values
            for i in range(0,raw[0]):
                self._status+= utils.to_byte(raw[i+1:i+2])
            logger.debug("%s status: %s" % (self.__class__.__name__, 
                     ' '.join(self._status)))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        stat = '';
        for s in self._status:
            if s == 0: stat += 'unhomed '
            elif s == 1: stat += 'home '
            else: stat += 'away '
        return "Count: %d\nStatus: %s" % (len(self._status), stat)
                
                
    ## Get Count
    #
    #  @return number of torques
    #
    #  @pydoc
    def get_count(self):
        """Get Count"""
        
        return len(self._status)
                
                
    ## Get Status
    #
    #  @param  joint joint of status to get
    #  @return the joint's status, or list of all status if joint is -1
    #
    #  @pydoc
    def get_status(self, joint=-1):
        """Get Status"""
        
        # singular
        if joint > -1 and joint < len(self._status):
            return self._status[joint]
        
        # multiple
        else:
            return self._status[:]
    
    
    # Class Properties
    ## Status Count
    count = property(fget=get_count, doc="Status Count")
    ## Joint Status
    status = property(fget=get_status, doc="Joint Status")


## @defgroup joint_torques Joint Torques
#  @ingroup data_joint_torques
#  \b Description: The torque being experienced by each joint. The payload 
#  will be 2n + 1 bytes in size, where n is the amount of integrated torque 
#  sensors.                                                                 \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Count</td><td>byte</td><td>1 byte</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  <tr><td>Torque</td><td>signed short</td><td>2n bytes</td><td>100</td>
#  <td>[-320,320]</td><td>N-m</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field  | Type         | Size     | Scale | Range      | Units
#  -------+--------------+----------+-------+------------+------
#  Count  | byte         | 1 byte   | -     | -          | -
#  Torque | signed short | 2n bytes | 100   | [-320,320] | N-m
#  @endmanonly                                                                \n
#
#  \b Count: The number of torque measurements being returned. The position of 
#  each torque sensor is manipulator-dependent.                               \n
#
#  \b Value: The torque the joint is experiencing. The direction of the torque 
#  is defined by the specific manipulator hardware.


## Horizon Message Payload - Joint Torques
#
#  Represents the payload of the data message 'joint torques'
#
#  @warning Data should not be modified once created
#
#  @since 0.4
#
#  @section data Joint Torque Data
#  @copydoc joint_torques
#
#  @pydoc
class HorizonPayload_JointTorques(HorizonPayload):
    """Horizon Message Payload - Joint Torques"""
    
    
    ## Create A Horizon Message Payload - Joint Torques
    #
    #  Constructor for the Horizon Message Payload - Joint Torques.           \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_JointTorques(torques, raw=None, version, timestamp)   \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_JointTorques(raw, version, timestamp)                 \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  torques        List of torques [-320,320]
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, torques = {}, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Joint Torques"""
        
        # Class Variables
        ## Joint Torques
        self._torques = {}
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test length
            if len(torques) > 256:
                logger.warning("Tried to create %s with too many torques!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Number of torques must be 0-256!")
                if not store_error: raise self.error  
                else: return
            data = utils.from_byte(len(torques))
            
            # test values
            for torque in torques:
                if torque < -320 or torque > 320:
                    logger.warning("Tried to create %s with bad torque!"\
                                 % self.__class__.__name__)
                    self.error = ValueError( "Torque must be [-320,320]!")
                    if not store_error: raise self.error  
                    else: return
                data += utils.from_short(int(torque*100))
            self._torques = torques
            
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) < 1 or len(raw) != raw[0] + 1:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Values
            for i in range(0,raw[0]):
                self._torques+= utils.to_short(raw[i*2+1:i*2+2])/100.0
            logger.debug("%s torques: %s" % (self.__class__.__name__, 
                     'N-m '.join(self._torques)+'N-m '))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Count: %d\nTorques: %s" % (len(self._torques), 
                                           'N-m '.join(self._torques) + 'N-m')
                
                
    ## Get Count
    #
    #  @return number of torques
    #
    #  @pydoc
    def get_count(self):
        """Get Count"""
        
        return len(self._torques)
                
                
    ## Get Torque
    #
    #  @param  joint joint of torque to get
    #  @return the joint's torque, or list of all torques if joint is -1
    #
    #  @pydoc
    def get_torque(self, joint=-1):
        """Get Torque"""
        
        # singular
        if joint > -1 and joint < len(self._torques):
            return self._torques[joint]
        
        # multiple
        else:
            return self._torques[:]
    
    
    # Class Properties
    ## Torque Count
    count = property(fget=get_count, doc="Torque Count")
    ## Joint Torque
    torque = property(fget=get_torque, doc="Joint Torque")



## @defgroup end_effector_position End Effector Position
#  @ingroup set_end_effector_position data_end_effector_position
#  \b Description: The desired end effector position in the robot frame. The 
#  robot frame definition is dependent on the specific manipulator hardware.\n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>x</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-32,32]</td><td>m</td></tr>
#  <tr><td>y</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-32,32]</td><td>m</td></tr>
#  <tr><td>z</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-32,32]</td><td>m</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field | Type         | Size    | Scale | Range    | Units
#  ------+--------------+---------+-------+----------+------
#  x     | signed short | 2 bytes | 1000  | [-32,32] | m
#  y     | signed short | 2 bytes | 1000  | [-32,32] | m
#  z     | signed short | 2 bytes | 1000  | [-32,32] | m
#  @endmanonly                                                                \n
#
#  \b x: The desired x position in the robot frame of the end effector origin.\n
#
#  \b y: The desired y position in the robot frame of the end effector origin.\n
#
#  \b z: The desired z position in the robot frame of the end effector origin.


## Horizon Message Payload - End Effector Position
#
#  Represents the payload of the command and data messages 
#  'end effector position'
#
#  @warning Data should not be modified once created
#
#  @since 0.4
#
#  @section data End Effector Position Data
#  @copydoc end_effector_position
#
#  @pydoc
class HorizonPayload_EndEffectorPosition(HorizonPayload):
    """Horizon Message Payload - End Effector Position"""
    
    
    ## Create A Horizon Message Payload - End Effector Position
    #
    #  Constructor for the Horizon Message Payload - End Effector Position
    #  Class.                                                                 \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_EndEffectorPosition(x, y, z, roll, raw=None, ...)     \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_EndEffectorPosition(raw, version, timestamp)          \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @param  x              The end effector's x-axis position
    #  @param  y              The end effector's y-axis position
    #  @param  z              The end effector's z-axis position
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, x = 0, y = 0, z = 0, store_error = False, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - End Effector Position"""
        
        # Class Variables
        ## X-Axis
        self._x = 0
        ## Y-Axis
        self._y = 0
        ## Z-Axis
        self._z = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test x
            if x < -32 or x > 32:
                logger.warning("Tried to create %s with bad x!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "X must be [-32,32]!")
                if not store_error: raise self.error  
                else: return
            self._x = x
            data += utils.from_short(int(x*1000))
            
            # test y
            if y < -32 or y > 32:
                logger.warning("Tried to create %s with bad y!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Y must be [-32,32]!")
                if not store_error: raise self.error  
                else: return
            self._y = y
            data += utils.from_short(int(y*1000))
            
            # test z
            if z < -32 or z > 32:
                logger.warning("Tried to create %s with bad z!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Z must be [-32,32]!")
                if not store_error: raise self.error  
                else: return
            self._z = z
            data += utils.from_short(int(z*1000))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 6:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract X
            self._x = utils.to_short(raw[0:2])/1000.0
            logger.debug("%s x: %f" % (self.__class__.__name__, 
                     self._x))
            
            # Extract Y
            self._y = utils.to_short(raw[2:4])/1000.0
            logger.debug("%s y: %f" % (self.__class__.__name__, 
                     self._y))
            
            # Extract Z
            self._z = utils.to_short(raw[4:6])/1000.0
            logger.debug("%s z: %f" % (self.__class__.__name__, 
                     self._z))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "X: %f\nY: %f\nZ: %f" % (self._x, self._y, self._z)
        
        
    ## Get the end effector's X pos
    #
    #  @return x pos
    #
    #  @pydoc
    def get_x(self):
        """Get the end effector's x pos."""
        
        return self._x
        
        
    ## Get the end effector's y pos
    #
    #  @return y pos
    #
    #  @pydoc
    def get_y(self):
        """Get the end effector's y pos."""
        
        return self._y
        
        
    ## Get the end effector's z pos
    #
    #  @return z pos
    #
    #  @pydoc
    def get_z(self):
        """Get the end effector's z pos."""
        
        return self._z
    
    
    # Class Properties
    ## x pos
    x = property(fget=get_x, doc="x pos")
    ## y pos
    y = property(fget=get_y, doc="y pos")
    ## z pos
    z = property(fget=get_z, doc="z pos")



## @defgroup end_effector_pose End Effector Pose
#  @ingroup set_end_effector_pose data_end_effector_pose
#  \b Description: The desired end effector position and orientation in the 
#  robot frame. The robot frame definition is dependent on the specific 
#  manipulator hardware.                                                    \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>x</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-32,32]</td><td>m</td></tr>
#  <tr><td>y</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-32,32]</td><td>m</td></tr>
#  <tr><td>z</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-32,32]</td><td>m</td></tr>
#  <tr><td>Roll</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  <tr><td>Pitch</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  <tr><td>Yaw</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field | Type         | Size    | Scale | Range    | Units
#  ------+--------------+---------+-------+----------+------
#  x     | signed short | 2 bytes | 1000  | [-32,32] | m
#  y     | signed short | 2 bytes | 1000  | [-32,32] | m
#  z     | signed short | 2 bytes | 1000  | [-32,32] | m
#  Roll  | signed short | 2 bytes | 1000  | [-π,π]   | rad
#  Pitch | signed short | 2 bytes | 1000  | [-π,π]   | rad
#  Yaw   | signed short | 2 bytes | 1000  | [-π,π]   | rad
#  @endmanonly                                                                \n
#
#  \b x: The desired x position in the robot frame of the end effector origin.\n
#
#  \b y: The desired y position in the robot frame of the end effector origin.\n
#
#  \b z: The desired z position in the robot frame of the end effector origin.\n
#
#  \b Roll: The desired roll of the end effector in the robot frame.          \n
#
#  \b Pitch: The desired pitch of the end effector in the robot frame.        \n
#
#  \b Yaw: The desired yaw of the end effector in the robot frame.


## Horizon Message Payload - End Effector Pose
#
#  Represents the payload of the command and data messages 'end effector pose'
#
#  @warning Data should not be modified once created
#
#  @since 0.4
#
#  @section data End Effector Pose Data
#  @copydoc end_effector_pose
#
#  @pydoc
class HorizonPayload_EndEffectorPose(HorizonPayload):
    """Horizon Message Payload - End Effector Pose"""
    
    
    ## Create A Horizon Message Payload - End Effector Pose
    #
    #  Constructor for the Horizon Message Payload - End Effector Pose Class. \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_EndEffectorPose(x, y, z, roll, raw=None, ...)         \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_EndEffectorPose(raw, version, timestamp)              \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  pitch          The end effector's pitch angle
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  roll           The end effector's roll angle
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @param  x              The end effector's x-axis position
    #  @param  y              The end effector's y-axis position
    #  @param  yaw            The end effector's yaw angle
    #  @param  z              The end effector's z-axis position
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, roll = 0, pitch = 0, yaw = 0, x = 0, y = 0, z = 0, 
                 store_error = False, raw = None, timestamp = 0, 
                 version = tuple([-1,0])):
        """Create A Horizon Message Payload - End Effector Pose"""
        
        # Class Variables
        ## X-Axis
        self._x = 0
        ## Y-Axis
        self._y = 0
        ## Z-Axis
        self._z = 0
        ## Roll Angle
        self._roll = 0
        ## Pitch Angle
        self._pitch = 0
        ## Yaw Angle
        self._yaw = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test x
            if x < -32 or x > 32:
                logger.warning("Tried to create %s with bad x!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "X must be [-32,32]!")
                if not store_error: raise self.error  
                else: return
            self._x = x
            data += utils.from_short(int(x*1000))
            
            # test y
            if y < -32 or y > 32:
                logger.warning("Tried to create %s with bad y!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Y must be [-32,32]!")
                if not store_error: raise self.error  
                else: return
            self._y = y
            data += utils.from_short(int(y*1000))
            
            # test z
            if z < -32 or z > 32:
                logger.warning("Tried to create %s with bad z!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Z must be [-32,32]!")
                if not store_error: raise self.error  
                else: return
            self._z = z
            data += utils.from_short(int(z*1000))
            
            # test roll
            if roll < -math.pi or roll > math.pi:
                logger.warning("Tried to create %s with bad roll!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Roll must be [-π,π]!")
                if not store_error: raise self.error  
                else: return
            self._roll = roll
            data += utils.from_short(int(roll*1000))
            
            # test pitch
            if pitch < -math.pi or pitch > math.pi:
                logger.warning("Tried to create %s with bad pitch!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Pitch must be [-π,π]!")
                if not store_error: raise self.error  
                else: return
            self._pitch = pitch
            data += utils.from_short(int(pitch*1000))
            
            # test yaw
            if yaw < -math.pi or yaw > math.pi:
                logger.warning("Tried to create %s with bad yaw!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Yaw must be [-π,π]!")
                if not store_error: raise self.error  
                else: return
            self._yaw = yaw
            data += utils.from_short(int(yaw*1000))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 12:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract X
            self._x = utils.to_short(raw[0:2])/1000.0
            logger.debug("%s x: %f" % (self.__class__.__name__, 
                     self._x))
            
            # Extract Y
            self._y = utils.to_short(raw[2:4])/1000.0
            logger.debug("%s y: %f" % (self.__class__.__name__, 
                     self._y))
            
            # Extract Z
            self._z = utils.to_short(raw[4:6])/1000.0
            logger.debug("%s z: %f" % (self.__class__.__name__, 
                     self._z))
            
            # Extract Roll
            self._roll = utils.to_short(raw[6:8])/1000.0
            logger.debug("%s roll: %f" % (self.__class__.__name__, 
                     self._roll))
            
            # Extract Pitch
            self._pitch = utils.to_short(raw[8:10])/1000.0
            logger.debug("%s pitch: %f" % (self.__class__.__name__, 
                     self._pitch))
            
            # Extract Yaw
            self._yaw = utils.to_short(raw[10:12])/1000.0
            logger.debug("%s yaw: %f" % (self.__class__.__name__, 
                     self._yaw))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "X: %f\nY: %f\nZ: %f\nRoll: %f\nPitch: %f\nYaw: %f"\
                %(self._x, self._y, self._z, self._roll, self._pitch, self._yaw)
        
        
    ## Get the end effector's Roll
    #
    #  @return roll angle
    #
    #  @pydoc
    def get_roll(self):
        """Get the end effector's Roll."""
        
        return self._roll
        
        
    ## Get the end effector's Pitch
    #
    #  @return pitch angle
    #
    #  @pydoc
    def get_pitch(self):
        """Get the end effector's Pitch."""
        
        return self._pitch
        
        
    ## Get the end effector's Yaw
    #
    #  @return yaw angle
    #
    #  @pydoc
    def get_yaw(self):
        """Get the end effector's Yaw."""
        
        return self._yaw
        
        
    ## Get the end effector's X pos
    #
    #  @return x pos
    #
    #  @pydoc
    def get_x(self):
        """Get the end effector's x pos."""
        
        return self._x
        
        
    ## Get the end effector's y pos
    #
    #  @return y pos
    #
    #  @pydoc
    def get_y(self):
        """Get the end effector's y pos."""
        
        return self._y
        
        
    ## Get the end effector's z pos
    #
    #  @return z pos
    #
    #  @pydoc
    def get_z(self):
        """Get the end effector's z pos."""
        
        return self._z
    
    
    # Class Properties
    ## Pitch Angle
    pitch = property(fget=get_pitch, doc="Pitch Angle")
    ## Roll Angle
    roll = property(fget=get_roll, doc="Roll Angle")
    ## Yaw Angle
    yaw = property(fget=get_yaw, doc="Yaw Angle")
    ## x pos
    x = property(fget=get_x, doc="x pos")
    ## y pos
    y = property(fget=get_y, doc="y pos")
    ## z pos
    z = property(fget=get_z, doc="z pos")


    
## @defgroup end_effector_orientation End Effector Orientation
#  @ingroup data_end_effector_orientation
#  \b Description: The current end effector position and orientation in the 
#  robot frame. The robot frame definition is dependent on the specific 
#  manipulator hardware.                                                    \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>x</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-32,32]</td><td>m</td></tr>
#  <tr><td>y</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-32,32]</td><td>m</td></tr>
#  <tr><td>z</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-32,32]</td><td>m</td></tr>
#  <tr><td>Roll</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  <tr><td>Pitch</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  <tr><td>Yaw</td><td>signed short</td><td>2 bytes</td><td>1000</td>
#  <td>[-π,π]</td><td>rad</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field | Type         | Size    | Scale | Range    | Units
#  ------+--------------+---------+-------+----------+------
#  x     | signed short | 2 bytes | 1000  | [-32,32] | m
#  y     | signed short | 2 bytes | 1000  | [-32,32] | m
#  z     | signed short | 2 bytes | 1000  | [-32,32] | m
#  Roll  | signed short | 2 bytes | 1000  | [-π,π]   | rad
#  Pitch | signed short | 2 bytes | 1000  | [-π,π]   | rad
#  Yaw   | signed short | 2 bytes | 1000  | [-π,π]   | rad
#  @endmanonly                                                                \n
#
#  \b x: The current x position in the robot frame of the end effector origin.\n
#
#  \b y: The current y position in the robot frame of the end effector origin.\n
#
#  \b z: The current z position in the robot frame of the end effector origin.\n
#
#  \b Roll: The current roll of the end effector in the robot frame.          \n
#
#  \b Pitch: The current pitch of the end effector in the robot frame.        \n
#
#  \b Yaw: The current yaw of the end effector in the robot frame.


## Horizon Message Payload - End Effector Orientation
#
#  Represents the payload of the data message 'end effector orientation'
#
#  @warning Data should not be modified once created
#
#  @since 0.4
#
#  @section data End Effector Orientation Data
#  @copydoc end_effector_orientation
#
#  @pydoc
class HorizonPayload_EndEffectorOrientation(HorizonPayload):
    """Horizon Message Payload - End Effector Orientation"""
    
    ## Create A Horizon Message Payload - End Effector Orientation
    #
    #  Constructor for the Horizon Message Payload - End Effector Orientation
    #  Class.                                                                 \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_EndEffectorOrientation(x, y, z, roll, raw=None, ...)  \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_EndEffectorOrientation(raw, version, timestamp)       \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  pitch          The end effector's pitch angle
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  roll           The end effector's roll angle
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @param  x              The end effector's x-axis position
    #  @param  y              The end effector's y-axis position
    #  @param  yaw            The end effector's yaw angle
    #  @param  z              The end effector's z-axis position
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, roll = 0, pitch = 0, yaw = 0, x = 0, y = 0, z = 0, 
                 store_error = False, raw = None, timestamp = 0, 
                 version = tuple([-1,0])):
        """Create A Horizon Message Payload - End Effector Orientation"""
        
        # Class Variables
        ## X-Axis
        self._x = 0
        ## Y-Axis
        self._y = 0
        ## Z-Axis
        self._z = 0
        ## Roll Angle
        self._roll = 0
        ## Pitch Angle
        self._pitch = 0
        ## Yaw Angle
        self._yaw = 0
        
        # Create Constructor
        if raw == None:
            data = []
            
            # test x
            if x < -32 or x > 32:
                logger.warning("Tried to create %s with bad x!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "X must be [-32,32]!")
                if not store_error: raise self.error  
                else: return
            self._x = x
            data += utils.from_short(int(x*1000))
            
            # test y
            if y < -32 or y > 32:
                logger.warning("Tried to create %s with bad y!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Y must be [-32,32]!")
                if not store_error: raise self.error  
                else: return
            self._y = y
            data += utils.from_short(int(y*1000))
            
            # test z
            if z < -32 or z > 32:
                logger.warning("Tried to create %s with bad z!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Z must be [-32,32]!")
                if not store_error: raise self.error  
                else: return
            self._z = z
            data += utils.from_short(int(z*1000))
            
            # test roll
            if roll < -math.pi or roll > math.pi:
                logger.warning("Tried to create %s with bad roll!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Roll must be [-π,π]!")
                if not store_error: raise self.error  
                else: return
            self._roll = roll
            data += utils.from_short(int(roll*1000))
            
            # test pitch
            if pitch < -math.pi or pitch > math.pi:
                logger.warning("Tried to create %s with bad pitch!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Pitch must be [-π,π]!")
                if not store_error: raise self.error  
                else: return
            self._pitch = pitch
            data += utils.from_short(int(pitch*1000))
            
            # test yaw
            if yaw < -math.pi or yaw > math.pi:
                logger.warning("Tried to create %s with bad yaw!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Yaw must be [-π,π]!")
                if not store_error: raise self.error  
                else: return
            self._yaw = yaw
            data += utils.from_short(int(yaw*1000))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 12:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError( "Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract X
            self._x = utils.to_short(raw[0:2])/1000.0
            logger.debug("%s x: %f" % (self.__class__.__name__, 
                     self._x))
            
            # Extract Y
            self._y = utils.to_short(raw[2:4])/1000.0
            logger.debug("%s y: %f" % (self.__class__.__name__, 
                     self._y))
            
            # Extract Z
            self._z = utils.to_short(raw[4:6])/1000.0
            logger.debug("%s z: %f" % (self.__class__.__name__, 
                     self._z))
            
            # Extract Roll
            self._roll = utils.to_short(raw[6:8])/1000.0
            logger.debug("%s roll: %f" % (self.__class__.__name__, 
                     self._roll))
            
            # Extract Pitch
            self._pitch = utils.to_short(raw[8:10])/1000.0
            logger.debug("%s pitch: %f" % (self.__class__.__name__, 
                     self._pitch))
            
            # Extract Yaw
            self._yaw = utils.to_short(raw[10:12])/1000.0
            logger.debug("%s yaw: %f" % (self.__class__.__name__, 
                     self._yaw))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "X: %f\nY: %f\nZ: %f\nRoll: %f\nPitch: %f\nYaw: %f"\
                %(self._x, self._y, self._z, self._roll, self._pitch, self._yaw)
        
        
    ## Get the end effector's Roll
    #
    #  @return roll angle
    #
    #  @pydoc
    def get_roll(self):
        """Get the end effector's Roll."""
        
        return self._roll
        
        
    ## Get the end effector's Pitch
    #
    #  @return pitch angle
    #
    #  @pydoc
    def get_pitch(self):
        """Get the end effector's Pitch."""
        
        return self._pitch
        
        
    ## Get the end effector's Yaw
    #
    #  @return yaw angle
    #
    #  @pydoc
    def get_yaw(self):
        """Get the end effector's Yaw."""
        
        return self._yaw
        
        
    ## Get the end effector's X pos
    #
    #  @return x pos
    #
    #  @pydoc
    def get_x(self):
        """Get the end effector's x pos."""
        
        return self._x
        
        
    ## Get the end effector's y pos
    #
    #  @return y pos
    #
    #  @pydoc
    def get_y(self):
        """Get the end effector's y pos."""
        
        return self._y
        
        
    ## Get the end effector's z pos
    #
    #  @return z pos
    #
    #  @pydoc
    def get_z(self):
        """Get the end effector's z pos."""
        
        return self._z
    
    
    # Class Properties
    ## Pitch Angle
    pitch = property(fget=get_pitch, doc="Pitch Angle")
    ## Roll Angle
    roll = property(fget=get_roll, doc="Roll Angle")
    ## Yaw Angle
    yaw = property(fget=get_yaw, doc="Yaw Angle")
    ## x pos
    x = property(fget=get_x, doc="x pos")
    ## y pos
    y = property(fget=get_y, doc="y pos")
    ## z pos
    z = property(fget=get_z, doc="z pos")



## @defgroup reset Reset
#  @ingroup set_reset
#  \b Description: Resets the control processor.                            \n\n
#
#  \b Message \b Fields:
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><i>Field</i></td>
#  <td><i>Type</i></td><td><i>Size</i></td><td><i>Scale</i></td>
#  <td><i>Range</i></td><td><i>Units</i></td></tr>
#  <tr><td>Passcode</td><td>unsigned short</td><td>2 bytes</td><td>-</td>
#  <td>-</td><td>-</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field    | Type           | Size    | Scale | Range | Units
#  ---------+----------------+---------+-------+-------+------
#  Passcode | unsigned short | 2 bytes | -     | -     | -
#  @endmanonly                                                                \n
#
#  \b Passcode: Fixed value of 0x3A18.

## Horizon Message Payload - Reset
#
#  Represents the payload of the command message 'reset'
#
#  @warning Data should not be modified once created
#
#  @since 0.7
#
#  @section data Reset Data
#  @copydoc reset
#
#  @pydoc
class HorizonPayload_Reset(HorizonPayload):
    """Horizon Message Payload - Reset"""
    
    ## Create A Horizon Message Payload - Reset
    #
    #  Constructor for the Horizon Message Payload - Reset Class.             \n
    #  The constructor can be called two different ways:
    #  - HorizonPayload_PlatformTime(raw=None, version, timestamp)            \n
    #    Create a command message payload to send.                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonPayload_PlatformTime(raw, version, timestamp)                 \n
    #    Parse raw data (most likely received) into payload variables.        \n 
    #    Version auto-detection is unsupported.
    #
    #  @param  raw            Raw Payload data byte list to parse
    #  @param  store_error    Suppress and store errors?
    #                         Errors will be stored to self.error
    #  @param  timestamp      Payload Send / Create Time (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, store_error = False, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Reset"""
        
        # Class Variables
        ## Passcode
        self.passcode = 0x3A18
        
        # Create Constructor
        if raw == None:
            data = []
            
            # passcode
            data += utils.from_unsigned_short(self.passcode)
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            
            # Verify Length
            if len(raw) != 2:
                logger.warning("Tried to create %s with bad length!"\
                                 % self.__class__.__name__)
                self.error = ValueError("Bad length!")
                if not store_error: raise self.error  
                else: return
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            # Extract Passcode
            self.passcode = utils.to_unsigned_short(raw)
            logger.debug("%s passcode: %d" % (self.__class__.__name__, self.passcode))
    
    
    ## Human Readable Payload String
    def print_format(self):
        """Return the payload as a human readable string"""
        
        return "Passcode: 0x%04X" % self.passcode
                

# Abstract class. This forms the basis for all the sensor config messages, as they are all
# sequences of pairs of offset/scale values.
class HorizonPayload_VariableSensorConfig(HorizonPayload):
    """Horizon Message Abstract Payload - Sensor Configuration"""

    def __init__(self, passcode = 0, offsets = [], scales = [], store_error = False, 
                 raw = None, timestamp = 0, version = tuple([-1,0])):
        
        # Class Variables
        self.passcode = passcode
        self.offsets = []
        self.scales = []
        
        # Create Constructor
        if raw == None:
            # Assume this is a config set going out.

            # Magic passcode
            data = utils.from_short(self.passcode)
            
            self.offsets = offsets
            self.scales = scales
            if len(self.offsets) != len(self.scales):
                raise ValueError("Offsets and scales are not the same length!")
        
            for offset, scale in zip(self.offsets, self.scales):   
                data += utils.from_short(int(offset * 1000))
                data += utils.from_short(int(scale * 1000))
            
            # Pass on to super-class
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            # For now, assume this is a result coming back.
            # (Depending on the payload length, we could infer whether this
            # is a config set operation or a config request result.)
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            for i in range(0, self._num_sensors(raw)):
                base = self._base_for_value(i)
                self.offsets.append(utils.to_short(raw[base:base + 2]) / 1000.0)
                self.scales.append(utils.to_short(raw[base + 2:base + 4]) / 1000.0)

    def _num_sensors(self, raw):
        return raw[0]

    def _base_for_value(self, i):
        return i * 4 + 1


class HorizonPayload_CurrentSensorConfig(HorizonPayload_VariableSensorConfig):
    """Horizon Message Payload - Current Sensor Configuration"""

    def print_format(self):
        lines = []
        for i in range(len(self.offsets)):
            lines.append("Current Sensor %d: Offset: %f,  Scale: %f" % (i + 1, self.offsets[i], self.scales[i]))
        return "\n".join(lines)


class HorizonPayload_VoltageSensorConfig(HorizonPayload_VariableSensorConfig):
    """Horizon Message Payload - Voltage Sensor Configuration"""

    def print_format(self):
        lines = []
        for i in range(len(self.offsets)):
            lines.append("Voltage Sensor %d: Offset: %f,  Scale: %f" % (i + 1, self.offsets[i], self.scales[i]))
        return "\n".join(lines)


class HorizonPayload_TemperatureSensorConfig(HorizonPayload_VariableSensorConfig):
    """Horizon Message Payload - Temperature Sensor Configuration"""

    def print_format(self):
        lines = []
        for i in range(len(self.offsets)):
            lines.append("Temperature Sensor %d: Offset: %f,  Scale: %f" % (i + 1, self.offsets[i], self.scales[i]))
        return "\n".join(lines)


class HorizonPayload_OrientationSensorConfig(HorizonPayload_VariableSensorConfig):
    """Horizon Message Payload - Orientation Sensor Configuration"""

    def __init__(self, passcode = 0, roll_offset = 0, roll_scale = 0, 
                 pitch_offset = 0, pitch_scale = 0, yaw_offset = 0, 
                 yaw_scale = 0, store_error = False, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        
        offsets = [roll_offset, pitch_offset, yaw_offset]
        scales = [roll_scale, pitch_scale, yaw_scale]
        
        HorizonPayload_VariableSensorConfig.__init__(self, passcode, offsets, scales, store_error, raw, timestamp)

        self.roll_offset, self.pitch_offset, self.yaw_offset = self.offsets
        self.roll_scale, self.pitch_scale, self.yaw_scale = self.scales

    def print_format(self):
        lines = []
        lines.append("Roll offset: %f,  Roll scale: %f" % (self.offsets[0], self.scales[0]))
        lines.append("Pitch offset: %f,  Pitch scale: %f" % (self.offsets[1], self.scales[1]))
        lines.append("Yaw offset: %f,  Yaw scale: %f" % (self.offsets[2], self.scales[2]))
        return "\n".join(lines)

    def _num_sensors(self, raw):
        return 3

    def _base_for_value(self, i):
        return i * 4


class HorizonPayload_GyroConfig(HorizonPayload_OrientationSensorConfig):
    """Horizon Message Payload - Gyro Configuration, identical to orientation sensor"""
    pass


class HorizonPayload_AccelerometerConfig(HorizonPayload_VariableSensorConfig):
    """Horizon Message Payload - Orientation Sensor Configuration"""

    def __init__(self, passcode = 0, x_offset = 0, x_scale = 0, y_offset = 0, 
                 y_scale = 0, z_offset = 0, z_scale = 0, store_error = False, 
                 raw = None, timestamp = 0, version = tuple([-1,0])):
        
        offsets = [x_offset, y_offset, z_offset]
        scales = [x_scale, y_scale, z_scale]
        
        HorizonPayload_VariableSensorConfig.__init__(self, passcode, offsets, scales, store_error, raw, timestamp)

        self.x_offset, self.y_offset, self.z_offset = self.offsets
        self.x_scale, self.y_scale, self.z_scale = self.scales

    def print_format(self):
        lines = []
        lines.append("x-axis offset: %f, scale: %f" % (self.offsets[0], self.scales[0]))
        lines.append("y-axis offset: %f, scale: %f" % (self.offsets[1], self.scales[1]))
        lines.append("z-axis offset: %f, scale: %f" % (self.offsets[2], self.scales[2]))
        return "\n".join(lines)

    def _num_sensors(self, raw):
        return 3

    def _base_for_value(self, i):
        return i * 4


class HorizonPayload_MagnetometerConfig(HorizonPayload_AccelerometerConfig):
    """Horizon Message Payload - Magnetometer Configuration, identical to accelerometer"""
    pass


# Abstract class. This forms the basis for all the sensor config messages, as they are all
# sequences of pairs of offset/scale values.
class HorizonPayload_RawSensor(HorizonPayload):
    """Horizon Message Abstract Payload - Raw Sensor Data"""

    def __init__(self, store_error = False, raw = None, timestamp = 0, version = tuple([-1,0])):
        
        # Class Variables
        self.raw_values = []
        
        # Create Constructor
        if raw != None:
            # Result coming back from firmware            
                
            # Pass on to super-class
            HorizonPayload.__init__(self, raw = raw, version = version, 
                                    store_error = store_error,
                                    timestamp = timestamp)
            
            for i in range(0, self._num_sensors(raw)):
                base = self._base_for_value(i)
                self.raw_values.append(utils.to_signed_short(raw[base:base + 2]))

    def _num_sensors(self, raw):
        return raw[0]

    def _base_for_value(self, i):
        return i * 2 + 1



class HorizonPayload_RawCurrentSensor(HorizonPayload_RawSensor):
    """Horizon Message Payload - Raw Current Data"""

    def __init__(self, store_error = False, raw = None, timestamp = 0, version = tuple([-1,0])):
        HorizonPayload_RawSensor.__init__(self, store_error, raw, timestamp)

        # Class Variables
        self.raw_currents = self.raw_values
        

    def print_format(self):
        lines = []
        for i in range(len(self.raw_currents)):
            lines.append("Raw Current %d: %d" % (i + 1, self.raw_currents[i]))
        return "\n".join(lines)



class HorizonPayload_RawVoltageSensor(HorizonPayload_RawSensor):
    """Horizon Message Payload - Raw Voltage Data"""

    def __init__(self, store_error = False, raw = None, timestamp = 0, version = tuple([-1,0])):
        HorizonPayload_RawSensor.__init__(self, store_error, raw, timestamp)

        # Class Variables
        self.raw_voltages = self.raw_values
        

    def print_format(self):
        lines = []
        for i in range(len(self.raw_voltages)):
            lines.append("Raw Voltage %d: %d" % (i + 1, self.raw_voltages[i]))
        return "\n".join(lines)


class HorizonPayload_RawTemperatureSensor(HorizonPayload_RawSensor):
    """Horizon Message Payload - Raw Temperature Data"""

    def __init__(self, store_error = False, raw = None, timestamp = 0, version = tuple([-1,0])):
        HorizonPayload_RawSensor.__init__(self, store_error, raw, timestamp)

        # Class Variables
        self.raw_temperatures = self.raw_values
        

    def print_format(self):
        lines = []
        for i in range(len(self.raw_temperatures)):
            lines.append("Raw Temperature %d: %d" % (i + 1, self.raw_temperatures[i]))
        return "\n".join(lines)


class HorizonPayload_RawOrientationSensor(HorizonPayload_RawSensor):
    """Horizon Message Payload - Raw Orientation Data"""

    def __init__(self, store_error = False, raw = None, timestamp = 0, version = tuple([-1,0])):
        HorizonPayload_RawSensor.__init__(self, store_error, raw, timestamp)

        # Class Variables
        self.raw_roll, self.raw_pitch, self.raw_yaw = self.raw_values

    def _num_sensors(self, raw):
        return 3

    def _base_for_value(self, i):
        return i * 2

    def print_format(self):
        lines = []
        lines.append("Raw Roll: %d" % self.raw_roll)
        lines.append("Raw Pitch: %d" % self.raw_pitch)
        lines.append("Raw Yaw: %d" % self.raw_yaw)
        return "\n".join(lines)


class HorizonPayload_RawGyro(HorizonPayload_RawOrientationSensor):
    """Horizon Message Payload - Raw Gyro Data, same as raw orientation data"""
    pass


class HorizonPayload_RawAccelerometer(HorizonPayload_RawSensor):
    """Horizon Message Payload - Raw Orientation Data"""

    def __init__(self, store_error = False, raw = None, timestamp = 0, version = tuple([-1,0])):
        HorizonPayload_RawSensor.__init__(self, store_error, raw, timestamp)

        # Class Variables
        self.raw_x, self.raw_y, self.raw_z = self.raw_values

    def _num_sensors(self, raw):
        return 3

    def _base_for_value(self, i):
        return i * 2

    def print_format(self):
        lines = []
        lines.append("Raw X: %d" % self.raw_x)
        lines.append("Raw Y: %d" % self.raw_y)
        lines.append("Raw Z: %d" % self.raw_z)
        return "\n".join(lines)


class HorizonPayload_RawMagnetometer(HorizonPayload_RawSensor):
    """Horizon Message Payload - Raw Magnetometer Data"""

    def __init__(self, store_error = False, raw = None, timestamp = 0, version = tuple([-1,0])):
        HorizonPayload_RawSensor.__init__(self, store_error, raw, timestamp)

        # Class Variables
        self.raw_x, self.raw_y, self.raw_z = self.raw_values

    def _num_sensors(self, raw):
        return 3

    def _base_for_value(self, i):
        return i * 2

    def print_format(self):
        lines = []
        lines.append("Raw X: %d" % self.raw_x)
        lines.append("Raw Y: %d" % self.raw_y)
        lines.append("Raw Z: %d" % self.raw_z)
        return "\n".join(lines)



class HorizonPayload_RestoreSystemConfig(HorizonPayload):
    """Horizon Message Payload - Restore System Configuration"""

    def __init__(self, passcode = 0x3A18, flags = 1, store_error = False, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Reset"""
        
        self.passcode = passcode
        self.flags = flags

        # Create Constructor
        if raw == None:
            data = utils.from_unsigned_short(self.passcode) + utils.from_byte(self.flags)
        
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            raise NotImplementedError('Payload %s cannot parse its response.' % self.__class__.__name__ )
    
    def print_format(self):
        """Return the payload as a human readable string"""
        return "Passcode: 0x%04X" % self.passcode




class HorizonPayload_StoreSystemConfig(HorizonPayload):
    """Horizon Message Payload - Store System Configuration"""

    def __init__(self, passcode = 0x3A18, store_error = False, raw = None, 
                 timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message Payload - Reset"""
        
        self.passcode = passcode
        
        # Create Constructor
        if raw == None:
            data = utils.from_unsigned_short(self.passcode)
        
            HorizonPayload.__init__(self, version = version, raw = data, 
                                    store_error = store_error,
                                    timestamp = timestamp)
        
        # Parse Constructor
        else:
            raise NotImplementedError('Payload %s cannot parse its response.' % self.__class__.__name__ )
     
    def print_format(self):
        """Return the payload as a human readable string"""
        return "Passcode: 0x%04X" % self.passcode



logger.debug("... clearpath.horizon.payloads loaded.")
