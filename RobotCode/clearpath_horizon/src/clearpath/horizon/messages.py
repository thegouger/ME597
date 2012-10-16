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
#  File: messages.py
#  Desc: Horizon Protocol Message Definition
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
    print ("ERROR: clearpath.horizon.messages is a module and can NOT be run"\
           " as a script!\nFor a command-line interface demo of Horizon, run:"\
           "\n  python -m clearpath.horizon.demo\n"\
           "For Horizon message forwarding, run:\n"\
           "  python -m clearpath.horizon.forward")

    # Exit Error
    import sys
    sys.exit(1)




################################################################################
# Module



## @package clearpath.horizon.messages 
#  Horizon Protocol Message Python Module
# 
#  Horizon Protocol Message Definition                                        \n
#  Abstracted from knowing message codes and payloads.                        \n
#  Supported Horizon version(s): 0.1 - 1.0
#
#  @author     Ryan Gariepy
#  @author     Malcolm Robert
#  @date       25/01/10
#  @req        clearpath.utils                                                \n
#              clearpath.horizon.codes                                        \n
#              clearpath.horizon.versioning
#  @version    1.0
#
#  @section USE
#
#  The intended purpose of this module is to provide a class that handles
#  message formatting and parsing. The class does not care about message
#  codes (except for version translation) or payload formatting.
#
#  Creation of a message to send requires instantiating HorizonMessage with the
#  parameters of code, payload, store_error, no_ack, timestamp, and version.
#  payload must be an instantiated HorizonPayload class and cannot be None.
#  The message can then be sent by passing the instance to the send_message
#  function of the appropriate HorizonProtocol or HorizonTransport object
#  or the raw data can be sent directly over a serial port or a socket by
#  using the method raw_string.
#
#  Once data has been received and is known to be exactly one message, it can
#  be parsed and turned into a HorizonMessage object by instantiating the class
#  with the parameters of raw, payload_type, store_error, and version where raw
#  is an integer list of the received data.
#
#  In either of the above cases, the HorizonMessage object should not be
#  modified or mutated once created. If changing data is desirable, create
#  a new HorizonMessage with the new data.
#
#  Various logging messages (debug, warning,...) are logged with the standard
#  Python logger to the log named 'clearpath.horizon.messages'.               \n
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
#  - Horizon support for v 0.1 - 1.0
#  - Python 2.6+ & 3.x compatible
#
#  @section License
#  @copydoc public_license
#
"""Horizon Protocol Message Definition 

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
from .  import codes            # Horizon Message Codes
from .  import versioning       # Horizon Protocol Versions 

# Required Python Modules
import logging                  # Logging Utilities


# Module Support
## Module Version
_version__  = "1.0"
"""Module Version"""
## SVN Code Revision
__revision__ = "$Revision: 425 $"
""" SVN Code Revision"""


## Supported Horizon Versions
versions = set()
"""Supported Horizon Versions"""


## Message Log
logger = logging.getLogger('clearpath.horizon.messages')
"""Horizon Messages Module Log"""
logger.setLevel(logging.DEBUG)
logger.addHandler(utils.NullLoggingHandler())
logger.propagate = False
logger.debug("Loading clearpath.horizon.messages ...")




################################################################################
# Horizon Message



## @defgroup format Data Format
#  @ingroup doc
#  @defgroup package Package
#  @ingroup format
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><b>0</b></td><td><b>1</b></td>
#  <td><b>2</b></td><td><b>3</b></td><td><b>4</b></td><td><b>5</b></td>
#  <td><b>6</b></td><td><b>7</b></td><td><b>8</b></td><td><b>9</b></td>
#  <td><b>10</b></td><td><b>11</b></td><td><b>12</b></td><td><b>13</b></td>
#  <td><b>14</b></td><td><b>15</b></td><td><b>16</b></td><td><b>17</b></td>
#  <td><b>18</b></td><td><b>19</b></td><td><b>20</b></td><td><b>21</b></td>
#  <td><b>22</b></td><td><b>23</b></td><td><b>24</b></td><td><b>25</b></td>
#  <td><b>26</b></td><td><b>27</b></td><td><b>28</b></td><td><b>29</b></td>
#  <td><b>30</b></td><td><b>31</b></td></tr>
#  <tr><td colspan="8">SOH</td><td colspan="16">LENGTH</td>
#  <td colspan="8">VERSION</td></tr>
#  <tr><td colspan="32">TIMESTAMP</td></tr>
#  <tr><td colspan="8">FLAGS</td><td colspan="16">MESSAGE TYPE</td>
#  <td colspan="8">STX</td></tr>
#  <tr><td colspan="32">PAYLOAD (VARIABLE LENGTH)</td></tr>
#  <tr><td colspan="16">&nbsp;</td><td colspan="16">CHECKSUM</td></tr>
#  </table>
#  @endhtmlonly 
#  @manonly
#  ------------------------------------------
#  | SOH      | LENGTH           | VERSION  |
#  |----------------------------------------|
#  | TIMESTAMP                              |
#  |----------------------------------------|
#  | FLAGS    | MESSAGE TYPE     | STX      |
#  |----------------------------------------|
#  | PAYLOAD (VARIABLE LENGTH)              |
#  |                    --------------------|
#  |                    | CHECKSUM          |
#  ------------------------------------------
#  @endmanonly
#
#  @defgroup fields Field Description
#  @ingroup format
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><b>Field</b></td>
#  <td><b>Size</b></td></tr>
#  <tr><td>SOH</td><td>1 byte</td></tr>
#  <tr><td>Length</td><td>2 bytes</td></tr>
#  <tr><td>Version</td><td>1 byte</td></tr>
#  <tr><td>Timestamp</td><td>4 bytes</td></tr>
#  <tr><td>FLAGS</td><td>1 byte</td></tr>
#  <tr><td>Message Type</td><td>2 bytes</td></tr>
#  <tr><td>STX</td><td>1-byte</td></tr>
#  <tr><td>Payload</td><td><i>n</i> bytes</td></tr>
#  <tr><td>Checksum</td><td>2 bytes</td></tr>
#  </table><br>
#  @endhtmlonly 
#  @manonly
#  Field        | Size
#  -------------+--------
#  SOH          | 1 byte
#  Length       | 2 bytes
#  Version      | 1 byte
#  Timestamp    | 4 bytes
#  FLAGS        | 1 byte
#  Message Type | 2 bytes
#  STX          | 1 byte
#  Payload      | n bytes
#  Checksum     | 2 bytes
#  @endmanonly
#
#  \b SOH                                                                     \n
#
#  0xAA, indicates start of data transmission.                              \n\n
#
#  \b Length                                                                  \n
#
#  First byte in the field: Number of bytes following the field (but not 
#  including). Second byte: Compliment of the first.                        \n\n
#
#  Example:                                                                   \n
#
#  Number of bytes following: 0x1D
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><b>Field</b></td>
#  <td><b>Value</b></td></tr>
#  <tr><td>Length[0]</td><td>0x1D</td></tr>
#  <tr><td>Length[1]</td><td>0xE2</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Field     | Value
#  ----------+------
#  Length[0] | 0x1D
#  Length[1] | 0xE2
#  @endmanonly                                                                \n
#
#  \b Version                                                                 \n
#
#  Version of protocol – currently 0x0.                                     \n\n
#
#  \b Timestamp                                                               \n
#
#  4 bytes representing the time of message transmission, in milliseconds. 
#  Little-endian byte order. When data is returned from the platform, the 
#  timestamp is the platform clock, which resets to 0 when the platform is 
#  reset. However, the platform clock can be changed via 0x0005 – “Set Platform 
#  Time”.                                                                   \n\n 
#
#  As the timestamp can represent at most 50 days, it is recommended 
#  that the program transmits “milliseconds since program start” instead of 
#  using a measure based off the Unix epoch (unless the top bits are to be
#  truncated).                                                              \n\n
#
#  For ease of processing any returned acknowledgements, no two sent messages 
#  should have the same timestamp. To avoid this, refrain from sending messages 
#  at a rate faster than 1 KHz. If serial communication hardware is in use, 
#  this will be enforced by the 115200 maximum baud rate.                   \n\n
#
#  \b Flags                                                                   \n
# 
#  One byte, as described below.
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><b>7</b></td><td><b>6</b></td>
#  <td><b>5</b></td><td><b>4</b></td><td><b>3</b></td><td><b>2</b></td>
#  <td><b>1</b></td><td><b>0</b></td></tr>
#  <tr><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td>
#  <td>Suppress ACK</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  ---------------------------------
#  | - | - | - | - | - | - | - | A |
#  ---------------------------------
#  @endmanonly                                                                \n
#
#  <i>[0] Suppress ACK (A)</i>                                                \n
#
#  Platform is not to acknowledge this command or request. Not recommended for 
#  use in the field, but may be of assistance in the initial development of 
#  custom implementations.                                                  \n\n
#
#  \b Message \b Type                                                         \n
#
#  The type of data contained in the payload.                               \n\n
#
#  \b STX                                                                     \n
#
#  0x55, indicates start of data payload.                                   \n\n
#
#  \b Payload                                                                 \n
#
#  Payload contents. Varies for each message type.                          \n\n
#
#  \b Checksum                                                                \n
#
#  16 bit CRC-CCITT, encompassing the entire message. See CRC Generation for 
#  further details about the checksum. Checksum is transmitted in a 
#  little-endian byte order.
#   


## Horizon Message
#
#  Represents a message to be transmitted to / received from the hardware.    \n
#
#  @warning Data should not be modified once created
#
#  @since 0.1
#
#  @section data Data Format
#  @subsection Package
#  @copydoc package
#  @subsection fields Field Description
#  @copydoc fields
#
#  @pydoc
class HorizonMessage():
    """Horizon Protocol Message"""
    
    
    # Horizon Message Constants
    ## Start of Header Flag
    SOH = 0xAA
    ## Start of Data Payload Flag
    STX = 0x55
    
    
    ## Create A Horizon Message
    #
    #  Constructor for the Horizon Message Class.                             \n
    #  The constructor can be called two different ways:
    #  - HorizonMessage(raw = None, version, ...)                             \n
    #    Create a message to send.                                            \n 
    #    Version auto-detection is unsupported.
    #  - HorizonMessage(version,raw,payload_type,store_error)                 \n
    #    Parse raw data (most likely received) into message variables.        \n 
    #    Version auto-detection is slightly supported.
    #
    #  @param  code           The message code
    #  @param  no_ack         Don't Acknowledge Flag
    #  @param  payload        The message payload
    #  @param  payload_type   The message payload class to instantiate 
    #                         (requires raw)
    #  @param  raw            Raw data buffer to parse
    #  @param  store_error    Store error and return?
    #                         False - raise errors
    #  @param  timestamp      Message Timestamp (milliseconds)
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws ChecksumError  If checksum verification fails
    #  @throws LookupError    If auto-detect version fails
    #  @throws ValueError     If values are out of range or if raw is invalid
    #
    #  @pydoc
    def __init__(self, code = 0x0000, payload = None, payload_type = None, 
                 store_error = False, no_ack = False, 
                 raw = None, timestamp = 0, version = tuple([-1,0])):
        """Create A Horizon Message"""
        
        # Class Variables
        ## Message Type
        self._code = 0x0000
        ## Raw Message Data (byte list)
        self._data = []
        ## Message Creation Error
        self._error = None 
        ## Don't Acknowledge Flag
        self._no_ack = False
        ## Message Payload
        self._payload = None
        ## Message Sequence ID
        #  @deprecated No longer in Horizon as of v0.4
        self._sequence = 0
        ## Message Timestamp
        self._timestamp = 0
        ## Message Version
        self.version = version
        
        
        # Create Constructor
        if raw == None:
            self._data += utils.from_byte(versioning.PROTOCOL_BYTE)
            
            # timestamp
            if timestamp < 0 or timestamp > 4294967295: # 4294967295 for int
                logger.warning("Tried to create %s with invalid timestamp %d!" \
                         % (self.__class__.__name__, timestamp))
                self._error = ValueError("Invalid timestamp!")
                if not store_error: raise self._error  
                else: return
            self._timestamp = timestamp
            logger.debug("%s: Timestamp: %d" % (self.__class__.__name__, 
                     self._timestamp))

            self._data += utils.from_unsigned_int(timestamp)
            
            # flags
            self._no_ack = no_ack
            logger.debug("%s: No Ack: %d" % (self.__class__.__name__, 
                     self._no_ack))
            if no_ack:
                self._data += utils.from_byte(1)
            else:
                self._data += utils.from_byte(0)
            
            # type
            if code < 0 or code > 65535:  # 65535 for short
                logger.warning("Tried to create %s with invalid message type " \
                             "0x%04x!" % self.__class__.__name__, code)
                self._error = ValueError("Invalid message type!")
                if not store_error: raise self._error  
                else: return
            self._code = code
            logger.debug("%s: Code: %d" % (self.__class__.__name__, 
                     self._code))
            self._data += utils.from_unsigned_short(code)
            
            
            # Payload
            self._data.append(HorizonMessage.STX)
            self._payload = payload
            self._data += payload.data
            
            # Update Length
            length = len(self._data)+2 # +2 for checksum
            logger.debug("%s: Length: %d" % (self.__class__.__name__, 
                     length))

            self._data = utils.from_byte(length) + \
                utils.from_byte(0xFF&(~(0xFF&length))) + \
                self._data
            self._data.insert(0,HorizonMessage.SOH)
            
            # Checksum
            checksum = utils.ccitt_checksum(self._data)
            self._data += utils.from_unsigned_short(checksum)     
            
            # Final Result
            logger.debug("%s: raw message data: %s" % (self.__class__.__name__, 
                     str(self)))
            
        # Parse Constructor
        else:
            self._data = raw
            logger.debug("%s: raw message data: %s" % (self.__class__.__name__, 
                     str(self)))
            
            # Perform Checksum First to filter out bad data
            checksum = utils.to_unsigned_short(raw[-2:])
            if utils.ccitt_checksum(raw[:-2]) != checksum:
                logger.warning("%s: Bad checksum!" % self.__class__.__name__)
                self._error = utils.ChecksumError("Bad Checksum!")
                if not store_error: raise self._error  
                else: return
            
            # SOH
            if raw[0] != HorizonMessage.SOH:
                logger.warning("%s: Bad SOH!" % self.__class__.__name__)
                self._error = ValueError("Invalid SOH!")
                if not store_error: raise self._error  
                else: return
            
            # Length
            length = 0
            # @deprecated
            length = utils.to_byte(raw[1:2])
            if length != len(raw) - 3 or length != 0xFF & (~raw[2]):
                logger.warning("%s: Bad length!" \
                                   % self.__class__.__name__)
                self._error = ValueError("Invalid Length!")
                if not store_error: raise self._error  
                else: return
                logger.debug("%s: Length: %d" % (self.__class__.__name__, 
                                                 length))
      
            # Timestamp
            self._timestamp = utils.to_unsigned_int(raw[4:8])
            logger.debug("%s: message timestamp: %d" %(self.__class__.__name__, 
                     self._timestamp))
                
            # Verify Flags
            if utils.to_byte(raw[8:9]) == 1:
                self._no_ack = True
            elif utils.to_byte(raw[8:9]) != 0: 
                logger.warning("%s: Bad flags!" \
                                 % self.__class__.__name__)
                self._error = ValueError("Invalid Flags!")
                if not store_error: raise self._error 
                else: return
            else:
                self._no_ack = False
            logger.debug("%s: no_ack flag: %d" %(self.__class__.__name__, 
                     self._no_ack))

            # Message Type
            self._code = utils.to_unsigned_short(raw[9:11])
            logger.debug("%s: type: 0x%04x" % (self.__class__.__name__, 
                     self._code))
            
            # STX
            pnext = 11
            if raw[pnext] != HorizonMessage.STX:
                logger.warning("%s: Bad STX!" % self.__class__.__name__)
                self._error = ValueError("Invalid STX!")
                if not store_error: raise self._error  
                else: return
            pnext = pnext+1
                
            # Payload
            self._payload = payload_type(raw = raw[pnext:-2], version = version,
                                         store_error = store_error,
                                         timestamp = timestamp)


    ## Message String Representation
    #
    #  Return the entire message in hex string
    # 
    #  @return String of hex info
    #
    #  @pydoc
    def __str__(self):
        """Return the entire message in hex string"""
        
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
        if self._error != None: store_error = True
        raw = None
        if self._data != None: raw = self._data[:]
        
        # create copy
        logger.debug("%s: Creating copy." % self.__class__.__name__)
        return HorizonMessage(store_error = store_error, raw = raw, 
                              version = self.version,
                              payload_type = self._payload.__class__)
    
    
    ## Translate Instance
    #
    #  @param  version The version to translate to
    #  @return A deep copy of this object in a different version
    #  @throws LookupError if version not supported
    #
    #  @pydoc
    def translate(self, version = tuple([1,0])):
        """Translate Instance"""
        
        # check version
        if version[0] == -1 or (version[0] == 0 and version[1] == 0) or\
                version not in self.versions:
            logger.warning("%s: Unsupported version %d.%d!" % (
                                self.__class__.__name__, version[0],version[1]))
            raise LookupError("Invalid / unsupported version!")
        
        # determine altered data
        store_error = False
        if self._error != None: store_error = True
        payload = self._payload.translate(version)
        code = self._code
        
        # create copy
        logger.debug("%s: Creating translated copy." % self.__class__.__name__)
        return HorizonMessage(store_error = store_error, raw = None, 
                              version = version, no_ack = self._no_ack,
                              timestamp = self._timestamp, code = code,
                              payload = payload)
    
    
    ## Human Readable Message String
    #
    #  Return the message as a human readable string
    #
    #  @return Human readable string representation of the message
    #
    #  @pydoc
    def print_format(self):
        """Return the message as a human readable string"""
        
        return "Version: %d.%d\nTime: %s\nNo ACK: %d\nType: %X\n%s" % (
                self.version[0], self.version[1], self._timestamp, 
                self._no_ack, self._code, self._payload.print_format())
    
    
    ## Raw Bytes Message Representation
    #
    #  Convert the message into raw bytes (character string for python 2.x) 
    #  useful for writing to devices.
    #
    #  @return Raw Bytes
    #
    #  @pydoc 
    def raw_string(self):
        """Returns the data converted into raw bytes."""
        
        return utils.to_bytes(self._data)
    
    
    ## Get Message Type
    #
    #  @return The message's type.
    #
    #  @pydoc
    def get_code(self):
        """Get Message Type"""
        
        return self._code
    
    
    ## Get Raw Data
    #
    #  @return The payload's raw data. (byte list)
    #
    #  @pydoc
    def get_data(self):
        """Get Raw Data"""
        
        return self._data[:]
        
        
    ## Get Creation Error
    #
    #  @return The error encountered during instantiation. 
    #          None if everything is fine.
    #
    #  @pydoc
    def get_error(self):
        """Get Creation Error"""
        
        return self._error
    
    
    ## Get No Ack Flag
    #
    #  @return The message's no_ack Flag.
    #
    #  @pydoc
    def get_no_ack(self):
        """Get No Ack Flag"""
        
        return self._no_ack
    
    
    ## Get Payload
    #
    #  @return The message's payload.
    #
    #  @pydoc
    def get_payload(self):
        """Get Payload"""
        
        return self._payload
    
    
    ## Get Sequence
    #
    #  @deprecated No longer in Horizon as of v0.4
    #  @return The message's sequence id.
    #
    #  @pydoc
    def get_sequence(self):
        """Get Sequence"""
        
        return self._sequence
    
    
    ## Get Timestamp
    #
    #  @return The message's timestamp.
    #
    #  @pydoc
    def get_timestamp(self):
        """Get Timestamp"""
        
        return self._timestamp
    
    
    ## Get Horizon Version
    #
    #  @return the horizon version of this payload
    #
    #  @pydoc
    def getversion(self):
        """Get Horizon Version"""
        
        return self.version
    
    
    # Class Properties
    ## Message Type
    code = property(fget=get_code, doc="Message Type")
    ## Raw Data
    data = property(fget=get_data, doc="Raw Data")
    ## Creation Error
    error = property(fget=get_error, doc="Creation Error")
    ## No Acknowledgment Flag
    no_ack = property(fget=get_no_ack, doc="No Acknowledgment Flag")
    ## Message Payload
    payload = property(fget=get_payload, doc="Message Payload")
    ## Message Sequence
    #  @deprecated No longer in Horizon as of v0.4
    sequence = property(fget=get_sequence, doc="Message Sequence")
    ## Message Timestamp
    timestamp = property(fget=get_timestamp, doc="Message Timestamp")



logger.debug("... clearpath.horizon.messages loaded.")
