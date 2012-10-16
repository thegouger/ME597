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
#  File: transports.py
#  Desc: Horizon Message Transport Controllers
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
    print ("ERROR: clearpath.horizon.transports is a module and can NOT be run"\
           " as a script!\nFor a command-line interface demo of Horizon, run:"\
           "\n  python -m clearpath.horizon.demo\n"\
           "For Horizon message forwarding, run:\n"\
           "  python -m clearpath.horizon.forward")

    # Exit Error
    import sys
    sys.exit(1)




################################################################################
# Module



## @package clearpath.horizon.transports 
#  Horizon Transports Python Module
# 
#  Horizon Message Transport Controllers                                      \n
#  Supported Horizon version(s): 0.1 - 1.0                                    \n
#                                                                             \n
#  Supported Transports:
#  - Serial (RS-232)
#  - Transmission Control Protocol (TCP) [with/without encryption]
#  - User Datagram Protocol (UDP)
#
#  @author     Ryan Gariepy
#  @author     Malcolm Robert
#  @date       18/01/10
#  @todo       Would be nice to support version translation in IP Servers, 
#              however, that would require additional work to detect the
#              version that clients are using which is currently not supported
#              by the current framework.
#  @todo       FIX encryption
#  @req        clearpath.utils                                                \n
#              clearpath.horizon.messages                                     \n
#              clearpath.horizon.payloads                                     \n
#              clearpath.horizon.router                                       \n
#              clearpath.horizon.versioning                                   \n
#              pyCrypto [http://www.dlitz.net/software/pycrypto/]             \n
#              -- for HorizonTransport_TCP_Client encryption                  \n
#              -- for HorizonTransport_TCP_Server encryption                  \n
#              pySerial [http://pyserial.sourceforge.net/]                    \n
#              -- for HorizonTransport_Serial
#  @version    1.0
#
#  @section USE
#
#  The intended purpose of this module is to provide functionality to send
#  and receive Horizon messages over the various transports with minimal
#  knowledge of messages and no knowledge of message payloads. Further, as
#  some transports support a one-to-many connection scheme, this module also
#  provides message routing. Note that as this module implements the transports
#  as defined in the Horizon specification, there is no support for version
#  translation. If version translation is desired, refer to the module 
#  clearpath.horizon.forward.
#
#  While the base class HorizonTransport is fully functional as a loop-back
#  device, the Horizon specification does not support loop-back devices and
#  connecting a loop-back device to a platform may have undesired side-effects.
#  A client transport is a transport that would be used by a client (controller,
#  logger, etc.) to connect to a platform, whereas a server is used to 
#  masquerade as a platform and accept client connections.
#
#  To use a transport synchronously, instantiate the associated HorizonTransport 
#  class, passing in appropriate information, and then call the open method.
#  The method send_message may now be used to send messages (failure may
#  indicate a lack of connection) and the method get_waiting to obtain received
#  messages.
#  To use a transport asynchronously, instantiate the associated
#  HorizonTransport class, passing in appropriate information. Create one or 
#  more non-blocking methods that accepts one argument (HorizonMessage) and 
#  pass them to the method add_handler. After calling the method open, messages 
#  will be passed to all created methods that were passed to add_handler upon 
#  receipt. Messages may be sent with the method send_message.
#  Alternatively, a transport may be used asynchronously by creating a subclass
#  of the associated HorizonTransport class, overriding the method 
#  message_received, which gets called each time a message arrives. After
#  instantiation and calling open, messages may be sent using the send_message
#  method.
#  Regardless of method of use, when the transport is no longer needed the
#  method close should be called and, at the very least, the HorizonTransport
#  class instance must be deleted (calls close if not already done so).
#
#  Various logging messages (debug, warning,...) are logged with the standard
#  Python logger to the log named 'clearpath.horizon.transports'.             \n
#  Messages are ignored by default; remove the only handler from it and turn
#  on propagation to get messages to propagate.
#
#  @section HISTORY
#
#  Versions 0.1 - 0.3 {Ryan Gariepy}
#  - Initial Creation as protocol.py
#
#  Version 0.4 {Malcolm Robert}
#  - Moved to horizon_protocol.py
#  - Added protocol abstraction
#  - Added TCP/IP
#  - Added UDP
#  - Added logging
#  - Added version support
#  - Added Doxygen documentation
#  - Changed version scheme to match Horizon doc
#  - Horizon support for v0.4
#
#  Version 0.5
#  - Horizon support for v0.5
#
#  Version 0.6
#  - Moved to protocol.py
#  - Horizon support for v0.6
#  - Improved search for header in read
#  - Added TCP encryption
#
#  Version 0.7
#  - Moved to transport.py
#  - Added Server transports
#  - Added IPv6 support
#  - Horizon support for v 0.1 - 0.7
#  - Python 2.6+ & 3.x compatible
#
#  Version 0.8
#  - Horizon support for v 0.1 - 0.8
#
#  Version 1.0
#  - Added one-to-many support
#  - Horizon support for v 0.1 - 1.0
#
#  @section License
#  @copydoc public_license
#
#  @defgroup hardware Hardware
#  @ingroup doc
#  
#  @copydoc serial
#
#  If higher bandwidth communication is required, the protocol may also be 
#  packaged verbatim within TCP/IP or UDP packets if desired, with one protocol 
#  message per packet. The default Horizon port for TCP/UDP is 7284.
#
#  @copydoc tcp
#
#  @copydoc udp
#
"""Horizon Message Transport Controllers

   Copyright © 2010 Clearpath Robotics, Inc.
   All rights reserved
   
   Created: 17/03/10
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
from .  import messages         # Horizon Protocol Message Definition
from .  import payloads         # Horizon Protocol Message Payload Definitions
from .  import router           # One-to-many Support
from .  import versioning       # Horizon Protocol Versions 
from .  import codes            # Horizon Protocol Versions 

# Required Python Modules
import datetime                 # Date & Time Manipulation
import logging                  # Logging Utilities
import socket                   # UDP Port Control
import sys                      # Python Interpreter Functionality
import threading                # Python Thread Support
import time                     # System Date & Time

# Version Dependent Modules
if sys.version_info[0] > 2:
    import queue                # Thread-safe Queue
else:
    import Queue as queue       # Thread-safe Queue

# Non-Standard Modules
try:
    import Crypto.Random        # Encryption Random Generator
except ImportError:
    pass
try:
    import serial               # Serial Port Control
except ImportError:
    pass


# Module Support
## Module Version
__version__  = "1.0"
"""Module Version"""
## SVN Code Revision
__revision__ = "$Revision: 488 $"
""" SVN Code Revision"""


## Supported Horizon Versions
versions = set()
"""Supported Horizon Versions"""


## Clearpath Default Horizon TCP/UDP Port
horizon_port = 7284             # 7284 -> 'PATH'
"""Clearpath Default Horizon TCP/UDP Port"""     


## Message Log
logger = logging.getLogger('clearpath.horizon.transports')
"""Horizon Transports Module Log"""
logger.setLevel(logging.NOTSET)
logger.addHandler(utils.NullLoggingHandler())
logger.propagate = False
logger.debug("Loading clearpath.horizon.transports ...")              




################################################################################
# Horizon Transport Controller



## Horizon Transport
#
#  Represents the Horizon message transport layer.                            \n
#  Handles sending and receiving messages.                                    \n
#  Provides both synchronous and asynchronous functionality.                  \n
#                                                                             \n
#  Synchronous:                                                               \n
#  - Do NOT use the methods add_handler, get_handlers, and remove_handler.
#  - Do NOT override message_received.
#  - Use get_waiting to retrieve messages received within the past 
#    store_timeout ms.                                                        \n
#                                                                             \n
#  Asynchronous:
#  - Do NOT use the method get_waiting.
#  - Override the method message_received and do NOT use the methods 
#    add_handler, get_handlers, and remove_handler; or
#  - Use the methods add_handler, get_handlers, and remove_handler and do NOT
#    override the method message_received.                                    \n
#                                                                             \n
#  Basic transport simply acts as a loopback device. The loopback device is 
#  not a part of the Horizon protocol supported transports and is intended for 
#  development use only.                                                      \n
#                                                                             \n
#  To be inherited for specific transport implementations and/or for 
#  asynchronous message handling.                                             \n
#  Implementation subclasses should only override __init__, __del__, __str__, 
#  _open, _close, _get_message, send_message, and get_name.                  \n
#  Message handling subclasses should only override __init__, __del__, 
#  message_received, add_handler, and get_waiting.                            \n
#  In both cases, __init__ and __del__ should call the parent's overridden
#  method.                                                                    \n
#
#  @since NA - Not in the Horizon Protocol
#  
#  @pydoc
class HorizonTransport(router.HorizonRouteable):
    """Horizon Transport Controller - Loopback Device"""
        
    ## Create A Horizon Transport
    #  
    #  Constructor for the Horizon message Transport controller.              \n
    #  Does NOT support version auto-detection.                               \n
    #                                                                         \n
    #  Override this method for subclass initialization.                      \n
    #  Overriding methods should call this method with subclass=True. 
    #
    #  @param  subclass       Is this method being called from the overriding
    #                         subclass' __init__ method?
    #  @param  store_timeout  The time to store an un-handled message for the 
    #                         method get_waiting in milliseconds,
    #                         0 - store indefinitely
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws TransportError Upon creation/initialization failure
    #  @throws ValueError     Upon bad arguments
    #
    #  @pydoc
    def __init__(self, store_timeout = 10000, version = tuple([-1,0]), 
                 subclass = False):
        """Create A Horizon Transport"""
        logger.debug("%s: Instance creation started..." % \
                     self.__class__.__name__)
        
        # Class Variables
        ## Message Handlers
        self._handlers = []
        ## Message Storage
        self._messages = queue.Queue()  # Format: tuple(timestamp,message)
        ## Is Open?
        self.opened = False
        ## Receive Thread
        self._receiver = threading.Thread(target = self._receive)
        ## Receive Looping?
        self._receiving = False
        ## Store Timeout
        self._store_timeout = store_timeout
        
        # Loop-back specific
        if subclass == False:
            ## Loop-back Link Buffer
            self._link = queue.Queue()
        

        # Verify timeout
        if store_timeout < 0:
            logger.error("%s: Invalid store timeout!" % self.__class__.__name__)
            raise ValueError ("Invalid store timeout!")
        logger.info("%s: Using %d ms store timeout." % (self.__class__.__name__,
                                                  self._store_timeout))
        
        # Setup Thread
        self._receiver.setDaemon(True)
    
        logger.debug("%s: ...instance creation complete." % \
                     self.__class__.__name__)
    
    
    ## Destroy A Horizon Transport
    #  
    #  Destructor for the Horizon message Transport protocol.                 \n
    #  Ensures the connection is closed by calling close.                     \n
    #                                                                         \n
    #  Override this method for subclass destruction.                         \n
    #  Overriding methods should call this method. 
    #
    #  @pydoc
    def __del__(self):
        """Destroy A Horizon Transport"""
        logger.debug("%s: Instance destruction started..." % \
                     self.__class__.__name__)
        
        # Cleanup Transport
        self.close()
        
        logger.debug("%s: ...instance destruction complete." % 
                     self.__class__.__name__)
        
        
    ## String Representation
    #
    #  Return the transport device name.                                      \n
    #                                                                         \n
    #  Override this method for subclasses implementing devices other than
    #  loop-back. For example, a port device would return the port.
    # 
    #  @return String of transport device name.
    #
    #  @pydoc
    def __str__(self):
        """Return the transport device name."""
        
        return 'loop-back'
    
    
    ## Horizon Transport Check Timeouts
    #  
    #  Message timeout checker.                                               \n
    #                                                                         \n
    #  Do not override this method.
    #  
    #  @pydoc
    def _timeout(self):
        """Horizon Transport Check Timeouts"""
        if self._store_timeout > 0 and (not self._messages.empty()):
            
            # Update Timestamp
            timestamp = 0
            t = datetime.datetime.today()
            timestamp = t.microsecond/1000 + t.second*1000 + \
                  t.minute*60*1000 + t.hour*60*60*1000 + t.day*24*60*60*1000
            while timestamp > 4294967295: timestamp -= 4294967295
            
            # Remove Timeout Messages
            while (not self._messages.empty()):
                msg_time = self._messages.queue[0][0]
                if (timestamp - msg_time >= self._store_timeout) or\
                        (timestamp < msg_time and 4294967295 - \
                         msg_time + timestamp >= self._store_timeout):
                    logger.warning("%s: message store timeout occurred." %\
                                   (self.__class__.__name__))
                    self._messages.get()
                else:
                    break

    
    ## Transport Receiver Loop
    #  
    #  Receiver Thread method. Loops on _receiving waiting for messages.      \n
    #  Calls _getMessage.                                                     \n
    #                                                                         \n
    #  Do not override this method.
    #
    #  @pydoc
    def _receive(self):
        """Transport Receiver Loop"""
        logger.debug("%s: Entering receive loop..." % self.__class__.__name__)
        
        # Continuous Loop
        while self._receiving:
            
            # Check for message
            try:
                message = self._get_message()
                if message != None:
                    logger.debug("%s: received message:\n %s" % \
                             (self.__class__.__name__, str(message)))
            except utils.TransportError as ex:
                logger.warning(
                        "%s: error in attempting to retrieve message:\n%s" %\
                        (self.__class__.__name__,ex))
                message = None
                
            # Handle Message
            if message != None:
                self.message_received(message)
            
            # Check store timeouts
            self._timeout()
            
            # Release Processor
            time.sleep(0.001)
        
        logger.debug("%s: ...receive loop exited." % self.__class__.__name__)
        
    
    ## Transport Device Open
    #
    #  Performs underlying transport device opening and required handshaking. \n
    #  Called by open.                                                        \n
    #                                                                         \n
    #  Override this method in subclasses implementing devices other than
    #  loop-back.
    #
    #  @throws TransportError upon opening failure
    #
    #  @pydoc
    def _open(self):
        """Transport Device Open"""
        
        # Nothing to do for loopback device
        logger.debug("%s: Loop-back port open." % self.__class__.__name__)
        
    
    ## Transport Device Close
    #
    #  Performs underlying transport device closing and required handshaking. \n
    #  Called by close. May be called even if already closed.                 \n
    #                                                                         \n
    #  Override this method in subclasses implementing devices other than
    #  loop-back.
    #
    #  @throws TransportError upon closing failure
    #
    #  @pydoc
    def _close(self):
        """Transport Device Close"""
        
        # Nothing to do for loopback device
        logger.debug("%s: Loop-back port closed." % self.__class__.__name__)
        
    
    ## Transport Device Get Horizon Message
    #
    #  Pulls a Horizon message from the device.                               \n 
    #  Non-blocking - returns None if no message waiting.                     \n       
    #  Called by receive thread.                                              \n
    #  Uses HorizonMessage(raw, payload_type, version, store_error=True) to 
    #  instantiate the message.                                               \n
    #                                                                         \n
    #  Override this method in subclasses implementing devices other than
    #  loop-back.
    #
    #  @throws TransportError upon receive failure
    #  @return received HorizonMessage or None
    #
    #  @pydoc
    def _get_message(self):
        """Transport Device Get Horizon Message"""
        if not self.opened: return None
        
        # Remove from link buffer
        msg = None
        logger.debug("%s: Loop-back port read started..." % \
                     self.__class__.__name__)
        try:
            msg = messages.HorizonMessage(raw = self._link.get(block=False),
                                        payload_type = payloads.HorizonPayload,
                                          store_error = True)
        except queue.Empty:
            msg = None
        logger.debug("%s: ...loop-back port read complete." % \
                     self.__class__.__name__)
        
        if msg != None:
            logger.info("%s: Message of %d bytes found:\n%s" % (
                    self.__class__.__name__, len(msg.data), 
                    ' '.join(map(utils.hex,msg.data))))
        return msg
    
    
    ## Horizon Transport Open
    #  
    #  Opens the transport device and starts the receiver thread.             \n
    #  Calls _open.                                                           \n
    #                                                                         \n
    #  Do not override this method.
    #
    #  @throws TransportError upon open failure
    #
    #  @pydoc
    def open(self):
        """Horizon Transport Open"""
        if not self.opened:
            logger.debug("%s: Device opening started..." % \
                     self.__class__.__name__)
        
            # Open Device
            self._open()
        
            # Start Thread
            self._receiving = True
            self._receiver.start()
            
            self.opened = True
            logger.debug("%s: ...device opening complete." % \
                     self.__class__.__name__)
        
    
    ## Horizon Transport Close
    #  
    #  Closes the transport device and stops the receiver thread.             \n
    #  Calls _close.                                                          \n
    #                                                                         \n
    #  Do not override this method.
    #
    #  @throws TransportError upon open failure
    #
    #  @pydoc
    def close(self):
        """Horizon Transport Close"""
        logger.debug("%s: Device closing started..." % \
                     self.__class__.__name__)
    
        # Cleanup Thread
        self._receiving = False
        self.opened = False
        
        # Cleanup Device
        self._close()
        logger.debug("%s: ...device closing complete." % \
                     self.__class__.__name__)
        
    
    ## Transport Device Send Horizon Message
    #
    #  Sends a Horizon message to the device. Blocks until send is complete.  \n
    #                                                                         \n
    #  Can be called three different ways:
    #  - send_message(message, raw=None)
    #    Sends the given HorizonMessage using HorizonMessage.raw_string()
    #  - send_message(message=None, raw=data)
    #    Sends the given Byte Array using clearpath.utils.to_ascii()
    #  - send_message(message=None, raw)
    #    Sends the given raw string                                           \n
    #                                                                         \n
    #  Override this method in subclasses implementing devices other than
    #  loop-back.
    #
    #  @param  message    The message to send, None if using raw
    #  @param  raw        The byte array or raw string to send, 
    #                     None if using message
    #  @throws TransportError upon send failure
    #
    #  @pydoc
    def send_message(self, message, raw = None):
        """Transport Device Send Horizon Message"""
        if not self.opened:
            logger.error("%s: Cannot send while closed!" % \
                           self.__class__.__name__)
            raise utils.TransportError ("Cannot send while closed!")
            
        # Add copy to link buffer
        logger.debug("%s: Loop-back port write started..." % \
                     self.__class__.__name__)
        if message != None:
            raw = message.raw_string()
        elif not isinstance(raw,str):
            raw = utils.to_ascii(raw)
        self._link.put(raw)
        logger.debug("%s: ...loop-back port write complete." % \
                     self.__class__.__name__)
        logger.info("%s: Wrote %d bytes:\n%s" % \
                            (self.__class__.__name__, len(raw),
                             ' '.join(map(utils.hex,map(ord,raw)))))
    
    
    ## Horizon Transport Message Routed
    #
    #  The router has determined that this transport is to receive a message.
    #  Called by the router.                                                  \n
    #                                                                         \n
    #  Do not override this method.
    #
    #  @param  message    The message to receive
    #  @throws TransportError upon receive failure
    #
    #  @pydoc
    def message_routed(self, message):
        """Horizon Transport Message Routed"""
        
        # Send it through the transport
        self.send_message(message = message)
        
    
    ## Transport Device Horizon Message Received
    #
    #  Handles receiving a message.                                           \n
    #  Cannot be blocking and cannot raise exceptions.                        \n
    #  Called by the receive thread.                                          \n
    #                                                                         \n
    #  Override this method in subclasses that handle messages.
    #
    #  @param  message    The message received
    #
    #  @pydoc
    def message_received(self, message):
        """Transport Device Horizon Message Received"""
        
        # Routing
        if self.route_message(message) == True:
            return
        
        # Handlers?
        if len(self._handlers) > 0:
            for handler in self._handlers:
                handler(message)
        
        # Storage?
        else:
            timestamp = 0
            t = datetime.datetime.today()
            timestamp = t.microsecond/1000 + t.second*1000 + \
                  t.minute*60*1000 + t.hour*60*60*1000 + t.day*24*60*60*1000
            while timestamp > 4294967295: timestamp -= 4294967295
            self._messages.put(tuple([timestamp,message]))
                
           
    ## Horizon Transport Add Message Handler
    #  
    #  Adds a message handler.                                                \n
    #  Asynchronous method of receiving messages without subclasses.          \n
    #  All handlers will be called each time a message is received.           \n
    #  There is no guarantee of the order in which handlers are called.       \n
    #  For backtracking, get_waiting() will be used.                          \n
    #                                                                         \n
    #  Override this method in subclasses that handle messages to prevent the
    #  use of handlers.
    #
    #  @param  backtrack      Call the new handler with any waiting messages?
    #  @param  handler        The Message Handler,
    #                         Must have one parameter: HorizonMessage
    #
    #  @pydoc
    def add_handler(self, handler, backtrack = False):
        """Horizon Transport Add Message Handler"""

        # Add Handler
        self._handlers.append(handler)
        logger.debug("%s: handler %s added." % \
                     (self.__class__.__name__, handler.__name__))
        
        # Backtrack
        if backtrack:
            for message in self.get_waiting(): handler(message)
            logger.debug("%s: handler %s backtracked." % \
                     (self.__class__.__name__, handler.__name__))
            

    ## Horizon Transport Remove Message Handler
    #  
    #  Removes a message handler.                                             \n
    #                                                                         \n
    #  Do not override this method.
    #
    #  @param  handler        The Message Handler to remove.
    #                         If None, remove all handlers.
    #
    #  @pydoc
    def remove_handler(self, handler=None):
        """Horizon Transport Remove Message Handler"""

        # Remove Handler
        if handler != None and handler in self._handlers:
            self._handlers.remove(handler)
            logger.debug("%s: Handler %s removed." % \
                     (self.__class__.__name__, handler.__name__))
        elif handler == None:
            self._handlers = []
            logger.debug("%s: All handlers removed." % \
                     (self.__class__.__name__))
            

    ## Horizon Transport Get Message Handlers
    #  
    #  Gets the list of message handlers.                                     \n
    #                                                                         \n
    #  Do not override this method.
    #
    #  @return List of handlers, empty if none. 
    #
    #  @pydoc
    def get_handlers(self):
        """Horizon Transport Get Message Handlers"""

        # Get handlers 
        logger.debug("%s: Returned %d handlers." % \
                     (self.__class__.__name__, len(self._handlers)))
        return self._handlers[:]
        
        
    ## Horizon Transport Get Waiting Messages
    #  
    #  Gets a list of waiting messages.                                       \n
    #  Synchronous method of receiving messages.                              \n
    #                                                                         \n
    #  Override this method in subclasses that handle messages to prevent the
    #  use of handlers.
    #
    #  @return list of messages, empty if none
    #  
    #  @pydoc
    def get_waiting(self):
        """Horizon Transport Get Waiting Messages"""
        msgs = []
        
        # Get Messages
        while not self._messages.empty():
            msgs.append(self._messages.get(block=False))
            
        logger.debug("%s: Returned %d waiting messages." % \
                     (self.__class__.__name__, len(msgs)))
        return msgs
    
    
    ## Is Open?
    #
    #  Do not override this method.
    #
    #  @return is the transport open?
    #
    #  @pydoc
    def is_open(self):
        """Is Open?"""
        
        return self.opened
    
    
    ## Get Store Timeout
    #
    #  Do not override this method.
    #
    #  @return the message store timeout in ms
    #
    #  @pydoc
    def get_store_timeout(self):
        """Get Store Timeout"""
        
        return self._store_timeout
    
    
    ## Set Store Timeout
    #
    #  Do not override this method.
    #
    #  @param store_timeout the message store timeout in ms
    #  @throws ValueError     Upon bad timeout
    #
    #  @pydoc
    def set_store_timeout(self,store_timeout):
        """Get Store Timeout"""
        
        # verify
        if store_timeout < 0:
            logger.error("%s: Invalid store timeout!" % self.__class__.__name__)
            raise ValueError ("Invalid store timeout!")
        logger.info("%s: Using %d ms store timeout." % (self.__class__.__name__,
                                                  self._store_timeout))
        
        self._store_timeout = store_timeout
    
    
    
    ## Transport Name
    #
    #  Return the transport name.                                             \n
    #                                                                         \n
    #  Override this method for subclasses implementing transports other than
    #  loop-back. For example, a port device would return the port type.
    #
    #  @return String of transport name.
    #
    #  @pydoc
    def get_name(self):
        """Return the transport name."""
        
        return 'Loop-Back'
    get_name = staticmethod(get_name)
    
    
    # Class Properties
    ## The Transport Name
    name = property(fget=__str__, doc="The Transport Name")
    ## Transport Opened?
    opened = property(fget=is_open, doc="Transport Opened?")
    ## Message Store Timeout
    store_timeout = property(fget=get_store_timeout, fset=set_store_timeout,
                             doc="Message Store Timeout")

  
################################################################################
# Horizon Serial Controller



## @defgroup serial Serial
#  @ingroup hardware
#  The protocol is a bidirectional binary serial protocol intended for use over 
#  RS-232 or similar hardware. Settings are \b 115200 \b baud, \b 8 \b data 
#  \b bits, \b no \b parity, \b 1 \b stop \b bit.
#
#  @defgroup connection Connection Reference
#  @ingroup hardware
#  If your platform is equipped with a serial connection, the following 
#  information is provided for your reference. No null modem cable is 
#  necessary – the cable can be directly plugged into a hardware serial port or 
#  any USB to serial converter.
#  
#  <table border='0'><tr><td>
#  <b>Female DB9 Connection (platform):</b>
#  @image html female.png
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><b>Pin</b></td><td><b>Name</b></td>
#  <td><b>Direction</b></td><td><b>Description</b></td></tr>
#  <tr><td>1</td><td>DCD</td><td>OUT</td><td>Unused (N/C)</td></tr>
#  <tr><td>2</td><td>TX</td><td>OUT</td><td>Data from platform</td></tr>
#  <tr><td>3</td><td>RX</td><td>IN</td><td>Commands to platform</td></tr>
#  <tr><td>4</td><td>DTR</td><td>IN</td><td>Unused</td></tr>
#  <tr><td>5</td><td>SGND</td><td>N/A</td><td>Ground</td></tr>
#  <tr><td>6</td><td>DSR</td><td>OUT</td><td>Unused</td></tr>
#  <tr><td>7</td><td>RTS</td><td>IN</td><td>Unused</td></tr>
#  <tr><td>8</td><td>CTS</td><td>OUT</td><td>Unused</td></tr>
#  <tr><td>9</td><td>RI</td><td>OUT</td><td>Unused</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#   -------------
#   \\ 5 4 3 2 1 /
#  * \\ 9 8 7 6 / *
#     ---------
#  PIN 1 - DCD  - OUT - Unused (N/C)                                          \n
#  PIN 2 - TX   - OUT - Data from platform                                    \n
#  PIN 3 - RX   - IN  - Commands to platform                                  \n
#  PIN 4 - DTR  - IN  - Unused (N/C)                                          \n
#  PIN 5 - SGND - N/A - Ground                                                \n
#  PIN 6 - DSR  - OUT - Unused (N/C)                                          \n
#  PIN 7 - RTS  - IN  - Unused (N/C)                                          \n
#  PIN 8 - CTS  - OUT - Unused (N/C)                                          \n
#  PIN 9 - RI   - OUT - Unused (N/C)
#  @endmanonly
#  
#  </td><td>
#  <b>Male DB9 Connection (control computer):</b>
#  @image html male.png
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'><td><b>Pin</b></td><td><b>Name</b></td>
#  <td><b>Direction</b></td><td><b>Description</b></td></tr>
#  <tr><td>1</td><td>DCD</td><td>IN</td><td>Unused</td></tr>
#  <tr><td>2</td><td>RX</td><td>IN</td><td>Data from platform</td></tr>
#  <tr><td>3</td><td>TX</td><td>OUT</td><td>Commands to platform</td></tr>
#  <tr><td>4</td><td>DTR</td><td>OUT</td><td>Unused</td></tr>
#  <tr><td>5</td><td>SGND</td><td>N/A</td><td>Ground</td></tr>
#  <tr><td>6</td><td>DSR</td><td>IN</td><td>Unused</td></tr>
#  <tr><td>7</td><td>RTS</td><td>OUT</td><td>Unused</td></tr>
#  <tr><td>8</td><td>CTS</td><td>IN</td><td>Unused</td></tr>
#  <tr><td>9</td><td>RI</td><td>IN</td><td>Unused</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#   -------------
#   \\ 1 2 3 4 5 /
#  * \\ 6 7 8 9 / *
#     ---------
#  PIN 1 - DCD  - IN  - Unused                                                \n
#  PIN 2 - RX   - IN  - Data from platform                                    \n
#  PIN 3 - TX   - OUT - Commands to platform                                  \n
#  PIN 4 - DTR  - OUT - Unused                                                \n
#  PIN 5 - SGND - N/A - Ground                                                \n
#  PIN 6 - DSR  - IN  - Unused                                                \n
#  PIN 7 - RTS  - OUT - Unused                                                \n
#  PIN 8 - CTS  - IN  - Unused                                                \n
#  PIN 9 - RI   - IN  - Unused      
#  @endmanonly
#
#  </td></tr></table>


## Horizon Serial Controller
#
#  Provides a method to send and receive messages over RS-232.                \n
#  Guarantees order of arrival and arrival.                                   \n
#  Low Bandwidth use only.
#
#  @req        pySerial [http://pyserial.sourceforge.net/]
#  @since 0.1
#
#  @section Hardware
#  @copydoc serial
#
#  @section connection Connection Reference
#  @copydoc connection
#
#  @pydoc
class HorizonTransport_Serial(HorizonTransport):
    """Horizon Transport Controller - Serial Device"""
    
    name = "Serial"
    port = ''
        
        
    # Check for pySerial 
    try:
        serial
        
        
        ## Create A Horizon Serial Transport
        #  
        #  Constructor for the Horizon message Serial Transport controller.   \n
        #  Does NOT support version auto-detection.                           \n
        #                                                                     \n
        #  Override this method for subclass initialization.                  \n
        #  Overriding methods should call this method with subclass=True. 
        #
        #  @param  port           serial port name
        #  @param  subclass       Is this method being called from the 
        #                         overriding subclass' __init__ method?
        #  @param  store_timeout  The time to store an un-handled message for  
        #                         the method get_waiting in milliseconds,
        #                         0 - store indefinitely
        #  @param  version        Horizon Protocol Version,
        #                         (-1,*) represents the newest version,
        #                         (0,*) auto-detect the version (if supported)
        #  @throws LookupError    If auto-detect version fails
        #  @throws TransportError Upon creation/initialization failure
        #  @throws ValueError     Upon bad arguments
        #
        #  @pydoc
        def __init__(self, port = None, store_timeout = 10000, subclass = False):
            """Create A Horizon Serial Transport"""
            logger.debug("%s: Instance creation started..." % \
                         self.__class__.__name__)
            
            # Initialize Parent
            HorizonTransport.__init__(self, store_timeout = store_timeout, subclass = True)
        
            if port == None:
                logger.error("%s: ... failed. Must specify port." + \
                                 "Use HorizonTransport_Serial.autodetect()." % \
                                 self.__class__.__name__)
                raise utils.TransportError \
                    ("Serial Transport creation failed!\n")

            # Class Variables
            self.port = port
            self._rbuff = []
            self._serial_port = None
        
            # Initialization
            logger.info("%s: Using serial port %s at 115200 Bd." % \
                            (self.__class__.__name__, self.port))
            try:
                logger.debug("%s: Serial port initialization started..." % \
                                 self.__class__.__name__)
                self._serial_port = serial.Serial(port=port, baudrate=115200, timeout=0)

                # The open() method re-opens the serial port.
                self._serial_port.close()
                logger.debug("%s: ...serial port initialization complete." % \
                                 self.__class__.__name__)                    
        
            # Creation failed
            except serial.SerialException as ex:
                logger.error("%s: ...serial port initialization failed:\n%s" % \
                                 (self.__class__.__name__,str(ex)))
                raise utils.TransportError \
                    ("Serial Transport creation failed!\n" + str(ex))
            except ValueError as ex:
                logger.error("%s: ...serial port initialization failed:\n"\
                                 "Unexpected error encountered:\n%s" % \
                                 (self.__class__.__name__,str(ex)))
                raise utils.TransportError \
                    ("Unexpected error encountered!\n" + str(ex))

            logger.debug("%s: ...instance creation complete." % \
                             self.__class__.__name__)
                        
        @staticmethod
        def autodetect(**kwargs):
            ports = utils.list_serial_ports()

            ping_request_code = codes.find_code('request_echo')
            ping_data_code = codes.find_code('data_echo')
            for trynum in range(5):
                for port in ports:
                    #print("Attempting connection on %s" % port)
                    transport = HorizonTransport_Serial(port)

                    try:
                        transport.open()

                        ping_request = messages.HorizonMessage(
                            code = ping_request_code, 
                            payload = payloads.HorizonPayload_Request())
                        transport.send_message(ping_request)

                        for sleeping in range(5):
                            time.sleep(0.1)
                            msgs = transport.get_waiting();
                            for timestamp, msg in msgs:
                                if msg.code == ping_data_code:
                                    # Do not close before returning it, as that
                                    # will end the receiver thread.
                                    return transport

                    except Exception as ex:
                        # Error with serial port. Move on.
                        pass

                    # Not this one. Move on.
                    transport.close()

            time.sleep(0.2)
            raise utils.TransportError("Unable to autodetect a serial Horizon device.")
            return None
                

        ## String Representation
        #
        #  Return the transport device name.
        # 
        #  @return String of transport device name.
        #
        #  @pydoc
        def __str__(self):
            """Return the transport device name."""
            return self.port
    

        ## Serial Transport Device Open
        #
        #  @see HorizonTransport._open(self)
        #
        #  @throws TransportError upon opening failure
        #
        #  @pydoc
        def _open(self):
            """Serial Transport Device Open"""
        
            # Open Device
            try:
                logger.debug("%s: Serial port opening started..." % \
                             self.__class__.__name__)
                self._serial_port.open()
                logger.debug("%s: ...serial port opening complete." % \
                             self.__class__.__name__)
        
            # Open failed
            except serial.SerialException as ex:
                logger.error("%s: ...serial port opening failed:\n%s" % \
                             (self.__class__.__name__, str(ex)))
                raise utils.TransportError \
                        ("Serial Port open failed!\n" + str(ex))
        
    
        ## Serial Transport Device Close
        #
        #  @see HorizonTransport._close(self)
        #
        #  @throws TransportError upon closing failure
        #
        #  @pydoc
        def _close(self):
            """Serial Transport Device Close"""
        
            # Close device
            logger.debug("%s: Serial port closing started..." % \
                         self.__class__.__name__)
            self._serial_port.close()
            logger.debug("%s: ...serial port closing complete." % \
                         self.__class__.__name__)
        
    
        ## Serial Transport Get Horizon Message
        #
        #  @see HorizonTransport._get_message(self)
        #
        #  @throws TransportError upon receive failure
        #  @return received Horizon Message or None
        #
        #  @pydoc
        def _get_message(self):
            """Serial Transport Device Get Horizon Message"""
            if not self.opened: return None
            read = 0
            try:
            
                # read as much as possible
                chars = [b'0']
                logger.debug("%s: Serial port read started..." % \
                             self.__class__.__name__)
                while(len(chars) > 0):
                    chars = self._serial_port.read(256)
                    if len(chars) > 0:
                        try:
                            getattr(serial,"serial_for_url")
                            if sys.version_info[0] > 2:
                                self._rbuff += chars
                            else:
                                self._rbuff += map(ord,chars)
                        except AttributeError:
                            self._rbuff += map(ord,chars)
                        read += len(chars)
                logger.debug("%s: ...serial port read complete." % \
                                 self.__class__.__name__)
            
            # Read Failed
            except Exception as ex:
                logger.error("%s: ...serial port read failed:\n%s" % \
                             (self.__class__.__name__,str(ex)))
                raise utils.TransportError \
                        ("Serial Message get failed!\n" + str(ex))
            if read > 0 :
                logger.info("%s: Read %d bytes." % (self.__class__.__name__, 
                                                    read))

            # Look for message start (SOH)
            disc = []
            while(len(self._rbuff) > 3 and (
                    self._rbuff[0] != messages.HorizonMessage.SOH or
                    self._rbuff[1] != 0xFF&(~self._rbuff[2]) or
                    self._rbuff[1] == 0)):
                disc.append(self._rbuff.pop(0))

            if len(disc) > 0:
                logger.info("%s: Discarded %d bytes:\n%s" % (
                        self.__class__.__name__, len(disc), 
                        ' '.join(map(utils.hex,disc))))

            if len(self._rbuff) < 3:
                return None
         
            length = self._rbuff[1] + 3
        
            # Look for next message start
            for i in range(1,len(self._rbuff)-2):
                if self._rbuff[i] == messages.HorizonMessage.SOH and \
                        self._rbuff[1] == 0xFF&(~(self._rbuff[2])) and \
                        self._rbuff[1] != 0:
                    if i < length:
                        length = i
                    break
            
            # Not all read yet
            if len(self._rbuff) < length:
                return None
            
            # Return Message
            raw = self._rbuff[0:length]
            self._rbuff = self._rbuff[length:]
            logger.info("%s: Message of %d bytes found:\n%s" % (
                        self.__class__.__name__, len(raw), 
                        ' '.join(map(utils.hex,raw))))

            return messages.HorizonMessage(raw = raw, 
                                           payload_type = payloads.HorizonPayload,
                                           store_error = True)
        
    
        ## Serial Transport Device Send Horizon Message
        #
        #  @see HorizonTransport.send_message(self,message)
        #
        #  @param  message    The message to send, None if using raw
        #  @param  raw        The byte array or raw string to send, 
        #                     None if using message
        #  @throws TransportError upon send failure
        #
        #  @pydoc
        def send_message(self, message, raw = None):
            """Serial Transport Device Send Horizon Message"""
            if not self.opened:
                logger.error("%s: Cannot send while closed!" % \
                               self.__class__.__name__)
                raise utils.TransportError ("Cannot send while closed!")
            
            # Send Message
            if message != None:
                raw = message.raw_string()
            elif not isinstance(raw,str) and not isinstance(raw,bytes):
                raw = utils.to_bytes(raw)
            try:
                logger.debug("%s: Serial port write started..." % \
                             self.__class__.__name__)
                try:
                    getattr(serial,"serial_for_url")
                    sent = self._serial_port.write(raw)
                    if sent == None:
                        logger.error("%s: Serial port write failed!" % \
                                     self.__class__.__name__)
                        raise utils.TransportError ("Write Failed!")
                except AttributeError:
                    if sys.version_info[0] > 2:
                        self._serial_port.write(map(chr,raw))
                    else:
                        self._serial_port.write(raw)
                    sent = len(raw)
                if sent == len(raw):
                    logger.debug("%s: ...serial port write complete." % \
                                 self.__class__.__name__)
                if isinstance(raw,str):
                    logger.info("%s: Wrote %d bytes:\n%s" % \
                            (self.__class__.__name__, sent,
                            ' '.join(map(utils.hex,map(ord,raw[:sent])))))
                else:
                    logger.info("%s: Wrote %d bytes:\n%s" % \
                            (self.__class__.__name__, sent,
                            ' '.join(map(utils.hex,raw[:sent]))))
                if sent < len(raw):
                    logger.error("%s: Serial port write incomplete!" % \
                                     self.__class__.__name__)
                    raise utils.TransportError ("Write Incomplete!")
                    
            # Send Failed
            except serial.SerialException as ex:
                logger.error("%s: ...serial port write failed:\n%s" % \
                             (self.__class__.__name__,str(ex)))
                raise utils.TransportError \
                        ("Serial Message send failed!\n" + str(ex))
            
        
    # No pySerial
    except NameError:
        
        
        ## Create A Horizon Serial Transport
        #  
        #  Constructor for the Horizon message Serial Transport controller.   \n
        #  Does NOT support version auto-detection.                           \n
        #                                                                     \n
        #  Override this method for subclass initialization.                  \n
        #  Overriding methods should call this method with subclass=True. 
        #
        #  @param  port           serial port name
        #  @param  subclass       Is this method being called from the 
        #                         overriding subclass' __init__ method?
        #  @param  store_timeout  The time to store an un-handled message for  
        #                         the method get_waiting in milliseconds,
        #                         0 - store indefinitely
        #  @param  version        Horizon Protocol Version,
        #                         (-1,*) represents the newest version,
        #                         (0,*) auto-detect the version (if supported)
        #  @throws LookupError    If auto-detect version fails
        #  @throws TransportError Upon creation/initialization failure
        #  @throws ValueError     Upon bad arguments
        #
        #  @pydoc
        def __init__(self, port, store_timeout = 10000, subclass = False):
            """Create A Horizon Serial Transport"""
            logger.debug("%s: Instance creation started..." % \
                         self.__class__.__name__)
            
            # Raise Error as pySerial doesn't exist
            logger.error("%s: Cannot create Horizon Serial Transport without"\
                         "pySerial!" % self.__class__.__name__)
            raise utils.TransportError ("pySerial not found!")
    
    

   
################################################################################
# Horizon IP Controllers



## Horizon Socket Controller
#
#  Provides a method to send and receive raw data using a Python socket.
#
#  @req        pyCrypto [http://www.dlitz.net/software/pycrypto/]
#              - for encryption
#  @since      0.4
#
class HorizonTransport_Socket(HorizonTransport):
    """Horizon Transport Controller - Socket Device"""
    
    
    ## Create A Horizon Socket Transport
    #  
    #  Constructor for the Horizon message Socket Transport controller.       \n
    #  Does NOT support version auto-detection.                               \n
    #                                                                         \n
    #  Override this method for subclass initialization.                      \n
    #  Overriding methods should call this method with subclass=True. 
    #
    #  @param  encryption     Encryption class to use, Must be a block cypher
    #                         class within the package Crypto.Cipher or None
    #  @param  host           dest host name or ip address (none for stream)
    #  @param  key            Encryption key to use if encryption is not None,
    #                         Must be a string that matches the encryption 
    #                         class's required key size
    #  @param  name           A human readable name for the socket
    #  @param  port           dest port number (none for stream)
    #  @param  sock           The connected socket to communicate with
    #  @param  subclass       Is this method being called from the 
    #                         overriding subclass' __init__ method?
    #  @param  store_timeout  The time to store an un-handled message for  
    #                         the method get_waiting in milliseconds,
    #                         0 - store indefinitely
    #  @throws LookupError    If auto-detect version fails
    #  @throws TransportError Upon creation/initialization failure
    #  @throws ValueError     Upon bad arguments
    #
    #  @pydoc
    def __init__(self, sock = None, host = 'localhost', port = horizon_port,
                 encryption = None, key = '', name = '',
                 store_timeout = 10000, subclass = False):
        """Create A Horizon Socket Transport"""
        logger.debug("%s: Instance creation started..." % \
                     self.__class__.__name__)
            
        # Initialize Parent
        HorizonTransport.__init__(self, store_timeout = store_timeout, subclass = True)
        
        # Class Variables
        ## Client/Server Address
        self._addr = None
        ## Crypt Controller
        self._crypt = None
        ## Last Received
        self._last = 0
        ## Socket Name
        self._name = name
        ## Random Number Generator
        self._rand = None
        ## Receive Buffer
        self._rbuff = []
        if sys.version_info[0] > 2:
            ## Decryption Buffer
            self._ubuff = bytearray()
        else:
            ## Decryption Buffer
            self._ubuff = ''
        ## Socket
        self._socket = sock
        if host != None and port != None:
            self._addr = (host,port)
        
        # Check for encryption
        if encryption != None:
                
            # notify of encryption use
            logger.info("%s: Attempting to use encryption..." % \
                        (self.__class__.__name__))
            
            # Check for pyCrypto
            try:
                Crypto.Random
                
            # No pyCrypto
            except NameError:
                logger.error("%s: Encryption not supported!" % \
                             self.__class__.__name__)
                raise utils.TransportError ("Encryption not supported!")
                
            # instantiate [en/de]crypter
            try:
                seed = 'clearpath_robots'
                self._crypt = encryption.new(key,
                                              encryption.MODE_CBC,
                                              seed[:encryption.block_size])
            except AttributeError:
                logger.error("%s: Bad encryption!" % \
                             self.__class__.__name__)
                raise utils.TransportError ("Bad encryption!")
                
            # instantiate random number generator
            self._rand = Crypto.Random.new()
            
        # verify socket
        if (not isinstance(sock,socket.socket)):
            logger.error("%s: Bad Socket!" % \
                             self.__class__.__name__)
            raise utils.TransportError ("Bad Socket!")
        
        logger.debug("%s: ...instance creation complete." % \
                     self.__class__.__name__)
        
        
    ## String Representation
    #
    #  Return the transport device name.
    # 
    #  @return String of transport device name.
    #
    #  @pydoc
    def __str__(self):
        """Return the transport device name."""
        
        return self._name
        
    
    ## Socket Transport Device Open
    #
    #  @see HorizonTransport._open(self)
    #
    #  @throws TransportError upon opening failure
    #
    #  @pydoc
    def _open(self):
        """Socket Transport Device Open"""
        
        # Set initial time
        t = datetime.datetime.today()
        timestamp = t.microsecond/1000 + t.second*1000 + \
                    t.minute*60*1000 + t.hour*60*60*1000 + \
                    t.day*24*60*60*1000
        while timestamp > 4294967295: timestamp -= 4294967295
        self._last = timestamp
        
        # Nothing else to do... already open
        
    
    ## Socket Transport Device Close
    #
    #  @see HorizonTransport._close(self)
    #
    #  @throws TransportError upon closing failure
    #
    #  @pydoc
    def _close(self):
        """Socket Transport Device Close"""
        if self.opened == False: return
        
        # Close device
        logger.debug("%s: Socket closing started..." % \
                     self.__class__.__name__)
        self._socket.close()
        logger.debug("%s: ...Socket closing complete." % \
                     self.__class__.__name__)
        
    
    ## Socket Transport Device Get Horizon Message
    #
    #  @see HorizonTransport._get_message(self)
    #
    #  @throws TransportError upon receive failure
    #  @return received Horizon Message or None
    #
    #  @pydoc
    def _get_message(self):
        """Socket Transport Device Get Horizon Message"""
        read = 0
        try:
            
            # read as much as possible
            chars = [b'0']
            addr = None
            logger.debug("%s: Socket read started..." % \
                     self.__class__.__name__)
            while(len(chars) > 0):
                try:
                    chars, addr = self._socket.recvfrom(1)
                except socket.error:
                    logger.debug("%s: ...Socket read complete." % \
                                 self.__class__.__name__)
                    break
                except socket.timeout:
                    logger.debug("%s: ...Socket read complete." % \
                                 self.__class__.__name__)
                    break
                if len(chars) > 0 and (self._addr == None or addr == None or 
                                       addr ==self._addr):
                    
                    # Encryption???
                    if self._crypt != None:
                        self._ubuff += chars
                        if len(self._ubuff) >= self._crypt.block_size:
                            logger.debug("%s: Decryption started..." % \
                                     self.__class__.__name__)
                
                            # perform the decryption
                            chars = self._crypt.decrypt(self._ubuff[:
                                                        self._crypt.block_size])
                            logger.debug("%s: ...decryption complete." % \
                                     self.__class__.__name__)
                        else:
                            return None
                        
                    if sys.version_info[0] > 2:
                        self._rbuff += chars
                    else:
                        self._rbuff += map(ord,chars)
                    read += len(chars)
                else:
                    logger.error("%s: ...Socket has been closed." % \
                                 (self.__class__.__name__))
                    self.close()
                    return None
            logger.debug("%s: ...Socket read complete." % \
                     self.__class__.__name__)
            
        # Read Failed
        except Exception as ex:
            logger.error("%s: ...Socket read failed:\n%s" % \
                     (self.__class__.__name__,str(ex)))
            raise utils.TransportError \
                    ("Socket Message get failed!\n" + str(ex))
        if read > 0 :
            logger.info("%s: Read %d bytes." % (self.__class__.__name__, read))
            
        # Look for message start (SOH XX ~XX)
        disc = []
        while(len(self._rbuff) > 3 and (
                self._rbuff[0] != messages.HorizonMessage.SOH or
                self._rbuff[1] != 0xFF&(~self._rbuff[2]) or
                self._rbuff[1] == 0)):
            disc.append(self._rbuff.pop(0))
        if len(disc) > 0:
            logger.info("%s: Discarded %d bytes:\n%s" % (
                        self.__class__.__name__, len(disc), 
                        ' '.join(map(utils.hex,disc))))
        if len(self._rbuff) < 3:
            return None
            
        # Extract Expected Message Length
        length = self._rbuff[1] + 3 
        
        # Look for next message start
        for i in range(1,len(self._rbuff)-2):
            if self._rbuff[i] == messages.HorizonMessage.SOH and \
                    self._rbuff[1] == 0xFF&(~self._rbuff[2]) and \
                    self._rbuff[1] != 0:
                if i < length:
                    length = i
                break
            
        # Not all read yet
        if len(self._rbuff) < length:
            return None
            
        # Return Message
        raw = self._rbuff[0:length]
        self._rbuff = self._rbuff[length:]
        logger.info("%s: Message of %d bytes found:\n%s" % (
                    self.__class__.__name__, len(raw), 
                    ' '.join(map(utils.hex,raw))))
        t = datetime.datetime.today()
        timestamp = t.microsecond/1000 + t.second*1000 + \
                    t.minute*60*1000 + t.hour*60*60*1000 + \
                    t.day*24*60*60*1000
        while timestamp > 4294967295: timestamp -= 4294967295
        self._last = timestamp
        return messages.HorizonMessage(payload_type = payloads.HorizonPayload,
                                       raw = raw, store_error = True)
        
    
    ## Socket Transport Device Send Horizon Message
    #
    #  @see HorizonTransport._send_message(self,message)
    #
    #  @param  message    The message to send, None if using raw
    #  @param  raw        The byte array or raw string to send, 
    #                     None if using message
    #  @throws TransportError upon send failure
    #
    #  @pydoc
    def send_message(self, message, raw = None):
        """Socket Client Transport Device Send Horizon Message"""
            
        # Message Data
        if message != None:
            raw = message.raw_string()
        elif not isinstance(raw,str) and not isinstance(raw,bytes):
            raw = utils.to_bytes(raw)
            
        # Encryption???
        if self._crypt != None:
            logger.debug("%s: Encryption started..." % \
                         self.__class__.__name__)
                
            # padd the bytes to a multiple of the block size
            raw += self._rand.read(self._crypt.block_size - 
                                         (len(raw)%self._crypt.block_size))
                
            # perform the encryption
            raw = self._crypt.encrypt(raw)
            logger.debug("%s: ...encryption complete." % \
                         self.__class__.__name__)
        
        # Send Message
        try:
            logger.debug("%s: Socket write started..." % \
                         self.__class__.__name__)
            if self._addr == None:
                sent = self._socket.send(raw)
            else:
                sent = self._socket.sendto(raw, self._addr)
            if sent == len(raw):
                logger.debug("%s: ...Socket write complete." % \
                             self.__class__.__name__)
            if isinstance(raw,str):
                logger.info("%s: Wrote %d bytes:\n%s" % \
                        (self.__class__.__name__, sent,
                        ' '.join(map(utils.hex,map(ord,raw[:sent])))))
            else:
                logger.info("%s: Wrote %d bytes:\n%s" % \
                        (self.__class__.__name__, sent,
                        ' '.join(map(utils.hex,raw[:sent]))))
            if sent < len(raw):
                logger.error("%s: Socket write incomplete!" % \
                                     self.__class__.__name__)
                raise utils.TransportError ("Write Incomplete!")
                
        # Send Failed
        except socket.error as ex:
            if ex.args[0] != 104:
                logger.error("%s: ...Socket has been closed." % \
                                 (self.__class__.__name__))
                self.close()
            logger.error("%s: ...Socket write failed:\n%s" % \
                         (self.__class__.__name__,str(ex)))
            raise utils.TransportError \
                    ("Socket Message send failed!\n" + str(ex))
    
    
    ## Last Time Message Was Received
    #
    #  Return the last time a message was received.
    #
    #  @return timestamp of last msg.
    #
    #  @pydoc
    def get_last_time(self):
        """Return the Time of the Last Message."""
        
        return self._last
    
    
    ## Transport Name
    #
    #  Return the transport name.
    #
    #  @return String of transport name.
    #
    #  @pydoc
    def get_name(self):
        """Return the transport name."""
        
        return 'Socket/IP'
    get_name = staticmethod(get_name)
    
    
    ## Server Address
    #
    #  Return The Server Address.
    #
    #  @return tuple(hostname,port)
    #
    #  @pydoc
    def get_address(self):
        """Return The Server Address."""
        
        if self._addr == None:
            return self._socket.getpeername()
        return self._addr
    
    
    # Class Properties
    ## The Server Address
    address = property(fget=get_address, doc="The Server Address")
    ## The Transport Name
    name = property(fget=__str__, doc="The Transport Name")
    
    

   
################################################################################
# Horizon TCP Controllers



## @defgroup tcp TCP
#  @ingroup hardware
#
#  The Transmission Control Protocol (TCP/IP), like Serial, provides guaranteed
#  message arrival and order of arrival.                                    \n\n 
#
#  @defgroup encryption Encryption
#  @ingroup doc
#
#  The TCP Horizon transport is the only transport scoped to support 
#  encryption. Encryption is performed on the entire message (before packet
#  embedding, after padding with random data to a multiple of the block size) 
#  with a block cipher using the standard Cipher Block Chaining (CBC) 
#  method of [en/de]cryption. The CBC is initialized with the seed 
#  "clearpath_robots", spliced to the apropriate block lengh, upon connection
#  between a client and server. If a single byte is missed, out of order, or not
#  sent from the client/server but processed through decryption then all 
#  subsequent data will be invalid. The connection must then be re-established. 
#  Currently, the supported block ciphers are:                                \n
#  AES, RC2, Blowfish, CAST, DES, and DES3.                                   \n
#
#  Availability and support for encryption ciphers are based on location and
#  cipher implementation. If the cipher implementation is Canadian made and 
#  public domain (does not require a license), then Canadian export laws allows 
#  it to be exported to countries that are not on Canada's Area Control Lists 
#  (ACLs). Otherwise, only Canadian made ciphers of limited strength are 
#  allowed to be exported to countries that are not on Canada's ACLs. The 
#  definition of this strength tends to vary over time as more advanced 
#  algorithms are developed and older algorithms are spread from country to 
#  country. If the implementation contains code of foreign origin then special
#  case export laws may apply. If export to a country is desired but not 
#  allowed by the above mentioned cases, then a permit must be obtained from 
#  the Government of Canada before export. Canada does not restrict the import 
#  or use of cryptography thus support for all ciphers is available in Canada.\n
#
#  Canadian ACLs may be found at:                                             \n
#  http://www.international.gc.ca/controls-controles/report-rapports/list_liste/index.aspx?lang=eng
#



## Horizon TCP Client Controller
#
#  Provides a method to send and receive messages over Transmission Control 
#  Protocol as a client (to talk to a platform).                              \n
#  Guarantees arrival and order of arrival.                                   \n
#  Can be used for high Bandwidth.                                            \n
#                                                                             \n
#  Encryption is performed with the Canadian made, Public Domain library, Python
#  Cryptography Toolkit. Due to export laws, this library is currently only
#  exportable the United States without a permit. The developers are currently 
#  in the process of allowing it to be exported without a permit to all 
#  countries except those on Canada's Area Control Lists. Note that export 
#  requires the library to be exported as-is and within the public domain.
#  Canada does not currently restrict the import or use of cryptography. 
#
#  @req        pyCrypto [http://www.dlitz.net/software/pycrypto/]
#              - for encryption
#  @since 0.4
#
#  @section Hardware
#  @copydoc tcp
#
class HorizonTransport_TCP_Client(HorizonTransport):
    """Horizon Transport Controller - TCP Client Device"""
    
    
    ## Supported Versions
    versions = versioning.HORIZON_VERSIONS.difference(set([tuple([0,1]),
                                            tuple([0,2]),
                                            tuple([0,3])]))
    """Supported Horizon Versions"""
    
    
    ## Create A Horizon TCP Client Transport
    #  
    #  Constructor for the Horizon message TCP Client Transport controller.   \n
    #  Does NOT support version auto-detection.                               \n
    #                                                                         \n
    #  Override this method for subclass initialization.                      \n
    #  Overriding methods should call this method with subclass=True. 
    #
    #  @param  encryption     Encryption class to use, Must be a block cypher
    #                         class within the package Crypto.Cipher or None
    #  @param  host           dest host name or ip address
    #  @param  ipv6           Is the network IPv6 (True) or IPv4 (False)
    #  @param  key            Encryption key to use if encryption is not None,
    #                         Must be a string that matches the encryption 
    #                         class's required key size
    #  @param  port           dest port number
    #  @param  subclass       Is this method being called from the 
    #                         overriding subclass' __init__ method?
    #  @param  store_timeout  The time to store an un-handled message for  
    #                         the method get_waiting in milliseconds,
    #                         0 - store indefinitely
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,*) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws TransportError Upon creation/initialization failure
    #  @throws ValueError     Upon bad arguments
    #
    #  @pydoc
    def __init__(self, host = 'localhost', port = horizon_port, 
                 encryption = None, key = '', ipv6 = False,
                 store_timeout = 10000, version = tuple([-1,0]), 
                 subclass = False):
        """Create A Horizon TCP Client Transport"""
        logger.debug("%s: Instance creation started..." % \
                     self.__class__.__name__)
            
        # Initialize Parent
        HorizonTransport.__init__(self, store_timeout = store_timeout, 
                                  version = version, subclass = True)
        
        # Class Variables
        ## TCP Server
        self._addr = (host,port)
        ## Encryption?
        self._encryption = None
        ## Encryption Key
        self._key = None
        ## Receive Buffer
        self._rbuff = []
        ## TCP Socket
        self._tcp_socket = None
        ## Socket Transport
        self._socket = None
        
        # Check for encryption
        if encryption != None:
                
            # notify of encryption use
            logger.info("%s: Attempting to use encryption..." % \
                        (self.__class__.__name__))
            
            # Check for pyCrypto
            try:
                Crypto.Random
                
            # No pyCrypto
            except NameError:
                logger.error("%s: Encryption not supported!" % \
                             self.__class__.__name__)
                raise utils.TransportError ("Encryption not supported!")
                
            # instantiate [en/de]crypter
            try:
                seed = 'clearpath_robots'
                encryption.new(key,encryption.MODE_CBC,
                               seed[:encryption.block_size])
                self._encryption = encryption
                self._key = key
            except AttributeError:
                logger.error("%s: Bad encryption!" % \
                             self.__class__.__name__)
                raise utils.TransportError ("Bad encryption!")
            
        # Check IPv6
        if ipv6 == True and not socket.has_ipv6:
            logger.error("%s: IPv6 not supported!" % self.__class__.__name__)
            raise utils.TransportError ("IPv6 not supported!")
            
        # Initialization
        logger.info("%s: Using tcp host %s at port %d." % \
                    (self.__class__.__name__, self._addr[0], self._addr[1]))
        try:
            logger.debug("%s: TCP port initialization started..." % \
                     self.__class__.__name__)
            socket.setdefaulttimeout(0.0)
            if(ipv6 == True):
                self._tcp_socket = socket.socket(socket.AF_INET6,
                                                 socket.SOCK_STREAM)
            else:
                self._tcp_socket = socket.socket(socket.AF_INET,
                                                 socket.SOCK_STREAM)
            self._tcp_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            logger.debug("%s: ...TCP port initialization complete." % \
                     self.__class__.__name__)
        
        # Init failed
        except Exception as ex:
            logger.error("%s: ...TCP port initialization failed:\n%s" % \
                         (self.__class__.__name__,str(ex)))
            raise utils.TransportError \
                    ("TCP Port Initialization failed!\n" + str(ex))
        logger.debug("%s: ...instance creation complete." % \
                     self.__class__.__name__)
        
        
    ## String Representation
    #
    #  Return the transport device name.
    # 
    #  @return String of transport device name.
    #
    #  @pydoc
    def __str__(self):
        """Return the transport device name."""
        
        return self._addr[0] + ":" + str(self._addr[1])
        
    
    ## TCP Client Transport Device Open
    #
    #  @see HorizonTransport._open(self)
    #
    #  @throws TransportError upon opening failure
    #
    #  @pydoc
    def _open(self):
        """TCP Client Transport Device Open"""
        
        # Open Device
        try:
            logger.debug("%s: TCP port opening started..." % \
                         self.__class__.__name__)
            errno = 115
            while errno == 115:
                try:
                    self._tcp_socket.connect(self._addr)
                    errno = 0
                except socket.error as fx:
                    if fx.args[0] != 115:
                        raise fx
            self._socket = HorizonTransport_Socket(sock = self._tcp_socket,
                                                   host = self._addr[0],
                                                   port = self._addr[1],
                                                   name = "%s:%d" % self._addr,
                                                   encryption =self._encryption,
                                                   key = self._key,
                                            store_timeout = self.store_timeout,
                                                   version = self.version)
            self._socket.opened = True
            logger.debug("%s: ...TCP port opening complete." % \
                         self.__class__.__name__)
        
        # Open failed
        except Exception as ex:
            logger.error("%s: ...TCP port opening failed:\n%s" % \
                         (self.__class__.__name__, str(ex)))
            raise utils.TransportError \
                    ("TCP Port open failed!\n" + str(ex))
        
    
    ## TCP Client Transport Device Close
    #
    #  @see HorizonTransport._close(self)
    #
    #  @throws TransportError upon closing failure
    #
    #  @pydoc
    def _close(self):
        """TCP Client Transport Device Close"""
        
        # Close device
        logger.debug("%s: TCP port closing started..." % \
                     self.__class__.__name__)
        self._tcp_socket.close()
        self._socket = None
        logger.debug("%s: ...TCP port closing complete." % \
                     self.__class__.__name__)
        
    
    ## TCP Client Transport Device Get Horizon Message
    #
    #  @see HorizonTransport._get_message(self)
    #
    #  @throws TransportError upon receive failure
    #  @return received Horizon Message or None
    #
    #  @pydoc
    def _get_message(self):
        """TCP Client Transport Device Get Horizon Message"""
        if not self.opened: return None
        if not self._socket.is_open(): 
            self.close()
            return None
        return self._socket._get_message()
        
    
    ## TCP Transport Device Send Horizon Message
    #
    #  @see HorizonTransport._send_message(self,message)
    #
    #  @param  message    The message to send, None if using raw
    #  @param  raw        The byte array or raw string to send, 
    #                     None if using message
    #  @throws TransportError upon send failure
    #
    #  @pydoc
    def send_message(self, message, raw = None):
        """TCP Client Transport Device Send Horizon Message"""
        if not self._socket.is_open(): 
            self.close()
        if not self.opened:
            logger.error("%s: Cannot send while closed!" % \
                         self.__class__.__name__)
            raise utils.TransportError ("Cannot send while closed!")
        
        self._socket.send_message(message, raw)
    
    
    ## Transport Name
    #
    #  Return the transport name.
    #
    #  @return String of transport name.
    #
    #  @pydoc
    def get_name(self):
        """Return the transport name."""
        
        return 'TCP/IP Client'
    get_name = staticmethod(get_name)
    
    
    ## Server Address
    #
    #  Return The Server Address.
    #
    #  @return tuple(hostname,port)
    #
    #  @pydoc
    def get_address(self):
        """Return The Server Address."""
        
        return self._addr
    
    
    # Class Properties
    ## The Server Address
    address = property(fget=get_address, doc="The Server Address")
    ## The Transport Name
    name = property(fget=__str__, doc="The Transport Name")
    

## Horizon TCP Server Controller
#
#  Provides a method to send and receive messages over Transmission Control 
#  Protocol as a server supporting multiple connections (to impersonate a 
#  platform).                                                                 \n
#  Guarantees arrival and order of arrival.                                   \n
#  Messages from different clients may be intersperced.                       \n
#  Can be used for high Bandwidth.                                            \n
#                                                                             \n
#  The TCP Horizon transport is the only transport scoped to support encryption.
#  Encryption is performed with the Canadian made, Public Domain library, Python
#  Cryptography Toolkit. Due to export laws, this library is currently only
#  exportable the United States without a permit. The developers are currently 
#  in the process of allowing it to be exported without a permit to all 
#  countries except those on Canada's Area Control Lists. Note that export 
#  requires the library to be exported as-is and within the public domain.
#  Canada does not currently restrict the import or use of cryptography. 
#
#  @req        pyCrypto [http://www.dlitz.net/software/pycrypto/]
#              - for encryption
#  @since 0.4
#
#  @section Hardware
#  @copydoc tcp
#
class HorizonTransport_TCP_Server(HorizonTransport):
    """Horizon Transport Controller - TCP Server Device"""
    
    
    ## Supported Versions
    versions = versioning.HORIZON_VERSIONS.difference(set([tuple([0,1]),
                                            tuple([0,2]),
                                            tuple([0,3])]))
    """Supported Horizon Versions"""
    
    
    ## Create A Horizon TCP Server Transport
    #  
    #  Constructor for the Horizon message TCP Server Transport controller.   \n
    #  Does NOT support version auto-detection.                               \n
    #                                                                         \n
    #  Override this method for subclass initialization.                      \n
    #  Overriding methods should call this method with subclass=True. 
    #
    #  @param  encryption     Encryption class to use, Must be a block cypher
    #                         class within the package Crypto.Cipher or None
    #  @param  ipv6           Is the network IPv6 (True) or IPv4 (False)
    #  @param  key            Encryption key to use if encryption is not None,
    #                         Must be a string that matches the encryption 
    #                         class's required key size
    #  @param  max            The maximum number of connections to support
    #  @param  port           port number to listen on
    #  @param  rec_timeout    The time to wait after the last received message
    #                         before closing the TCP connection in milliseconds,
    #                         0 - wait indefinitely
    #  @param  send_all       Send all messages to all clients connected
    #                         regardless of original sender/requestor (2), 
    #                         Send data messages to all clients connected
    #                         regardless of original requestor (1),
    #                         Perform full routing (0)
    #  @param  subclass       Is this method being called from the 
    #                         overriding subclass' __init__ method?
    #  @param  store_timeout  The time to store an un-handled message for  
    #                         the method get_waiting in milliseconds,
    #                         0 - store indefinitely
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,*) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws TransportError Upon creation/initialization failure
    #  @throws ValueError     Upon bad arguments
    #
    #  @pydoc
    def __init__(self, port = horizon_port, encryption = None, key = '', 
                 rec_timeout = 300000, ipv6 = False,
                 max = 5, send_all = 0, store_timeout = 10000, 
                 version = tuple([-1,0]), subclass = False):
        """Create A Horizon TCP Server Transport"""
        logger.debug("%s: Instance creation started..." % \
                     self.__class__.__name__)
            
        # Initialize Parent
        HorizonTransport.__init__(self, store_timeout = store_timeout, 
                                  version = version, subclass = True)
        
        # Class Variables
        ## TCP Client Sockets
        self._clients = []
        ## Encryption?
        self._encryption = None
        ## Encryption Key
        self._key = None
        ## Max connections
        self._max = max
        ## Platform Routeable
        self._platform = None
        ## TCP Server
        self._port = port
        ## Receive Buffer
        self._rbuff = []
        ## Receive Timeout
        self._rec_timeout = rec_timeout
        ## Horizon Message Router
        self._router = None
        ## Send All?
        self._send_all = send_all
        ## TCP Socket
        self._tcp_socket = None
        
        # Check for encryption
        if encryption != None:
                
            # notify of encryption use
            logger.info("%s: Attempting to use encryption..." % \
                        (self.__class__.__name__))
            
            # Check for pyCrypto
            try:
                Crypto.Random
                
            # No pyCrypto
            except NameError:
                logger.error("%s: Encryption not supported!" % \
                             self.__class__.__name__)
                raise utils.TransportError ("Encryption not supported!")
                
            # instantiate [en/de]crypter
            try:
                seed = 'clearpath_robots'
                encryption.new(key,encryption.MODE_CBC,
                               seed[:encryption.block_size])
                self._encryption = encryption
                self._key = key
            except AttributeError:
                logger.error("%s: Bad encryption!" % \
                             self.__class__.__name__)
                raise utils.TransportError ("Bad encryption!")
            
        # Check IPv6
        if ipv6 == True and not socket.has_ipv6:
            logger.error("%s: IPv6 not supported!" % self.__class__.__name__)
            raise utils.TransportError ("IPv6 not supported!")
            
        # Initialization
        logger.info("%s: Using tcp localhost at port %d." % \
                    (self.__class__.__name__, self._port))
        try:
            logger.debug("%s: TCP port initialization started..." % \
                     self.__class__.__name__)
            socket.setdefaulttimeout(0.0)
            if(ipv6 == True):
                self._tcp_socket = socket.socket(socket.AF_INET6,
                                                 socket.SOCK_STREAM)
            else:
                self._tcp_socket = socket.socket(socket.AF_INET,
                                                 socket.SOCK_STREAM)
            self._tcp_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            logger.debug("%s: ...TCP port initialization complete." % \
                     self.__class__.__name__)
        
        # Init failed
        except Exception as ex:
            logger.error("%s: ...TCP port initialization failed:\n%s" % \
                         (self.__class__.__name__,str(ex)))
            raise utils.TransportError \
                    ("TCP Port Initialization failed!\n" + str(ex))
        logger.debug("%s: ...instance creation complete." % \
                     self.__class__.__name__)
    
    
    ## String Representation
    #
    #  Return the transport name.
    # 
    #  @return String of transport name.
    #
    #  @pydoc
    def __str__(self):
        """Return the transport name."""
        
        return "localhost:%d" % (self._port)
        
    
    ## TCP Server Transport Device Open
    #
    #  @see HorizonTransport._open(self)
    #
    #  @throws TransportError upon opening failure
    #
    #  @pydoc
    def _open(self):
        """TCP Server Transport Device Open"""
        
        # Open Device
        try:
            logger.debug("%s: TCP port opening started..." % \
                         self.__class__.__name__)
            self._tcp_socket.bind(tuple(['',self._port]))
            self._tcp_socket.listen(self._max)
            logger.debug("%s: ...TCP port opening complete." % \
                         self.__class__.__name__)
            
            # Instantiate router
            self._platform = router.HorizonRouteable()
            self._platform._version = self._version
            self._platform.message_routed = self.message_received
            def tmp():
                return self.__str__()
            self._platform.__str__ = tmp
            self._router = router.HorizonRouter(platform = self._platform, 
                                                clients = [], 
                                                send_all = self._send_all)
        
        # Open failed
        except Exception as ex:
            logger.error("%s: ...TCP port opening failed:\n%s" % \
                         (self.__class__.__name__, str(ex)))
            raise utils.TransportError \
                    ("TCP Port open failed!\n" + str(ex))
        
    
    ## TCP Single Server Transport Device Close
    #
    #  @see HorizonTransport._close(self)
    #
    #  @throws TransportError upon closing failure
    #
    #  @pydoc
    def _close(self):
        """TCP Single Server Transport Device Close"""
        
        # Close device
        logger.debug("%s: TCP port closing started..." % \
                     self.__class__.__name__)
        self._router = None
        self._platform = None
        self._tcp_socket.close()
        logger.debug("%s: ...TCP port closing complete." % \
                     self.__class__.__name__)
        
    
    ## TCP Server Transport Device Get Horizon Message
    #
    #  @see HorizonTransport._get_message(self)
    #
    #  @throws TransportError upon receive failure
    #  @return received Horizon Message or None
    #
    #  @pydoc
    def _get_message(self):
        """TCP Server Transport Device Get Horizon Message"""
        if not self.opened: return None
        
        # new connection?
        if len(self._clients) < self._max:
            self._tcp_socket.listen(self._max - len(self._clients))
            try:
                sock, addr = self._tcp_socket.accept()
                self._clients.append(HorizonTransport_Socket(
                                sock = sock,
                                host = None,
                                name = "%s:%d" % addr,
                                store_timeout = 1,
                                version = self._version,
                                encryption = self._encryption,
                                key = self._key))
                self._clients[-1].open()
                self._router.add_client(self._clients[-1])
                logger.info("%s: New connection to %s:%d." % \
                         (self.__class__.__name__,self._clients[-1].address[0],
                         self._clients[-1].address[1]))
            except socket.error:
                pass
            except socket.timeout:
                pass
            
        # update timestamp
        t = datetime.datetime.today()
        timestamp = t.microsecond/1000 + t.second*1000 + \
                    t.minute*60*1000 + t.hour*60*60*1000 + \
                    t.day*24*60*60*1000
        while timestamp > 4294967295: timestamp -= 4294967295
            
        # Old connection closed?
        for i in range(len(self._clients),0,-1):
            last = self._clients[i-1].get_last_time()
            if not self._clients[i-1].is_open():
                logger.warning("%s: Connection to %s lost!" % \
                         (self.__class__.__name__,self._clients[i-1].name))
                self._router.remove_client(self._clients[i-1])
                self._clients.remove(self._clients[i-1])
        
            # Connection Timeout?
            elif ((timestamp - last >= self._rec_timeout) or\
                    (timestamp < last and 4294967295 - \
                    last + timestamp >= self._rec_timeout)):
                logger.warning("%s: Connection to %s timed-out!" % \
                         (self.__class__.__name__,self._clients[i-1].name))
                self._router.remove_client(self._clients[i-1])
                self._clients[i-1].close()
                self._clients.remove(self._clients[i-1])
        
        return None
        
    
    ## TCP Server Transport Device Send Horizon Message
    #
    #  @see HorizonTransport._send_message(self,message)
    #
    #  @param  message    The message to send
    #  @param  raw        None - Not supported
    #  @throws TransportError upon send failure
    #
    #  @pydoc
    def send_message(self, message, raw = None):
        """TCP Server Transport Device Send Horizon Message"""
        if not self.opened:
            logger.error("%s: Cannot send while closed!" % \
                         self.__class__.__name__)
            raise utils.TransportError ("Cannot send while closed!")
        if raw != None:
            logger.error("%s: TCP Server doesn't support raw send!" % \
                         self.__class__.__name__)
            raise utils.TransportError ("TCP Server doesn't support raw send!")
        
        # Send Message
        try:
            self._platform.route_message(message = message)
        except utils.TransportError as ex:
            for i in range(len(self._clients),0,-1):
                if not self._clients[i-1].is_open():
                    return
            raise ex
    
    ## Has Client
    #
    #  Is a client connected?
    #
    #  @return Is a client connected?
    #
    #  @pydoc
    def has_client(self):
        """Is a client connected?"""
        
        return len(self._clients) > 0
    
    
    ## Get Clients
    #
    #  Return a list of client addresses.
    #
    #  @return List of tuple([host,port])
    #
    #  @pydoc
    def get_clients(self):
        """Return a list of client addresses."""
        clis = []
        for c in self._clients:
            clis.append(c.get_address())
        return clis
    
    
    ## Transport Name
    #
    #  Return the transport name.
    #
    #  @return String of transport name.
    #
    #  @pydoc
    def get_name(self):
        """Return the transport name."""
        
        return 'TCP/IP Server'
    get_name = staticmethod(get_name)
    
    
    ## Server Address
    #
    #  Return The Server Address.
    #
    #  @return tuple(hostname,port)
    #
    #  @pydoc
    def get_address(self):
        """Return The Server Address."""
        
        return tuple('localhost',self._port)
    
    
    # Class Properties
    ## The Server Address
    address = property(fget=get_address, doc="The Server Address")
    ## The Transport Name
    name = property(fget=__str__, doc="The Transport Name")
    

   
################################################################################
# Horizon UDP Controllers



## @defgroup udp UDP
#  @ingroup hardware
#
#  The User Datagram Protocol (UDP) allows for very high bandwidth usage. 
#  However, UDP cannot gaurentee message order of arrival or even message
#  arrival. Since the Horizon protocol does not use acknowledgements for 
#  data messages it sends to the client, Horizon UDP does not gaurantee the 
#  arrival of request data. Use TCP if request data is critical to operation. 
#



## Horizon UDP Client Controller
#
#  Provides a method to send and receive messages over User Datagram Protocol
#  as a client (to talk to a platform).                                       \n
#  Does not gaurentee order of arrival or arrival.                            \n
#  Can be used for very high Bandwidth.
#
#  @since 0.4
#
#  @warning UDP does not gaurentee message delivery and since the Horizon device
#  does not use for acknowledgements for data parsels it sends to the client,
#  Horizon UDP does not gaurentee the arrival of request data. Use TCP if
#  request data is critical to operation.  
#
#  @section Hardware
#  @copydoc udp
#
class HorizonTransport_UDP_Client(HorizonTransport):
    """Horizon Transport Controller - UDP Client Device"""
    
    
    ## Supported Versions
    versions = versioning.HORIZON_VERSIONS.difference(set([tuple([0,1]),
                                            tuple([0,2]),
                                            tuple([0,3])]))
    """Supported Horizon Versions"""
    
    
    ## Create A Horizon UDP Client Transport
    #  
    #  Constructor for the Horizon message UDP Client Transport controller.   \n
    #  Does NOT support version auto-detection.                               \n
    #                                                                         \n
    #  Override this method for subclass initialization.                      \n
    #  Overriding methods should call this method with subclass=True. 
    #
    #  @param  host           dest host name or ip 
    #  @param  ipv6           Is the network IPv6 (True) or IPv4 (False)
    #  @param  local_port     incoming port number
    #  @param  port           dest port number
    #  @param  subclass       Is this method being called from the 
    #                         overriding subclass' __init__ method?
    #  @param  store_timeout  The time to store an un-handled message for  
    #                         the method get_waiting in milliseconds,
    #                         0 - store indefinitely
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,*) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws TransportError Upon creation/initialization failure
    #  @throws ValueError     Upon bad arguments
    #
    #  @pydoc
    def __init__(self, host = 'localhost', port = horizon_port, 
                 local_port = horizon_port, ipv6 = False,
                 store_timeout = 10000, version = tuple([-1,0]), 
                 subclass = False):
        """Create A Horizon UDP Client Transport"""
        logger.debug("%s: Instance creation started..." % \
                     self.__class__.__name__)
            
        # Initialize Parent
        HorizonTransport.__init__(self, store_timeout = store_timeout, 
                                  version = version, subclass = True)
        
        # Class Variables
        ## UDP Remote Address
        self._addr = (host,port)
        ## UDP Bind Port
        self._port = local_port
        ## Receive Buffer
        self._rbuff = []
        ## UDP Socket
        self._udp_socket = None
        ## Socket Transport
        self._socket = None
            
        # Check IPv6
        if ipv6 == True and not socket.has_ipv6:
            logger.error("%s: IPv6 not supported!" % self.__class__.__name__)
            raise utils.TransportError ("IPv6 not supported!")
        
        # Initialization
        logger.info("%s: Using udp host %s at port %d and local port %d." % \
                    (self.__class__.__name__, self._addr[0], self._addr[1], 
                     self._port))
        try:
            logger.debug("%s: UDP port initialization started..." % \
                     self.__class__.__name__)
            socket.setdefaulttimeout(0.0)
            if ipv6 == True:
                self._udp_socket = socket.socket(socket.AF_INET6,
                                                 socket.SOCK_DGRAM)
            else:
                self._udp_socket = socket.socket(socket.AF_INET,
                                                 socket.SOCK_DGRAM)
            logger.debug("%s: ...UDP port initialization complete." % \
                     self.__class__.__name__)
        
        # Init failed
        except Exception as ex:
            logger.error("%s: ...UDP port initialization failed:\n%s" % \
                         (self.__class__.__name__,str(ex)))
            raise utils.TransportError \
                    ("UDP Port Initialization failed!\n" + str(ex))
        logger.debug("%s: ...instance creation complete." % \
                     self.__class__.__name__)
    
    
    ## String Representation
    #
    #  Return the transport name.
    # 
    #  @return String of transport name.
    #
    #  @pydoc
    def __str__(self):
        """Return the transport name."""
        
        return "localhost:"+self._port+" --> "+self._addr[0]+":"+self._addr[1]
        
    
    ## UDP Client Transport Device Open
    #
    #  @see HorizonTransport._open(self)
    #
    #  @throws TransportError upon opening failure
    #
    #  @pydoc
    def _open(self):
        """UDP Client Transport Device Open"""
        
        # Open Device
        try:
            logger.debug("%s: UDP port opening started..." % \
                         self.__class__.__name__)
            self._udp_socket.bind(('',self._port))
            self._socket = HorizonTransport_Socket(sock = self._udp_socket,
                                                   host = self._addr[0],
                                                   port = self._addr[1],
                                                   name = "%s:%d" % self._addr,
                                            store_timeout = self.store_timeout,
                                                   version = self.version)
            self._socket.opened = True
            logger.debug("%s: ...UDP port opening complete." % \
                         self.__class__.__name__)
        
        # Open failed
        except Exception as ex:
            logger.error("%s: ...UDP port opening failed:\n%s" % \
                         (self.__class__.__name__, str(ex)))
            raise utils.TransportError \
                    ("UDP Port open failed!\n" + str(ex))
    
    
    ## UDP Client Transport Device Close
    #
    #  @see HorizonTransport._close(self)
    #
    #  @throws TransportError upon closing failure
    #
    #  @pydoc
    def _close(self):
        """UDP Client Transport Device Close"""
        
        # Close device
        logger.debug("%s: UDP port closing started..." % \
                     self.__class__.__name__)
        self._udp_socket.close()
        self._socket = None
        logger.debug("%s: ...UDP port closing complete." % \
                     self.__class__.__name__)
        
    
    ## UDP Client Transport Device Get Horizon Message
    #
    #  @see HorizonTransport._get_message(self)
    #
    #  @throws TransportError upon receive failure
    #  @return received Horizon Message or None
    #
    #  @pydoc
    def _get_message(self):
        """UDP Client Transport Device Get Horizon Message"""
        if not self.opened: return None
        if not self._socket.is_open(): 
            self.close()
            return None
        return self._socket._get_message()
        
    
    ## UDP Client Transport Device Send Horizon Message
    #
    #  @see HorizonTransport.send_message(self,message)
    #
    #  @param  message    The message to send, None if using raw
    #  @param  raw        The byte array or raw string to send, 
    #                     None if using message
    #  @throws TransportError upon send failure
    #
    #  @pydoc
    def send_message(self, message, raw = None):
        """UDP Client Transport Device Send Horizon Message"""
        if not self._socket.is_open(): 
            self.close()
        if not self.opened:
            logger.error("%s: Cannot send while closed!" % \
                         self.__class__.__name__)
            raise utils.TransportError ("Cannot send while closed!")
        
        self._socket.send_message(message, raw)
    
    
    ## Transport Name
    #
    #  Return the transport name.
    #
    #  @return String of transport name.
    #
    #  @pydoc
    def get_name(self):
        """Return the transport name."""
        
        return 'UDP/IP Client'
    get_name = staticmethod(get_name)
    
    
    ## Server Address
    #
    #  Return The Server Address.
    #
    #  @return tuple(hostname,port)
    #
    #  @pydoc
    def get_address(self):
        """Return The Server Address."""
        
        return self._addr
    
    
    ## Local Port
    #
    #  Return The Local Port.
    #
    #  @return port
    #
    #  @pydoc
    def get_port(self):
        """Return The Local Port."""
        
        return self._port
    
    
    # Class Properties
    ## The Server Address
    address = property(fget=get_address, doc="The Server Address")
    ## The Transport Name
    name = property(fget=__str__, doc="The Transport Name")
    ## The Local Port
    port = property(fget=get_port, doc="The Local Port")
    
    

## Horizon UDP Server Controller
#
#  Provides a method to send and receive messages over User Datagram Protocol
#  as a server supporting multiple connections (to impersonate an advanced 
#  platform).                                                                 \n
#  Does not gaurentee order of arrival or arrival.                            \n
#  Can be used for very high Bandwidth.
#
#  @since 0.4
#
#  @warning UDP does not gaurentee message delivery and since the Horizon device
#  does not use for acknowledgements for data parsels it sends to the client,
#  Horizon UDP does not gaurentee the arrival of request data. Use TCP if
#  request data is critical to operation.  
#
#  @section Hardware
#  @copydoc udp
#
class HorizonTransport_UDP_Server(HorizonTransport):
    """Horizon Transport Controller - UDP Server Device"""
    
    
    ## Supported Versions
    versions = versioning.HORIZON_VERSIONS.difference(set([tuple([0,1]),
                                            tuple([0,2]),
                                            tuple([0,3])]))
    """Supported Horizon Versions"""
    
    
    ## Create A Horizon UDP Server Transport
    #  
    #  Constructor for the Horizon message UDP Server Transport controller.   \n
    #  Does NOT support version auto-detection.                               \n
    #                                                                         \n
    #  Override this method for subclass initialization.                      \n
    #  Overriding methods should call this method with subclass=True. 
    #
    #  @param  ipv6           Is the network IPv6 (True) or IPv4 (False)
    #  @param  max            The maximum number of connections to support
    #  @param  local_port     port number to listen on
    #  @param  rec_timeout    The time to wait after the last received message
    #                         before closing the TCP connection in milliseconds,
    #                         0 - wait indefinitely
    #  @param  send_all       Send all messages to all clients connected
    #                         regardless of original sender/requestor (2), 
    #                         Send data messages to all clients connected
    #                         regardless of original requestor (1),
    #                         Perform full routing (0)
    #  @param  subclass       Is this method being called from the 
    #                         overriding subclass' __init__ method?
    #  @param  store_timeout  The time to store an un-handled message for  
    #                         the method get_waiting in milliseconds,
    #                         0 - store indefinitely
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,*) auto-detect the version (if supported)
    #  @throws LookupError    If auto-detect version fails
    #  @throws TransportError Upon creation/initialization failure
    #  @throws ValueError     Upon bad arguments
    #
    #  @pydoc
    def __init__(self, local_port = horizon_port, rec_timeout = 300000, max = 5, 
                 send_all = False, store_timeout = 10000, ipv6 = False,
                 version = tuple([-1,0]), subclass = False):
        """Create A Horizon UDP Server Transport"""
        logger.debug("%s: Instance creation started..." % \
                     self.__class__.__name__)
            
        # Initialize Parent
        HorizonTransport.__init__(self, store_timeout = store_timeout, 
                                  version = version, subclass = True)
        
        # Class Variables
        ## UDP Client Sockets
        self._clients = []
        ## Max connections
        self._max = max
        ## Platform Routeable
        self._platform = None
        ## UDP Server
        self._port = local_port
        ## Receive Buffers
        self._rbuff = {}        # Format: { addr:[] }
        ## Receive Timeout
        self._rec_timeout = rec_timeout
        ## Horizon Message Router
        self._router = None
        ## Send All?
        self._send_all = send_all
        ## UDP Socket
        self._udp_socket = None
            
        # Check IPv6
        if ipv6 == True and not socket.has_ipv6:
            logger.error("%s: IPv6 not supported!" % self.__class__.__name__)
            raise utils.TransportError ("IPv6 not supported!")
            
        # Initialization
        logger.info("%s: Using udp localhost at port %d." % \
                    (self.__class__.__name__, self._port))
        try:
            logger.debug("%s: UDP port initialization started..." % \
                     self.__class__.__name__)
            socket.setdefaulttimeout(0.0)
            if(ipv6 == True):
                self._udp_socket = socket.socket(socket.AF_INET6,
                                                 socket.SOCK_DGRAM)
            else:
                self._udp_socket = socket.socket(socket.AF_INET,
                                                 socket.SOCK_DGRAM)
            logger.debug("%s: ...UDP port initialization complete." % \
                     self.__class__.__name__)
        
        # Init failed
        except Exception as ex:
            logger.error("%s: ...UDP port initialization failed:\n%s" % \
                         (self.__class__.__name__,str(ex)))
            raise utils.TransportError \
                    ("UDP Port Initialization failed!\n" + str(ex))
        logger.debug("%s: ...instance creation complete." % \
                     self.__class__.__name__)
    
    
    ## String Representation
    #
    #  Return the transport name.
    # 
    #  @return String of transport name.
    #
    #  @pydoc
    def __str__(self):
        """Return the transport name."""
        
        return "localhost:%d" % (self._port)
        
    
    ## UDP Server Transport Device Open
    #
    #  @see HorizonTransport._open(self)
    #
    #  @throws TransportError upon opening failure
    #
    #  @pydoc
    def _open(self):
        """UDP Server Transport Device Open"""
        
        # Open Device
        try:
            logger.debug("%s: UDP port opening started..." % \
                         self.__class__.__name__)
            self._udp_socket.bind(tuple(['',self._port]))
            logger.debug("%s: ...UDP port opening complete." % \
                         self.__class__.__name__)
            
            # Instantiate router
            self._platform = router.HorizonRouteable()
            self._platform._version = self._version
            self._platform.message_routed = self.message_received
            def tmp():
                return self.__str__()
            self._platform.__str__ = tmp
            self._router = router.HorizonRouter(platform = self._platform, 
                                                clients = [], 
                                                send_all = self._send_all)
        
        # Open failed
        except Exception as ex:
            logger.error("%s: ...UDP port opening failed:\n%s" % \
                         (self.__class__.__name__, str(ex)))
            raise utils.TransportError \
                    ("UDP Port open failed!\n" + str(ex))
        
    
    ## UDP Server Transport Device Close
    #
    #  @see HorizonTransport._close(self)
    #
    #  @throws TransportError upon closing failure
    #
    #  @pydoc
    def _close(self):
        """UDP Server Transport Device Close"""
        
        # Close device
        logger.debug("%s: UDP port closing started..." % \
                     self.__class__.__name__)
        self._router = None
        self._platform = None
        self._udp_socket.close()
        logger.debug("%s: ...UDP port closing complete." % \
                     self.__class__.__name__)
        
    
    ## UDP Server Transport Device Get Horizon Message
    #
    #  @see HorizonTransport._get_message(self)
    #
    #  @throws TransportError upon receive failure
    #  @return received Horizon Message or None
    #
    #  @pydoc
    def _get_message(self):
        """UDP Server Transport Device Get Horizon Message"""
        if not self.opened: return None
        
        # read as much as possible
        read = 0
        try:    
            chars = [b'0']
            addr = None
            logger.debug("%s: Socket read started..." % \
                     self.__class__.__name__)
            while(len(chars) > 0):
                try:
                    chars, addr = self._udp_socket.recvfrom(1)
                except socket.error:
                    logger.debug("%s: ...Socket read complete." % \
                                 self.__class__.__name__)
                    break
                except socket.timeout:
                    logger.debug("%s: ...Socket read complete." % \
                                 self.__class__.__name__)
                    break
                if len(chars) > 0:
                    if addr not in self._rbuff:
                        self._rbuff[addr] = []
                    if sys.version_info[0] > 2:
                        self._rbuff[addr] += chars
                    else:
                        self._rbuff[addr] += map(ord,chars)
                    read += len(chars)
                else:
                    logger.error("%s: ...Socket has been closed." % \
                                 (self.__class__.__name__))
                    self.close()
                    return None
            logger.debug("%s: ...Socket read complete." % \
                     self.__class__.__name__)
        except Exception as ex:
            logger.error("%s: ...Socket read failed:\n%s" % \
                     (self.__class__.__name__,str(ex)))
            raise utils.TransportError \
                    ("Socket Message get failed!\n" + str(ex))
        if read > 0 :
            logger.info("%s: Read %d bytes." % (self.__class__.__name__, read))
            
        # Check all Clients
        for addr in self._rbuff.keys():
            
            # Look for message start (SOH XX ~XX)
            disc = []
            while(len(self._rbuff[addr]) > 3 and (
                    self._rbuff[addr][0] != messages.HorizonMessage.SOH or
                    self._rbuff[addr][1] != 0xFF&(~self._rbuff[addr][2]) or
                    self._rbuff[addr][1] == 0)):
                disc.append(self._rbuff[addr].pop(0))
            if len(disc) > 0:
                logger.info("%s: Discarded %d bytes:\n%s" % (
                        self.__class__.__name__, len(disc), 
                        ' '.join(map(utils.hex,disc))))
            if len(self._rbuff[addr]) < 3:
                continue
            
            # Extract Expected Message Length
            length = self._rbuff[addr][1] + 3 
        
            # Look for next message start
            for i in range(1,len(self._rbuff[addr])-2):
                if self._rbuff[addr][i] == messages.HorizonMessage.SOH and \
                        self._rbuff[addr][1]==0xFF&(~self._rbuff[addr][2]) and \
                        self._rbuff[addr][1] != 0:
                    if i < length:
                        length = i
                    break
            
            # Not all read yet
            if len(self._rbuff[addr]) < length:
                continue
            
            # Extract Message
            raw = self._rbuff[addr][0:length]
            self._rbuff[addr] = self._rbuff[addr][length:]
            logger.info("%s: Message of %d bytes found:\n%s" % (
                    self.__class__.__name__, len(raw), 
                    ' '.join(map(utils.hex,raw))))
            msg = messages.HorizonMessage(version = self._version, 
                                        payload_type = payloads.HorizonPayload,
                                       raw = raw, store_error = True)
            
            # update timestamp
            t = datetime.datetime.today()
            timestamp = t.microsecond/1000 + t.second*1000 + \
                        t.minute*60*1000 + t.hour*60*60*1000 + \
                        t.day*24*60*60*1000
            while timestamp > 4294967295: timestamp -= 4294967295
        
            # find connection
            for client in self._clients:
                if client.address == addr:
                    client._last = timestamp
                    client.route_message(msg)
                    continue
            
            # new connection
            if len(self._clients) >= self._max:
                continue
            self._clients.append(HorizonTransport_Socket(
                                sock = self._udp_socket,
                                host = addr[0],
                                port = addr[1],
                                name = "%s:%d" % addr,
                                store_timeout = 1,
                                version = self._version))
            self._clients[-1].opened = True
            self._router.add_client(self._clients[-1])
            logger.info("%s: New connection to %s:%d." % \
                         (self.__class__.__name__,self._clients[-1].address[0],
                         self._clients[-1].address[1]))
            client._last = timestamp
            client.route_message(msg)
            
            
        # update timestamp
        t = datetime.datetime.today()
        timestamp = t.microsecond/1000 + t.second*1000 + \
                    t.minute*60*1000 + t.hour*60*60*1000 + \
                    t.day*24*60*60*1000
        while timestamp > 4294967295: timestamp -= 4294967295
            
        # Connection Timeout?
        for i in range(len(self._clients),0,-1):
            last = self._clients[i-1].get_last_time()
            if ((timestamp - last >= self._rec_timeout) or\
                    (timestamp < last and 4294967295 - \
                    last + timestamp >= self._rec_timeout)):
                logger.warning("%s: Connection to %s timed-out!" % \
                         (self.__class__.__name__,self._clients[i-1].name))
                self._router.remove_client(self._clients[i-1])
                self._clients[i-1].opened = False
                self._clients.remove(self._clients[i-1])
        
        return None
        
    
    ## UDP Server Transport Device Send Horizon Message
    #
    #  @see HorizonTransport._send_message(self,message)
    #
    #  @param  message    The message to send
    #  @param  raw        None - Not supported
    #  @throws TransportError upon send failure
    #
    #  @pydoc
    def send_message(self, message, raw = None):
        """UDP Server Transport Device Send Horizon Message"""
        if not self.opened:
            logger.error("%s: Cannot send while closed!" % \
                         self.__class__.__name__)
            raise utils.TransportError ("Cannot send while closed!")
        if raw != None:
            logger.error("%s: UDP Server doesn't support raw send!" % \
                         self.__class__.__name__)
            raise utils.TransportError ("UDP Server doesn't support raw send!")
        
        # Send Message
        self._platform.route_message(message = message)
    
    
    ## Has Client
    #
    #  Is a client connected?
    #
    #  @return Is a client connected?
    #
    #  @pydoc
    def has_client(self):
        """Is a client connected?"""
        
        return len(self._clients) > 0
    
    
    ## Get Clients
    #
    #  Return a list of client addresses.
    #
    #  @return List of tuple([host,port])
    #
    #  @pydoc
    def get_clients(self):
        """Return a list of client addresses."""
        clis = []
        for c in self._clients:
            clis.append(c.get_address())
        return clis
    
    
    ## Transport Name
    #
    #  Return the transport name.
    #
    #  @return String of transport name.
    #
    #  @pydoc
    def get_name(self):
        """Return the transport name."""
        
        return 'UDP/IP Server'
    get_name = staticmethod(get_name)
    
    
    ## Server Address
    #
    #  Return The Server Address.
    #
    #  @return tuple(hostname,port)
    #
    #  @pydoc
    def get_address(self):
        """Return The Server Address."""
        
        return tuple('localhost',self._port)
    
    
    # Class Properties
    ## The Server Address
    address = property(fget=get_address, doc="The Server Address")
    ## The Transport Name
    name = property(fget=__str__, doc="The Transport Name")
   


logger.debug("... clearpath.horizon.transports loaded.")
