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
#  File: protocol.py
#  Desc: Horizon Protocol Message Handlers
#  Auth: Ryan Gariepy
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
    print ("ERROR: clearpath.horizon.protocol is a module and can NOT be run"\
           " as a script!\nFor a command-line interface demo of Horizon, run:"\
           "\n  python -m clearpath.horizon.demo\n"\
           "For Horizon message forwarding, run:\n"\
           "  python -m clearpath.horizon.forward")

    # Exit Error
    import sys
    sys.exit(1)




################################################################################
# Module



## @package clearpath.horizon.protocol 
#  Horizon Protocol Python Module
# 
#  Horizon Protocol Message Handlers                                          \n
#  Abstracted from knowing messages & underlying transport.                   \n
#  Supported Horizon version(s): 0.1 - 1.0
#
#  @author     Ryan Gariepy
#  @author     Malcolm Robert
#  @date       18/01/10
#  @todo       Implement the protocol server wrapper & document in use
#  @req        clearpath.utils                                                \n
#              clearpath.horizon.codes                                        \n
#              clearpath.horizon.messages                                     \n
#              clearpath.horizon.payloads                                     \n
#              clearpath.horizon.transports                                   \n
#              clearpath.horizon.versioning
#  @version    1.0
#
#  @section HORIZON
#  @copydoc overview1
#
#  @section USE
#
#  The intended purpose of this module is to provide a layer between the 
#  Horizon interface (or emulator) and the low-level transports that will 
#  automatically handle acknowledgments and message formatting. Only 
#  HorizonPayload classes, message codes, and errors are exposed.
#
#  The Client protocol wrapper, HorizonProtocol_Client, is used for clients 
#  that connect to a platform to automatically handle acknowledgments. Upon
#  initialization, the class is told which transport to instantiate and with
#  what arguments along with timeout values (how long to wait for an 
#  acknowledgment, to store a message) and the number or times to retry sending
#  a message upon errors/timeout. Before the instance can be used, the method
#  open must be called to perform the underlying transport open and similarly,
#  upon completion of use, close must be called.
#
#  Just like the HorizonTransport class, the client can be used multiple ways.
#  To use synchronously, the method send_message is used to send messages 
#  and the method get_waiting is used to obtain received messages.
#  To receive asynchronously, create one or more non-blocking methods that 
#  accepts one argument (HorizonPayload) and pass them to the method 
#  add_handler. Data messages will be passed to these methods upon receipt. 
#  Alternatively, to use asynchronously, create a subclass of the client class 
#  overriding the method message_received, which gets called each time a data
#  message arrives.
#  To send asynchronously (do not wait for acknowledgment), create a method with
#  one argument (Exception) and pass it to handler when calling send_message.
#  This method will be called with the encountered error or None for a 
#  successful send upon receipt of an acknowledgment.
#  Alternatively, if acknowledgments are not important, pass True for no_ack
#  when calling send_message. The method will not block and will assume that 
#  the send was successful.
#
#  The server protocol wrapper, HorizonProtocol_Server, is used for programs
#  that want to emulate a platform and have clients connect to it.
#  The server has yet to be developed.
#
#  Various logging messages (debug, warning,...) are logged with the standard
#  Python logger to the log named 'clearpath.horizon.protocol'.               \n
#  Messages are ignored by default; remove the only handler from it and turn
#  on propagation to get messages to propagate.
#
#  @section HISTORY
#  Version 0.1 - 0.3 {Ryan Gariepy}
#  - Initial Creation as protocol.py
#
#  Version 0.4 {Malcolm Robert}
#  - Move to horizon_protocol.py
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
#  - Extracted transports to transports.py
#  - Horizon support for v0.7
#
#  Version 0.8
#  - Horizon support for v 0.1 - 0.8
#
#  Version 1.0
#  - Horizon support for v 0.1 - 1.0
#  - Python 2.6+ & 3.x compatible
#
#  @section License
#  @copydoc public_license
#
#  @defgroup overview1 Overview Part I
#  @ingroup overview
#  This protocol is meant as a simple way for users of the various Clearpath 
#  Robotics research offerings to interface with the Clearpath Robotics 
#  hardware. It includes several features intended to increase communication 
#  reliability, while keeping message overhead and protocol complexity low. 
#  For the sake of rapid prototyping, it is not intended for multiple devices 
#  to be simultaneously connected to each communication line, removing the need 
#  for addressing or negotiation.
#
"""Horizon Protocol Message Handlers

   Copyright © 2010 Clearpath Robotics, Inc.
   All rights reserved
   
   Created: 18/01/10
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
from .  import messages         # Horizon Protocol Message Definition
from .  import payloads         # Horizon Protocol Message Payload Definitions
from .  import transports       # Horizon Transport Definitions
from .  import versioning       # Horizon Protocol Versions 

# Required Python Modules
import datetime                 # Date & Time Manipulation
import logging                  # Logging Utilities
import sys                      # Python Interpreter Functionality
import time                     # System Date & Time

# Version Dependent Modules
if sys.version_info[0] > 2:
    import queue                # Thread-safe Queue
else:
    import Queue as queue       # Thread-safe Queue


# Module Support
## Module Version
__version__  = "1.0"
"""Module Version"""
## SVN Code Revision
__revision__ = "$Revision: 389 $"
""" SVN Code Revision"""



## Message Log
logger = logging.getLogger('clearpath.horizon.protocol')
"""Horizon Protocol Module Log"""
logger.setLevel(logging.NOTSET)
logger.addHandler(utils.NullLoggingHandler())
logger.propagate = False
logger.debug("Loading clearpath.horizon.protocol ...")         




################################################################################
# Horizon Protocol Controllers



## @defgroup flow Data Flow
#  @ingroup format
#  The below figure shows a typical sequence of commands. The platform is 
#  sending data \b (D) with a regular frequency when the control computer sends
#  a command \b (C). The platform acknowledges the command immediately \b (A) 
#  and continues sending data.
#  @image html flow.png
#  @manonly
#
#  PC TX --------------------------- C ---------------------
#                                     \\
#  PC RX - D --- D --- D --- D --- D - A D --- D --- D --- D
#  @endmanonly



## Horizon Protocol Controller Client
#
#  Represents the Horizon message transport protocol used for connecting to
#  a platform.                                                                \n
#  Handles sending and receiving messages as well as acknowledgments.         \n
#  Acknowledgments are not exposed.                                           \n
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
#  To be inherited for asynchronous message handling.                         \n
#  Message handling subclasses should only override __init__, __del__, 
#  message_received, add_handler, and get_waiting.                            \n
#  The methods __init__ and __del__ should call the parent's overridden
#  method.                                                                    \n
#                                                                             \n
#  If system time is used then only milliseconds, seconds, minutes, hours, and
#  day of month will be used. Otherwise, time since start will loop back to 
#  zero after reaching the maximum value (4294967295).                        \n
#  Additionally, the chassis time is reset upon boot and thus may need altering
#  for synchronization with the set_chassis_time command.
#
#  @since 0.1
#
#  @section flow Data Flow
#  @copydoc flow
#  
#  @pydoc
class HorizonProtocol_Client():
    """Horizon Transport Protocol Controller - Client Device"""
    
    
        
    ## Create A Horizon Protocol Client
    #  
    #  Constructor for the Horizon message Transport protocol client.         \n
    #  Performs the initial creation and initialization of the underlying 
    #  transport.                                                             \n
    #  Does NOT support version auto-detection.                               \n
    #                                                                         \n
    #  Refer to the transport's __init__ method for argument specifications.
    #                                                                         \n
    #  Override this method for subclass initialization.                      \n
    #  Overriding methods should call this method. 
    #
    #  @param  retries        The number of times to retry sending a message
    #                         that received a timeout or checksum error
    #  @param  send_timeout   The time to wait for an acknowledgment
    #                         in milliseconds, 0 - wait indefinitely
    #  @param  store_timeout  The time to store an un-handled message for the 
    #                         method get_waiting in milliseconds,
    #                         0 - store indefinitely
    #  @param  sys_time       Use system time (True) instead of time since 
    #                         instantiation (False)?
    #  @param  transport      The HorizonTransport class to use.
    #  @param  transport_args Dictionary of arguments to pass to the transport's
    #                         __init__ method. Do NOT include version or
    #                         store_timeout as these will be populated.
    #  @param  version        Horizon Protocol Version,
    #                         (-1,*) represents the newest version,
    #                         (0,0) auto-detect the version (if supported)
    #  @see    clearpath.horizon.transports.HorizonTransport_Serial.__init__
    #  @see    clearpath.horizon.transports.HorizonTransport_TCP_Client.__init__
    #  @see    clearpath.horizon.transports.HorizonTransport_UDP_Client.__init__
    #  @throws LookupError    If auto-detect version fails
    #  @throws TransportError Upon transport creation/initialization failure
    #  @throws ValueError     From bad arguments
    #
    #  @pydoc
    def __init__(self, transport, transport_args,
                 retries = 5, sys_time = False, send_timeout = 500, 
                 store_timeout = 10000, version = tuple([-1,0])):
        """Create A Horizon Protocol Client"""
        logger.debug("%s: Instance creation started..." % \
                     self.__class__.__name__)
        
        # Class Variables
        ## Message Acknowledgments
        self._acks = {}         # Format: { timestamp:ack }
        ## Message Handlers
        self._handlers = {}     # Format: { code:[handler] }
        ## Message Storage
        self._messages = {}     # Format: { code:tuple([timestamp,queue]) }
        ## Retry Count
        self._retries = retries
        ## Send Timeout
        self._send_timeout = send_timeout
        ## Sent Messages
        self._sent = {}         # Format: timestamp:(message,key,handler,tries)
        ## Start Time
        self._start_time = datetime.datetime.today()
        ## Store Timeout
        self._store_timeout = store_timeout
        ## Use System Time?
        self._sys_time = sys_time
        ## Transport Interface
        self._transport = None

        self.version = version

     
        # Verify retries
        if retries < 0:
            logger.error("%: Invalid retries!" % self.__class__.__name__)
            raise ValueError ("Invalid retries!")
        logger.info("%s: Using %d retries." % (self.__class__.__name__,
                                               self._retries))
        
        # Verify timeouts
        if send_timeout < 0:
            logger.error("%s: Invalid send timeout!" % self.__class__.__name__)
            raise ValueError ("Invalid send timeout!")
        logger.info("%s: Using %d ms send timeout." % (self.__class__.__name__,
                                                  self._send_timeout))
        if store_timeout < 0:
            logger.error("%s: Invalid store timeout!" % self.__class__.__name__)
            raise ValueError ("Invalid store timeout!")
        logger.info("%s: Using %d ms store timeout." % (self.__class__.__name__,
                                                  self._store_timeout))
        
        # System Time
        if(self._sys_time):
            logger.info("%s: Using system time." % self.__class__.__name__)
        else:
            logger.info("%s: Using program run time." % self.__class__.__name__)
    
        # Create underlings
        transport_args['store_timeout'] = 1
        self._transport = transport(**transport_args)
        if not isinstance(self._transport,transports.HorizonTransport):
            logger.error("%s: Invalid transport!" % self.__class__.__name__)
            raise ValueError ("Invalid transport!")
        self._transport.message_received = self._received
        self._transport._timeout = self._timeout
        logger.debug("%s: ...instance creation complete." % \
                     self.__class__.__name__)
    
    
    ## Destroy A Horizon Protocol Client
    #  
    #  Destructor for the Horizon message Transport protocol client.          \n
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
    #  Return the transport name.
    # 
    #  @return String of transport name.
    #
    #  @pydoc
    def __str__(self):
        """Return the transport name."""
        
        return str(self._transport)
    
    
    ## Horizon Protocol Check Timeouts
    #  
    #  Message send timeout checker.                                          \n
    #                                                                         \n
    #  Do not override this method.
    #  
    #  @pydoc
    def _timeout(self):
        """Horizon Protocol Check Timeouts"""
        
        # Update Timestamp
        timestamp = 0
        if self._sys_time:
            t = datetime.datetime.today()
            timestamp = t.microsecond/1000 + t.second*1000 + \
                    t.minute*60*1000 + t.hour*60*60*1000 + t.day*24*60*60*1000
        else:
            t = (datetime.datetime.today() - self._start_time)
            timestamp = t.microseconds/1000 + t.seconds*1000 + \
                    t.days*24*60*60*1000
        while timestamp > 4294967295: timestamp -= 4294967295
        
        # Check send timeouts
        if(self._send_timeout > 0):
            for stamp in set(self._sent.keys()):
                
                # Free space
                if self._store_timeout > 0 and \
                        (timestamp - stamp >= self._send_timeout + 
                         self.store_timeout) or (timestamp < stamp and 
                        4294967295 - stamp + timestamp >= self._send_timeout +
                        self._store_timeout):
                    self._acks.pop(self._sent[stamp][1])
                    self._sent.pop(stamp)
                    
                # Send Timeout
                elif stamp in self._sent and \
                        (timestamp - stamp >= self._send_timeout) or\
                        (timestamp < stamp and \
                         4294967295 - stamp + timestamp >= self._send_timeout):
                    (msg,key,handler,tries) = self._sent[stamp]
                    if self._acks[key] != None: continue
                        
                    # Retry Send
                    if tries < self._retries: 
                        logger.warning("%s: sent message timeout. Retry %d." %\
                                 (self.__class__.__name__, (tries+1)))
                        self.send_message(code = msg.code, payload= msg.payload,
                                      handler = handler, tries = tries+1,
                                      key = key)
                        self._sent.pop(msg.timestamp)
                            
                    # Message Retries Maxed Out
                    else:
                        logger.warning("%s: message timeout occurred." % \
                                 (self.__class__.__name__))
                        self._acks[key] = utils.TimeoutError(
                                                    "Message Timeout Occurred!")
                        self._sent.pop(msg.timestamp)
                        if handler != None:
                            handler(error=self._acks[key])
                            self._acks.pop(key)
                     
                # Stop searching       
                else:
                    break

    
    ## Horizon Transport Received Message
    #  
    #  Message received handler.                                              \n
    #                                                                         \n
    #  Do not override this method.
    #
    #  @param  message    The message received
    #
    #  @pydoc
    def _received(self, message):
        """Horizon Transport Receiver Loop"""
        
        # Update Timestamp
        timestamp = 0
        if self._sys_time:
            t = datetime.datetime.today()
            timestamp = t.microsecond/1000 + t.second*1000 + \
                    t.minute*60*1000 + t.hour*60*60*1000 + t.day*24*60*60*1000
        else:
            t = (datetime.datetime.today() - self._start_time)
            timestamp = t.microseconds/1000 + t.seconds*1000 + \
                    t.days*24*60*60*1000
        while timestamp > 4294967295: timestamp -= 4294967295
        
        # Received Acknowledgment?
        isack = -1
        if message.timestamp in self._sent.keys(): 
            isack = message.timestamp
        if isack > -1:
            (msg,key,handler,tries) = self._sent[isack]
            self._sent.pop(msg.timestamp)
                
            # Test Flags
            payload = payloads.HorizonPayload_Ack(store_error = True,
                                                  version = self.version,
                                                  raw = message._payload.data)
            if (payload == None) or \
                    (payload.error != None or payload.bad_checksum):
                    
                # Retry Send
                if tries < self._retries: 
                    logger.warning("%s: sent message failed. Retry %d." % \
                             (self.__class__.__name__, (tries+1)))
                    self.send_message(code = msg.code, payload = msg.payload,
                                      handler = handler, tries = tries+1,
                                      key = key)
                        
                # Message Retries Maxed Out
                else:
                    logger.warning("%s: sent message failed." % \
                             (self.__class__.__name__))
                    self._acks[key] = utils.TransportError(
                                                        "Send Message Failed!")
                    if handler != None:
                        handler(error=self._acks[key])
                        self._acks.pop(key)
                            
            # No resend error
            else:
                ack = 0
                if payload.bad_code == True:
                    ack = utils.UnsupportedCodeError(
                                            "Acknowledgment says Bad Code.")
                elif payload.bad_format == True:
                    ack = utils.FormatError("Acknowledgment says Bad Format.")
                elif payload.bad_values == True:
                    ack = ValueError("Acknowledgment says Bad Values.")
                elif payload.bad_frequency == True:
                    ack = utils.SubscriptionError(
                                        "Acknowledgment says Bad Frequency.")
                elif payload.bad_code_count == True:
                    ack = utils.SubscriptionError(
                                "Acknowledgment says Too Many Subscriptions.")
                elif payload.bad_bandwidth == True:
                    ack = utils.SubscriptionError(
                                    "Acknowledgment says Not Enough Bandwidth.")
                self._acks[key] = ack
                if handler != None:
                    if ack == 0:
                        handler(error=None)
                    else:
                        handler(error=ack)
                    self._acks.pop(key)
                    
        # Valid Message
        elif message.code >= 0x8000:
            payload = codes.PAYLOAD_MAP[codes.HORIZON_CODES[message.code]]
            self.message_received(message.code, payload(store_error = True,
                                                        raw = message.payload.data,
                                                        timestamp = message.timestamp,
                                                        version = message.version), timestamp)
            
        # Check store timeouts
        if self._store_timeout > 0:
            for code in self._messages:
                msge = None
                if not self._messages[code].empty():
                    msge = self._messages[code].queue[0][0]
                while (not self._messages[code].empty()) and \
                        msge != None and \
                        (timestamp - msge >=self._store_timeout)or\
                        (timestamp < msge and 4294967295 - \
                         msge + timestamp >= self._store_timeout):
                    logger.warning("%s: message 0x%04X store timeout "\
                                           "occurred."%(self.__class__.__name__, 
                                                          code))
                    self._messages[code].get()
                    if not self._messages[code].empty():
                        msge = self._messages[code].queue[0][0]
    
    
    ## Horizon Transport Open
    #  
    #  Opens the transport device.                                            \n
    #                                                                         \n
    #  Do not override this method.
    #
    #  @throws TransportError upon open failure
    #
    #  @pydoc
    def open(self):
        """Horizon Transport Open"""
        if not self._transport.is_open():
        
            # Open Device
            self._transport.open()
        
    
    ## Horizon Transport Close
    #  
    #  Closes the transport device.                                           \n
    #                                                                         \n
    #  Do not override this method.
    #
    #  @throws TransportError upon open failure
    #
    #  @pydoc
    def close(self):
        """Horizon Transport Close"""
        
        # Cleanup Device
        if self._transport != None:
            self._transport.close()

    
    ## Horizon Protocol Send Message
    #  
    #  Sends a command over the transport.                                    \n
    #                                                                         \n
    #  Do not override this method.                                           \n
    #                                                                         \n
    #  If handler is None then this method will act synchronously and block
    #  until acknowledgment (or non-blocking with no_ack). Otherwise this
    #  method will act asynchronously and have handler called upon completion.
    #
    #  @param  code                   The Message code
    #  @param  handler                Acknowledgment Message Handler
    #                                 Must have one parameter: 
    #                                   Exception error (None = success).
    #  @param  key                    Key used for first send attempt,
    #                                 -1 for first attempt
    #  @param  no_ack                 Suppress Acknowledgment? (assume success?)
    #  @param  payload                The Message Payload
    #  @param  tries                  Previous number of send tries
    #  @throws FormatError            If acknowledgment says bad payload.
    #  @throws IOError                If not opened
    #  @throws SubscriptionError      If acknowledgment says bad subscription.
    #  @throws TimeoutError           If timeout occurred
    #  @throws TransportError         upon send failure
    #  @throws UnsupportedCodeError   If acknowledgment says bad code.
    #  @throws ValueError             If message creation fails or 
    #                                 if acknowledgment says bad values.
    #
    #  @pydoc
    def send_message(self, code = 0x0000, payload = payloads.HorizonPayload(),
                     no_ack = False, handler = None, tries = 0, key = -1):
        """Horizon Protocol Send Message"""
        if not self._transport.is_open(): 
            raise IOError ("Transport has not been opened!")

        # Update Timestamp
        timestamp = 0
        if self._sys_time:
            t = datetime.datetime.today()
            timestamp = t.microsecond/1000 + t.second*1000 + \
                t.minute*60*1000 + t.hour*60*60*1000 + t.day*24*60*60*1000
        else:
            t = (datetime.datetime.today() - self._start_time)
            timestamp = t.microseconds/1000 + t.seconds*1000 + \
                t.days*24*60*60*1000
        while timestamp > 4294967295: timestamp -= 4294967295
        if sys.version_info[0] < 3: timestamp = long(timestamp)
        else: timestamp = int(timestamp)

        logger.info("%s: Message 0x%04X will be sent with timestamp %d." %\
                        (self.__class__.__name__, code, timestamp))
        if key < 0:
            key = timestamp
        
        # Create Message
        message = messages.HorizonMessage(code = code, 
                        payload = payload, timestamp = timestamp, 
                        version = self.version, no_ack = no_ack)
        
        # Handler(s)
        if no_ack == False:
        
            # Setup Handler
            self._acks[key] = None
            self._sent[message.timestamp] = tuple([message,key,handler,tries])
            
        # Send Message
        self._transport.send_message(message)
        
        # Synchronize?
        if no_ack == False and handler == None and key == timestamp and key in self._acks:
            logger.debug("%s: Send synchronization started..." % \
                     self.__class__.__name__)
            while self._acks[key] == None: time.sleep(0.001)
            logger.debug("%s: ...send synchronization complete." % \
                     self.__class__.__name__)
            if self._acks[key] != 0:
                ack = self._acks[key]
                self._acks.pop(key)
                raise ack
            self._acks.pop(key)
    
    
    ## Horizon Message Received
    #
    #  Handles receiving a message.                                           \n
    #  Cannot be blocking and cannot raise exceptions.                        \n
    #                                                                         \n
    #  Override this method in subclasses that handle messages.
    #
    #  @param  code      The message type
    #  @param  payload   The message payload received
    #  @param  timestamp The time the message was received
    #
    #  @pydoc
    def message_received(self, code = 0x0000, 
                         payload = payloads.HorizonPayload(),
                         timestamp = 0):
        """Horizon Message Received"""
        
        # Handlers?
        handled = False
        if code in self._handlers and len(self._handlers[code]) > 0:
            for handler in self._handlers[code]:
                handler(code,payload,timestamp)
                handled = True
        if 0 in self._handlers and len(self._handlers[0]) > 0:
            for handler in self._handlers[0]:
                handler(code,payload,timestamp)
                handled = True
        
        # Storage?
        if handled == False:
            if code not in self._messages:
                self._messages[code] = queue.Queue()
            self._messages[code].put(tuple([timestamp,payload]))
            
            
    ## Horizon Protocol Add Data Message Handler
    #  
    #  Adds a data message code's handler.                                    \n
    #  Asynchronous method of receiving messages.                             \n
    #  The handler will be called each time a non-ack message of the code is
    #  received.                                                              \n
    #  Specify a code of 0 to have the handler called on all non-ack messages.\n
    #                                                                         \n
    #  Do not override this method.
    #
    #  @param  backtrack      Call the new handler with any waiting messages?
    #  @param  code           The Message code
    #  @param  handler        The Message Handler. 
    #                         Must have three parameters: 
    #                            int code,
    #                            HorizonPayload payload,
    #                            long timestamp
    #
    #  @pydoc
    def add_handler(self, handler, code=0x0000, backtrack = False):
        """Horizon Protocol Add Data Message Handler"""

        # Add Handler
        if code not in self._handlers: self._handlers[code] = []
        self._handlers[code].append(handler)
        logger.debug("%s handler %s added for code 0x%04x" % \
                     (self.__class__.__name__, handler.__name__, code))
        
        # Backtrack
        if backtrack:
            logger.debug("%s handler %s backtrack for code 0x%04x" % \
                     (self.__class__.__name__, handler.__name__, code))
            for message in self.get_waiting(code): handler(message[0],
                                                           message[1],
                                                           message[2])
            

    ## Horizon Protocol Remove Data Message Handler
    #  
    #  Removes a data message code's handler.                                 \n
    #                                                                         \n
    #  Do not override this method.
    #
    #  @param  code           The Message code
    #  @param  handler        The Message Handler to remove.
    #                         If None, remove all handlers.
    #
    #  @pydoc
    def remove_handler(self, handler=None, code=0x0000):
        """Horizon Protocol Remove Data Message Handler"""

        # Remove Handler
        if code in self._handlers: 
            if handler != None and handler in self._handlers[code]:
                self._handlers[code].remove(handler)
                logger.debug("%s handler %s removed for code 0x%04x" % \
                     (self.__class__.__name__, handler.__name__, code))
            elif handler == None:
                self._handlers[code] = []
                logger.debug("%s handlers removed for code 0x%04x" % \
                     (self.__class__.__name__, code))
            

    ## Horizon Protocol Get Data Message Handler(s)
    #  
    #  Gets a data message code's handler list.                               \n
    #                                                                         \n
    #  Do not override this method.
    #
    #  @param  code           The Message code
    #  @return List of handlers, empty if none. 
    #
    #  @pydoc
    def get_handlers(self, code=0x0000):
        """Horizon Protocol Get Data Message Handler(s)"""

        # No Handlers
        if code not in self._handlers:
            logger.debug("%s returned 0 handlers for code 0x%04x" % \
                     (self.__class__.__name__, code))
            return []
        
        # Get handlers 
        logger.debug("%s returned %d handlers for code 0x%04x" % \
                     (self.__class__.__name__, len(self._handlers[code]), code))
        return self._handlers[code][:]
        
        
    ## Horizon Protocol Get Waiting Messages
    #  
    #  Gets a list of waiting messages.                                       \n
    #  Synchronous method of receiving messages.                              \n
    #  Specify a code of 0 to get all non-ack messages.                       \n
    #                                                                         \n
    #  Do not override this method.
    #
    #  @param  code           The Message code
    #  @return                list of tuple(code,payload,timestamp), empty
    #  
    #  @pydoc
    def get_waiting(self, code=0x0000):
        """Horizon Protocol Get Waiting Messages"""
        msgs = []
        
        # Check for all
        if code == 0:
            for c in self._messages.keys():
                while not self._messages[c].empty():
                    msge = self._messages[c].get()
                    msgs.append(tuple([c,msge[1],msge[0]]))
                    
        # Single code
        if code in self._messages:
            while not self._messages[code].empty():
                msge = self._messages[code].get()
                msgs.append(tuple([code,msge[1],msge[0]]))
            
        logger.debug("%s returned %d waiting messages for code 0x%04x" % \
                     (self.__class__.__name__, len(msgs), code))
        return msgs
    
    
    ## Is Opened?
    #
    #  @return is the transport opened?
    #
    #  @pydoc
    def is_open(self):
        """Is Opened?"""
        
        return self._transport.is_open()
    
    
    ## Get Transport
    #
    #  @return the underlying transport being used
    #
    #  @pydoc
    def get_transport(self):
        """Get Transport"""
        
        return self._transport
    
    
    ## Get Max Retries
    #
    #  @return the number of times to send a message before all hope is lost
    #
    #  @pydoc
    def get_retries(self):
        """Get Max Retries"""
        
        return self._retries
    
    
    ## Set Max Retries
    #
    #  @param tries number of times to send a message before all hope is lost
    #  @throws ValueError upon bad retry count
    #
    #  @pydoc
    def set_retries(self,tries):
        """Set Max Retries"""
        
        if tries < 0:
            logger.error("%: Invalid retries!" % self.__class__.__name__)
            raise ValueError ("Invalid retries!")
        logger.info("%s: Using %d retries." % (self.__class__.__name__,
                                               tries))
        
        self._retries = tries
    
    
    ## Get Start Time
    #
    #  @return the transport's start time
    #
    #  @pydoc
    def get_start_time(self):
        """Get Start Time"""
        
        return self._start_time.microsecond/1000 + \
                self._start_time.second*1000 + \
                self._start_time.minute*60*1000 + \
                self._start_time.hour*60*60*1000 + \
                self._start_time.day*24*60*60*1000
    
    
    ## Is System Time?
    #
    #  @return is the timestamp the system time?
    #
    #  @pydoc
    def is_sys_time(self):
        """Is System Time?"""
        
        return self._sys_time
    
    
    ## Get Send Timeout
    #
    #  @return the message send timeout
    #
    #  @pydoc
    def get_send_timeout(self):
        """Get Send Timeout"""
        
        return self._send_timeout
    
    
    ## Set Send Timeout
    #
    #  @param timeout the message send timeout
    #  @throws ValueError upon bad timeout
    #
    #  @pydoc
    def set_send_timeout(self,timeout):
        """Set Send Timeout"""
        
        if timeout < 0:
            logger.error("%s: Invalid send timeout!" % self.__class__.__name__)
            raise ValueError ("Invalid send timeout!")
        logger.info("%s: Using %d ms send timeout." % (self.__class__.__name__,
                                                  timeout))
        
        self._send_timeout = timeout
    
    
    ## Get Store Timeout
    #
    #  @return the message store timeout
    #
    #  @pydoc
    def get_store_timeout(self):
        """Get Store Timeout"""
        
        return self._store_timeout
    
    
    ## Set Store Timeout
    #
    #  @param timeout the message store timeout
    #  @throws ValueError upon bad timeout
    #
    #  @pydoc
    def set_store_timeout(self,timeout):
        """Set Store Timeout"""
        
        if timeout < 0:
            logger.error("%s: Invalid store timeout!" % self.__class__.__name__)
            raise ValueError ("Invalid store timeout!")
        logger.info("%s: Using %d ms store timeout." % (self.__class__.__name__,
                                                        timeout))
        
        self._store_timeout = timeout
    
    
    ## Get Horizon Version
    #
    #  @return the horizon version of this transport
    #
    #  @pydoc
    def get_version(self):
        """Get Horizon Version"""
        
        return self._version
    
    
    # Class Properties
    ## Transport Opened?
    opened = property(fget=is_open, doc="Transport Opened?")
    ## Message Retries
    retries = property(fget=get_retries, fset=set_retries,doc="Message Retries")
    ## Start Time
    start_time = property(fget=get_start_time, doc="Start Time")
    ## System Time
    sys_time = property(fget=is_sys_time, doc="System Time")
    ## Message Send Timeout
    send_timeout = property(fget=get_send_timeout, fset=set_send_timeout,
                            doc="Message Send Timeout")
    ## Message Store Timeout
    store_timeout = property(fget=get_store_timeout, fset=set_store_timeout,
                             doc="Message Store Timeout")
 
## Horizon Protocol Controller Server
#
#  Represents the Horizon message transport protocol used for connecting to
#  clients (to impersonate a platform).                                       \n
#  Handles sending and receiving messages as well as acknowledgments.         \n
#  Acknowledgments are not exposed except to the point of verification of 
#  received messages.                                                         \n
#  Provides both synchronous and asynchronous functionality.                  \n
#                                                                             \n
#  If system time is used then only milliseconds, seconds, minutes, hours, and
#  day of month will be used. Otherwise, time since start will loop back to 
#  zero after reaching the maximum value (4294967295).                        \n
#  Additionally, the chassis time is expected to be reset upon boot and thus 
#  should support being changed for synchronization with the set_chassis_time 
#  command.
#
#  @todo  Implement Me
#  @since 0.1
#
#  @section flow Data Flow
#  @copydoc flow
#  
#  @pydoc
class HorizonProtocol_Server():
    """Horizon Transport Protocol Controller - Server Device"""
    pass

logger.debug("... clearpath.horizon.protocol loaded.")
