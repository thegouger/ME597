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
#  File: horizon/__init__.py
#  Desc: Horizon Python Module
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
# Script - Package __init__ shouldn't be able to be run as a script.



# Check if run as a script
if __name__ == "__main__":
    
    # Warn of Module ONLY status
    print ("ERROR: clearpath.horizon is a module and can NOT be run"\
           " as a script!\nFor a command-line interface demo of Horizon, run:"\
           "\n  python -m clearpath.horizon.demo\n"\
           "For Horizon message forwarding, run:\n"\
           "  python -m clearpath.horizon.forward")

    # Exit Error
    import sys
    sys.exit(1)




################################################################################
# Module



## @package clearpath.horizon
#  Horizon Python Module
# 
#  Horizon Interface.                                                         
#  Supported Horizon version(s): 0.1 - 1.0
#
#  @author     Ryan Gariepy
#  @author     Malcolm Robert
#  @date       14/01/10
#  @req        clearpath.utils                                                
#              clearpath.horizon.codes                                        
#              clearpath.horizon.protocol                                     
#              clearpath.horizon.transports                                   
#              clearpath.horizon.versioning
#  @version    1.0
#
#  @section HORIZON
#  @copydoc overview
#
#  @section TERMS
#  @copydoc terms
#
#  @section USE
#
#  The intended purpose of this module is to provide a 1:1 interface for Horizon
#  (1:1 -> one function for every command and request). This class uses 
#  HorizonProtocol_Client and only exposes HorizonPayload classes and 
#  exceptions so that the program using it does not need to know the protocol.
#
#  Upon initialization, the class is told which transport to instantiate and 
#  with what arguments along with timeout values (how long to wait for an 
#  acknowledgment, to store a message, to wait for a data response) and the 
#  number or times to retry sending a message upon errors/timeout. Before the 
#  instance can be used, the method open must be called to perform the 
#  underlying protocol and transport open and similarly, upon completion of use, 
#  close must be called.
#
#  Just like the HorizonProtocol_Client and the HorizonTransport classes, the 
#  interface can be used multiple ways.
#  To use synchronously, the command and request methods are used to send 
#  messages and the method get_waiting is used to obtain received messages.
#  To receive asynchronously, create one or more non-blocking methods that 
#  accepts one argument (HorizonPayload) and pass them to the method 
#  add_handler. Data messages will be passed to these methods upon receipt. 
#  Alternatively, to use asynchronously, create a subclass of the interface 
#  class overriding the method message_received, which gets called each time a 
#  data message arrives.
#  To send asynchronously (do not wait for acknowledgment), create a method with
#  one argument (Exception) and pass it to handler when calling the command and
#  request methods. This method will be called with the encountered error or 
#  None for a successful send upon receipt of an acknowledgment.
#
#  The ability to suppress acknowledgments, using the no_ack flag within a 
#  message, is not exposed and not made available to the programs that use this
#  interface because, while suppressing acknowledgments allows for higher 
#  bandwidth usage, it removes reliability and also prevents knowledge of
#  whether or not the interface is actually connected to a platform.
#  Advanced programs may still use this ability by directly using the protocol
#  client through the method get_protocol.
#
#  Various logging messages (debug, warning,...) are logged with the standard
#  Python logger to the log named 'clearpath.horizon'.                        
#  Messages are ignored by default; remove the only handler from it and turn
#  on propagation to get messages to propagate.
#
#  @section HISTORY
#  Version 0.1 - 0.3 {Ryan Gariepy}
#  - Initial Creation as protocol_demo.py
#
#  Version 0.4 {Malcolm Robert}
#  - Move to horizon.py
#  - Added logging
#  - Added version support
#  - Added Doxygen documentation
#  - Changed version scheme to match Horizon doc
#  - Horizon support for v0.4 messages
#
#  Version 0.5
#  - Added TCP and UDP support
#  - Horizon support for v0.5
#
#  Version 0.6
#  - Move to horizon package __init__.py
#  - Horizon support for v0.6
#
#  Version 0.7
#  - Added Encryption Support
#  - Horizon support for v0.7
#
#  Version 0.8
#  - Horizon support for v0.8
#
#  Version 1.0
#  - Horizon support for v 0.1 - 1.0
#  - Python 2.6+ & 3.x compatible
#
#  @section License
#  @copydoc public_license
#  
#  @defgroup overview Overview
#  @ingroup doc
#
#  @copydoc overview1
#
#  @copydoc overview2
#
#  @defgroup overview2 Overview Part II
#  @ingroup overview
#  Additionally, no setup commands are necessary. Each platform's default 
#  firmware is set up to allow for movement out-of-the-box – no characterization 
#  or controls development necessary. However, low-level servo access is still
#  available for those who wish to develop their own controllers.           
#
#  Please consult the specific documentation for your platform for any safety 
#  considerations, terrain limits, or any other operating requirements.
#
#  @defgroup terms Terms
#  @ingroup doc
#  \b Platform: The Clearpath Robotics hardware being used, including the 
#  microcontroller (one end of the serial line).                            
#
#  \b Mobile \b Platform: A platform which is not fixed to the ground and is 
#  capable of controlled movement.                                          
#
#  \b Control \b computer: The computer issuing commands to the platform (the 
#  other end of the serial line).                                           
#
#  \b Little-endian \b byte \b order: The first byte received of a multi-byte 
#  field is the least significant byte.                                     
#
#  \b DOF: Degree of freedom, referring to an independent joint in a manipulator 
#  arm or actuated mount (such as a camera gimbal)                          
#
#  \b Differential-drive \b steering: Also referred to as “skid-steer”, where 
#  turning is achieved by varying left and right wheel speeds independently.
#
#  \b Ackermann \b steering: The steering geometry used in cars and trucks 
#  whereby the front wheels pivot for turning.
#
"""Horizon Interface

   Copyright © 2010 Clearpath Robotics, Inc.
   All rights reserved
   
   Created: 14/01/10
   Author:  Ryan Gariepy & Malcolm Robert
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


# A note on cmd:
# To take advantage of Python's dynamic creation and public accessible features,
# the cmd class HorizonDemo implements very few command methods and instead of
# requiring a function for each function in the Horizon Interface, HorizonDemo 
# uses the __dict__ property to automatically grab functions from Horizon.
# Each function that is grabbed has a doc string that starts with "syntax: ",
# has an entry in Horizon.VERSION_MAP matching __name__ and its versions,
# and must have default values for each parameter (other than self) so that
# HorizonDemo can know the parameter types. Further, grabbed functions cannot
# have *lst or **dct parameters and all parameter values must be representable
# by a string other than the ignored async parameter. The line passed by cmd is 
# separated by whitespace and then converted into the appropriate type for its 
# referential parameter. Any ValueError(s) raised are treated as parameter 
# errors and the error string is directly outputted to the user. HorizonDemo 
# will ignore any methods that do not have horizon.version in their
# VERSION_MAP set.


# Required Clearpath Modules
from .. import utils            # Clearpath Utilities
from .  import codes            # Horizon Message Codes
from .  import protocol         # Horizon Protocol Interface
from .  import transports       # Horizon Message Transports
from .  import versioning       # Horizon Protocol Versions

# Required Python Modules
import datetime                 # Date & Time Manipulation
import logging                  # Logging Utilities
import time                     # System Date & Time
import inspect                  # For parameter comprehension in command decorators


# Module Support
## Module Version
__version__  = "1.0"
"""Module Version"""
## SVN Code Revision
__revision__ = "$Revision: 495 $"
""" SVN Code Revision"""


## Message Log
logger = logging.getLogger('clearpath.horizon')
"""Horizon Module Log"""
logger.setLevel(logging.DEBUG)
logger.addHandler(utils.NullLoggingHandler())
logger.propagate = True
logger.debug("Loading clearpath.horizon ...")    




################################################################################
# Horizon Interface


## @defgroup mobile Mobile Platform Frame of Reference
#  @ingroup doc
#  The reference frame fixed to mobile platforms is based on \b ISO \b 8855 
#  (“Vehicle Dynamics and Road-Holding Ability – Vocabulary”). The direction of 
#  the axes differ from those used for roll, pitch, and yaw in aircraft, and 
#  care should be taken to ensure that the data is being interpreted correctly.
#  @image html chassis.gif
#  @manonly
#
#      z
#      ^  y
#     -|-/------------- 
#    / |/             /|     Counter-Clockwise:
#   /  /----> x      / /     - around x = roll
#  /----------------/ / =>   - around y = pitch
#  |                |/       - around z = yaw
#  ------------------
#  @endmanonly


## Adaptor translating from a horizon function argument spec to send function parameters
#
#  A horizon command function has the form
#    command_type(self, paramter1 = default, paramter2 = default, handler = None)
#  And must call and sender function of the form
#    self._send(name = 'command_type', {'paramter1':value1, 'paramater2':value2}, handler)
#
#  CommandSender takes the arguments and keyword arguments passed to the horizon command, 
#  places them in the dictionary form expected by the sender function, then calls the 
#  sender function.
#
#  @param command_func   The function whose argument spec describes args and kargs
#  @param send_func_name The name of the send function to use; this must be an attribute of 
#                        command_func's 'self'
#  @param args           Formal arguments passed to command_func.  
#                        (Will contain command_func's self)
#  @param kargs          Keyword arguments passed to command_func
#
#  @pydoc
def command_sender(command_func, send_func_name, args, kargs):
    """Adaptor translating from horizon function argument spec to send function paramters"""

    func_argspec = inspect.getargspec(command_func)[0]
    func_defaults = inspect.getargspec(command_func)[3]

    # In addition to pulling in any special set-up logic, this also functions
    # as a way of validating that the decorator has been called with the correct
    # arguments.  If this succeeds, we know we can construct a full parameter list
    command_func(args, kargs)

    # We don't separate out the self argument in this function's argument list
    # because this causes the indices for func_argspec (which includes 'self') and
    # args (which wouldn't) to be one-off, which causes all sorts of ugly
    func_self = args[0];
    handler = None
    cmd_args = {}

    # Fetch formal arguments into dictionary, pulling names from the argspec
    for inx in range(1,len(args)): 
        if( (func_argspec[inx]=='handler')  ):
            # This is the handler argument, which is kept separate
            handler = args[inx];
        else:
            # The rest are the command parameters
            cmd_args.update( {func_argspec[inx]: args[inx]} )

    # Pull out the handler argument if it's in kargs
    if( kargs.has_key('handler') ):
        handler = kargs['handler']
        del kargs['handler']

    # Remaining keyword arguments can go straight into the parameter dict
    cmd_args.update(kargs)

    # Fill in default values for parameters which weren't provided in args or kargs
    # Since the defaults list is right-aligned with the args list, we need to count back
    for inx in range(-1,-len(func_argspec),-1):  # Intend to miss 1st arg, which is self
        if not (func_argspec[inx]=='handler') and not cmd_args.has_key( func_argspec[inx] ) :
            if( (func_defaults == None) or (inx < -len(func_defaults) ) ):
                print 'cmd_name', ' is missing a parameter: ', func_argspec[inx]
                return None
            cmd_args.update( {func_argspec[inx]: func_defaults[inx]} )

    sender = getattr(func_self, send_func_name)
    return sender(command_func.__name__, cmd_args, handler)



## Decorator for horizon commands which use '_send_command' as a sender function
#
#  @pydoc
def api_command(func):
    """Decorator for horizon commands which use '_send_command' as a sender function"""

    def wrapper_func(*args, **kargs):
        return command_sender(func, '_send_command', args, kargs)

    wrapper_func.__doc__ = func.__doc__
    return wrapper_func



## Decorator for horizon commands which use '_send_request' as a sender function
#
#  @pydoc
def api_request(func):
    """Decorator for horizon commands which use '_send_request' as a sender function"""

    def wrapper_func(*args, **kargs):
        return command_sender(func, '_send_request', args, kargs);

    wrapper_func.__doc__ = func.__doc__
    return wrapper_func


## Horizon Interface.
#
#  Interface for interaction over a transport using the Horizon protocol.
#  Contains direct 1-1 function mappings for commands and requests.
#  Does not care about Chasis configuration: allows access to all functions
#  regardless of danger.
#
#  @note  Names used for a function directly match the names used within 
#         HORIZON_CODES.
#  @since 0.1
#
#  @section Platform
#  @copydoc mobile
#
#  @pydoc
class Horizon():
    """Horizon Interface."""
    
    version = (1, 1)

    ## Create a Horizon Interface.
    #
    #  Constructor for the Horizon Interface class.                           
    #  Performs the initial creation and initialization of the underlying 
    #  protocol and transport through instantiation.                          
    #  Supports version auto-detection (will take place in open) by sending
    #  a request for the platform information message.
    #
    #  @param  retries        The number of times to retry sending a message
    #                         that received a timeout or checksum error
    #  @param  rec_timeout    The time to wait for a data message after
    #                         sending a request, 0 - wait indefinitely
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
    #  @see    clearpath.horizon.protocol.HorizonProtocol_Client.__init__
    #  @throws LookupError    If auto-detect version fails
    #  @throws TransportError Upon transport creation/initialization failure
    #  @throws ValueError     From bad arguments
    #
    #  @pydoc
    def __init__(self, transport = transports.HorizonTransport_Serial.autodetect, 
                 transport_args = {}, rec_timeout = 5000, retries = 5, 
                 sys_time = False, send_timeout = 1000, store_timeout = 10000):
        """Create a Horizon Interface."""
        logger.debug("%s: Instance creation started..." % \
                     self.__class__.__name__)
        
        # Class Variables
        self._args = {}
        version = tuple([-1,0])
        self._auto = version[0] == 0 and version[1] == 0
        self._handlers = {}     # Format: name:[tuple([handler,wrapper])]
        self._protocol = None
        self._received = {}     # Format: class:payload
        self._rec_timeout = rec_timeout

        
        # Verify timeout
        if rec_timeout < 0:
            logger.error("%s: Invalid rec timeout!" % self.__class__.__name__)
            raise ValueError ("Invalid rec timeout!")
        logger.info("%s: Using %d ms rec timeout." % (self.__class__.__name__,
                                                  self._rec_timeout))
        
        # Setup Protocol
        self._args['transport'] = transport
        self._args['transport_args'] = transport_args
        self._args['retries'] = retries
        self._args['sys_time'] = sys_time
        self._args['send_timeout'] = send_timeout
        self._args['store_timeout'] = store_timeout
        self._args['version'] = self.version
        

        # Create the protocol
        self._protocol = protocol.HorizonProtocol_Client(**self._args)
        def tmp(code, payload, timestamp):
            name = codes.HORIZON_CODES[code]
            if self.message_received(name, payload, timestamp) == False:
                protocol.HorizonProtocol_Client.message_received(self._protocol, 
                                                                 code, payload,
                                                                 timestamp)
        self._protocol.message_received = tmp
    
        logger.debug("%s: ...instance creation complete." % \
                     self.__class__.__name__)
                
        
    ## String Representation
    def __str__(self):
        return str(self._protocol)


    ## Destroy a Horizon Interface.
    #
    #  Horizon Interface class destructor.                                    
    #  Cleans up the connection if operational by calling close() and
    #  cleans-up anything else.
    #
    def __del__(self):
        logger.debug("%s: Instance destruction started..." % \
                     self.__class__.__name__)
        
        # Close Connection
        self.close()
        
        logger.debug("%s: ...instance destruction complete." % 
                     self.__class__.__name__)
        
        
    ## Data Receive Handler
    #
    #  Temporary handler for receiving 1-off data.
    #
    #  @param name      The message name
    #  @param payload   The received payload.
    #  @param timestamp The time received.
    #
    def _receive_data(self, name, payload, timestamp):        
        self._received[payload.__class__] = payload
        
           
    ## Open the Connection.
    #
    #  Opens the connection so that Horizon can start to be used.
    #
    #  @throws LookupError    If auto-detect version fails
    #  @throws TransportError upon open failure
    #
    def open(self):
        if not self._protocol.is_open():
            logger.debug("%s: Device opening started..." % \
                     self.__class__.__name__)
        
            # Open Connection
            self._protocol.open()
            
            # Version Auto-detection
            if self._auto:
                # Get version from device
                firm = self.request_firmware_info()
                self.version = firm.version

            logger.debug("%s: ...device opening complete." % \
                             self.__class__.__name__)

        
        
    ## Close the Connection.
    #
    #  Closes the connection if it was opened.
    #
    def close(self):
        logger.debug("%s: Device closing started..." % \
                     self.__class__.__name__)
        
        # Close Connection
        if self._protocol != None and self._protocol.opened:
            self._protocol.close()
        logger.debug("%s: ...device closing complete." % \
                     self.__class__.__name__)
                
                
    #-------------------------------- Safeties ---------------------------------
                
    
    ## Emergency Stop
    #
    #  Sends the emergency stop signal.
    #
    #  @pydoc
    def emergency_stop(self):
        """syntax: emergency_stop
           -- Sends the emergency stop signal."""
        
        try:
            self.set_safety_status(codes.PAYLOAD_MAP['set_safety_status']
                                   .EMERGENCY_STOP)
        except Exception:
            pass
      
    ## Emergency Stop
    #
    #  Shortcut to emergency_stop
    #
    #  @pydoc
    def estop(self):
        """syntax: estop
           -- Shortcut to emergency_stop."""
        
        self.emergency_stop()
            
    
    ## Reset
    #
    #  Sends the reset signal.
    #
    #  @pydoc
    def reset(self):
        """syntax: reset
           -- Sends the reset signal."""
        
        try:
            self.set_reset()
        except Exception:
            pass



                
                
    #-------------------------------- Commands ---------------------------------
    
    ## Command - SET PLATFORM INFO
    #
    #  Clearpath-internal only.
    #
    #  @pydoc
    def set_platform_info(self, passcode = 0, model = '', revision = 0, serial = 0, handler = None):
        """syntax: set_platform_info <model> <revision> <serial> """
        
        args = { 'passcode': passcode, 'model': model, 'revision': revision, 'serial': serial }

        self._send_command(name = 'set_platform_info', 
                           args = args, handler = handler)


    
    ## Command - SET PLATFORM NAME
    #
    #  Sets a human-readable name for the platform.                           
    #  The name must be at most 63 characters long and must be NULL terminated
    #  (thus max 64 characters).
    #
    #  @pydoc
    def set_platform_name(self, name = 'Clearpath1', handler = None):
        """syntax: set_platform_name <name> 
           -- Sets a human-readable name for the platform."""
           
        args = {'name':name}
        self._send_command(name = 'set_platform_name', 
                           args = args, handler = handler)
        
    
    ## Command Code: SET PLATFORM TIME
    #
    #  Sets the platform's internal clock.                                    
    #  The time must be at most 50 days.
    #
    #  @pydoc
    def set_platform_time(self, time = 0, handler = None):
        """syntax: set_platform_time <time> 
           -- Sets the platform's internal clock."""
  
        args = {'time': time}
        self._send_command(name = 'set_platform_time', 
                           args = args, handler = handler)

    
    ## Command Code: SET SAFETY SYSTEM
    #
    #  Sets the platform's safety system flags.
    #
    #  @pydoc
    def set_safety_status(self, flags = 0x0000, handler = None):
        """syntax: set_safety_status <flags> 
           -- Sets the platform's safety system flags."""
        
        # Create Arguments
        args = {'flags':flags}
        
        # Send Message
        self._send_command(name = 'set_safety_status', 
                           args = args, handler = handler)
        

    ## Command Code: SET DIFFERENTIAL SPEED
    #
    #  Sets the platform's independent wheel speeds.
    #
    #  @pydoc
    def set_differential_speed(self, left_speed = 0.0, right_speed = 0.0, 
                               left_accel = 0.0, right_accel = 0.0, handler = None):
        """syntax: set_differential_speed <lspeed> <rspeed> <laccel>
                   <raccel>-- Sets the platform's independent wheel speeds."""

        # Create Arguments
        args = {'left_speed':left_speed, 'right_speed':right_speed,
                'left_accel':left_accel, 'right_accel':right_accel}
        
        # Send Message
        self._send_command(name = 'set_differential_speed', 
                           args = args, handler = handler)

    
    ## Command Code: SET DIFFERENTIAL CONTROL
    #
    #  Sets the platform's differential control constants.
    #
    #  @pydoc
    def set_differential_control(self, l_p = 0.0, l_i = 0.0, l_d = 0.0, 
                          l_feed = 0.0, l_stiction = 0.0, l_limit = 0.0, 
                          r_p = 0.0, r_i = 0.0, r_d = 0.0, r_feed = 0.0, 
                          r_stiction = 0.0, r_limit = 0.0, handler = None):
        """syntax: set_differential_control <lP> <lI> <lD> <lfeed>
                   <lstiction> <llimit> <rP> <rI> <rD> <rfeed> <rstiction> 
                   <rlimit>
                   -- Sets the platform's differential control constants."""
        
        # Create Arguments
        args = {'l_p':l_p, 'l_i':l_i, 'l_d':l_d, 'l_feed':l_feed, 
                'l_stiction':l_stiction, 'l_limit':l_limit, 'r_p':r_p, 
                'r_i':r_i, 'r_d':r_d, 'r_feed':r_feed, 'r_stiction':r_stiction, 
                'r_limit':r_limit}
        
        # Send Message
        self._send_command(name = 'set_differential_control', 
                           args = args, handler = handler)
    

    ## Command Code: SET DIFFERENTIAL OUTPUT
    #
    #  Sets the platform's differential motor outputs.
    #
    #  @pydoc
    def set_differential_output(self, left = 0.0, right = 0.0, handler = None):
        """syntax: set_differential_outputs <left> <right> 
           -- Sets the platform's differential motor outputs."""
           
        # Create Arguments
        args = {'left':left, 'right':right}
        
        # Send Message
        self._send_command(name = 'set_differential_output', 
                           args = args, handler = handler)

  
    ## Command Code: SET ACKERMANN OUTPUT
    #
    #  Sets the platform's Ackermann servo positions.
    #
    #  @pydoc
    def set_ackermann_output(self, steering = 0.0, throttle = 0.0, brake = 0.0, 
                             handler = None):
        """syntax: set_ackermann_output <steering> <throttle> <brake> 
           -- Sets the platform's Ackermann servo positions."""
       
        # Create Arguments
        args = {'steering':steering, 'throttle':throttle, 'brake':brake}
        
        # Send Message
        self._send_command(name = 'set_ackermann_output', 
                           args = args, handler = handler)

    ## Command Code: SET VELOCITY
    #
    #  Sets the platform's target velocity.
    #
    #  @pydoc
    def set_velocity(self, trans = 0.0, rot = 0.0, accel = 0.0, 
                          handler = None):
        """syntax: set_velocity <trans> <rot> <accel>
           -- Sets the platform's translational speed (trans), 
           rotational speed (rot) and translational acceleration (accel)."""
  
        # Create Arguments
        args = {'trans':trans, 'rot':rot, 'accel':accel}
        
        # Send Message
        self._send_command(name = 'set_velocity', 
                           args = args, handler = handler)
    
    
    ## Command Code: SET TURN
    #
    #  Sets the platform's target turn.
    #
    #  @pydoc
    def set_turn(self, trans = 0.0, turn = 0.0, accel = 0.0, 
                          handler = None):
        """syntax: set_turn <trans> <turn> <accel>
           -- Sets the platform's translational speed (trans), 
           turn radius (turn) and translational acceleration (accel)."""
   
        # Create Arguments
        args = {'trans':trans, 'turn':turn, 'accel':accel}
        
        # Send Message
        self._send_command(name = 'set_turn', 
                           args = args, handler = handler)
        
        
    ## Command Code: SET MAX SPEED 
    #
    #  Sets the platform's maximum translational speed.
    #
    #  @pydoc
    def set_max_speed(self, forward = 0.0, reverse = 0.0, handler = None):
        """syntax: set_max_speed <forward> <reverse>
           -- Sets the platform's maximum translational speeds."""
    
        # Create Arguments
        args = {'forward':forward, 'reverse':reverse}
        
        # Send Message
        self._send_command(name = 'set_max_speed', 
                           args = args, handler = handler)
        
        
    ## Command Code: SET MAX ACCELERATION
    #
    #  Sets the platform's maximum translational accelerations.
    #
    #  @pydoc
    def set_max_acceleration(self, forward = 0.0, reverse = 0.0, handler =None):
        """syntax: set_max_acceleration <forward> <reverse>
           -- Sets the platform's maximum translational accelerations."""
        
        # Create Arguments
        args = {'forward':forward, 'reverse':reverse}
        
        # Send Message
        self._send_command(name = 'set_max_acceleration', 
                           args = args, handler = handler)

    
    ## Command Code: SET GEAR
    #
    #  Sets the platform's transmission gear.
    #
    #  @pydoc
    def set_gear(self, gear = 0, handler = None):
        """syntax: set_gear <gear>
           -- Sets the platform's transmission gear."""
        
        args = {'gear':gear}
        self._send_command(name = 'set_gear', 
                           args = args, handler = handler)
    
    
    ## Command Code: SET GPADC OUTPUT
    #
    #  Sets the value of the GPADC Output Channels.
    #
    #  @pydoc
    def set_gpadc_output(self, values = {0:0}, handler = None):
        """syntax: set_gpadc_output <value> [<value> ...]
           -- Sets the value of the GPADC Output Channels.
              <value> = channel:output"""
        
        # Create Arguments
        args = {'values':values}
        
        # Send Message
        self._send_command(name = 'set_gpadc_output', 
                           args = args, handler = handler)
    
    ## Command Code: SET GPIO DIRECTION
    #
    #  Sets the directions of the GPIO Channels.
    #
    #  @pydoc
    def set_gpio_direction(self, mask = 0, direction = 0, handler = None):
        """syntax: set_gpio_direction <mask> <direction>
           -- Sets the directions of the GPIO Channels."""
        
        # Create Arguments
        args = {'mask':mask, 'direction':direction}
        
        # Send Message
        self._send_command(name = 'set_gpio_direction', 
                           args = args, handler = handler)
    
    
    ## Command Code: SET GPIO OUTPUT
    #
    #  Sets the outputs of the GPIO Channels.
    #
    #  @pydoc
    def set_gpio_output(self, mask = 0, output = 0, handler = None):
        """syntax: set_gpio_output <mask> <output>
           -- Sets the outputs of the GPIO Channels."""
        
        # Create Arguments
        args = {'mask':mask, 'output':output}
        
        # Send Message
        self._send_command(name = 'set_gpio_output', 
                           args = args, handler = handler)
    
    
    ## Command Code: SET PAN/TILT/ZOOM
    #
    #  Sets the camera mount's pan, tilt, and zoom positions.
    #
    #  @pydoc
    def set_pan_tilt_zoom(self, mount = 0, pan = 0.0, tilt = 0.0, 
                                   zoom = 1.0, handler = None):
        """syntax: set_pan_tilt_zoom <mount> <pan> <tilt> <zoom>
           -- Sets the position of the camera mount."""
        
  
        # Create Arguments
        args = {'mount':mount, 'pan':pan, 'tilt':tilt, 'zoom':zoom}
        
        # Send Message
        self._send_command(name = 'set_pan_tilt_zoom', 
                           args = args, handler = handler)
    
    
    ## Command Code: SET ABSOLUTE JOINT POSITION
    #
    #  Sets the angles of the joints.
    #
    #  @pydoc
    def set_absolute_joint_position(self, angles = {0:0.0}, handler = None):
        """syntax: set_absolute_joint_position <angle> [<angle> ...]
           -- Sets the angles of the joints.
              <angle> = joint:angle"""
        
        # Create Arguments
        args = {'angles':angles}
        
        # Send Message
        self._send_command(name = 'set_absolute_joint_position', 
                           args = args, handler = handler)
    
    
    ## Command Code: SET RELATIVE JOINT POSITION
    #
    #  Sets the angles of the joints.
    #
    #  @pydoc
    def set_relative_joint_position(self, angles = {0:0.0}, handler = None):
        """syntax: set_relative_joint_position <angle> [<angle> ...]
           -- Sets the angles of the joints.
           <angle> = joint:angle"""
        
        # Create Arguments
        args = {'angles':angles}
        
        # Send Message
        self._send_command(name = 'set_relative_joint_position', 
                           args = args, handler = handler)
    
    
    ## Command Code: SET JOINT CONTROL
    #
    #  Sets the joint's control constants.
    #
    #  @pydoc
    def set_joint_control(self, joint = 0, p = 0.0, i = 0.0, d = 0.0, 
                          feed = 0.0, stiction = 0.0, limit = 0.0, 
                          handler = None):
        """syntax: set_joint_control <joint> <P> <I> <D> <feed> 
           <stiction> <limit> 
           -- Sets the joint's control constants."""
        
        # Create Arguments
        args = {'joint':joint, 'p':p, 'i':i, 'd':d, 'feed':feed, 
                'stiction':stiction, 'limit':limit}
        
        # Send Message
        self._send_command(name = 'set_joint_control', 
                           args = args, handler = handler)
    
    
    ## Command Code: SET JOINT HOMING
    #
    #  Run a joint's homing sequence.
    #
    #  @pydoc
    def set_joint_homing(self, joint = 0, handler = None):
        """syntax: set_joint_homing <joint>
           -- Run a joint's homing sequence."""
        
        # Create Arguments
        args = {'joint':joint}
        
        # Send Message
        self._send_command(name = 'set_joint_homing', 
                           args = args, handler = handler)
    
    
    ## Command Code: SET END EFFECTOR POSITION
    #
    #  Sets the end effector's position.
    #
    #  @pydoc
    def set_end_effector_position(self, x = 0.0, y = 0.0, z = 0.0, handler = None):
        """syntax: set_end_effector_position <x> <y> <x> 
           -- Sets the end effector's position."""
           
        # Create Arguments
        args = {'x':x, 'y':y, 'z':z}
        
        # Send Message
        self._send_command(name = 'set_end_effector_position', 
                           args = args, handler = handler)
    

    ## Command Code: SET END EFFECTOR POSE
    #
    #  Sets the end effector's position and orientation.
    #
    #  @pydoc
    def set_end_effector_pose(self, x = 0.0, y = 0.0, z = 0.0, roll = 0.0, 
                                   pitch = 0.0, yaw = 0.0, handler = None):
        """syntax: set_end_effector_pose <x> <y> <x> <roll> <pitch> <yaw> 
           -- Sets the end effector's position and orientation."""
        
        # Create Arguments
        args = {'x':x, 'y':y, 'z':z, 'roll':roll, 'pitch':pitch, 'yaw':yaw}
        
        # Send Message
        self._send_command(name = 'set_end_effector_pose', 
                           args = args, handler = handler)
    
    
    ## Command Code: SET RESET
    #
    #  Resets the control processor.
    #
    #  @pydoc
    def set_reset(self, handler = None):
        """syntax: set_reset
           -- Resets the control processor."""
        
        self._send_command(name = 'set_reset', args = {}, handler = handler)


    ## Command Code: RESTORE SYSTEM CONFIGURATION
    #
    #  Restores the saved system configuration. Use flags = 0x2 to restore from factory defaults.
    #
    #  @pydoc
    def restore_system_config(self, passcode = 0x3A18, flags = 0x1, handler = None):
        """syntax: restore_system_config <passcode> <flags>"""
        args = { 'flags': flags, 'passcode': passcode }
        self._send_command(name = 'restore_system_config', 
                           args = args, handler = handler)


    ## Command Code: STORE SYSTEM CONFIGURATION
    #
    #  Save current settings in the stored system configuration memory.
    #
    #  @pydoc
    def store_system_config(self, passcode = 0x3A18, handler = None):
        """syntax: store_system_config <passcode>"""
        args = { 'passcode': passcode }
        self._send_command(name = 'store_system_config', 
                           args = args, handler = handler)


    ## Command Code: SET CURRENT SENSOR CALIBRATION
    #
    #  Set the offsets and scales for the current sensor.
    #
    #  @pydoc
    def set_current_sensor_config(self, passcode = 0, offsets = [ 0.0 ], scales = [ 0.0 ], handler = None):
        """Not usable from command-line interface."""
        args = { 'passcode': passcode, 'offsets': offsets, 'scales': scales }
        self._send_command(name = 'set_current_sensor_config',
                           args = args, handler = handler)


    ## Command Code: SET VOLTAGE SENSOR CALIBRATION
    #
    #  Set the offsets and scales for the voltage sensor.
    #
    #  @pydoc
    def set_voltage_sensor_config(self, passcode = 0, offsets = [ 0.0 ], scales = [ 0.0 ], handler = None):
        """Not usable from command-line interface."""
        args = { 'passcode': passcode, 'offsets': offsets, 'scales': scales }
        self._send_command(name = 'set_voltage_sensor_config',
                           args = args, handler = handler)


    ## Command Code: SET TEMPERATURE SENSOR CALIBRATION
    #
    #  Set the offsets and scales for the temperature sensor.
    #
    #  @pydoc
    def set_temperature_sensor_config(self, passcode = 0, offsets = [ 0.0 ], scales = [ 0.0 ], handler = None):
        """Not usable from command-line interface."""
        args = { 'passcode': passcode, 'offsets': offsets, 'scales': scales }
        self._send_command(name = 'set_temperature_sensor_config',
                            args = args, handler = handler)


    ## Command Code: SET ORIENTATION SENSOR CALIBRATION
    #
    #  Set the offsets and scales for the temperature sensor.
    #
    #  @pydoc
    def set_orientation_sensor_config(self, passcode = 0,
                                      roll_offset = 0.0, roll_scale = 0.0,
                                      pitch_offset = 0.0, pitch_scale = 0.0,
                                      yaw_offset = 0.0, yaw_scale = 0.0, handler = None):
        """Not usable from command-line interface."""
        args = { 'passcode': passcode,
                 'roll_offset': roll_offset, 'roll_scale': roll_scale,
                 'pitch_offset': pitch_offset, 'pitch_scale': pitch_scale,
                 'yaw_offset': yaw_offset, 'yaw_scale': yaw_scale}
        self._send_command(name = 'set_orientation_sensor_config',
                           args = args, handler = handler)


    ## Command Code: SET MAGNETOMETER CALIBRATION
    #
    #  Set the offsets and scales for the magnetometer.
    #
    #  @pydoc
    def set_magnetometer_config(self, passcode = 0,
                                      x_offset = 0.0, x_scale = 0.0,
                                      y_offset = 0.0, y_scale = 0.0,
                                      z_offset = 0.0, z_scale = 0.0, handler = None):
        """Not usable from command-line interface."""
        args = { 'passcode': passcode,
                 'x_offset': x_offset, 'x_scale': x_scale,
                 'y_offset': y_offset, 'y_scale': y_scale,
                 'z_offset': z_offset, 'z_scale': z_scale}
        self._send_command(name = 'set_magnetometer_config',
                           args = args, handler = handler)


    ## Command Code: SET GYRO CALIBRATION
    #
    #  Set the offsets and scales for the gyroscope.
    #
    #  @pydoc
    def set_gyro_config(self, passcode = 0,
                        roll_offset = 0.0, roll_scale = 0.0,
                        pitch_offset = 0.0, pitch_scale = 0.0,
                        yaw_offset = 0.0, yaw_scale = 0.0, handler = None):
        """Not usable from command-line interface."""
        args = { 'passcode': passcode,
                 'roll_offset': roll_offset, 'roll_scale': roll_scale,
                 'pitch_offset': pitch_offset, 'pitch_scale': pitch_scale,
                 'yaw_offset': yaw_offset, 'yaw_scale': yaw_scale}
        self._send_command(name = 'set_gyro_config',
                           args = args, handler = handler)


    ## Command Code: SET ACCELEROMETER CALIBRATION
    #
    #  Set the offsets and scales for the accelerometer.
    #
    #  @pydoc
    def set_accelerometer_config(self, passcode = 0,
                                 x_offset = 0.0, x_scale = 0.0,
                                 y_offset = 0.0, y_scale = 0.0,
                                 z_offset = 0.0, z_scale = 0.0, handler = None):
        """Not usable from command-line interface."""
        args = { 'passcode': passcode,
                 'x_offset': x_offset, 'x_scale': x_scale,
                 'y_offset': y_offset, 'y_scale': y_scale,
                 'z_offset': z_offset, 'z_scale': z_scale}
        self._send_command(name = 'set_accelerometer_config',
                           args = args, handler = handler)


    ## Send a Command
    #
    #  Sends the command to the device.                                       
    #  Synchronous:  Blocks until acknowledgment received or timeout.         
    #  Asynchronous: Immediately returns and has handler called upon ack.
    #
    #  @param  args                   Arguments to create the payload with.
    #  @param  handler                The command completion handler.
    #                                 Must have one parameter: Exception.
    #                                 If None then synchronous method is used.
    #  @param  name                   The command name (as in HORIZON_NAMES).
    #  @throws FormatError            If acknowledgment says bad payload.
    #  @throws IOError                If not opened
    #  @throws SubscriptionError      If acknowledgment says bad subscription.
    #  @throws TimeoutError           If timeout occurred
    #  @throws TransportError         upon send failure
    #  @throws UnsupportedCodeError   If acknowledgment says bad code.
    #  @throws ValueError             If message creation fails or 
    #                                 if acknowledgment says bad values.
    #
    def _send_command(self, name, args, handler = None):
        if not self._protocol.opened:
            self.close()
            raise IOError ("Horizon has not been opened!")
        
        # Create Payload & find code
        args['version'] = self.version
        payload = codes.PAYLOAD_MAP[name](**args)
        code = codes.find_code(name)
            
        # Send the Message
        try:
            self._protocol.send_message(code = code, payload = payload,
                                             handler = handler)
        except utils.TransportError as ex:
            if handler != None:
                handler(utils.TransportError(ex))
        except ValueError as ex:
            if handler != None:
                handler(ValueError(ex))
                
    
    #-------------------------------- Requests ---------------------------------

    ## Request Code: REQUEST ECHO  
    #
    #  Ping the platform.
    #
    #  @pydoc
    def request_echo(self, subscription = 0, handler = None):
        """syntax: request_echo
           -- Ping the platform."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_echo', 
                                  handler = handler, args = args)
        

    ## Request Code: REQUEST PLATFORM INFORMATION
    #
    #  Get the platform's information.
    #
    #  @pydoc
    def request_platform_info(self, subscription = 0, handler = None):
        """syntax: request_platform_information
           -- Get the platform's information."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_platform_info', 
                                  handler = handler, args = args)
    
  
    ## Request Code: REQUEST PLATFORM NAME
    #
    #  Get the human-readable name for the platform.
    #
    #  @pydoc
    def request_platform_name(self, subscription = 0, handler = None):
        """syntax: request_platform_name
           -- Get the human-readable name for the platform."""

        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_platform_name', 
                                  handler = handler, args = args)
        
        
    ## Request Code: REQUEST FIRMWARE INFORMATION
    #
    #  Get the firmware information.
    #
    #  @pydoc
    def request_firmware_info(self, subscription = 0, handler = None):
        """syntax: request_firmware_information
           -- Get the firmware information."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_firmware_info', 
                                  handler = handler, args = args)
    
    
    ## Request Code: REQUEST SYSTEM STATUS
    #
    #  Get the platform status data.
    #
    #  @pydoc
    def request_system_status(self, subscription = 0, handler = None):
        """syntax: request_system_status
           -- Get the platform status data."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_system_status', 
                                  handler = handler, args = args)
        

    ## Request Code: REQUEST PROCESSOR STATUS
    #
    #  Get the processor status data.
    #
    #  @pydoc
    def request_processor_status(self, subscription = 0, handler = None):
        """syntax: request_processor_status
           -- Get the processor status data."""
        
        # Create Arguments
        args = {'subscription': subscription}
        
        # Send the Request
        return self._send_request(name = 'request_processor_status', 
                                  handler = handler, args = args)


    ## Request Code: REQUEST POWER STATUS
    #
    #  Get the platform power status data.
    #
    #  @pydoc
    def request_power_status(self, subscription = 0, handler = None):
        """syntax: request_power_status
           -- Get the platform's power status."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_power_status', 
                                  handler = handler, args = args)
        
        
    ## Request Code: REQUEST SAFETY SYSTEM
    #
    #  Get the platform safety status data.
    #
    #  @pydoc
    def request_safety_status(self, subscription = 0, handler = None):
        """syntax: request_safety_status
           -- Get the platform's safety system status."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_safety_status', 
                                  handler = handler, args = args)
        
        
    ## Request Code: REQUEST DIFFERENTIAL SPEED
    #
    #  Get the platform's wheel speeds.
    #
    #  @pydoc
    def request_differential_speed(self, subscription = 0, handler = None):
        """syntax: request_differential_speed
           -- Get the platform's wheel speeds."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_differential_speed', 
                                  handler = handler, args = args)
        
        
    ## Request Code: REQUEST DIFFERENTIAL CONTROL
    #
    #  Get the platform's differential control constants.
    #
    #  @pydoc
    def request_differential_control(self, subscription = 0, handler = None):
        """syntax: request_differential_control
           -- Get the platform's differential control constants."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_differential_control', 
                                  handler = handler, args = args)
        
    ## Request Code: REQUEST DIFFERENTIAL OUTPUT
    #
    #  Get the platform's raw motor output.
    #
    #  @pydoc
    def request_differential_output(self, subscription = 0, handler = None):
        """syntax: request_differential_motors
           -- Get the platform's raw motor output."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_differential_output', 
                                  handler = handler, args = args)
        
    
    ## Request Code: REQUEST ACKERMANN SETPOINT
    #
    #  Get the platform's Ackermann servo position setpoints.
    #
    #  @pydoc
    def request_ackermann_output(self, subscription = 0, handler = None):
        """syntax: request_ackermann_output
           -- Get the platform's Ackermann servo positions."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_ackermann_output', 
                                  handler = handler, args = args)
        
    
    
    ## Request Code: REQUEST VELOCITY 
    #
    #  Get the target translational speed, rotational speed, and 
    #  translational acceleration.
    #
    #  @pydoc
    def request_velocity(self, subscription = 0, handler = None):
        """syntax: request_velocity
           -- Get the target translational speed, rotational speed,"""\
        """ and translational acceleration."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_velocity', 
                                  handler = handler, args = args)
        
        
    ## Request Code: REQUEST TURN
    #
    #  Get the target translational speed, turn radius, and 
    #  translational acceleration.
    #
    #  @pydoc
    def request_turn(self, subscription = 0, handler = None):
        """syntax: request_turn
           -- Get the target translational speed, turn radius,"""\
        """ and translational acceleration."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_turn', 
                                  handler = handler, args = args)
        
        
    ## Request Code: REQUEST MAX SPEED
    #
    #  Get the platform's maximum translational speeds.
    #
    #  @pydoc
    def request_max_speed(self, subscription = 0, handler = None):
        """syntax: request_max_speed
           -- Get the platform's maximum translational speeds."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_max_speed', 
                                  handler = handler, args = args)
        
    ## Request Code: REQUEST MAX ACCELERATION
    #
    #  Get the platform's maximum translational accelerations.
    #
    #  @pydoc
    def request_max_acceleration(self, subscription = 0, handler = None):
        """syntax: request_max_acceleration
           -- Get the platform's maximum translational accelerations."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_max_acceleration', 
                                  handler = handler, args = args)
        
    ## Request Code: REQUEST GEAR STATUS
    #
    #  Get the platform's transmission target gear and action.
    #
    #  @pydoc
    def request_gear_status(self, subscription = 0, handler = None):
        """syntax: request_gear_status
           -- Get the platform's transmission target gear and action."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_gear_status', 
                                  handler = handler, args = args)
    
    
    ## Request Code: REQUEST GPADC OUTPUT
    #
    #  Get the value of the GPADC output channels.
    #
    #  @pydoc
    def request_gpadc_output(self, subscription = 0, channel=0, handler = None):
        """syntax: request_gpadc_output
           -- Get the value of the GPADC output channels."""
        
        # Create Arguments
        args = {'subscription':subscription, 'channel':channel}
        
        # Send the Request
        return self._send_request(name = 'request_gpadc_output', 
                                  handler = handler, args = args)
        
        
    ## Request Code: REQUEST GPIO
    #
    #  Get the values and directions of the GPIO channels.
    #
    #  @pydoc
    def request_gpio(self, subscription = 0, handler = None):
        """syntax: request_gpio
           -- Get the values and directions of the GPIO channels."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_gpio', 
                                  handler = handler, args = args)
    
    
    ## Request Code: REQUEST GPADC INPUT
    #
    #  Get the value of the GPADC input channels.
    #
    #  @pydoc
    def request_gpadc_input(self, subscription = 0, handler = None):
        """syntax: request_gpadc_input
           -- Get the value of the GPADC input channels."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_gpadc_input', 
                                  handler = handler, args = args)
        

    ## Request Code: REQUEST PAN/TILT/ZOOM
    #
    #  Get the position of the camera mount.
    #
    #  @pydoc
    def request_pan_tilt_zoom(self, mount = 0, subscription = 0, 
                                       handler = None):
        """syntax: request_pan_tilt_zoom <mount>
           -- Get the position of the camera mount."""
        
        # Create Arguments
        args = {'subscription':subscription, 'mount':mount}
        
        # Send the Request
        return self._send_request(name = 'request_pan_tilt_zoom', 
                                  handler = handler, args = args)
        
        
    ## Request Code: REQUEST DISTANCE
    #
    #  Get the distance sensor values.
    #
    #  @pydoc
    def request_distance(self, subscription = 0, handler = None):
        """syntax: request_distance
           -- Get the distance sensor values."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_distance', 
                                  handler = handler, args = args)
        
        
    ## Request Code: REQUEST DISTANCE & TIMING
    #
    #  Get the vehicle's distance sensor values and times.
    #
    #  @pydoc
    def request_distance_timing(self, subscription = 0, handler = None):
        """syntax: request_distance_timing
           -- Get the vehicle's distance sensor values and times."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_distance_timing', 
                                  handler = handler, args = args)
    
    
    ## Request Code: REQUEST PLATFORM ORIENTATION
    #
    #  Get the vehicle's orientation.
    #
    #  @pydoc
    def request_platform_orientation(self, subscription = 0, handler = None):
        """syntax: request_platform_orientation
           -- Get the vehicle's orientation."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_platform_orientation', 
                                  handler = handler, args = args)
        
        
    ## Request Code: REQUEST PLATFORM ROTATION
    #
    #  Get the vehicle's rotational rates.
    #
    #  @pydoc
    def request_platform_rotation(self, subscription = 0, handler =None):
        """syntax: request_platform_rotation
           -- Get the vehicle's rotational rates."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_platform_rotation', 
                                  handler = handler, args = args)
        
        
    ## Request Code: REQUEST PLATFORM ACCELERATION
    #
    #  Get the vehicle's acceleration.
    #
    #  @pydoc
    def request_platform_acceleration(self, subscription = 0, handler = None):
        """syntax: request_platform_acceleration
           -- Get the vehicle's acceleration."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_platform_acceleration', 
                                  handler = handler, args = args)
        
        
    ## Request Code: REQUEST PLATFORM 6-AXIS
    #
    #  Get the vehicle's acceleration and rotation.
    #
    #  @pydoc
    def request_platform_6axis(self, subscription = 0, handler = None):
        """syntax: request_platform_6axis
           -- Get the vehicle's acceleration and rotation."""
        
        # Create Arguments
        args = {'subscription': subscription}
        
        # Send the Request
        return self._send_request(name = 'request_platform_6axis', 
                                  handler = handler, args = args)
        
        
    ## Request Code: REQUEST PLATFORM 6-AXIS & ORIENTATION
    #
    #  Get the vehicle's acceleration, rotation and orientation.
    #
    #  @pydoc
    def request_platform_6axis_orientation(self, subscription = 0, handler = None):
        """syntax: request_platform_6axis_orientation
           -- Get the vehicle's acceleration, rotation, and orientation."""
        
        # Create Arguments
        args = {'subscription': subscription}
        
        # Send the Request
        return self._send_request(name = 'request_platform_6axis_orientation', 
                                  handler = handler, args = args)
    

    ## Request Code: REQUEST PLATFORM 6-AXIS & ORIENTATION
    #
    #  Get the vehicle's acceleration, rotation and orientation.
    #
    #  @pydoc
    def request_platform_magnetometer(self, subscription = 0, handler = None):
        """syntax: request_platform_magnetometer
           -- Get the vehicle's magnetometer data."""
        
        # Create Arguments
        args = {'subscription': subscription}
        
        # Send the Request 
        return self._send_request(name = 'request_platform_magnetometer', 
                                  handler = handler, args = args)


    ## Request Code: REQUEST ENCODERS
    #
    #  Get the distances and speeds of the encoders.
    #
    def request_encoders(self, subscription = 0, handler = None):
        """syntax: request_encoders
           -- Get the distances and speeds of the encoders."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_encoders', 
                                  handler = handler, args = args)
    
    
    ## Request Code: REQUEST RAW ENCODERS
    #
    #  Get the raw tick counts of the encoders.
    #
    #  @pydoc
    def request_raw_encoders(self, subscription = 0, handler = None):
        """syntax: request_raw_encoders
           -- Get the raw tick counts of the encoders."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_raw_encoders', 
                                  handler = handler, args = args)
        

    ## Request Code: REQUEST ABSOLUTE JOINT POSITION
    #
    #  Get the joints absolute positions.
    #
    #  @pydoc
    def request_absolute_joint_position(self, subscription = 0, handler = None):
        """syntax: request_absolute_joint_position
           -- Get the joints absolute positions."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_absolute_joint_position', 
                                  handler = handler, args = args)
    
    
    ## Request Code: REQUEST RELATIVE JOINT POSITION
    #
    #  Get the joints relative positions.
    #
    #  @pydoc
    def request_relative_joint_position(self, subscription = 0, handler = None):
        """syntax: request_relative_joint_position
           -- Get the joints relative positions."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_relative_joint_position', 
                                  handler = handler, args = args)
    
    
    ## Request Code: REQUEST JOINT CONTROL
    #
    #  Get the joint control constants.
    #
    #  @pydoc
    def request_joint_control(self, joint = 0, subscription = 0, handler= None):
        """syntax: request_joint_control <joint>
           -- Get the joint control constants."""
        
        # Create Arguments
        args = {'subscription':subscription, 'joint':joint}
        
        # Send the Request
        return self._send_request(name = 'request_joint_control', 
                                  handler = handler, args = args)
    
    
    ## Request Code: REQUEST JOINT HOMING STATUS
    #
    #  Get the joint homing status.
    #
    #  @pydoc
    def request_joint_homing_status(self, subscription = 0, handler = None):
        """syntax: request_joint_homing_status
           -- Get the joint homing status."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_joint_homing_status', 
                                  handler = handler, args = args)
    
    ## Request Code: REQUEST JOINT TORQUES
    #
    #  Get the joint torques.
    #
    #  @pydoc
    def request_joint_torques(self, subscription = 0, handler = None):
        """syntax: request_joint_torques
           -- Get the joint torques."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_joint_torques', 
                                  handler = handler, args = args)
    
    
    ## Request Code: REQUEST END EFFECTOR POSITION
    #
    #  Get the desired end effector position.
    #
    #  @pydoc
    def request_end_effector_position(self, subscription = 0, handler = None):
        """syntax: request_end_effector_position
           -- Get the desired end effector position."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_end_effector_position', 
                                  handler = handler, args = args)
    
    ## Request Code: REQUEST END EFFECTOR POSE
    #
    #  Get the desired end effector position and orientation.
    #
    #  @pydoc
    def request_end_effector_pose(self, subscription = 0, handler = None):
        """syntax: request_end_effector_pose
           -- Get the desired end effector position and orientation."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_end_effector_pose', 
                                  handler = handler, args = args)
    
    
    ## Request Code: REQUEST END EFFECTOR ORIENTATION
    #
    #  Get the current end effector position and orientation.
    #
    #  @pydoc
    def request_end_effector_orientation(self, subscription = 0, handler= None):
        """syntax: request_end_effector_orientation
           -- Get the current end effector position and orientation."""
        
        # Create Arguments
        args = {'subscription':subscription}
        
        # Send the Request
        return self._send_request(name = 'request_end_effector_orientation', 
                                  handler = handler, args = args)
        

    ## Request Code: REQUEST CURRENT SENSOR CONFIG
    #
    #  Get the current sensor calibration
    #
    #  @pydoc
    def request_current_sensor_config(self, subscription = 0, handler = None):
        """syntax: request_current_sensor_config"""
        return self._send_request(name = 'request_current_sensor_config', 
                                  handler = handler, args = {'subscription': subscription})


    ## Request Code: REQUEST VOLTAGE SENSOR CONFIG
    #
    #  Get the voltage sensor calibration
    #
    #  @pydoc
    def request_voltage_sensor_config(self, subscription = 0, handler = None):
        """syntax: request_voltage_sensor_config"""
        return self._send_request(name = 'request_voltage_sensor_config', 
                                  handler = handler, args = {'subscription': subscription})


    ## Request Code: REQUEST TEMPERATURE SENSOR CONFIG
    #
    #  Get the temperature sensor calibration
    #
    #  @pydoc
    def request_temperature_sensor_config(self, subscription = 0, handler = None):
        """syntax: request_temperature_sensor_config"""
        return self._send_request(name = 'request_temperature_sensor_config', 
                                  handler = handler, args = {'subscription': subscription})


    ## Request Code: REQUEST ORIENTATION SENSOR CONFIG
    #
    #  Get the orientation sensor calibration
    #
    #  @pydoc
    def request_orientation_sensor_config(self, subscription = 0, handler = None):
        """syntax: request_orientation_sensor_config"""
        return self._send_request(name = 'request_orientation_sensor_config', 
                                  handler = handler, args = {'subscription': subscription})


    ## Request Code: REQUEST MAGNETOMETER CONFIG
    #
    #  Get the magnetometer calibration
    #
    #  @pydoc
    def request_magnetometer_config(self, subscription = 0, handler = None):
        """syntax: request_magnetometer_config"""
        return self._send_request(name = 'request_magnetometer_config', 
                                  handler = handler, args = {'subscription': subscription})


    ## Request Code: REQUEST GYRO CONFIG
    #
    #  Get the gyro calibration
    #
    #  @pydoc
    def request_gyro_config(self, subscription = 0, handler = None):
        """syntax: request_gyro_config"""
        return self._send_request(name = 'request_gyro_config', 
                                  handler = handler, args = {'subscription': subscription})


    ## Request Code: REQUEST ACCELEROMETER CONFIG
    #
    #  Get the accelerometer calibration
    #
    #  @pydoc
    def request_accelerometer_config(self, subscription = 0, handler = None):
        """syntax: request_accelerometer_config"""
        return self._send_request(name = 'request_accelerometer_config', 
                                  handler = handler, args = {'subscription': subscription})


    ## Request Code: REQUEST RAW CURRENT SENSOR
    #
    #  Get the raw current sensor readings
    #
    #  @pydoc
    def request_raw_current_sensor(self, subscription = 0, handler = None):
        """syntax: request_raw_current_sensor"""
        return self._send_request(name = 'request_raw_current_sensor', 
                                  handler = handler, args = {'subscription': subscription})


    ## Request Code: REQUEST RAW VOLTAGE SENSOR
    #
    #  Get the raw voltage sensor readings
    #
    #  @pydoc
    def request_raw_voltage_sensor(self, subscription = 0, handler = None):
        """syntax: request_raw_voltage_sensor"""
        return self._send_request(name = 'request_raw_voltage_sensor', 
                                  handler = handler, args = {'subscription': subscription})


    ## Request Code: REQUEST RAW TEMPERATURE SENSOR
    #
    #  Get the raw temperature sensor readings
    #
    #  @pydoc
    def request_raw_temperature_sensor(self, subscription = 0, handler = None):
        """syntax: request_raw_temperature_sensor"""
        return self._send_request(name = 'request_raw_temperature_sensor', 
                                  handler = handler, args = {'subscription': subscription})


    ## Request Code: REQUEST RAW ORIENTATION SENSOR
    #
    #  Get the raw orientation sensor readings
    #
    #  @pydoc
    def request_raw_orientation_sensor(self, subscription = 0, handler = None):
        """syntax: request_raw_orientation_sensor"""
        return self._send_request(name = 'request_raw_orientation_sensor', 
                                  handler = handler, args = {'subscription': subscription})


    ## Request Code: REQUEST RAW MAGNETOMETER SENSOR
    #
    #  Get the raw current sensor readings
    #
    #  @pydoc
    def request_raw_magnetometer(self, subscription = 0, handler = None):
        """syntax: request_raw_magnetometer"""
        return self._send_request(name = 'request_raw_magnetometer', 
                                  handler = handler, args = {'subscription': subscription})


    ## Request Code: REQUEST RAW GYRO
    #
    #  Get the raw current sensor readings
    #
    #  @pydoc
    def request_raw_gyro(self, subscription = 0, handler = None):
        """syntax: request_raw_gyro"""
        return self._send_request(name = 'request_raw_gyro', 
                                  handler = handler, args = {'subscription': subscription})


    ## Request Code: REQUEST RAW ACCELEROMETER
    #
    #  Get the raw accelerometer readings
    #
    #  @pydoc
    def request_raw_accelerometer(self, subscription = 0, handler = None):
        """syntax: request_raw_accelerometer"""
        return self._send_request(name = 'request_raw_accelerometer', 
                                  handler = handler, args = {'subscription': subscription})



     
    ## Send a Request
    #
    #  Sends the request to the device.                                       
    #  Synchronous:  Blocks until acknowledgment and one response received or 
    #                timeout.                                                 
    #  Asynchronous: Immediately returns and has handler called upon ack.     
    #  Subscription: Rest of data received through handlers added with 
    #                'add_handler' or by calling 'get_waiting_data'.
    #
    #  @param  args                   Arguments to create the payload with.
    #  @param  handler                The command completion handler.
    #                                 Must have one parameter: Exception.
    #                                 If None then synchronous method is used.
    #  @param  name                   The command name (as in HORIZON_NAMES).
    #  @throws FormatError            If acknowledgment says bad payload.
    #  @throws IOError                If not opened
    #  @throws SubscriptionError      If acknowledgment says bad subscription.
    #  @throws TimeoutError           If timeout occurred
    #  @throws TransportError         upon send failure
    #  @throws UnsupportedCodeError   If acknowledgment says bad code.
    #  @throws ValueError             If message creation fails or 
    #                                 if acknowledgment says bad values.
    #
    def _send_request(self, name, args, handler = None):
        if not self._protocol.opened: 
            self.close()
            raise IOError ("Horizon has not been opened!")
        
        # Synchronous -> prepare 1-off handler
        if handler == None:
            self.add_handler(handler=self._receive_data,request=name)
            self._received[codes.PAYLOAD_MAP[codes.RESPONSE_MAP[name]]] = None
        
        # Send the Message
        self._send_command(name = name, args = args, handler = handler)
        
        # Wait for first response
        if handler == None and ('subscription' not in args or 
                ((self.version[0] > 0 or self.version[1] > 3) and 
                 args['subscription'] != 0xFFFF) or ((not(self.version[0] > 0 
                or self.version[1] > 3)) and args['subscription'] != 0xFF)):
            timestamp = 0
            t = datetime.datetime.today()
            timestamp = t.microsecond/1000 + t.second*1000 + \
                    t.minute*60*1000 + t.hour*60*60*1000 + t.day*24*60*60*1000
            time2 = timestamp
            received = None
            while ((time2 - timestamp <= self._rec_timeout) or\
                        (time2 < timestamp and 4294967295 - \
                         timestamp + time2 <= self._rec_timeout))  and \
                    received == None:
                t = datetime.datetime.today()
                time2 = t.microsecond/1000 + t.second*1000 + \
                    t.minute*60*1000 + t.hour*60*60*1000 + t.day*24*60*60*1000
                received = self._received[codes.PAYLOAD_MAP[
                                                    codes.RESPONSE_MAP[name]]]
                time.sleep(0.001)
            self.remove_handler(handler=self._receive_data,
                                request=name)
            self._received[codes.PAYLOAD_MAP[codes.RESPONSE_MAP[name]]] = None
            if received == None:
                raise utils.TimeoutError (
                                    "Timeout Occurred waiting for response!")
            elif received.error != None:
                raise received.error
            return received
            
            
    #------------------------------ Subscriptions ------------------------------
    
    
    ## Horizon Message Received
    #
    #  Handles receiving a message.                                           
    #  Cannot be blocking and cannot raise exceptions.                        
    #                                                                         
    #  Override this method in subclasses that handle messages.
    #
    #  @param  name       The message name
    #  @param  payload    The message payload received
    #  @param  timestamp  The time the message was received
    #  @return Was the payload handled?
    #
    def message_received(self, name, payload, timestamp):
        return False
    
    
    ## Add Subscription Data Handler
    #  
    #  Adds a data subscription handler to be called when data is received.   
    #  Asynchronous method of getting subscription Data.
    #
    #  @param  backtrack  Call the new handler with any waiting data?
    #  @param  request    The name of the request method used to create the 
    #                     subscription or the request code.
    #                     If None then it returns all subscription data.
    #  @param  handler    The Message Handler. 
    #                     Must have three parameters: 
    #                        str name,
    #                        HorizonPayload payload,
    #                        long timestamp
    #
    def add_handler(self, handler, backtrack = False, request = None):
        # Convert request method to message codes
        code = 0
        if request != None:
            code = codes.find_code(codes.RESPONSE_MAP[request])
            
        # Create Conversion Wrapper
        def tmp(code, payload, timestamp, handler = handler):
            handler(codes.HORIZON_CODES[code],payload,timestamp)
            
        # Add the Handler
        self._protocol.add_handler(handler = tmp, code = code, 
                                    backtrack = backtrack)
        if request not in self._handlers: self._handlers[request] = []
        self._handlers[request].append(tuple([handler,tmp]))
        logger.debug("%s: handler %s added for request %s" % \
                     (self.__class__.__name__, handler.__name__, request))
    
    
    ## Remove Subscription Data Handler
    #  
    #  Removes a data subscription handler.
    #
    #  @param  handler   The Handler to remove.
    #                    If None, remove all handlers. 
    #  @param  request   The name of the request method used to create the 
    #                    subscription or the request code.
    #
    def remove_handler(self, handler = None, request = None):
        # Convert request method to message codes
        code = 0
        if request != None:
            code = codes.find_code(codes.RESPONSE_MAP[request])
            
        # No Handler
        if handler == None:
            self._protocol.remove_handler(handler = None, code = code)
            if request in self._handlers: self._handlers[request] = []
            logger.debug("%s: handlers removed for request %s" % \
                     (self.__class__.__name__, request))
            
        # Handler
        else:
            # Remove Handler
            for h in self._handlers[request]:
                if h[0] == handler:
                    self._protocol.remove_handler(handler = h[1], code = code)
                    self._handlers[request].remove(h)
                    break
            logger.debug("%s: handler %s removed for request %s" % \
                     (self.__class__.__name__, handler.__name__, request))
            

    ## Get Subscription Data Handler(s)
    #  
    #  Gets a subscription's handler list.
    #
    #  @param  request   The name of the request method used to create the 
    #                    subscription or the request code.
    #  @return List of handlers, empty if none. 
    #
    def get_handlers(self, request = None):

        # No Handlers
        if request not in self._handlers:
            return []
        
        # Get handlers
        lst = []
        for tup in self._handlers[request]:
            lst.append(tup[0])
        return lst
    
        
    ## Get Waiting Data
    #
    #  Returns payload(s) from any waiting messages.                          
    #  Synchronous method of getting subscription Data.
    #
    #  @param  request The name of the request method used to create the 
    #                  subscription.
    #                  If None then it returns all subscription data.
    #  @return list of tuple(name,payload,timestamp), empty if no waiting data.
    #                  Invalid messages are returned as HorizonPayload_Null.
    #
    def get_waiting_data(self, request = None):
        
        # Convert request method to message codes
        code = 0
        if request != None:
            code = codes.find_code(codes.RESPONSE_MAP[request])
        
        # Retrieve and Return Waiting Messages
        lst = []
        for tup in self._protocol.get_waiting(code=code):
            lst.append(tuple([codes.HORIZON_CODES[tup[0]],
                          tup[1],tup[2]]))
        return lst
            
            
    #------------------------------- Properties --------------------------------
        

    ## Horizon Device Open?
    #
    #  @return is the horizon device open?
    #
    def is_open(self):        
        if self._protocol.opened == False:
            self.close()
            return False
        return True
        

    ## Horizon Device Alive?
    #
    #  @return is the horizon device alive?
    #
    def is_alive(self):
        # Echo
        try:
            self.request_echo()
            return True
        except Exception:
            return False
                
            
    ## Get Device Time
    #
    #  Note that the time returned has a delay from transmission.
    #
    #  @return device time in milliseconds
    #
    def get_device_time(self):        
        # Get Time
        echo = self.request_echo()
        return echo.timestamp
                
            
    ## Get Program Start Time
    #
    #  @return program start time in milliseconds as stored in HorizonTransport
    #
    def get_start_time(self):
        return self._protocol.start_time
                    
                    
    ## Get the Underlying Protocol
    #
    #  @warning Highly inadvisable to use this method and even more inadvisable
    #  to alter the resultant protocol. Only provided for advanced usability.
    #
    #  @return underlying HorizonProtocol_Client
    #
    def get_protocol(self):
        return self._protocol
        

    
    # Class Properties
    ## Horizon Device Alive
    alive = property(fget=is_alive, doc="Horizon Device Alive")
    ## Horizon Device Time
    device_time = property(fget=get_device_time, doc="Horizon Device Time")
    ## Horizon Device Open
    opened = property(fget=is_open, doc="Horizon Device Open")
    ## Horizon Start Time
    start_time = property(fget=get_start_time, doc="Horizon Start Time")
 

################################################################################
# Horizon Document



## @defgroup horizon_doc Horizon
#  @{
#  @image html clearpath.jpg
#  @htmlonly
#  <h1>Horizon Communication Protocol</h1>
#  <h2><center>For use with the Clearpath Robotics research platforms
#  </center><h2>
#  @endhtmlonly
#  @manonly
#     _____
#    /  _  \\
#   / _/ \\  \\
#  / / \\_/   \\
# /  \\_/  _   \\  ___  _    ___   ___   ____   ____   ___   _____  _   _
# \\  / \\_/ \\  / /  _\\| |  | __| / _ \\ | ┌┐ \\ | ┌┐ \\ / _ \\ |_   _|| | | |
#  \\ \\_/ \\_/ /  | |  | |  | └─┐| |_| || └┘ / | └┘_/| |_| |  | |  | └─┘ |
#   \\  \\_/  /   | |_ | |_ | ┌─┘|  _  || |\\ \\ | |   |  _  |  | |  | ┌─┐ |
#    \\_____/    \\___/|___||___||_| |_||_| \\_\\|_|   |_| |_|  |_|  |_| |_|
#            ROBOTICS™
#  
#  Horizon Communication Protocol                                             
#  For use with the Clearpath Robotics research platforms
#  @endmanonly
#
#  @section Revision History
#
#  @copydoc history
#
#  @section Overview
#
#  @copydoc overview
#
#  @section Terms
#
#  @copydoc terms
#
#  @section Hardware
#
#  @copydoc hardware
#
#  @section connection Connection Reference
#
#  @copydoc connection
#
#  @section mobile Mobile Platform Frame of Reference
#
#  @copydoc mobile
#
#  @section representation Data Representation
#
#  @copydoc representation
#
#  @section format Data Format
#
#  @subsection Package
#
#  @copydoc package
#  @subsection fields Field Description
#
#  @copydoc fields
#
#  @subsection acknowledgments Acknowledgments
#
#  @copydoc acknowledgments
#  @subsection flow Data Flow
#
#  @copydoc flow
#
#  @section encryption Encryption
#  @copydoc encryption
#
#  @section commands Commands (0x0001 - 0x3FFF)
#  @copydoc commands
#
#  @section requests Requests (0x4000 - 0x7FFF)
#  @copydoc requests
#
#  @section datas Data (0x8000 - 0xBFFF)
#  @copydoc datas
#
#  @section crc CRC Generation
#  @copydoc crc
#
#  @section License
#  @copydoc public_license
#
#  @}
#
#  @defgroup doc Documentation
#  @{
#  @}
