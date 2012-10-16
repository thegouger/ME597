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
#  File: router.py
#  Desc: Horizon Message Routing
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
# Script Code - First Run



# Check if run as a script
if __name__ == "__main__":
    
    # Warn of Module ONLY status
    print ("ERROR: clearpath.horizon.router is a module and can NOT be run"\
           " as a script!\nFor a command-line interface demo of Horizon, run:"\
           "\n  python -m clearpath.horizon.demo\n"\
           "For Horizon message forwarding, run:\n"\
           "  python -m clearpath.horizon.forward")

    # Exit Error
    import sys
    sys.exit(1)




################################################################################
# Module



## @package clearpath.horizon.router 
#  Horizon Message Routing Python Module
# 
#  Horizon Message Routing                                                    \n
#  Supported Horizon version(s): 0.1 - 1.0
#
#  @author     Malcolm Robert
#  @date       26/03/10
#  @req        clearpath.utils                                                \n
#              clearpath.horizon                                              \n
#              clearpath.horizon.messages                                     \n
#              clearpath.horizon.payloads                                     \n
#              clearpath.horizon.versioning
#  @version    1.0
#
#  @section USE
#
#  The intended use of this module is to provide functionality to allow multiple
#  clients to be connected to a platform at the same time without the clients
#  receiving unwanted/unneeded messages. Further, this module is also intended
#  to provide the ability to translate messages between a client and a platform
#  that are using different versions of the Horizon specification.
#
#  To use the router, the clients and platform objects (whether or not they are
#  transports) must be subclasses of the HorizonRouteable object, overriding
#  the method message_routed to handle getting a message from the router and the
#  property version to tell the router what version of the Horizon specification
#  the object is using. Further, when the object needs a message routed, it 
#  should call the method route_message. After instantiating the clients and 
#  platform, instantiate the router by passing in pointers to the instances
#  before opening all objects (if transports or similar). Messages should
#  now automatically be routed to the proper destination.
#
#  At any time after instantiation, additional clients can be added by using
#  the method add_client, and clients can be removed by using the method
#  remove_client. Destruction of the router does not require any special
#  handling, though it is probably best to do so before closing and destroying
#  the clients and platform so that no messages get routed to a closed object.
#
#  Various logging messages (debug, warning,...) are logged with the standard
#  Python logger to the log named 'clearpath.horizon.router'.                 \n
#  Messages are ignored by default; remove the only handler from it and turn
#  on propagation to get messages to propagate.
#
#  @section HISTORY
#
#  Version 1.0
#  - Initial Creation
#  - Horizon support for v 0.1 - 1.0
#  - Python 2.6+ & 3.x compatible
#
#  @section License
#  @copydoc public_license
#
"""Horizon Message Routing

   Copyright © 2010 Clearpath Robotics, Inc.
   All rights reserved
   
   Created: 26/03/10
   Authors: Malcolm Robert
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
from .  import versioning       # Horizon Protocol Versions 

# Required Python Modules
import datetime                 # Date & Time Manipulation
import fractions                # Math Utils
import logging                  # Logging Utilities


# Module Support
## Module Version
__version__  = "1.0"
"""Module Version"""
## SVN Code Revision
__revision__ = "$Revision: 389 $"
""" SVN Code Revision"""


## Supported Horizon Versions
versions = set()
"""Supported Horizon Versions"""


## Message Log
logger = logging.getLogger('clearpath.horizon.router')
"""Horizon Transports Module Log"""
logger.setLevel(logging.NOTSET)
logger.addHandler(utils.NullLoggingHandler())
logger.propagate = False           
logger.debug("Loading clearpath.horizon.router ...") 




################################################################################
# Horizon Route-able Interface



## Horizon Route-able Interface
#
#  Represents an object that can have Horizon messages routed to/from it.     \n
#  The object receives messages by having message_routed called and the object
#  sends messages by calling route_message (will be overridden by router).    \n
#  The Horizon specification version, obtainable by the class property version,
#  is the version that the interface expects messages to be in.               \n
#                                                                             \n
#  To be inherited for specific implementations. No default implementation
#  provided.
#
#  @since NA - Not in the Horizon Protocol
#  
#  @pydoc
class HorizonRouteable():
    """Horizon Route-able Interface"""
    
    
    ## Supported Versions (Override to match subclass' supported versions) 
    versions = set()
    """Supported Horizon Versions"""
    
        
    ## Create A Horizon Route-able Interface
    #
    #  @pydoc
    def __init__(self):
        """Create A Horizon Route-able Interface"""   
        
        # Class Variables
        ## Protocol Version
        self._version = tuple([0,0])
                        
                        
    ## String Representation
    #
    #  Return the Route-able name.
    # 
    #  @return String of Route-able name.
    #
    #  @pydoc
    def __str__(self):
        """Return the Route-able name."""
    
        return "default"
        
    
    ## Horizon Route-able Interface Message Routed
    #
    #  The router has determined that this interface is to receive a message.
    #  Called by the router.
    #
    #  @param  message    The message to receive
    #  @throws TransportError upon receive failure
    #
    #  @pydoc
    def message_routed(self, message):
        """Horizon Route-able Interface Message Routed"""
        pass
        
    
    ## Horizon Route-able Interface Route Message
    #
    #  The interface has a message that needs to be routed.
    #  Called by the interface, overridden by the router.
    #
    #  @param  message    The message to route.
    #  @return handled    Was the message handled?
    #  @throws TransportError Upon send failure or routing failure.
    #
    #  @pydoc
    def route_message(self, message):
        """Horizon Route-able Interface Route Message"""
        return False
    
    
    ## Get Horizon Version
    #
    #  @return the horizon version in use
    #
    #  @pydoc
    def get_version(self):
        """Get Horizon Version"""
        
        return self._version
    
    
    # Class Properties
    ## Route-able Name
    name = property(fget=__str__,doc="Route-able Name")
    ## Horizon Version
    version = property(fget=get_version,doc="Horizon Version")
    
    
    
# HorizonRouteable Version Support
versions = versions.union(HorizonRouteable.versions)




################################################################################
# Horizon Message Router



## Horizon Message Router
#
#  Represents the Horizon message router.                                     \n
#  Routes messages between one platform and many clients.
#
#  @since 0.1
#  
#  @pydoc
class HorizonRouter():
    """Horizon Message Router"""
    
    
    ## Supported Versions
    versions = versioning.HORIZON_VERSIONS.copy()
    """Supported Horizon Versions"""
    
        
    ## Create A Horizon Message Router
    #  
    #  Constructor for the Horizon Message Router.                            \n
    #  Does NOT support version auto-detection.
    #
    #  @param  clients     The already instantiated HorizonRouteable clients
    #  @param  platform    The already instantiated HorizonRouteable platform
    #  @param  send_all    Send all messages to all clients connected
    #                      regardless of original sender/requestor (2), 
    #                      Send data messages to all clients connected
    #                      regardless of original requestor (1),
    #                      Perform full routing (0)
    #  @throws LookupError If auto-detect version fails
    #  @throws ValueError  Upon bad arguments
    #
    #  @pydoc
    def __init__(self, platform = None, clients = [], send_all = 0):
        """Create A Horizon Message Router"""
        logger.debug("%s: Instance creation started..." % \
                     self.__class__.__name__)
        
        # Class Variables
        ## Message Ack Routes
        self._acks = {}         # Format: {code:[tuple([timestamp,index])]}
        ## Client Routeables
        self._clients = clients
        ## Routing Level
        self._level = send_all
        ## Platform Routeables
        self._platform = platform
        ## Data Routes
        self._routes = {}       # Format: {code:[tuple([index,frequency,last])]}
        ## Protocol Version
        self._version = tuple([0,0])
        
        # Verify Routing Level
        if send_all > 2 or send_all < 0:
            logger.error("%s: Un-supported send_all!" % self.__class__.__name__)
            raise ValueError ("send_all must be [0,2]!")
        
        # Add override hooks
        def tmp(message):
            return (self.route_message(message,source=-1))
        self._platform.route_message = tmp
        for i in range(0,len(self._clients)):
            def tmp(message, source=i):
                return self.route_message(message,source=source)
            self._clients[i].route_message = tmp
    
        logger.debug("%s: ...instance creation complete." % \
                     self.__class__.__name__)
        
        
    ## Horizon Message Router Route A Message
    #
    #  Sends the message to the proper clients and/or platform.
    #  Called by HorizonRouteable.route_message.
    #
    #  @param  message The message to route
    #  @param  source  The index of the client sender or -1 for platform sender.
    #  @return handled Was the message handled
    #  @throws TransportError Upon send failure or routing failure.
    #
    #  @pydoc
    def route_message(self, message = None, source = -1):
        """Horizon Message Router Route A Message."""
        handled = False
        
        # Send All Mode
        if self._level == 2:
            
            # Send to platform?
            if source > -1:
                if message.version != self._platform.version:
                    self._platform.message_routed(
                                    message.translate(self._platform.version))
                else:
                    self._platform.message_routed(message)
                handled = True
                
            # Send to All Clients
            else:
                for client in self._clients:
                    if message.version != client.version:
                        client.message_routed(message.translate(client.version))
                    else:
                        client.message_routed(message)
                    handled = True
                    
        # Other Modes
        else:
            
            # From Platform?
            if source == -1:
                
                # Ack?
                if message.code < 0x8000:
                    
                    # Check all waiting clients for code
                    if message.code in self._acks:
                        for (timestamp,index) in self._acks[message.code][:]:
                            
                            # Found ?
                            if (self._platform.version[0] == 0 and 
                                    self._platform.version[1] < 4 and
                                    timestamp == message.sequence) or (
                                    (self._platform.version[0] > 0 or 
                                    self._platform.version[1] > 3) and
                                    timestamp == message.timestamp):
                                if message.version != \
                                            self._clients[index].version:
                                    self._clients[index].message_routed(
                                        message.translate(
                                                self._clients[index].version))
                                else:
                                    self._clients[index].message_routed(message)
                                self._acks[message.code].remove(tuple(
                                                        [timestamp,index]))
                                handled = True
                                break
                
                # Data
                else:
                    
                    # All Data Mode
                    if self._level == 1:
                        for client in self._clients:
                            if message.version != client.version:
                                client.message_routed(
                                            message.translate(client.version))
                            else:
                                client.message_routed(message)
                            handled = True
                            
                    # Full Routing Mode
                    else:
                    
                        # Update Timestamp
                        timestamp = 0
                        t = datetime.datetime.today()
                        timestamp = t.microsecond/1000 + t.second*1000 + \
                              t.minute*60*1000 + t.hour*60*60*1000 + \
                              t.day*24*60*60*1000
                        while timestamp > 4294967295: timestamp -= 4294967295
                        
                        # Check all waiting clients for code
                        if message.code in self._routes:
                            for (index,freq,last) in \
                                    self._routes[message.code][:]:
                                
                                # 1-off?
                                if freq == 0:
                                    if message.version != \
                                            self._clients[index].version:
                                        self._clients[index].message_routed(
                                            message.translate(
                                                self._clients[index].version))
                                    else:
                                        self._clients[index].message_routed(
                                                                        message)
                                    self._routes[message.code].remove(tuple(
                                                            [index,freq,last]))
                                    handled = True
                                
                                # Subscription?
                                elif last == -1 or \
                                        ((timestamp - last >= (1000.0/freq)) or\
                                        (timestamp < last and 4294967295 - \
                                         last + timestamp >= (1000.0/freq))):
                                    if message.version != \
                                            self._clients[index].version:
                                        self._clients[index].message_routed(
                                            message.translate(
                                                self._clients[index].version))
                                    else:
                                        self._clients[index].message_routed(
                                                                        message)
                                    self._routes[message.code].remove(tuple(
                                                            [index,freq,last]))
                                    self._routes[message.code].append(tuple(
                                                        [index,freq,timestamp]))
                                    handled = True
                                    
            # From Client?
            else:
                    
                # Add Ack to routes
                if message.code not in self._acks:
                    self._acks[message.code] = []
                if message.no_ack == False:
                    if self._platform.version[0] == 0 and \
                            self._platform.version[1] < 4:
                        self._acks[message.code].append(tuple([message.sequence,
                                                               source]))
                    else:
                        self._acks[message.code].append(tuple([
                                                            message.timestamp,
                                                               source]))
                
                # Command?
                if message.code < 0x4000:
                    if message.version != self._platform.version:
                        self._platform.message_routed(
                                    message.translate(self._platform.version))
                    else:
                        self._platform.message_routed(message)
                    handled = True
                    
                # Request
                else:
                    
                    # All Data Mode
                    if self._level == 1:
                        if message.version != self._platform.version:
                            self._platform.message_routed(
                                    message.translate(self._platform.version))
                        else:
                            self._platform.message_routed(message)
                        handled = True
                            
                    # Full Routing Mode
                    else:
                        response = codes.RESPONSE_MAP[codes.HORIZON_CODES[message.code]]
                        code = codes.find_code(response)
                        freq = 0
                        if self._platform.version[0] == 0 and \
                                self._platform.version[1] < 4:
                            freq = message.data[0]
                        else:
                            freq = utils.to_unsigned_short(message.data[0:2])
                        if (code not in self._routes):
                            self._routes[code] = []
                        
                        # 1?
                        if freq == 0:
                            if len(self._routes[code]) == 0:
                                if message.version != self._platform.version:
                                    self._platform.message_routed(
                                        message.translate(
                                                        self._platform.version))
                                else:
                                    self._platform.message_routed(message)
                                handled = True
                            else:
                                for i in range(0,len(self._routes[code])):
                                    if self._routes[code][i][0] == source:
                                        self._routes[code].pop(i)
                                self._routes[code].append(tuple([source,
                                                                         freq,
                                                                         -1]))
                                if message.no_ack == False:
                                    msg2 = None
                                    if self._platform.version[0] == 0 and \
                                            self._platform.version[1] < 4:
                                        msg2 = messages.HorizonMessage(
                                                code = message.code,
                                                payload = \
                                                payloads.HorizonPayload_Ack(
                                                timestamp = message.timestamp,
                                                version = self._clients[source
                                                                    ].version),
                                                sequence = message.sequence,
                                                timestamp = message.timestamp,
                                            version = self._clients[source
                                                                    ].version)
                                        self._acks[message.code].remove(tuple(
                                                            [message.sequence,
                                                               source]))
                                    else:
                                        msg2 = messages.HorizonMessage(
                                                code = message.code,
                                                payload = \
                                                payloads.HorizonPayload_Ack(
                                                timestamp = message.timestamp,
                                            version = self._clients[source
                                                                    ].version),
                                                timestamp = message.timestamp,
                                            version = self._clients[source
                                                                    ].version)
                                        self._acks[message.code].remove(tuple([
                                                            message.timestamp,
                                                               source]))
                                    self._clients[source].message_routed(msg2)
                                    handled = True
                            
                        # off
                        elif (self._platform.version[0] == 0 and 
                                self._platform.version[1] < 4 and 
                                freq == 0xFF) or (
                                (self._platform.version[0] > 0 or 
                                 self._platform.version[1] > 3) and
                                freq == 0xFFFF):
                            
                            # remove routes
                            for i in range(0,len(self._routes[code])):
                                if self._routes[code][i][0] == source:
                                    self._routes[code].pop(i)
                                    
                            # msg needed?
                            if len(self._routes[code]) == 0:
                                if message.version != self._platform.version:
                                    self._platform.message_routed(
                                        message.translate(
                                                        self._platform.version))
                                else:
                                    self._platform.message_routed(message)
                                handled = True
                            else:
                                if message.no_ack == False:
                                    msg2 = None
                                    if self._platform.version[0] == 0 and \
                                            self._platform.version[1] < 4:
                                        msg2 = messages.HorizonMessage(
                                                code = message.code,
                                                payload = \
                                                payloads.HorizonPayload_Ack(
                                                timestamp = message.timestamp,
                                            version = self._clients[source
                                                                    ].version),
                                                sequence = message.sequence,
                                                timestamp = message.timestamp,
                                            version = self._clients[source
                                                                    ].version)
                                        self._acks[message.code].remove(tuple(
                                                            [message.sequence,
                                                               source]))
                                    else:
                                        msg2 = messages.HorizonMessage(
                                                code = message.code,
                                                payload = \
                                                payloads.HorizonPayload_Ack(
                                                timestamp = message.timestamp,
                                            version = self._clients[source
                                                                    ].version),
                                                timestamp = message.timestamp,
                                            version = self._clients[source
                                                                    ].version)
                                        self._acks[message.code].remove(tuple([
                                                            message.timestamp,
                                                               source]))
                                    self._clients[source].message_routed(msg2)
                                    handled = True
                        
                        # subscription
                        else:
                            
                            # update routes
                            max = -1
                            lcm = 1
                            for i in range(0,len(self._routes[code])):
                                if self._routes[code][i][0] == source:
                                    self._routes[code].pop(i)
                                else:
                                    if self._routes[code][i][1] > max:
                                        max = self._routes[code][i][1]
                                    lcm = (lcm * self._routes[code][i][1]) / \
                                    fractions.gcd(lcm,self._routes[code][i][1])
                            self._routes[code].append(tuple([source,freq, -1]))
                            lcm = (lcm * freq) / fractions.gcd(lcm,freq)
                            
                            # msg needed?
                            if max == -1 or lcm > max:
                                if lcm != freq:
                                    raw = message.data
                                    if self._platform.version[0] == 0 and \
                                            self._platform.version[1] < 4:
                                        raw[0] = lcm
                                        ack = payloads.HorizonPayload(raw = raw,
                                                timestamp = message.timestamp,
                                                version = message.version)
                                        message = messages.HorizonMessage(
                                                    code = message.code,
                                                    payload = ack,
                                                timestamp = message.timestamp,
                                                    version = message.version,
                                                    sequence = message.sequence)
                                    else:
                                        raw[0:2] =utils.from_unsigned_short(lcm)
                                        ack = payloads.HorizonPayload(raw = raw,
                                                timestamp = message.timestamp,
                                                version = message.version)
                                        message = messages.HorizonMessage(
                                                    code = message.code,
                                                    payload = ack,
                                                timestamp = message.timestamp,
                                                    version = message.version)
                                self._platform.message_routed(message)
                                handled = True
                                
                            # Ack needed?
                            else:
                                if message.no_ack == False:
                                    msg2 = None
                                    if self._platform.version[0] == 0 and \
                                            self._platform.version[1] < 4:
                                        msg2 = messages.HorizonMessage(
                                                code = message.code,
                                                payload = \
                                                payloads.HorizonPayload_Ack(
                                                timestamp = message.timestamp,
                                            version = self._clients[source
                                                                    ].version),
                                                sequence = message.sequence,
                                                timestamp = message.timestamp,
                                            version = self._clients[source
                                                                    ].version)
                                        self._acks[message.code].remove(tuple(
                                                            [message.sequence,
                                                               source]))
                                    else:
                                        msg2 = messages.HorizonMessage(
                                                code = message.code,
                                                payload = \
                                                payloads.HorizonPayload_Ack(
                                                timestamp = message.timestamp,
                                            version = self._clients[source
                                                                    ].version),
                                                timestamp = message.timestamp,
                                            version = self._clients[source
                                                                    ].version)
                                        self._acks[message.code].remove(tuple([
                                                            message.timestamp,
                                                               source]))
                                    self._clients[source].message_routed(msg2)
                                    handled = True
                                    
        return handled
    
    
    ## Horizon Message Router Add Client
    #  
    #  Adds an already instantiated HorizonRouteable as a client.
    #
    #  @param  client The new client to add.
    #
    #  @pydoc
    def add_client(self, client = None):
        """Horizon Message Router Add Client"""
            
        # Add Handler
        self._clients.append(client)
        def tmp(message, source = len(self._clients)-1):
            return self.route_message(message,source=source)
        self._clients[len(self._clients)-1].route_message = tmp
        logger.info("%s: client %s added." % \
                     (self.__class__.__name__, str(client)))
            

    ## Horizon Message Router Remove Client
    #  
    #  Removes a client.
    #
    #  @param  client The client to remove
    #
    #  @pydoc
    def remove_client(self, client = None):
        """Horizon Message Router Remove Client"""

        # Remove Client
        for c in range(0,len(self._clients)):
            if self._clients[c] == client:
                
                # remove routes
                for key in self._routes.keys():
                    for i in range(0,len(self._routes[key])):
                        if self._routes[key][i][0] == c:
                            self._routes[key].pop(i)
                            
                # remove acks
                for key in self._acks.keys():
                    for (timestamp,index) in self._acks[key][:]:
                            if index == c:
                                self._acks[key].remove(tuple(
                                                        [timestamp,index]))
                                break
                            
                # remove client
                self._clients[c] = 0
                break;
        logger.info("%s: Client %s removed." % \
                     (self.__class__.__name__, client.name))
    
    
    

logger.debug("... clearpath.horizon.router loaded.")
