#! /usr/bin/env python
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
#  File: forward.py
#  Desc: Horizon Message Forwarder 
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
# Script Run Method Check



# Check if run as a script
if __name__ == "__main__":
    
    # Check if run with '-m'
    try:
        from .. import utils
        
    # Not run with '-m'
    except:
        
        # Notify of proper run method
        print ("ERROR: clearpath.horizon.forward is a module and must be run "\
               "by:\n  python -m clearpath.horizon.forward")

        # Exit Error
        import sys
        sys.exit(1)




################################################################################
# Module



## @package clearpath.horizon.forward 
#  Horizon Protocol Forwarding Server Python Module
# 
#  Horizon Message Forwarder                                                  \n
#  Supported Horizon version(s): 0.1 - 1.0
#
#  @author     Malcolm Robert
#  @date       17/03/10
#  @req        clearpath.utils                                                \n
#              clearpath.horizon                                              \n
#              clearpath.horizon.router                                       \n
#              clearpath.horizon.transports                                   \n
#              clearpath.horizon.versioning                                   \n
#              pyCrypto [http://www.dlitz.net/software/pycrypto/]             \n
#              -- for encryption                                              \n
#              pySerial [http://pyserial.sourceforge.net/]                    \n
#              -- for serial support
#  @version    1.0
#
#  @section USE
#
#  The intended use of this module is to provide a wrapper for HorizonRouter for
#  transport specific functionality as well as provide a script to run the
#  wrapper as a server.
#
#  While the wrapper (HorizonForwarder) can be used directly, it is recommended
#  that HorizonRouter be used instead because the router provides more control
#  and configuration than the wrapper. To use the forwarder, instantiate it with
#  parameters for the desired transports and then call open. All messages will
#  now be forwarded correctly without any additional work. During the use of
#  the forwarder, transports can be added and removed with the methods 
#  add_client and remove_client respectively. For ease of use, the
#  forwarder also has the method serve_forever which can be called to prevent
#  an application from closing by blocking forever. When the forwarder is no
#  longer needed, the method close should be called before deleting the object.
#
#  This script is executed by running
#  'python -m clearpath.horizon.forward'
#  To list the required and optional command-line arguments run
#  'python -m clearpath.horizon.forward --help'
#
#  Various logging messages (debug, warning,...) are logged with the standard
#  Python logger to the log named 'clearpath.horizon.forward'.                \n
#  Messages are ignored by default; remove the only handler from it and turn
#  on propagation to get messages to propagate.
#  When this module is run as a script, logging messages are turned on according
#  to command-line arguments.
#
#  @section HISTORY
#
#  Version 0.6
#  - Initial Creation as horizon_forward.py
#  - Horizon support for v 0.6
#  - Supports one-to-one Forwarding
#
#  Version 0.7
#  - Horizon support for v 0.7
#  - Move to forward.py
#  - Added one-to-many support
#  - Added encryption
#
#  Version 1.0
#  - Horizon support for v 0.1 - 1.0
#  - Python 2.6+ & 3.x compatible
#
#  @section License
#  @copydoc public_license
#
"""Horizon Message Forwarder

   Copyright © 2010 Clearpath Robotics, Inc.
   All rights reserved
   
   Created: 17/03/10
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
from .  import logger as horizon_logger
                                # Horizon Interface Logger 
from .  import router           # One-to-many Support
from .  import transports       # Horizon Transport Definitions
from .  import versioning       # Horizon Protocol Versions 

# Required Python Modules
import logging                  # Logging Utilities
import optparse                 # Command-Line Options Parsing
import re                       # Regular Expressions
import sys                      # Python Interpreter Functionality
import time                     # System Date & Time

# Non-Standard Modules
try:
    import Crypto.Random        # Encryption Random Generator
    try:
        import Crypto.Cipher.AES    
                                # AES block cipher
    except ImportError:
        pass
    try:
        import Crypto.Cipher.Blowfish
                                # Blowfish block cipher
    except ImportError:
        pass
    try:
        import Crypto.Cipher.CAST   
                                # CAST block cipher
    except ImportError:
        pass
    try:
        import Crypto.Cipher.DES    
                                # DES block cipher
    except ImportError:
        pass
    try:
        import Crypto.Cipher.DES3   
                                # DES3 block cipher
    except ImportError:
        pass
    try:
        import Crypto.Cipher.ARC2    
                                # RC2 block cipher
    except ImportError:
        pass
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
__revision__ = "$Revision: 239 $"
"""SVN Code Revision"""


## Supported Horizon Versions
versions = set()
"""Supported Horizon Versions"""


## Message Log
logger = logging.getLogger('clearpath.horizon.forward')
"""Horizon Forward Module Log"""
logger.setLevel(logging.NOTSET)
logger.addHandler(utils.NullLoggingHandler())
logger.propagate = False
logger.debug("Loading clearpath.horizon.forward ...")




################################################################################
# Horizon Message Forwarder



## Horizon Message Forwarder
#
#  Forwards messages between a platform and one or more clients.
#
#  @since 0.1
#
#  @pydoc
class HorizonForwarder():
    """Horizon Message Forwarder"""
    
    
    ## Supported Versions
    versions = versioning.HORIZON_VERSIONS.copy()
    """Supported Horizon Versions"""
    
      
    ## Create A Horizon Message Forwarder
    #
    #  Constructor for the Horizon Message Forwarder Class.
    #
    #  @param  clients        A list of HorizonTransport classes to use as 
    #                         servers for clients to connect to.
    #  @param  clients_args   A list of dictionaries containing arguments for
    #                         the clients' respective __init__ methods.
    #  @param  platform       The HorizonTransport class to use to connect to
    #                         the platform.
    #  @param  platform_args  A dictionary containing arguments for the 
    #                         platform's __init__ method.
    #  @param  send_all       Send all messages to all clients regardless of 
    #                         original sender/requestor (2),
    #                         Send data messages to all clients regardless of 
    #                         original requestor (1),
    #                         Perform full routing (0)
    #  @throws LookupError    If auto-detect version fails
    #  @throws TransportError Upon creation/initialization failure
    #  @throws ValueError     Upon bad arguments
    #
    #  @pydoc
    def __init__(self, platform, clients, platform_args, clients_args,
                 send_all = 0):
        """Create A Horizon Command-Line Controller"""
        logger.debug("%s: Instance creation started..." % \
                     self.__class__.__name__)
        
        # Class Variables
        ## Client Transports
        self._clients = []
        ## Platform Transport
        self._platform = None
        ## Is Open?
        self._opened = False
        ## Message Router
        self._router = None
        ## Routing Level
        self._send_all = send_all
        
        # instantiate platform
        self._platform = platform(**platform_args)
        
        # instantiate clients
        for i in range(0,len(clients)):
            self._clients.append(clients[i](**clients_args[i]))
    
        logger.debug("%s: ...instance creation complete." % \
                     self.__class__.__name__)
            
            
    ## Destroy A Horizon Forwarder
    #  
    #  Destructor for the Horizon Forwarder.                                  \n
    #  Ensures the connections are closed by calling close.                   \n
    #
    #  @pydoc
    def __del__(self):
        """Destroy A Horizon Forwarder"""
        logger.debug("%s: Instance destruction started..." % \
                     self.__class__.__name__)
        
        # Cleanup Transports
        self.close()
        
        logger.debug("%s: ...instance destruction complete." % 
                     self.__class__.__name__)
    
    
    ## Horizon Transports Open
    #  
    #  Opens all the transport devices.
    #
    #  @throws TransportError upon open failure
    #
    #  @pydoc
    def open(self):
        """Horizon Transports Open"""
        if not self._opened:
            logger.debug("%s: Devices opening started..." % \
                     self.__class__.__name__)
            
            # Open platform & clients
            self._platform.open()
            for client in self._clients:
                client.open()
                
            # Instantiate Router
            self._router = router.HorizonRouter(self._platform, self._clients,
                                                self._send_all)
            
            self._opened = True
            logger.debug("%s: ...devices opening complete." % \
                     self.__class__.__name__)
    
    
    ## Horizon Transports Close
    #  
    #  Closes all the transport devices.
    #
    #  @pydoc
    def close(self):
        """Horizon Transports Close"""
        logger.debug("%s: Devices closing started..." % \
                     self.__class__.__name__)
        
        # close clients
        for i in range(len(self._clients),0,-1):
            self._clients[i-1].close()
            if self._router != None:
                self._router.remove_client(self._clients[i-1])
            self._clients.pop(i-1)
        
        # close platform
        self._platform.close()
        
        self._opened = False 
        logger.debug("%s: ...devices closing complete." % \
                     self.__class__.__name__)
    
    
    ## Horizon Message Forwarder Add Client
    #  
    #  Adds a HorizonTransport as a client.
    #
    #  @param  client      The HorizonTransport class to use for the new client
    #  @param  client_args A dictionary containing arguments for the client's 
    #                      __init__ method
    #  @throws TransportError upon open failure
    #
    #  @pydoc
    def add_client(self, client, client_args):
        """Horizon Message Forwarder Add Client"""
        
        # instantiate & open client
        self._clients.append(client(**client_args))
        self._clients[-1].open()

        # Add to router
        self._router.add_client(self._clients[-1])
        logger.info("%s: client %s added." % \
                     (self.__class__.__name__, str(client)))
    
    
    ## Get Clients
    #
    #  Return a list of clients.
    #
    #  @return List of HorizonTransport
    #
    #  @pydoc
    def get_clients(self):
        """Return a list of clients."""
        
        return self._clients[:]
            

    ## Horizon Message Forwarder Remove Client
    #  
    #  Removes a client.
    #
    #  @param  client The client to remove, must be obtained from get_clients
    #
    #  @pydoc
    def remove_client(self, client = None):
        """Horizon Message Forwarder Remove Client"""

        # Remove Client
        for c in range(0,len(self._clients)):
            if self._clients[c] == client:
                
                # remove from router
                self._router.remove_client(client)
                            
                # remove client
                client.close()
                self._clients.pop(c)
                break;
        logger.info("%s: Client %s removed." % \
                     (self.__class__.__name__, str(client)))
    
    
    ## Serve Forever
    #
    #  Simply blocks forever.
    #
    #  @pydoc
    def serve_forever(self):
        """Server Forever"""
        if not self._opened: return
        logger.info("%s: Serving forever..." % \
                     self.__class__.__name__)
        
        while True:
            time.sleep(0.001)
    
    
    ## Is Open?
    #
    #  @return are the transports open?
    #
    #  @pydoc
    def is_open(self):
        """Is Open?"""
        
        return self._opened
    
    
    # Class Properties
    ## Forwarder Opened?
    opened = property(fget=is_open, doc="Forwarder Opened?")
    
    
    # Version Dependencies
    versions = versions.intersection(router.HorizonRouter.versions)
    versions = versions.intersection(transports.versions)
    
    
    
# HorizonForwarder Version Support
versions = versions.union(HorizonForwarder.versions)




logger.debug("... clearpath.horizon.forward loaded.")




################################################################################
# Script Code


## Bootstrap the Horizon Message Forwarder.
#
#  Main Program for Horizon Message Forwarder
#  - Gets a list of serial ports
#  - Parses Command-Line Options
#  - Initializes the Horizon Forwarder
#  - Launches control Loop
#  - Cleans-up after completion
#
#  @pydoc
def __main():
    """Bootstrap the Horizon Message Forwarder."""
    
    # Setup Logger
    qui_re = re.compile(r'^((-q)|(--quiet))$')
    for arg in sys.argv[1:]:
        if qui_re.match(arg):
            qui_re = None
            break
    if qui_re != None:
        while len(horizon_logger.handlers) > 0: 
            horizon_logger.removeHandler(horizon_logger.handlers[0])
        while len(logger.handlers) > 0: 
            logger.removeHandler(logger.handlers[0])
        logger_handler = logging.StreamHandler()
        logger_handler.setFormatter(
                                logging.Formatter("%(levelname)s: %(message)s"))
        horizon_logger.addHandler(logger_handler)
        horizon_logger.setLevel(logging.WARNING)
        logger.propagate = True
    
    # Check verbosity
    ver_re = re.compile(r'^((-v)|(--verbose))$')
    for arg in sys.argv[1:]:
        if ver_re.match(arg):
            horizon_logger.setLevel(logging.INFO)
            logger.info("Verbose mode entered.")
            break
    
    # Get list of available serial ports
    logger.info("Looking for serial ports...")
    ports = []
    try:
        serial
        ports = utils.list_serial_ports()
    except OSError as ex:
        logging.critical(ex)
        sys.exit(1)
    except IOError as ex:
        logging.critical(ex)
        sys.exit(1)
    except NameError:
        logger.warning("Serial not supported. Please install pySerial.")
    if len(ports) > 0:
        ports.sort()
        logger.info("...located %d serial ports: %s" %(len(ports),
                                                       ', '.join(ports)))
        
    # Get Command-Line Options
    logger.info("Parsing command-line arguments...")
    usage = 'usage: python -m clearpath.horizon.forward <platform> <client> '\
                '[<client2> ...] [options]\n\n'
    try:
        serial
        if len(ports) == 0: raise NameError("")
        usage = usage + '<platform>  = <serial> | (tcp | udp):<address>\n' + \
                        '<client>    = <serial> | (tcp | udp):<port>\n' + \
                        '<serial>    = ' + ' | '.join(ports) + '\n'
    except NameError:
        usage = usage + '<platform>  = (tcp | udp):<address>\n' + \
                        '<client>    = (tcp | udp):<port>\n'
    usage = usage + '<address>   = <host>:<port>\n' + \
                    '<host>      = computer hostname or IP address\n' + \
                    '<port>      = computer port number (1-65535)'
    desc = 'Horizon Message Forwarder: Forward messages between a platform ' + \
           'device connected at <platform> and one or more client devices ' + \
           'connected at <client>, <client2>, ....'
    vers = 'Horizon Protocol Document version to use for %s communication. ' + \
           'Supported version(s): '
    ver = []
    for v in sorted(versions): ver += ['%d.%d' % v]
    vers += ', '.join(ver) + ' - Default version: %d.%d' % \
                (max(versions)[0], max(versions)[1])  
    p = optparse.OptionParser(usage=usage,description=desc)
    p.add_option('--quiet', '-q', action='store_true', 
                 help='Do not output to stdin and stderr. '\
                 'Useful for running as a background process. '\
                 'This flag overrides --verbose and --debug_transports.')
    p.add_option('--verbose', '-v', action='count', 
                 help='Print detailed messages. Use twice for debug messages.')
    encr = []
    try:
        Crypto.Random
        try:
            Crypto.Cipher.AES
            encr += ['AES']
        except NameError:
            pass
        try:
            Crypto.Cipher.Blowfish
            encr += ['Blowfish']
        except NameError:
            pass
        try:
            Crypto.Cipher.CAST
            encr += ['CAST']
        except NameError:
            pass
        try:
            Crypto.Cipher.DES
            encr += ['DES']
        except NameError:
            pass
        try:
            Crypto.Cipher.DES3
            encr += ['DES3']
        except NameError:
            pass
        try:
            Crypto.Cipher.ARC2
            encr += ['RC2']
        except NameError:
            pass
        p.add_option('--client_encryption', action='append', type='str',
                 nargs=2,
                 help='Select the encryption cipher to use for client TCP '\
                 'communication ['+', '.join(encr)+']. '\
                 'Not all choices may be available; refer to '\
                 '`python -m clearpath.horizon.demo --doc` for details.', 
                 metavar='C CIPHER')
        p.add_option('--client_key', action='append', type='str',
                     metavar='C KEY', nargs=2,
                 help='The encryption key to use with --client_encryption C. '\
                 'The key length is dependent upon the chosen cipher.')
    except NameError:
        pass
    p.add_option('--client_max', action='append', type='int', metavar='C MAX', 
                 nargs=2, help='Maximum number of client connections to '\
                 'accept for TCP/UDP transport client C.')
    p.add_option('--client_timeout', action='append', type='int',
                 metavar='C TIME', nargs=2, 
                 help='Maximum time of inactivity before closing client '\
                 'connections to TCP/UDP transport client C.')
    p.add_option('--client_version', action='append', type='str', nargs=2, 
                 metavar='C VERSION', help=vers % 'client C')
    p.add_option('--debug_transports', action='store', type='choice',
                 choices=['DEBUG','INFO','WARNING','ERROR','NONE'], 
                 help='Select the debug level for transports.py. '\
                 'INFO is useful for general debug and DEBUG is useful for' \
                 ' crash location detection. This flag automatically enables'\
                 ' verbose debug messages (-v -v) regardless of LEVEL.', 
                 metavar='LEVEL')
    try:
        Crypto.Random
        p.add_option('--platform_encryption', action='store', type='choice',
                 choices=encr, 
                 help='Select the encryption cipher to use for platform TCP '\
                 'communication ['+', '.join(encr)+']. '\
                 'Not all choices may be available; refer to '\
                 '`python -m clearpath.horizon.demo --doc` for details.', 
                 metavar='CIPHER')
        p.add_option('--platform_key', action='store',type='str', metavar='KEY', 
                 help='The encryption key to use with --platform_encryption. '\
                 'The key length is dependent upon the chosen cipher.')
    except NameError:
        pass
    p.add_option('--platform_version', action='store',type='str', 
                 default='%d.%d' % (max(versions)[0], max(versions)[1]), 
                 metavar='VERSION', help=vers % 'platform')
    options, arguments = p.parse_args()
    logger.info("...command-line arguments parsed.")
    
    # Check Number of Command-Line Arguments
    if len(arguments) < 1:
        p.print_usage()
        sys.stderr.write("error: no devices specified\n")
        sys.exit(1)
    elif len(arguments) < 2:
        p.print_usage()
        sys.stderr.write("error: no client specified\n")
        sys.exit(1)
        
    # Address Regular Expression
    addr_re1 = re.compile(r'^((?:[a-zA-Z0-9]+' \
                           '(?:[a-zA-Z0-9\\-]+[a-zA-Z0-9])?)' \
                           '(?:\\.(?:[a-zA-Z0-9]+(?:[a-zA-Z0-9\\-]+[a-zA-Z0-9]'\
                           ')?))*):([1-9][0-9]{0,4})$')
    addr_re2 = re.compile(r'^((?:[a-zA-Z0-9\\-]{1,63})' \
                           '(?:\\.(?:[a-zA-Z0-9\\-]' \
                           '{1,63}))*):([1-9][0-9]{0,4})$')
    addr_re3 = re.compile(r'^([a-zA-Z0-9\\-\\.]{1,255}):' \
                           '((?:6553[0-5])|(?:655' \
                           '[0-2][0-9])|(?:65[0-4][0-9]{2})|(?:6[0-4][0-9]{3})'\
                           '|(?:[1-5][0-9]{4})|(?:[1-9][0-9]{0,3}))$')
    
    # Check verbosity
    if options.debug_transports != None:
        logger_handler.setFormatter(logging.Formatter(
                                        "%(name)s: %(levelname)s: %(message)s"))
        horizon_logger.setLevel(logging.DEBUG)
        logger.info("Debug mode entered.")
        transports.logger.propagate = True
        if options.debug_transports == 'DEBUG':
            transports.logger.setLevel(logging.DEBUG)
        elif options.debug_transports == 'INFO':
            transports.logger.setLevel(logging.INFO)
        elif options.debug_transports == 'WARNING':
            transports.logger.setLevel(logging.WARNING)
        elif options.debug_transports == 'ERROR':
            transports.logger.setLevel(logging.ERROR)
        else:
            transports.logger.propagate = False
    elif options.verbose != None and options.verbose > 1:
        horizon_logger.setLevel(logging.DEBUG)
        logger.info("Debug mode entered.")
    
    # Check for platform serial port
    platform = None
    platform_args = {}
    if arguments[0] in ports:
        platform = transports.HorizonTransport_Serial
        platform_args = {'port':arguments[0]}
        logger.info("Platform using serial port %s." % arguments[0])
            
    # Check for platform tcp
    elif arguments[0].startswith("tcp:") and \
                addr_re1.match(arguments[0][4:]) and \
                addr_re2.match(arguments[0][4:]) and \
                addr_re3.match(arguments[0][4:]):
        ptcp = addr_re1.match(arguments[0][4:]).groups()
        platform = transports.HorizonTransport_TCP_Client
        platform_args = {'host':ptcp[0],'port':ptcp[1]}
        logger.info("Platform using TCP address %s:%s." % \
                            (ptcp[0],ptcp[1]))
            
    # Check for platform udp
    elif arguments[0].startswith("udp:") and \
                addr_re1.match(arguments[0][4:]) and \
                addr_re2.match(arguments[0][4:]) and \
                addr_re3.match(arguments[0][4:]):
        pudp = addr_re1.match(arguments[0][4:]).groups()
        platform = transports.HorizonTransport_UDP_Client
        platform_args = {'host':pudp[0],'port':pudp[1],'local_port':pudp[1]}
        logger.info("Platform using UDP address %s:%s." % \
                            (pudp[0], pudp[1]))
        
    # Invalid Device
    if platform == None:
        p.print_usage()
        sys.stderr.write("error: invalid platform device" \
                             " specified\n")
        sys.exit(1)
        
    # Check Platform Version
    vgroups = options.platform_version.split('.')
    if len(vgroups) == 2:
        platform_args['version'] = tuple([int(vgroups[0]),int(vgroups[1])])
    if len(vgroups) != 2 or platform_args['version'] == None or \
                not platform_args['version'] in versions:
        p.print_usage()
        sys.stderr.write("error: option "\
                             "--platform_version: " + \
                        "unsupported value\n")
        sys.exit(1)
        
    # Check platform serial version
    if platform == transports.HorizonTransport_Serial and \
            not platform_args['version'] in \
            transports.HorizonTransport_Serial.versions:
        p.print_usage()
        ex = "error: option --platform_version: " \
             "unsupported value\nSupported serial version(s): "
        for v in sorted(transports.HorizonTransport_Serial.versions): 
            ex += '%d.%d ' % v
        sys.stderr.write(ex + '\n')
        sys.exit(1)
    
    # Check platform TCP version
    elif platform == transports.HorizonTransport_TCP_Client and \
            not platform_args['version'] in \
            transports.HorizonTransport_TCP_Client.versions:
        p.print_usage()
        ex = "error: option --platform_version: " \
             "unsupported value\nSupported TCP version(s): "
        for v in sorted(transports.HorizonTransport_TCP_Client.versions): 
            ex += '%d.%d ' % v
        sys.stderr.write(ex + '\n')
        sys.exit(1)
    
    # Check platform UDP version
    elif platform == transports.HorizonTransport_UDP_Client and \
            not platform_args['version'] in \
            transports.HorizonTransport_UDP_Client.versions:
        p.print_usage()
        ex = "error: option --platform_version: " \
             "unsupported value\nSupported UDP version(s): "
        for v in sorted(transports.HorizonTransport_UDP_Client.versions): 
            ex += '%d.%d ' % v
        sys.stderr.write(ex + '\n')
        sys.exit(1)
    logger.info("Platform using Horizon Specification v%d.%d" % \
                platform_args['version'])
        
    # Encryption?
    try:
        Crypto.Random
        
        # Check for platform encryption
        if options.platform_encryption != None or options.platform_key != None:
        
            # All required arguments?
            if platform != transports.HorizonTransport_TCP_Client:
                p.print_usage()
                ex = "error: platform encryption requires TCP."
                sys.stderr.write(ex + '\n')
                sys.exit(1)
            elif options.platform_encryption == None or \
                    options.platform_key == None:
                p.print_usage()
                ex="error: platform encryption requires both "\
                     "--platform_encryption and --platform_key flags."
                sys.stderr.write(ex + '\n')
                sys.exit(1)
                
            # Key Size?
            try:
                if options.platform_encryption == 'AES':
                    Crypto.Cipher.AES.new(options.platform_key,
                                          Crypto.Cipher.AES.MODE_ECB)
                    platform_args.setdefault('encryption',Crypto.Cipher.AES)
                elif options.platform_encryption == 'Blowfish':
                    Crypto.Cipher.Blowfish.new(options.platform_key,
                                          Crypto.Cipher.Blowfish.MODE_ECB)
                    platform_args.setdefault('encryption',
                                             Crypto.Cipher.Blowfish)
                elif options.platform_encryption == 'CAST':
                    Crypto.Cipher.CAST.new(options.platform_key,
                                          Crypto.Cipher.CAST.MODE_ECB)
                    platform_args.setdefault('encryption',Crypto.Cipher.CAST)
                elif options.platform_encryption == 'DES':
                    Crypto.Cipher.DES.new(options.platform_key,
                                          Crypto.Cipher.DES.MODE_ECB)
                    platform_args.setdefault('encryption',Crypto.Cipher.DES)
                elif options.platform_encryption == 'DES3':
                    Crypto.Cipher.DES3.new(options.platform_key,
                                          Crypto.Cipher.DES3.MODE_ECB)
                    platform_args.setdefault('encryption',Crypto.Cipher.DES3)
                elif options.platform_encryption == 'RC2':
                    Crypto.Cipher.ARC2.new(options.platform_key,
                                          Crypto.Cipher.ARC2.MODE_ECB)
                    platform_args.setdefault('encryption',Crypto.Cipher.ARC2)
            except ValueError as ex:
                p.print_usage()
                ex = "error: option --platform_key: %s." % \
                        str(ex)
                sys.stderr.write(ex + '\n')
                sys.exit(1)
            platform_args.setdefault('key',options.platform_key)
                
    except NameError:
        pass
        
    # Check Clients
    clients = []
    clients_args = []
    for c in range(0,len(arguments)-1):
        
        # Check for serial port
        client = None
        client_args = {}
        if arguments[c+1] in ports:
            client = transports.HorizonTransport_Serial
            client_args = {'port':arguments[c+1]}
            logger.info("Client %d using serial port %s." %(c+1,arguments[c+1]))
                
        # Check for tcp
        elif arguments[c+1].startswith("tcp:") and \
                    arguments[c+1][4:].isdigit() and \
                int(arguments[c+1][4:]) > 0 and \
                int(arguments[c+1][4:]) < 65536:
            ctcp = int(arguments[c+1][4:])
            client = transports.HorizonTransport_TCP_Server
            client_args = {'port':ctcp}
            logger.info("Client %d using TCP port %s." % \
                                (c+1,ctcp))
                
        # Check for udp
        elif arguments[c+1].startswith("udp:") and \
                arguments[c+1][4:].isdigit() and \
                int(arguments[c+1][4:]) > 0 and \
                int(arguments[c+1][4:]) < 65536:
            cudp = int(arguments[c+1][4:])
            client = transports.HorizonTransport_UDP_Server
            client_args = {'local_port':cudp}
            logger.info("Client %d using UDP port %d." % \
                                (c+1, cudp))
            
        # Invalid Device
        if client == None:
            p.print_usage()
            sys.stderr.write("error: invalid client %s device"\
                             " specified\n" % (c+1))
            sys.exit(1)
        
        # Check Version
        vgroups = []
        if options.client_version != None: 
            for v in options.client_version:
                if v[0] == str((c+1)):
                    vgroups = v[1].split('.')
        if len(vgroups) == 2:
            client_args['version'] = tuple([int(vgroups[0]),int(vgroups[1])])
        elif len(vgroups) == 0:
            client_args['version'] = platform_args['version']
        if (len(vgroups) != 2 and len(vgroups) != 0) or \
                    client_args['version'] == None or \
                    not client_args['version'] in versions:
            p.print_usage()
            sys.stderr.write("error: option "\
                                 "--client_version %d: " + \
                            "unsupported value\n" % ((c+1)))
            sys.exit(1)
            
        # Check serial version
        if client == transports.HorizonTransport_Serial and \
                not client_args['version'] in \
                transports.HorizonTransport_Serial.versions:
            p.print_usage()
            ex = "error: option --client_version %d: " \
                 "unsupported value\nSupported serial version(s): " % ((c+1))
            for v in sorted(transports.HorizonTransport_Serial.versions): 
                ex += '%d.%d ' % v
            sys.stderr.write(ex + '\n')
            sys.exit(1)
        
        # Check TCP version
        elif client == transports.HorizonTransport_TCP_Server and \
                not client_args['version'] in \
                transports.HorizonTransport_TCP_Server.versions:
            p.print_usage()
            ex = "error: option --client_version %d: " \
                 "unsupported value\nSupported TCP version(s): " % ((c+1))
            for v in sorted(transports.HorizonTransport_TCP_Server.versions): 
                ex += '%d.%d ' % v
            sys.stderr.write(ex + '\n')
            sys.exit(1)
        
        # Check UDP version
        elif client == transports.HorizonTransport_UDP_Server and \
                not client_args['version'] in \
                transports.HorizonTransport_UDP_Server.versions:
            p.print_usage()
            ex = "error: option --client_version %d: " \
                 "unsupported value\nSupported UDP version(s): " %  ((c+1))
            for v in sorted(transports.HorizonTransport_UDP_Server.versions): 
                ex += '%d.%d ' % v
            sys.stderr.write(ex + '\n')
            sys.exit(1)
        logger.info("Client %d using Horizon Specification v%d.%d" % \
                    ((c+1),client_args['version'][0],client_args['version'][1]))
        
        # Encryption?
        try:
            Crypto.Random
            
            # Check for encryption
            enc = ''
            key = ''
            if options.client_encryption != None:
                for e in options.client_encryption:
                    if e[0] == str((c+1)):
                        enc = e[1]
            if options.client_key != None:
                for k in options.client_key:
                    if k[0] == str((c+1)):
                        key = k[1]
            if enc != '' or key != '':
            
                # All required arguments?
                if client != transports.HorizonTransport_TCP_Server:
                    p.print_usage()
                    ex = "error: client %d encryption "\
                            "requires TCP." % ((c+1))
                    sys.stderr.write(ex + '\n')
                    sys.exit(1)
                elif enc == '' or key == '':
                    p.print_usage()
                    ex="error: client %d encryption requires "\
                        "both --client_encryption and --client_key flags." % (
                                                                        (c+1))
                    sys.stderr.write(ex + '\n')
                    sys.exit(1)
                elif enc not in encr:
                    p.print_usage()
                    ex="error: client %d encryption cipher "\
                        "unsupported!" % ((c+1))
                    sys.stderr.write(ex + '\n')
                    sys.exit(1)
                    
                # Key Size?
                try:
                    if enc == 'AES':
                        Crypto.Cipher.AES.new(key,
                                              Crypto.Cipher.AES.MODE_ECB)
                        client_args.setdefault('encryption',Crypto.Cipher.AES)
                    elif enc == 'Blowfish':
                        Crypto.Cipher.Blowfish.new(key,
                                              Crypto.Cipher.Blowfish.MODE_ECB)
                        client_args.setdefault('encryption',
                                                 Crypto.Cipher.Blowfish)
                    elif enc == 'CAST':
                        Crypto.Cipher.CAST.new(key,
                                              Crypto.Cipher.CAST.MODE_ECB)
                        client_args.setdefault('encryption',Crypto.Cipher.CAST)
                    elif enc == 'DES':
                        Crypto.Cipher.DES.new(key,
                                              Crypto.Cipher.DES.MODE_ECB)
                        client_args.setdefault('encryption',Crypto.Cipher.DES)
                    elif enc == 'DES3':
                        Crypto.Cipher.DES3.new(key,
                                              Crypto.Cipher.DES3.MODE_ECB)
                        client_args.setdefault('encryption',Crypto.Cipher.DES3)
                    elif enc == 'RC2':
                        Crypto.Cipher.ARC2.new(key,
                                              Crypto.Cipher.ARC2.MODE_ECB)
                        client_args.setdefault('encryption',Crypto.Cipher.ARC2)
                except ValueError as ex:
                    p.print_usage()
                    ex = "error: option --client_key %d: %s."\
                             % ((c+1),str(ex))
                    sys.stderr.write(ex + '\n')
                    sys.exit(1)
                client_args.setdefault('key',key)
                    
        except NameError:
            pass
        
        # Check Max Connections
        ma = 1
        if options.client_max != None:
            for m in options.client_max:
                if m[0] == (c+1):
                    ma = m[1]
        client_args.setdefault('max',ma)
        logger.info("Client %d allowing a maximum of %d connections." % \
                    ((c+1),ma))
        
        # Check Timeout
        timeout = 300000
        if options.client_timeout != None:
            for t in options.client_timeout:
                if t[0] == (c+1):
                    timeout = t[1]
        client_args.setdefault('rec_timeout',timeout)
        logger.info("Client %d using a connection timeout of %d ms." % \
                    ((c+1),timeout))
        
        # Add to list
        clients.append(client)
        clients_args.append(client_args)
        
    # Try to initialize Forwarder
    forwarder = None
    err = 0
    try:
        
        # Init Forwarder
        logger.debug("Initializing Horizon Forwarder...")
        forwarder = HorizonForwarder(platform=platform,clients=clients,
                                     platform_args=platform_args,
                                     clients_args=clients_args)
        logger.debug("...Horizon Forwarder Initialized.")
        
        # open the communications
        logger.debug("Opening Communications...")
        forwarder.open()
        logger.debug("...Communications Open.")
        
        # Start serve Loop
        logger.info("Horizon Forwarder Serving.")
        forwarder.serve_forever()
    
    # Error Occurred
    except Exception as ex:
        logging.critical(ex)
        err = 1
    
    # Command Quit
    except KeyboardInterrupt:
        print ("") # Only catch to prevent program closing before clean-up
        logger.debug("Server close requested.")
               
    # Cleanup
    finally:
        if forwarder != None:
            logger.debug("Closing Communications...")
            forwarder.close()
            logger.debug("...Communications Closed.")
        
    # Finish Program
    logger.info("Program Complete!")
    sys.exit(err)




################################################################################
# Script


# Check if run as a script
if __name__ == "__main__":
    
    # Run the main method
    __main()

    # Exit Bad - Should not reach so if it does: error
    sys.exit(1)
