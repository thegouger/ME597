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
#  File: versioning.py
#  Desc: Horizon Protocol Versions
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
    print ("ERROR: clearpath.horizon.versioning is a module and can NOT be run"\
           " as a script!\nFor a command-line interface demo of Horizon, run:"\
           "\n  python -m clearpath.horizon.demo\n"\
           "For Horizon message forwarding, run:\n"\
           "  python -m clearpath.horizon.forward")

    # Exit Error
    import sys
    sys.exit(1)




################################################################################
# Module



## @package clearpath.horizon.versioning
#  Horizon Protocol Versions Python Module
# 
#  Horizon Message Versions                                                   \n
#  Supported Horizon version(s): 0.1 - 1.0
#
#  @author     Malcolm Robert
#  @date       07/04/10
#  @req        clearpath.utils
#  @version    1.0
#
#  @section USE
#
#  The intended purpose of this module is to provide a listing of all possible
#  Horizon versions. No additional functionality is provided. The main reason
#  for this module is to help prevent cyclic import dependencies.
#
#  The variable HORIZON_VERSIONS is a set containing all Horizon versions. Each
#  version is a tuple made up of (major,minor). Note that versions with a major
#  number of 0 are pre-release versions, the use of which are discouraged.
#  PROTOCOL_MAP is a dictionary mapping Horizon Protocol document versions
#  (those in HORIZON_VERSIONS) to the version byte in the message.
#  DOCUMENT_MAP is a dictionary mapping the version byte in the message to
#  one (or more) Horizon Protocol document version.
#
#  All modules within the clearpath.horizon package should contain the field
#  versions which is a set containing all of the Horizon Protocol versions
#  that the module is compatible with. Further, all classes within these
#  modules should likewise have a versions field. It should further be noted
#  that the module version (contained within __version__) is not an accurate
#  representation of the file or program version but rather the newest 
#  Horizon Protocol version that that module supports.
#
#  Various logging messages (debug, warning,...) are logged with the standard
#  Python logger to the log named 'clearpath.horizon.versioning'.             \n
#  Messages are ignored by default; remove the only handler from it and turn
#  on propagation to get messages to propagate.
#
#  @section HISTORY
#
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
#  - Move to versioning.py
#  - Horizon support for v 0.1 - 1.0
#  - Python 2.6+ & 3.x compatible
#
#  @section License
#  @copydoc public_license
#
#  @defgroup history Revision History
#  @ingroup doc
#  @htmlonly
#  <table border='1' cellspacing='0' cellpadding='0'>
#  <tr style='background-color: #B3B3B3'>
#  <td><i>Version</i></td><td><i>Date</i></td><td><i>Description</i></td></tr>
#  <tr><td>0.1</td><td>08/09/09</td><td>Initial Release</td></tr>
#  <tr><td>0.2</td><td>02/10/09</td><td>Supporting pan/tilt/zoom cameras</td>
#  </tr>
#  <tr><td>0.3</td><td>28/10/09</td><td>Modification to timestamp definition, 
#  additional platform control options</td></tr>
#  <tr><td>0.4</td><td>18/01/10</td><td>Removing sequence ID from header, 
#  adding manipulator control and feedback, E-Stop commands</td></tr>
#  <tr><td>0.5</td><td>15/02/10</td><td>Refining GPIO/GPADC interface</td></tr>
#  <tr><td>0.6</td><td>10/03/10</td><td>Initial definition of Platform 
#  Information data message</td></tr>
#  <tr><td>0.7</td><td>16/03/10</td><td>Adding processor reset command, scale 
#  changes on velocities</td></tr>
#  <tr><td>0.8</td><td>26/03/10</td><td>Adding power status, raw encoder data
#  </td></tr>
#  <tr><td>1.0</td><td>26/03/10</td><td>Public Release</td></tr>
#  </table>
#  @endhtmlonly
#  @manonly
#
#  Version | Date     | Description
#  --------+----------+-------------------------------------------------
#  0.1     | 08/09/09 | Initial Release
#  0.2     | 02/10/09 | Supporting pan/tilt/zoom cameras
#  0.3     | 28/10/09 | Modification to timestamp definition,
#          |          | Additional platform control options
#  0.4     | 18/01/10 | Removing sequence ID from header, 
#          |          | Adding manipulator control and feedback, 
#          |          | E-Stop commands
#  0.5     | 15/02/10 | Refining GPIO/GPADC interface
#  0.6     | 10/03/10 | Initial definition of Platform Information data
#  0.7     | 16/03/10 | Adding processor reset command, 
#          |          | Scale changes on velocities
#  0.8     | 26/03/10 | Adding power status and raw encoder data
#  1.0     | 26/03/10 | Public Release
#  @endmanonly
#
"""Horizon Protocol Versions

   Copyright © 2010 Clearpath Robotics, Inc.
   All rights reserved
   
   Created: 07/04/10
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

# Required Python Modules
import logging                  # Logging Utilities


# Module Support
## Module Version
__version__  = "1.0"
"""Module Version"""
## SVN Code Revision
__revision__ = "$Revision: 389 $"
""" SVN Code Revision"""


## All Horizon Versions
HORIZON_VERSIONS = set([tuple([0,1]),
                        tuple([0,2]),
                        tuple([0,3]),
                        tuple([0,4]),
                        tuple([0,5]),
                        tuple([0,6]),
                        tuple([0,7]),
                        tuple([0,8]),
                        tuple([1,0])])
"""All Horizon Versions"""


## Version Conversion Dictionary (Document Version -> Protocol Byte)
PROTOCOL_BYTE = 0x00


## Version Conversion Dictionary (Protocol Byte -> Document Version)
DOCUMENT_MAP = { 0:[tuple([0,1]),
                    tuple([0,2]),
                    tuple([0,3]),
                    tuple([0,4]),
                    tuple([0,5]),
                    tuple([0,6]),
                    tuple([0,7]),
                    tuple([0,8]),
                    tuple([1,0])] }
"""Version Conversion Dictionary (Protocol Byte -> Document Version)"""


## Supported Horizon Versions
versions = HORIZON_VERSIONS.copy()
"""Supported Horizon Versions"""


## Message Log
logger = logging.getLogger('clearpath.horizon.versioning')
"""Horizon Protocol Versions Module Log"""
logger.setLevel(logging.NOTSET)
logger.addHandler(utils.NullLoggingHandler())
logger.propagate = False
logger.debug("Loading clearpath.horizon.versioning ...")
logger.debug("... clearpath.horizon.versioning loaded.")
