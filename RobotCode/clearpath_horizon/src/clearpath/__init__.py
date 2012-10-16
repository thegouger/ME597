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
#  File: clearpath/__init__.py
#  Desc: Namespace encapsulation for Clearpath Robotics Python Modules.
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
    import sys
    
    # Warn of Module ONLY status
    if (sys.version_info[0] > 2 and sys.version_info[1] > 0) or \
            (sys.version_info[0] == 2 and sys.version_info[1] > 6):
        print ("ERROR: clearpath is a module and can NOT be run as a script!\n"
           "For a listing of installed Clearpath Robotics Inc. modules, run:"\
           "\n  python -m clearpath")
    else:
        print ("ERROR: clearpath is a module and can NOT be run as a script!\n"
           "For a listing of installed Clearpath Robotics Inc. modules, run:"\
           "\n  python -m clearpath.__main__")

    # Exit Error
    sys.exit(1)




################################################################################
# Module



## @package clearpath
#  Clearpath Robotics Module
# 
#  Namespace encapsulation for Clearpath Robotics Python Modules.
#
#  @author     Malcolm Robert
#  @date       17/03/10
#  @version    1.0
#
#  @section USE
#
#  The intended use of this package is to provide a namespace for Clearpath
#  Robotics, Inc. Python code to separate it out from the global namespace and
#  provide collision avoidance. Distributed Clearpath packages and modules
#  should, upon installation, check for the existence of this package, create
#  it if it doesn't exist, and then install themselves as children of this
#  package.
#
#  All clearpath modules should have the field __version__ which is a string
#  containing the module version number (ex. '1.0') and the field __revision__
#  which contains the subversion revision number for the module file from the
#  last time the file was changed. Changes to the revision field should not
#  alter any interfaces or functionality. Further, modules which can be run
#  as scripts should have the module-wide method __main() which is called
#  immediately upon loading.
#
#  A listing of the installed Clearpath modules can be obtained by running
#  'python -m clearpath' for Python 2.7+ and Python 3.1+ or 
#  'python -m clearpath.__main__' for Python 2.6- and Python 3.0.
#
#  @section HISTORY
#
#  Version 1.0
#  - Initial Creation
#  - Python 2.5+ & 3.x compatible
#
#  @section License
#  @copydoc public_license
#
#  @defgroup public_license Public License
#  @ingroup licensing
#
#  Copyright © 2010 Clearpath Robotics, Inc.                                  \n
#  All Rights Reserved.                                                     \n\n
#                                                                             
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#      - Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#      - Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in the
#        documentation and/or other materials provided with the distribution.
#      - Neither the name of Clearpath Robotics, Inc. nor the
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
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.             \n\n
#
#  Please send comments, questions, or patches to code@clearpathrobotics.com
#
#  @defgroup private_license Private License
#  @ingroup licensing
#
#  Copyright © 2010 Clearpath Robotics, Inc.                                  \n
#  All Rights Reserved.                                                     \n\n
#
#  Confidential, not for public release.
#
#  @defgroup licensing Licensing
#  @{
#  @}
#
"""Namespace encapsulation for Clearpath Robotics Python Modules.

   Copyright © 2010 Clearpath Robotics, Inc.
   All rights reserved
   
   Created: 17/03/10
   Author:  Malcolm Robert
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


# Module Support
## Module Version
__version__  = "1.0"
"""Module Version"""
## SVN Code Revision
__revision__ = "$Revision: 239 $"
""" SVN Code Revision"""
