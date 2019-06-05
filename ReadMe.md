MHEFK tracking for ackermann vehicles                        {#mainpage}
============

## Description

## Dependencies

This package requires of the following system libraries and packages

* [cmake](https://www.cmake.org "CMake's Homepage"), a cross-platform build system.
* [doxygen](http://www.doxygen.org "Doxygen's Homepage") and [graphviz](http://www.graphviz.org "Graphviz's Homepage") to generate the documentation.
* stdc++ and pthread libraries.

Under linux all of these utilities are available in ready-to-use packages.

Under MacOS most of the packages are available via [fink](http://www.finkproject.org/ "Fink's Homepage")

This package also requires of the following IRI libraries:

## Compilation and installation

Download this repository and create a build folder inside:

``` mkdir build ```

Inside the build folder execute the following commands:

``` cmake .. ```

The default build mode is DEBUG. That is, objects and executables include debug information.

The RELEASE build mode optimizes for speed. To build in this mode execute instead
``` cmake .. -DCMAKE_BUILD_TYPE=RELEASE ```

The release mode will be kept until next time cmake is executed.

``` make ``` 

In case no errors are reported, the generated libraries (if any) will be located at the
_lib_ folder and the executables (if any) will be located at the _bin_ folder.

In order to be able to use the library, it it necessary to copy it into the system.
To do that, execute

``` make install ```

as root and the shared libraries will be copied to */usr/local/lib/iridrivers* directory
and the header files will be copied to */usr/local/include/iridrivers* dierctory. At
this point, the library may be used by any user.

To remove the library from the system, exceute

``` make uninstall ```

as root, and all the associated files will be removed from the system.

To generate the documentation execute the following command:

``` make doc ```

## How to use it

To use this library in an other library or application, in the CMakeLists.txt file, first it is necessary to locate if the library has been installed or not using the following command

``` FIND_PACKAGE(MHEFK tracking for ackermann vehicles) ```

In the case that the package is present, it is necessary to add the header files directory to the include directory path by using

``` INCLUDE_DIRECTORIES(${MHEFK tracking for ackermann vehicles_INCLUDE_DIR}) ```

and it is also necessary to link with the desired libraries by using the following command

``` TARGET_LINK_LIBRARIES(<executable name> ${MHEFK tracking for ackermann vehicles_LIBRARY}) ```

## Disclaimer  

Copyright (C) 2009-2018 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Mantainer IRI labrobotics (labrobotica@iri.upc.edu)

This package is distributed in the hope that it will be useful, but without any warranty. It is provided "as is" without warranty of any kind, either expressed or implied, including, but not limited to, the implied warranties of merchantability and fitness for a particular purpose. The entire risk as to the quality and performance of the program is with you. should the program prove defective, the GMR group does not assume the cost of any necessary servicing, repair  or correction.

In no event unless required by applicable law the author will be liable to you for damages, including any general, special, incidental or consequential damages arising out of the use or inability to use the program (including but not limited to loss of data or data being rendered inaccurate or losses sustained by you or third parties or a failure of the program to operate with any other programs), even if the author has been advised of the possibility of such damages.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>

