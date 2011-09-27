This project contains source code and a sample program to perform
Hierarchical approximate convex decomposition (HACD) based on the
work of Khaled Mamou

Released: July 7, 2011

This source has been modified by John W. Ratcliff to add the
following features:

(1) Removed ALL of the STL code
(2) Provided a stripped down implementation of the STL map/set
	based on the EASTL written by Paul Pedriana
(3) Provides a single compatibility header file called
	'PxSimpleTypes.h' that allows you to capture all memory
	allocations define all data types.
(4) Provides a simple wrapper API that is easy to use.

The source is provided with a solution and project file
for Visual Studio 2008.

Most all of the code should compile on any platform with
relatively few changes required.

The source code for HACD is located in:

.\HACD\include  : Internal private header files
.\HACD\public	: Public interface and wrapper API
.\HACD\src		: Implementation sources

To run the sample; from the command line just type:

TestHACD hornbug.obj -c 500 -m 5 -merge 5

You can experiment with different concavity levels.

The test application will load the Wavefront OBJ sample file
'hornbug.obj' and output the convex hulls as a Wavefront OBJ file
called 'ConvexDecomposition.obj' which you can view in any mesh viewer.

Send questions/comments to John W. Ratcliff at

mailto:jratcliffscarab@gmail.com
