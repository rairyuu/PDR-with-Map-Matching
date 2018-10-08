# PDR-with-Map-Matching
A solution for indoor positioning. Based on PDR. With map matching. Using chest mounted IMU.

Got 1st place in IPIN 2017 competition Track 2. Got 2nd place in IPIN 2018 competition Track 2 (0.2m worse than 1st).

# System requirement
Hardware:
NGIMU
http://x-io.co.uk/ngimu/


Environment:

----Python 2.7.15

--------scipy 1.0.0

--------numpy 1.14.0

--------OpenCV 2.4.10

--------pyOSC 0.3.5b5294
https://pypi.org/project/pyOSC/

# A brief introduction
"MapEditor" is one of our tool program.
It helps to produce the input of "RealTime-MapMatching-MultiFloor".
To run it, you need to prepare one "core.txt" file, which contains several initial parameters of the map.
There is a template, "template/core.txt".
"core.txt" should begin with header "byUser".
Another header "byProgram" could only be used by the program.
Other parameters:

----floor,[floor number],[image 0]...

----height,[floor height]

----start,[start floor(from 0)]

----start_point,[start x],[start y]

----dir,[direction in rad]

----scale,[pixel to meter]

To save the map, press "0", then press "1".
The produced map consists of "core.txt", "node.txt", "link.txt", "arrow.txt", "polygon.txt" and image files.
Copy these file to "RealTime-MapMatching-MultiFloor/map/".

"RealTime-MapMatching-MultiFloor" is our proposed system.
After running, the data is logged in "log/".

# Future work
Detailed document is coming soon.
If you have any questions, please contact:
luchuanhua@limu.ait.kyushu-u.ac.jp
or
845485344@qq.com
