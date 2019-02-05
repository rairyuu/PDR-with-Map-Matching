# PDR-with-Map-Matching
A solution for indoor positioning. Based on PDR. With map matching. Using chest mounted IMU.

Got 1st place in IPIN 2017 competition Track 2. Got 2nd place in IPIN 2018 competition Track 2 (0.2m worse than 1st).

Our paper has been published in MDPI Sensors!

```
@article{lu2019indoor,
  title={Indoor Positioning System Based on Chest-Mounted IMU},
  author={Lu, Chuanhua and Uchiyama, Hideaki and Thomas, Diego and Shimada, Atsushi and Taniguchi, Rin-ichiro},
  journal={Sensors},
  volume={19},
  number={2},
  pages={420},
  year={2019},
  publisher={Multidisciplinary Digital Publishing Institute}
}
```

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

Support Python 3 now, Environment:

----Python 3.6.7

--------scipy 1.0.0

--------numpy 1.14.0

--------OpenCV 3.4.2

--------python-osc 1.7.0

We suggest you to install from Anaconda, you can run

```
conda create -n py36 python=3.6 scipy=1.0.0 numpy=1.14.0 opencv
pip install python-osc
```

to create the environment easily.

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

----dir,[initial heading(in rad)]

----scale,[pixel to meter]

To save the map, press "0", then press "1".
The produced map consists of "core.txt", "node.txt", "link.txt", "arrow.txt", "polygon.txt" and image files.
Copy these file to "RealTime-MapMatching-MultiFloor/map/".

"RealTime-MapMatching-MultiFloor" is our proposed system.
After running, the data is logged in "log/".

# Computational efficiency
In our system, the part which takes most time for processing is, detecting if the particles hit the walls.
In "map_matching.py", I did some optimization, but this is not enough, especially for huge maps with thousands walls.
There are several methods, which can solve this problem (turning this process to O(1)), but I did not implement them in my code.
For example, some grid-based methods.
After the pre-processing (may take some time), the further process will become extremely fast.
If you want to use particle filter based map matching, and there are many walls in the map, I strongly recommand you to implement these methods.

# Future work
Detailed document is coming soon.
If you have any questions, please contact:
luchuanhua@limu.ait.kyushu-u.ac.jp
or
845485344@qq.com
