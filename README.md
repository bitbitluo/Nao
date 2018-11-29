# Nao机器人程序

#### Nao高尔夫最后一关.py
这个程序为湖南省程序设计竞赛中的二等奖，但是如果正常运行，其应该为第一名。比赛时，忘记更改代码，使用了调试代码，所以击球时，出现了5厘米左右的误差,希望能够帮助使用Nao进行高尔夫比赛的同学。

#### SVM物体识别.py
这个程序为SVM图片识别程序，不过，你需要自己收集样本，识别的精度还算可以。  

#### 多进程的红球识别功能.py  
在程序设计竞赛上，没有任何一组做多进程找球功能（使用的是多进程，因为在Python中，多线程是伪多线程，它只能使用一个cpu），不过，我也没有用，因为后期在准备考试。多进程可能大量缩短程序的执行的时间。  

#### 机器人倒地处理程序.py
我们的程序专门针对了比赛使用的Nao机器人做了步态处理，所以，其有一定的稳定性，比赛时，没有出现过摔倒的状况。这个程序是我自己独创的，它会不断的去检测Nao机器是否摔倒，然后去停止Nao机器人的Python脚本。当Nao机器人站起来以后，但又可以使脚本继续运行。  

#### 黄杆识别.py
黄杆识别模块，这个也是很多队伍没有写出的功能，主要运用的是OpenCV，之所以使用的是旋转矩形，是因为，Nao机器的头部旋转，可能会导致画面倾斜，而旋转矩形可以处理这个问题。

#### License  
> Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

#### 前方高能  

![](https://github.com/bitbitluo/Nao/blob/master/img/naos.jpg)

