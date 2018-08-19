# Object-Aware Guidance for Autonomous Scene Reconstruction
Object-aware autoscanning system capable of exploring, reconstructing, and understanding an unknown scene within *one navigation pass*. You can find more information about this project from our [paper](http://kevinkaixu.net/papers/liu_sig18_nbo.pdf) and [project website](http://kevinkaixu.net/projects/nbo.html).

![scanning example](https://github.com/againxx/nbo-autoscanning/blob/master/doc/teaser.jpg)

## Citation
If you find our work useful in your research, please consider citing:
```
@article {liu_nbo_sig18,
    title = {Object-Aware Guidance for Autonomous Scene Reconstruction},
    author = {Ligang Liu and Xi Xia and Han Sun and Qi Shen and Juzhan Xu and Bin Chen and Hui Huang and Kai Xu},
    journal = {ACM Transactions on Graphics},
    volume = {37},
    number = {4},
    pages = {Article 104},
    year = {2018}
}
```

## Installation
The code is tested on Ubuntu 14.04. Since our autoscanning system is based on [ElasticFusion](https://github.com/mp3guy/ElasticFusion)--a real-time dense visual SLAM method, you may need install their requirements first. There are also some other dependencies that we list below.
* [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)
* OpenCV >= 3.0
* [cuDNN 5.1](https://developer.nvidia.com/cudnn)
When you have all of the dependencies installed, build the Core, Seg and CatkinWorkSpace one by one.

## Usage

### Parameters
In addition to ElasticFusion original parameters, we add several new parameters that you can use when launching `iroboscan`. Here we list all parameters as follows:
* -cal : Loads a camera calibration file specified as fx fy cx cy.
* -l : Processes the specified .klg log file.
* -p : Loads ground truth poses to use instead of estimated pose.
* -c : Surfel confidence threshold (default 10).
* -d : Cutoff distance for depth processing (default 3m).
* -i : Relative ICP/RGB tracking weight (default 10).
* -ie : Local loop closure residual threshold (default 5e-05).
* -ic : Local loop closure inlier threshold (default 35000).
* -cv : Local loop closure covariance threshold (default 1e-05).
* -pt : Global loop closure photometric threshold (default 115).
* -ft : Fern encoding threshold (default 0.3095).
* -t : Time window length (default 200).
* -s : Frames to skip at start of log.
* -e : Cut off frame of log.
* -f : Flip RGB/BGR.
* -icl : Enable this if using the ICL-NUIM dataset (flips normals to account for negative focal length on that data).
* -o : Open loop mode.
* -rl : Enable relocalisation.
* -fs : Frame skip if processing a log to simulate real-time.
* -q : Quit when finished a log.
* -fo : Fast odometry (single level pyramid).
* -nso : Disables SO(3) pre-alignment in tracking.
* -r : Rewind and loop log forever.
* -ftf : Do frame-to-frame RGB tracking.
* -sc : Showcase mode (minimal GUI).
* -sg : Run in a simulated gazebo environment (newly added).
* -rr : Use ROS topic as real live input (newly added).

### Demo

### Auto Segmentation
Besides `iroboscan` package, we also provide another package `autoseg` which can be used to automatically segment CAD models into pre-segmented components. For more details, please refer to our [paper](http://kevinkaixu.net/papers/liu_sig18_nbo.pdf) and `AutoSpin.py`.

## Benchmark
There is an extra benchmark that you can find [here](https://github.com/againxx/OASC-Benchmark).
