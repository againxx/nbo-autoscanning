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

## Introduction

### Installation
The code is tested on Ubuntu 14.04. Since our autoscanning system is based on [ElasticFusion](https://github.com/mp3guy/ElasticFusion)--a real-time dense visual SLAM method, you may need install their requirements first. There are also some other dependencies that we list below.
* OpenCV >= 3.0
* cuDNN 5.1
