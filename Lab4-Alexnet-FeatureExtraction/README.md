# AlexNet Feature Extraction
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This lab guides you through using AlexNet and TensorFlow to build a feature extraction network.

## Setup
Before you start the lab, you should first install:
* Python 3
* TensorFlow
* NumPy
* SciPy
* matplotlib

## Getting Started
* Original location of sources: https://github.com/udacity/CarND-Alexnet-Feature-Extraction
* Download ![Training data](https://d17h27t6h515a5.cloudfront.net/topher/2016/October/580a829f_train/train.p)
* Download ![AlexNet weights](https://d17h27t6h515a5.cloudfront.net/topher/2016/October/580d880c_bvlc-alexnet/bvlc-alexnet.npy)
* Make sure the downloaded files are in the code directory as the code.
* ![Alexnet](https://papers.nips.cc/paper/4824-imagenet-classification-with-deep-convolutional-neural-networks.pdf)

AlexNet is a popular base network for transfer learning because its structure is relatively straightforward, it's not too big, and it performs well empirically.

There is a TensorFlow implementation of AlexNet (adapted from ![Michael Guerhoy and Davi Frossard](http://www.cs.toronto.edu/~guerzhoy/tf_alexnet/)) in alexnet.py. You're not going to edit anything in this file but it's a good idea to skim through it to see how AlexNet is defined in TensorFlow. Coming up, you'll practice using AlexNet for inference on the image set it was trained on. After that, you'll extract AlexNet's features and use them to classify images from the ![German Traffic Sign Recognition Benchmark dataset](http://benchmark.ini.rub.de/?section=gtsrb&subsection=dataset).

