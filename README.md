# ASE-Encoder

+ This work is an implement for the paper "Adaptive Soft-Encoding: A General Unsupervised Encoding Method for Place Recognition", which is waiting for review in a journal.

+ The adaptive soft-encoding (ASE) is a general unsupervised method, which is an improvement of the soft-encoding[1] to provide it with adaptability. The ASE encodes input sensor data into hierarchical feature vectors, including a training phase and an encoding phase. Different from current encoding methods, the ASE considers the distribution of input data in a subdimensional interval which is divided by analyzing the amount of information in each dimension of training data. Then, the encoded feature vector of the ASE is obtained by combining probability densities of fitted GMMs to reflect to distribution characteristics of input data in different subdimensional intervals.

+ The ASE has some advantages. First, encoding results of the ASE have high distinguishability. Second, the ASE is adaptive to input data with any types and dimensions. Therefore, the ASE-based place recognition method can work well for different local descriptors from data obtained by different sensors, e.g. SIFTs and ORBs from images, SRDs and 3D SUs from LiDAR data. Third, the ASE only needs a small amount of training data and can quickly generate an effective data representation model without need of GPU acceleration, which means the ASE can be applied for online navigation of autonomous vehicles and mobile robots.

### Notice！

+ In order to protect our idea from stealing during the review, we only release a demo which that can be tested and part of source codes. Once the paper is accepted, the complete source files are publicly published.

## How to run the demo

+ sudo apt-get install gcc g++ build-essential cmake 
+ sudo apt-get install autotools-dev libicu-dev libbz2-dev libboost-all-dev
+ sudo apt-get install libeigen3-dev
+ sudo apt-get install git
+ sudo ap-get update
+ git clone git@github.com:wdyiwdwd/ASE-Encoder.git
+ cd ASE-Encoder
+ mkdir bin & mkdir build
+ cd build
+ cmake ..
+ make
+ ../bin/Test


In this demo, the ASE encodes the input data into a feature vector, which is a low-dimentianal representation of the input data and extracts the distribution characteristic of them.


