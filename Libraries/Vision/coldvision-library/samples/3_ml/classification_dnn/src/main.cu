#include <iostream>
#include "opencv_dnn.h"
#include "caffe_dnn.h"

using namespace std;

int main(int argc, char **argv)
{
	string imagePath = (argc > 1) ? argv[1] : "../data/img_test_1/img_color_0.jpg";
	string dataPath = (argc > 2) ? argv[2] : "../data/";

	{ // test of OpenCV DNN CPU implementation with GoogLeNet trained on ILSVRC 2012 (BVLC GoogLeNet in Model Zoo)
		string modelTxtPath = dataPath + "bvlc_googlenet/bvlc_googlenet.prototxt";
		string modelBinPath = dataPath + "bvlc_googlenet/bvlc_googlenet.caffemodel";
		string classesPath  = dataPath + "bvlc_googlenet/synset_words.txt";
		testOpenCVDnnWithCaffeModel(modelTxtPath, modelBinPath, classesPath, imagePath);
	}

	{ // test of Caffe DNN CPU implementation with AlexNet trained on ILSVRC 2012 (BVLC Reference CaffeNet in Model Zoo)
		bool cpuOnly = true;
		string modelPath   = dataPath + "bvlc_caffenet/deploy.prototxt";
		string trainedPath = dataPath + "bvlc_caffenet/bvlc_reference_caffenet.caffemodel";
		string meanPath    = dataPath + "bvlc_caffenet/imagenet_mean.binaryproto";
		string labelPath   = dataPath + "bvlc_caffenet/synset_words.txt";
		testCaffeDnn(modelPath, trainedPath, meanPath, labelPath, imagePath, cpuOnly);
	}

	{ // test of Caffe DNN GPU implementation with AlexNet trained on ILSVRC 2012 (BVLC Reference CaffeNet in Model Zoo)
		bool cpuOnly = false;
		string modelPath   = dataPath + "bvlc_caffenet/deploy.prototxt";
		string trainedPath = dataPath + "bvlc_caffenet/bvlc_reference_caffenet.caffemodel";
		string meanPath    = dataPath + "bvlc_caffenet/imagenet_mean.binaryproto";
		string labelPath   = dataPath + "bvlc_caffenet/synset_words.txt";
		testCaffeDnn(modelPath, trainedPath, meanPath, labelPath, imagePath, cpuOnly);
	}

	{ // test of Caffe DNN GPU implementation with AlexNet trained on bvlc_oxford102 (BVLC Reference CaffeNet in Model Zoo)
		bool cpuOnly = false;
		string modelPath   = dataPath + "bvlc_oxford102/AlexNet/deploy.prototxt";
		string trainedPath = dataPath + "bvlc_oxford102/AlexNet/snapshot_iter_50000.caffemodel";
		string meanPath    = dataPath + "bvlc_oxford102/AlexNet/imagenet_mean.binaryproto";
		string labelPath   = dataPath + "bvlc_oxford102/data/synset_words.txt";
		testCaffeDnn(modelPath, trainedPath, meanPath, labelPath, imagePath, cpuOnly);
	}

	{ // test of Caffe DNN GPU implementation with AlexNet trained on the prototype dataset (BVLC Reference CaffeNet in Model Zoo)
		bool cpuOnly = false;
		string modelPath   = dataPath + "model_prototype/deploy.prototxt";
		string trainedPath = dataPath + "model_prototype/snapshot_iter_100.caffemodel";
		string meanPath    = dataPath + "model_prototype/imagenet_mean.binaryproto";
		string labelPath   = dataPath + "model_prototype/synset_words.txt";
		testCaffeDnn(modelPath, trainedPath, meanPath, labelPath, imagePath, cpuOnly);
	}

	return 0;
} //main
