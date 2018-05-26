// TextDetection.cpp : main function
// By JiaheZhang , 2018/05/24
// Computer Vision project
// This project performs a text detector based on SWT(Stroke Width Transform)

#include "stdafx.h"
#include "TextDetector.h"
#include <io.h>
#include <direct.h>
#include <opencv2\opencv.hpp>

using namespace cv;
using namespace std;


int main(int argc, char *argv[]) {
	string filename = "test.jpg";   // default input file
	string outputPath = "outputs/";
	if (_access(outputPath.c_str(), 6) == -1) {
		_mkdir(outputPath.c_str());
	}
	ImageWriter iw(outputPath);

	// Check if input a filename in command line
	if (argc == 1) {  // No extra input arguments
		string fileTmp;
		cout << "Please input a image filename(Press Enter for default " << filename << " ):  " << endl;
		cin >> noskipws >> fileTmp;
		if (fileTmp.size() > 0 && fileTmp.at(0) != '\n') {  //Not Null input
			filename = fileTmp;
		}
	}

	// Read Image Data
	Mat src = imread(filename);
	if (!src.data) {
		cout << "Read Image File Failure!" << endl;
		system("pause");
		return 0;
	}

	imshow("src", src);
	
	
	
	// SWT - Stroke Width Transform
	TextDetector td(src, iw);
	td.prePorcess();
	td.strokeWidthTransform();
	cout << "SWT Finished" << endl;
	
	td.connectedComponentTwoPass();
	cout << "CC Finished" << endl;
	td.calcBoundingRect();
	td.connectedComponentFilter();


	vector<Rect> boxes = td.getResult();
	for (size_t i = 0;i < boxes.size();i++) {
		Rect currentBox = boxes[i];
		rectangle(src, currentBox, Scalar(255, 0, 0), 1);
	}
	imshow("boxes", src);


	waitKey(0);
	system("pause");

	return 0;
}

