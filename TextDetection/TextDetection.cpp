// TextDetection.cpp : main function
// By JiaheZhang , 2018/05/24
// TIEI, Tianjin University
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
	/* Set the path of input files*/
	char inputPath[100];
	memset(inputPath, 0, 100);
	_getcwd(inputPath, 100); // Get current path
	strcat_s(inputPath, "\\images\\*");  // Format: "\\filename\\*"
	string filename = "";

	/* Set the output files info*/
	
	string outputPath = "outputs/";
	if (_access(outputPath.c_str(), 6) == -1) {
		_mkdir(outputPath.c_str());
	}
	ImageWriter iw(outputPath);  // Defined in TextDetector.h

	/* User Interface */
	if (argc == 1) {  // No extra input arguments
		vector<string> files;
		getFiles(inputPath, files);
		sort(files.begin(), files.end());
		cout << "Choose one of the following files as input:" << endl;
		for (size_t i = 0;i < files.size();i++) {
			cout << "[" << i << "]" << " " << files[i] << endl;
		}
		cout << "Your Choice (Type in the No.): ";
		int input;
		cin >> input;
		if (input > files.size() - 1) {
			cout << "Invalid Input!" << endl;
			system("pause");
			return -1;
		}
		filename = files[input];
	}
	else {
		cout << "Invalid Input Arguments!" << endl;
		system("pause");
		return -1;
	}

	/* Read Image Data */
	filename = ((string)inputPath).substr(0,size((string)inputPath)-1) + filename;
	cout << filename << endl;
	Mat src = imread(filename);
	if (!src.data) {
		cout << "Read Image File Failure!" << endl;
		system("pause");
		return -1;
	}
	cout << "Read file [" << filename << "] success!" << endl;


	/* Run the text detection */
	TextDetector td(src, iw, true, false);
	td.runDetection();

	/* Finish!*/
	system("pause");
	return 0;
}

