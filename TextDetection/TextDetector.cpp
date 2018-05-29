#include "stdafx.h"
#include "TextDetector.h"
#include <vector>
#include <math.h>

#define MAX_STROKE_STEPS 20  //Set the maximum stroke search steps
#define SAME_DIRECTION_TH CV_PI/2   // The threshold to judge if the direction is same

TextDetector::TextDetector(Mat src, ImageWriter &iw) {
	TextDetector::srcImg = src;
	TextDetector::rows = src.rows;
	TextDetector::cols = src.cols;
	TextDetector::iw = iw;
	TextDetector::isSaveMiddleResult = true;
	TextDetector::isShowMiddleResult = true;
}

TextDetector::TextDetector(Mat src, ImageWriter &iw, bool isSaveMiddleResult, bool isShowMiddleResult) {
	TextDetector::srcImg = src;
	TextDetector::rows = src.rows;
	TextDetector::cols = src.cols;
	TextDetector::iw = iw;
	TextDetector::isSaveMiddleResult = isSaveMiddleResult;
	TextDetector::isShowMiddleResult = isShowMiddleResult;
}

void TextDetector::runDetection() {
	prePorcess();
	strokeWidthTransform();
	cout << "SWT Finished!" << endl;
	connectedComponentTwoPass();
	cout << "Connected Component Finished!" << endl;
	calcBoundingRect();
	cout << "Start filtering the candidate regions! (This might take some time)" << endl;
	connectedComponentFilter();
	cout << "Successfully Finished! :-) " << endl;
	if (isShowMiddleResult) {
		waitKey(0);
	}
	
}

void TextDetector::prePorcess() {
	/* Pre-Processing 
	 * SWT needs an input of binary edge file */

	/* RGB to Gray */
	grayImg.create(rows, cols, CV_32F);  // Create a float mat
	cvtColor(srcImg, grayImg, CV_RGB2GRAY);
	/* Canny Edge Detector */
	Canny(grayImg, edgeMap, 50, 150);
	if (isShowMiddleResult) {
		imshow("gray", grayImg);
		imshow("canny", edgeMap);
	}
	if (isSaveMiddleResult) {
		iw.writeImage(srcImg, "Source.jpg");
		iw.writeImage(grayImg, "Gray.jpg");
		iw.writeImage(edgeMap, "CannyEdge.jpg");
	}
}

void TextDetector::strokeWidthTransform() {

	/* Find gradient horizontal and vertical gradient */
	// Using Sobel
	Mat dx(rows, cols, CV_32F);
	Mat dy(rows, cols, CV_32F);
	Sobel(grayImg, dx, dx.type(), 1, 0);
	Sobel(grayImg, dy, dy.type(), 0, 1);
	// Calculate the matrix of gradient direction
	//Mat tanTheta(rows, cols, CV_32FC1);  // Type: Float
	//divide(dy, dx, tanTheta); 


	/* Get all the Edge Points */
	// TODO: Iteration efficiency can be improved here
	vector<Point> edges;
	for (int i = 0;i < rows;i++) {
		for (int j = 0;j < cols;j++) {
			if (edgeMap.at<uchar>(i, j) > 0) {
				edges.push_back(Point(j, i));
			}
		}
	}


	/* Initialize the mat with maximun value */
	swtMap = Mat::ones(rows, cols, CV_16U) * 65535;

	/* First pass of SWT */
	size_t lengthEdges = edges.size();
	vector<StrokePoint> strokePoints; // Stroke points & corresponding ray path
	for (size_t i = 0;i < lengthEdges;i++) {
		// Initialization
		Point current = edges[i];
		int steps = 2;
		//float theta = atan(tanTheta.at<float>(current));
		float currentDx = dx.at<float>(current);
		float currentDy = dy.at<float>(current);
		float mag = sqrt(pow(currentDx, 2) + pow(currentDy, 2));
		if (mag != 0) {
			currentDx = currentDx / mag;
			currentDy = currentDy / mag;
		}
		vector<Point> rayPoints;
		rayPoints.push_back(current);
		bool isStroke = false;
		Point nextCache = current;

		// Follow the Ray
		while (steps < MAX_STROKE_STEPS) {
			Point next;
			next.x = (int)(current.x + currentDx  * searchDirection * steps);
			next.y = (int)(current.y + currentDy  * searchDirection * steps);

			steps++;

			// Break loop if out of bounds. 
			if (next.x < 1 || next.y < 1 || next.x >= cols || next.y >= rows) {
				break;
			}
			if (next == nextCache) {
				continue;
			}
			nextCache = next;

			// Record next point of the ray
			rayPoints.push_back(next);

			// If another edge pixel has been found
			if (edgeMap.at<uchar>(next) > 0) {
				//float oppositeTheta = atan(tanTheta.at<float>(next));
				float nextDx = dx.at<float>(next);
				float nextDy = dy.at<float>(next);
				float mag = sqrt(pow(nextDx, 2) + pow(nextDy, 2));
				if (mag != 0) {
					nextDx = nextDx / mag;
					nextDy = nextDy / mag;
				}
				// Gradient direction roughtly opposite -> The edge point is a stroke point

				double delta = acos(currentDx * -nextDx + currentDy * -nextDy);
				if (delta < SAME_DIRECTION_TH) {
					isStroke = true;
					double strokeWidth = euclideanDistance(next, current);
					StrokePoint spTmp(next, strokeWidth, rayPoints);
					strokePoints.push_back(spTmp);
					// Iterate all ray points and populate with the minimum stroke width
					for (size_t k = 0;k < rayPoints.size();k++) {
						swtMap.at<ushort>(rayPoints.at(k)) = min(swtMap.at<ushort>(rayPoints.at(k)), (ushort)strokeWidth);
					}
					if (currentDy == 0) {
						int iii = 0;
					}
					break;
				}
			}
		}
	}

	/* Second Pass: Iterate all the stroke points found above
	 * and take its median on its ray path as the final stroke width */
	for (size_t i = 0;i < strokePoints.size();i++) {
		StrokePoint current = strokePoints.at(i);
		vector<Point> currentRayPath = current.getRayPath();

		// ***Calculate the median value
		// Get all the stroke width on the ray path
		vector<double> strokeWidthOnRay;
		for (size_t j = 0;j < currentRayPath.size();j++) {
			strokeWidthOnRay.push_back(swtMap.at<ushort>(currentRayPath.at(j)));
		}

		// Get the median value
		double strokeMedian = getMedianFromVector(strokeWidthOnRay);

		// Iterate all ray points and populate with the minimum stroke width
		for (int j = 0;j < currentRayPath.size();j++) {
			swtMap.at<ushort>(currentRayPath.at(j)) = min(swtMap.at<ushort>(currentRayPath.at(j)), (ushort)strokeMedian);
		}


	}

	/* Remove 65535 */
	for (int i = 0;i < rows;i++) {
		for (int j = 0;j < cols;j++) {
			if (swtMap.at<ushort>(i, j) == 65535) {
				swtMap.at<ushort>(i, j) = 0;
			}
		}
	}

	/* Erode and Dilate*/
	//Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));
	//erode(swtMap, swtMap, element);
	//dilate(swtMap, swtMap, element);
	//dilate(swtMap, swtMap, element);


	/* Output swtMap if required */
	if (isSaveMiddleResult) {
		iw.writeImage(swtMap * 4000 / 65535 * 255, "SWT_Map.jpg");
	}
	if (isShowMiddleResult) {
		imshow("SWT", swtMap * 4000);
	}
}

void TextDetector::connectedComponentTwoPass() {
	// connected component analysis (4-component)  
	// use two-pass algorithm  
	// 1. first pass: label each foreground pixel with a label  
	// 2. second pass: visit each labeled pixel and merge neighbor labels  
	//   
	// foreground pixel: swtMap(x,y) = 1  
	// background pixel: swtMap(x,y) = 0  
	// 
	// Code in this method is modified from: blog.csdn.net/icvpr/article/details/10259577

	/*  Check the input
	if (swtMap.empty() ||
		swtMap.type() != CV_8UC1) {
		return;
	}
	*/

	/* 1. first pass  */

	labelMap.release();
	swtMap.convertTo(labelMap, CV_32SC1);

	int label = 1;  // start by 2  
	//std::vector<int> labelSet;
	labelSet.push_back(0);   // background: 0  
	labelSet.push_back(1);   // foreground: 1  

	for (int i = 1; i < rows; i++) {
		int* data_preRow = labelMap.ptr<int>(i - 1);
		int* data_curRow = labelMap.ptr<int>(i);
		for (int j = 1; j < cols; j++) {
			if (data_curRow[j] > 0) {
				std::vector<int> neighborLabels;
				neighborLabels.reserve(2);
				int leftPixel = data_curRow[j - 1];
				int upPixel = data_preRow[j];
				if (leftPixel > 1) {
					neighborLabels.push_back(leftPixel);
				}
				if (upPixel > 1) {
					neighborLabels.push_back(upPixel);
				}

				if (neighborLabels.empty()) {
					labelSet.push_back(++label);  // assign to a new label  
					data_curRow[j] = label;
					labelSet[label] = label;
				}
				else {
					std::sort(neighborLabels.begin(), neighborLabels.end());
					int smallestLabel = neighborLabels[0];
					data_curRow[j] = smallestLabel;

					// save equivalence  
					for (size_t k = 1; k < neighborLabels.size(); k++) {
						int tempLabel = neighborLabels[k];
						int& oldSmallestLabel = labelSet[tempLabel];
						if (oldSmallestLabel > smallestLabel) {
							labelSet[oldSmallestLabel] = smallestLabel;
							oldSmallestLabel = smallestLabel;
						}
						else if (oldSmallestLabel < smallestLabel) {
							labelSet[smallestLabel] = oldSmallestLabel;
						}
					}
				}
			}
		}
	}

	/* Update equivalent labels  */
	// Assigned with the smallest label in each equivalent label set  
	for (size_t i = 2; i < labelSet.size(); i++) {
		int curLabel = labelSet[i];
		int preLabel = labelSet[curLabel];
		while (preLabel != curLabel) {
			curLabel = preLabel;
			preLabel = labelSet[preLabel];
		}
		labelSet[i] = curLabel;
	}


	/* 2. second pass  */
	for (int i = 0; i < rows; i++) {
		int* data = labelMap.ptr<int>(i);
		for (int j = 0; j < cols; j++) {
			int& pixelLabel = data[j];
			pixelLabel = labelSet[pixelLabel];
		}
	}

	/* Output labelMap if required */
	if (isSaveMiddleResult) {
		iw.writeImage(labelMap * 4000 / 65535 * 255, "InitialConnectionLabels.jpg");
	}
	if (isShowMiddleResult) {
		imshow("labels", labelMap * 4000);
	}
}

void TextDetector::calcBoundingRect() {
	size_t labelsCnt = labelSet.size() - 2;
	labelPoints = new vector<Point>[labelsCnt];   // The labels value begins from 2

	/* Use the labelMap data to fill labelPoints */
	for (int i = 0;i < rows;i++) {
		int* data_curRow = labelMap.ptr<int>(i);
		for (int j = 0;j < cols;j++) {
			int currentLabel = data_curRow[j];
			if (currentLabel > 1 && currentLabel < labelsCnt) {
				labelPoints[currentLabel - 2].push_back(Point(j, i));
			}
		}
	}

	/* Get bounding box of each label */
	for (int k = 0;k < labelsCnt;k++) {
		vector<Point> currentPointSet = labelPoints[k];
		Rect currentBoundingRect = boundingRect(currentPointSet);
		Boundings0.push_back(currentBoundingRect);
	}
	resultBoundings = Boundings0;

	/* Output boundings Image if required */
	Mat outMat = drawBoundingsOnMat(srcImg, resultBoundings);
	if (isSaveMiddleResult) {
		iw.writeImage(outMat, "Boundings_Before_Filter.jpg");
	}
	if (isShowMiddleResult) {
		imshow("Boundings Before Filter", outMat);
	}

}

void TextDetector::connectedComponentFilter() {
	// TODO: Improve the efficiency of this method
	/* Remove some boundings using five conditions */
	resultBoundings.clear();
	std::cout << "Finished 00%" ;
	for (size_t i = 0;i < Boundings0.size();i++) {
		std::cout << "\b\b\b" << cv::format("%.2d", i * 100 / Boundings0.size()) << "%" ;
		Rect currentBox = Boundings0[i];
		vector<Point> currentLabels = labelPoints[i];
		if (currentLabels.size() < 2) {
			continue;
		}
		vector<double> labelStrokes;
		for (size_t j = 0;j < currentLabels.size();j++) {
			labelStrokes.push_back(swtMap.at<ushort>(currentLabels[j]));
		}
		double sum = std::accumulate(std::begin(labelStrokes), std::end(labelStrokes), 0.0);
		double accum = 0.0;
		double mean = sum / labelStrokes.size();
		std::for_each(std::begin(labelStrokes), std::end(labelStrokes), [&](const double d) {
			accum += (d - mean)*(d - mean);
		});

		double stdev = sqrt(accum / (labelStrokes.size() - 1)); // Variance 

		// Basic parameters
		double varianceSW = stdev;
		double meanSW = mean;
		int height = currentBox.size().height;
		int width = currentBox.size().width;
		

		// Conditions
		bool cond1 = height > 10 && height < 100;
		if (!cond1) {
			continue;
		}
		bool cond2 = varianceSW / meanSW > 0.08;  // 0.5
		if (!cond2) {
			continue;
		}
		double aspectRatio = height * 1.0 / width;
		double diameter = sqrt(pow(height, 2) + pow(width, 2));
		double medianSW = getMedianFromVector(labelStrokes);
		bool cond3 = diameter / medianSW > 2;  // 10 
		if (!cond3) {
			continue;
		}
		bool cond4 = aspectRatio > 0.1 && aspectRatio < 10;  // 0.1-10
		if (!cond4) {
			continue;
		}
		bool cond5 = true;  // Condition 5 is to remove overlap boxes, which is implemented in next step

		if (cond5) {   // cond1 && cond2 && cond3 && cond4 && cond5
			resultBoundings.push_back(currentBox);
		}
	}
	std::cout << "\b\b\b\b All!" << endl;

	/* Combine if overlap */
	if (resultBoundings.size() < 2) {
		return;
	}

	vector<int> reserveMask(resultBoundings.size(), 1);
	for (size_t i = 0;i < resultBoundings.size() - 1;i++) {
		//vector<int>::iterator it = find(indexToErease.begin(), indexToErease.end(), i);
		//if (it != indexToErease.end()) { // If i is in the vector indexToErease
			//continue;
		//}
		if (reserveMask[i] == 0) {
			continue;
		}
		for (size_t j = i + 1;j < resultBoundings.size();j++) {
			if (reserveMask[j] == 0) {
				continue;
			}
			double ratio = overlapRatio(resultBoundings[i], resultBoundings[j]);
			double yDis = yDistance(resultBoundings[i], resultBoundings[j]);
			double xDis = xDistance(resultBoundings[i], resultBoundings[j]);
			int minDis = xDis > yDis ? yDis : xDis;
			// only consider Y axis here will connect regions into line ratio > 0 || yDis < 2
			if (ratio > 0 || (yDis < 5 && xDis < 10)){  // Overlap or very near on Y axis (yDis < 5 && xDis < 2)
				resultBoundings[i] = resultBoundings[i] | resultBoundings[j]; // Union of both Rect
				reserveMask[j] = 0;
			}
		}
	}

	/* TODO: Combine if nearby */
	

	vector<Rect> boxTmp;
	for (size_t i = 0;i < resultBoundings.size();i++) {
		if (reserveMask[i] > 0) {
			boxTmp.push_back(resultBoundings[i]);
		}
	}
	resultBoundings.clear();
	resultBoundings = boxTmp;
	cout << "Filter finished!" << endl;

	/* Sort the bounding boxes */
	// TODO: This is needed for character recognition

	/* Output boundings Image if required */
	Mat outMat = drawBoundingsOnMat(srcImg, resultBoundings);
	if (isSaveMiddleResult) {
		iw.writeImage(outMat, "Final_Result.jpg");
	}
	if (isShowMiddleResult) {
		imshow("Final Result", outMat);
	}
}

vector<Rect> TextDetector::getResult() {
	return  resultBoundings;
}

void getFiles(string path, vector<string>& files) {
	// Ref: blog.csdn.net/u012005313/article/details/50687297
	// File Handles
	_finddata_t file;
	intptr_t  lf = 0;
	// File info
	lf = _findfirst(path.c_str(), &file);
	while (_findnext(lf, &file) == 0) { 
		if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0)
			continue;
		files.push_back(file.name);
	}
}

double euclideanDistance(Point a, Point b) {
	Point tmp = a - b;
	return sqrt(pow(tmp.x, 2) + pow(tmp.y, 2));
}

double getMedianFromVector(vector<double> input) {
	size_t sizeVec = input.size();
	if (sizeVec == 0) {
		return 0;
	}
	else if (sizeVec == 1) {
		return input.at(0);
	}
	// Bubble Sorting

	for (size_t ii = 1;ii < sizeVec;ii++) {
		for (size_t jj = 0;jj < sizeVec - 1;jj++) {
			if (input.at(jj) > input.at(ii)) {
				swap(input[ii], input[jj]);
			}
		}
	}
	// Get the median value
	double median = input.at((sizeVec + 1) / 2 - 1);
	return median;
}

double overlapRatio(const Rect & r1, const Rect & r2) {
	int x1 = r1.x;
	int y1 = r1.y;
	int width1 = r1.width;
	int height1 = r1.height;

	int x2 = r2.x;
	int y2 = r2.y;
	int width2 = r2.width;
	int height2 = r2.height;

	int endx = max(x1 + width1, x2 + width2);
	int startx = min(x1, x2);
	int width = width1 + width2 - (endx - startx);

	int endy = max(y1 + height1, y2 + height2);
	int starty = min(y1, y2);
	int height = height1 + height2 - (endy - starty);

	double ratio = 0.0;
	double Area, Area1, Area2;

	if (width <= 0 || height <= 0)
		return 0.0;
	else {
		Area = width*height;
		Area1 = width1*height1;
		Area2 = width2*height2;
		ratio = Area / (Area1 + Area2 - Area);
	}

	return ratio;
}

double yDistance(const Rect & r1, const Rect & r2) {
	int y1Up = r1.y;
	int y1Down = r1.y + r1.height;
	int y2Up = r2.y;
	int y2Down = r2.y + r2.height;

	vector<int> disTmp;
	disTmp.push_back(abs(y1Up - y2Up));
	disTmp.push_back(abs(y1Down - y2Down));
	disTmp.push_back(abs(y1Up - y2Down));
	disTmp.push_back(abs(y2Up - y1Down));
	
	auto minPosition = min_element(disTmp.begin(), disTmp.end());

	return *minPosition;
}

double xDistance(const Rect & r1, const Rect & r2) {
	int x1Left = r1.x;
	int x1Right = r1.x + r1.width;
	int x2Left = r2.x;
	int x2Right = r2.x + r2.width;

	vector<int> disTmp;
	//disTmp.push_back(abs(x1Left - x2Up));
	//disTmp.push_back(abs(x1Right - x2Right));
	disTmp.push_back(abs(x1Left - x2Right));
	disTmp.push_back(abs(x2Left - x1Right));

	auto minPosition = min_element(disTmp.begin(), disTmp.end());

	return *minPosition;
}

Mat drawBoundingsOnMat(Mat src, vector<Rect> boundings) {
	Mat tmp;
	src.copyTo(tmp);
	for (size_t i = 0;i < boundings.size();i++) {
		Rect currentBox = boundings[i];
		rectangle(tmp, currentBox, Scalar(0, 0, 255), 2);
	}
	return tmp;
}

ImageWriter::ImageWriter() {
}

ImageWriter::ImageWriter(String path) {
	ImageWriter::path = path;
}

void ImageWriter::writeImage(Mat src, String filename) {
	String wholePath = path + format("%.2d", cnt) + "_" + filename;
	cout << "Output File [" << wholePath << "] Saved!" << endl;
	if (imwrite(wholePath, src)) {   // if successful
		savedFiles.push_back(wholePath);
		cnt++;
	}
}

StrokePoint::StrokePoint(Point p, double width, vector<Point> ray) {
	StrokePoint::x = p.x;
	StrokePoint::y = p.y;
	StrokePoint::strokeWidth = width;
	StrokePoint::rayPath = ray;
}

vector<Point> StrokePoint::getRayPath() {
	return rayPath;
}

double StrokePoint::getStrokeWidth() {
	return strokeWidth;
}
