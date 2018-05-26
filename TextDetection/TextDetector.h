#include <opencv2\opencv.hpp>
#include <list>
#include <vector>
#include <numeric>

using namespace cv;
using namespace std;

double euclideanDistance(Point a, Point b);
double getMedianFromVector(vector<double>);
double overlapRatio(const Rect &r1, const Rect &r2);
double yDistance(const Rect &r1, const Rect &r2); // Calulate the distance between 2 Rect on Y axis

class StrokePoint : public cv::Point { // Extend from cv::Point
private:
	double strokeWidth;
	vector<Point> rayPath; // Corresponding ray path. Store it at the first pass for the second pass to save time.
public:
	StrokePoint(Point p, double width, vector<Point> ray);
	// The getters and setters
	vector<Point> getRayPath();
	double getStrokeWidth();
};

class ImageWriter {
private:
	int cnt = 1;
	String path;
	list<String> savedFiles;
public:
	ImageWriter();
	ImageWriter(String path);
	void writeImage(Mat src, String filename);
};

class TextDetector {

private:
	bool isSaveMiddleResult = true;
	int rows, cols;
	int searchDirection = -1;  //gradient direction is either 1 to detect dark text on light background or -1 to detect light text on dark background.
	ImageWriter iw;
	Mat srcImg;  // The origianl image
	Mat grayImg;
	Mat edgeMap; // Input of SWT

	Mat swtMap; // Result of SWT
	Mat labelMap; // Connected Component Labels
	vector<int> labelSet;
	vector<Point>* labelPoints; // Connected Component Labels Info
	vector<Rect> Boundings0; // Boundings before filtered

	vector<Rect> resultBoundings; // All the charactor boundings
	
public:
	TextDetector(Mat src, ImageWriter &iw);
	TextDetector(Mat src, ImageWriter &iw, bool isSaveMiddleResult);
	void prePorcess();
	void strokeWidthTransform();
	void connectedComponentTwoPass(); // Connected Component Analysis
	void calcBoundingRect();
	void connectedComponentFilter();

	vector<Rect> getResult(); // Return the result boundings vector<Rect> resultBoundings

};

