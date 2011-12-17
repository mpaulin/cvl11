#include "motionFiltering.hpp"

Point3d triangulate(const Mat& P1, const Mat& P2, const Point2d& x1,
		const Point2d& x2) {

	Mat A(6, 6, CV_64F);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 4; j++) {
			A.at<double> (i, j) = P1.at<double> (i, j);
			A.at<double> (i + 3, j) = P2.at<double> (i, j);
		}
	}

	A.at<double> (0, 4) = -x1.x;
	A.at<double> (1, 4) = -x1.y;
	A.at<double> (2, 4) = -1;

	A.at<double> (3, 5) = -x2.x;
	A.at<double> (4, 5) = -x2.y;
	A.at<double> (5, 5) = -1;

	A.at<double> (3, 4) = 0;
	A.at<double> (4, 4) = 0;
	A.at<double> (5, 4) = 0;
	A.at<double> (0, 5) = 0;
	A.at<double> (1, 5) = 0;
	A.at<double> (2, 5) = 0;


	SVD svd(A);

	Point3d X;

	X.x = svd.vt.at<double> (5, 0) / svd.vt.at<double> (5, 3);
	X.y = svd.vt.at<double> (5, 1) / svd.vt.at<double> (5, 3);
	X.z = svd.vt.at<double> (5, 2) / svd.vt.at<double> (5, 3);

	return X;
}

pair<double, double> linearRegression(const vector<Point2d>& points) {
	Mat A(points.size(), 2, CV_64F);
	Mat B(points.size(), 1, CV_64F);
	for (unsigned int i = 0; i < points.size(); i++) {
		A.at<double> (i, 0) = points[i].x;
		A.at<double> (i, 1) = 1;
		B.at<double> (i, 0) = points[i].y;
		//cout << points[i].x << " " << points[i].y << endl;
	}
	Mat X = A.inv(DECOMP_SVD) * B;
	double a = X.at<double> (0, 0);
	double b = X.at<double> (1, 0);

	return pair<double, double> (a, b);
}

Plane::Plane() {
}

Plane::Plane(const vector<Point3d>& points) {
	Mat A = Mat(points.size(), 3, CV_64F);
	for (unsigned int i = 0; i < points.size(); i++) {
		A.at<double> (i, 0) = points[i].x;
		A.at<double> (i, 1) = points[i].y;
		A.at<double> (i, 2) = 1;
	}
	SVD svd(A,SVD::FULL_UV);
	a = svd.vt.at<double> (2, 0);
	b = svd.vt.at<double> (2, 1);
	c = 0;
	d = svd.vt.at<double> (2, 2);

	//Normalize to have the same direction every time
	a = a/d;
	b = b/d;
	c = c/d;
	d = d/d;

	first = points[0];
	last = points[points.size()-1];
	for(unsigned int i = 0;i<points.size();i++){
		if(project(first).x < project(points[i]).x){
			first = points[i];
		}
		if(project(last).x > project(points[i]).x){
			last = points[i];
		}
	}
}

pair<Vec3d, Vec3d> Plane::getBase() {
	Vec3d b1(-b, a, 0);
	b1 *= 1 / cv::norm(b1);
	Vec3d b2(0, 0, 1);
	return pair<Vec3d, Vec3d> (b1, b2);
}

Point3d Plane::getOrigin() {
	// TODO maybe replace by a point closer to the samples
	return Point3d(-d / a, 0, 0);
}

Point2d Plane::project(const Point3d& p) {
	Point3d O = getOrigin();
	pair<Vec3d, Vec3d> base = getBase();
	return Point2d((Mat(p - O)).dot(base.first), (Mat(p - O)).dot(base.second));
}

Point3d Plane::retroProject(const Point2d& p) {
	Point3d O = getOrigin();
	pair<Vec3d, Vec3d> base = getBase();
	return O + p.x * Point3d(base.first) + p.y * Point3d(base.second);
}

void Plane::render(Mat& image, const Mat& P) {
	first.z=0;
	last.z = 0;
	renderLine(image, P, retroProject(project(first)), retroProject(project(last)), Scalar(255, 0, 255));
	pair<Vec3d, Vec3d> b = getBase();
	Point3d p = retroProject(project(first));
	Point3d p1(p.x+100*b.first[0],p.y+100*b.first[1],p.z+100*b.first[2]);
	Point3d p2(p.x+100*b.second[0],p.y+100*b.second[1],p.z+100*b.second[2]);
	renderLine(image,P,p,p1,Scalar(255,0,0));
	renderLine(image,P,p,p2,Scalar(255,0,0));
}

Parabola::Parabola() {
	a=b=c=0;
}

Parabola::Parabola(const vector<Point3d>& points, const vector<double>& times,
		double g) {


	this->points = points;
	this->times = times;

	plane = Plane(points);

	//Evaluate Vx0
	vector<Point2d> vx(points.size());
	for (unsigned int i = 0; i < vx.size(); i++) {
		vx[i].x = times[i];
		vx[i].y = plane.project(points[i]).x;
	}

	v0x = linearRegression(vx).first;
	//cout << "v0x found : " << v0x << endl;
	vector<Point2d> pts(points.size());
	for (unsigned int i = 0; i < pts.size(); i++) {
		Point2d p = plane.project(points[i]);
		pts[i].x = p.x;
		pts[i].y = p.y - g * p.x * p.x / (2*v0x);
	}
	pair<double, double> bc = linearRegression(pts);

	a = g /(2*v0x);
	b = bc.first;
	c = bc.second;
	for (unsigned int i = 0; i < pts.size(); i++) {
			Point2d p = plane.project(points[i]);
	}
}

double Parabola::eval(double x) {
	return c + x * (b + x * a);
}

double Parabola::getError(double lambdaPlaneError) {
	double error = 0;
	for (unsigned int i = 0; i < points.size(); i++) {
		Point2d p = plane.project(points[i]);
		Point3d pp = plane.retroProject(p);
		double errParabola = (eval(p.x) - p.y);
		double errPlane = (pp.x-points[i].x)*(pp.x-points[i].x)+(pp.x-points[i].y)*(pp.x-points[i].y)+(pp.x-points[i].z)*(pp.x-points[i].z);
		error += errParabola * errParabola + lambdaPlaneError*errPlane;
	}
	error = sqrt(error) / (points.size()*points.size());
	return error;
}

void Parabola::render(Mat& image, const Mat& P, const Scalar& col) {
	if(a!=0){
		plane.render(image, P);
		double steps = 1000;
		double beg = plane.project(plane.first).x;
		double end = plane.project(plane.last).x;
		for (int i = 0; i <= steps; i++) {
			double x = beg + (end-beg) * i / steps;
			Point2d po(x, eval(x));
			Point3d X = plane.retroProject(po);

			renderPoint(image, P, X, col);
		}

		for (vector<Point3d>::const_iterator it = points.begin(); it
		!= points.end(); it++) {
			Point3d p(it->x, it->y, 0);
			renderPoint(image, P, p, Scalar(255, 255, 255));
			renderPoint(image, P, *it, Scalar(125, 125, 125));
			renderLine(image,P,p,*it,Scalar(0,0,0));
			renderLine(image,P,p,plane.retroProject(plane.project(p)),Scalar(125,0,0));
		}
	}
}

Parabola motionFilter(Balls& balls, MotionFilteringParameters params) {
	list<Balls::Trajectory3D> trajs;
	double minError = 10000000;
	Parabola minParabola;
	vector<bool> hasMatchedRight(balls.trajectories_right.size(), false);
	for (list<Balls::Trajectory>::iterator it = balls.trajectories_left.begin(); it
	!= balls.trajectories_left.end(); it++) {
		if (it->length < params.minToConsider)
			continue;
		bool hasMatched = false;
		int iRight = 0;
		for (list<Balls::Trajectory>::iterator it2 =
				balls.trajectories_right.begin(); it2
				!= balls.trajectories_right.end(); it2++) {
			iRight++;
			bool toAdd = true;
			if (it2->length < params.minToConsider)
				continue;
			vector<Point3d> traj;
			vector<int> frames;
			int i1 = 0;
			int i2 = 0;

			while (i1 < it->length && i2 < it2->length) {
				int f1 = it->getFrame(i1);
				int f2 = it2->getFrame(i2);
				if (f1 == f2) {
					if (abs(it->getEllipse(i1).center.y
							- it2->getEllipse(i2).center.y)>params.maxVertDistLR) {
						cout << "Nope : "<< abs(it->getEllipse(i1).center.y - it2->getEllipse(i2).center.y) << endl;
						toAdd = false;
						break;
					}
					Point3d p = triangulate(balls.cameras_left[f1],
							balls.cameras_right[f2], it->getEllipse(i1).center,
							it2->getEllipse(i2).center);
					//Undistort manually...
					p.y = sqrt(p.y*p.y+p.x*p.x);
					i1++;
					i2++;

					traj.push_back(p);
					frames.push_back(f1);

				} else if (f1 < f2)
					i1++;
				else if (f2 < f1)
					i2++;
			}
			if (toAdd) {
				vector<double> times(frames.size());
				for (unsigned int i = 0; i < frames.size(); i++) {
					times[i] = balls.times[frames[i]];
				}

				Parabola p(traj, times, params.g);

				double error = p.getError(params.lambdaPlaneError);
				cout << "Error = " << error << endl;
				if (error < params.maxMeanError){
					trajs.push_back(Balls::Trajectory3D(traj, frames));
					hasMatched = true;
					hasMatchedRight[iRight] = true;
					if (error < minError) {
						minError = error;
						minParabola = p;
					}
				}
			}
		}
	}

	balls.trajectories3D = trajs;
	return minParabola;
}
