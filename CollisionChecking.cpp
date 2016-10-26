///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Lee Chen, Eleni Litsa
// Date: 9/15/2016
//////////////////////////////////////

#include "CollisionChecking.h"


// Intersect the point (x,y) with the set of rectangles.  If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles)
{
    bool valid = true;
    for (std::vector<Rectangle>::const_iterator i = obstacles.begin(); i != obstacles.end(); ++i){
        double xmin = i->x;
        double xmax = i->x + i->width;
        double ymin = i->y;
        double ymax = i->y + i->height;
        if (x<=xmax && xmin<=x && y<=ymax && ymin<=y){
            valid = false;
            break;
        }
    }
    return valid;
}

// Intersect a circle with center (x,y) and given radius with the set of rectangles.  If the circle
// lies outside of all obstacles, return true
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle>& obstacles)
{
    bool valid = true;
    for (std::vector<Rectangle>::const_iterator i = obstacles.begin(); i != obstacles.end(); ++i){
        double xmin = i->x;
        double xmax = i->x + i->width;
        double ymin = i->y;
        double ymax = i->y + i->height;
        
        struct Vertex
        {
            // Coordinates of the rectangle vertices
            double x;
            double y;
        };
        std::vector<Vertex> vertices;
        Vertex v;
        v.x = xmin;
        v.y = ymin;
        vertices.push_back(v);
        v.x = xmin;
        v.y = ymax;
        vertices.push_back(v);
        v.x = xmax;
        v.y = ymin;
        vertices.push_back(v);
        v.x = xmax;
        v.y = ymax;
        vertices.push_back(v);
        
        if (x<=(xmax + radius) && (xmin - radius)<=x && y<=ymax && ymin<=y){
            valid = false;
            break;
        }
        else if ((y<=(ymax + radius) && (ymin - radius)<=y && x<=xmax && xmin<=x)){
            valid = false;
            break;
        }
        else {
            for (std::vector<Vertex>::const_iterator j = vertices.begin(); j != vertices.end(); ++j){
                double xv = j -> x;
                double yv = j -> y;
                double temp = sqrt((x-xv)*(x-xv)+(y-yv)*(y-yv));
                if (temp <= radius){
                    valid = false;
                    break;
                }
            }
        }
    }
    return valid;
}


// returns whether the value k is between the values k1 and k2
bool valueInRange(double k, double k1, double k2){
	//case for k2>k1
	if (k>=k1 and k<=k2){
		return true;
	}
	//case for k2<k1
	else if(k<=k1 and k>=k2){
		return true;
	}
	else{
		return false;
	}

}
// returns whether the ranges defined by s1k1 and s1k2 overlaps with the
// range defined by s2k1 and s2k2.
bool rangesOverlap(double s1k1, double s1k2, double s2k1, double s2k2){
	
	if(valueInRange(s1k1,s2k1,s2k2)){
		return true;
	}
	else if(valueInRange(s1k2,s2k1,s2k2)){
		return true;
	}
	else if(valueInRange(s2k1,s1k1,s1k2)){
		return true;
	}
	else if(valueInRange(s2k2,s1k1,s1k2)){
		return true;
	}
	else{
		return false;
	}
}

// Intersects the line segment defined by x1, y1, x2, y2, with lines of the Rectangle rect.
// Returns true if the line segment does not intersect with the rectangle
bool isValidSegment(double x1, double y1, double x2, double y2, Rectangle &rect){
	
	double x_s1;
	double y_s2;
	double x_s3;
	double y_s4;

	// if the line segment is verticle
	if(x1==x2){
		x_s1 = x1; //side 1:  y=rect.y
		y_s2 = 0;   //side 2   x=rect.x  (doesn't matter because of the project description)
		x_s3 = x1;//side 3   y=rect.y+rect.height
		y_s4 = 0; //side 4   x=rect.x+rect.width (doesn't matter because of the project description)
	}
	else{
		double m=(y1-y2)/(x1-x2);
		double b=y1-m*x1;

		x_s1 = (rect.y-b)/m; //side 1:  y=rect.y
		y_s2 = m*rect.x+b;   //side 2   x=rect.x
		x_s3 = (rect.y+rect.height-b)/m;//side 3   y=rect.y+rect.height
		y_s4 = m*(rect.x+rect.width)+b; //side 4   x=rect.x+rect.width
	}
	
	//checking if the solutions above are in side the segments and the obstacle
	if(valueInRange(rect.y,y1,y2) and 
		rangesOverlap(x1,x2,rect.x,rect.x+rect.width) and
		valueInRange(x_s1,rect.x,rect.x+rect.width) and
		valueInRange(x_s1,x1,x2)){
		return false;
	}
	else if(valueInRange(rect.x,x1,x2) and 
		rangesOverlap(y1,y2,rect.y,rect.y+rect.height) and
		valueInRange(y_s2,rect.y,rect.y+rect.height) and
		valueInRange(y_s2,y1,y2)){
		return false;
	}
	else if(valueInRange(rect.y+rect.height,y1,y2) and 
		rangesOverlap(x1,x2,rect.x,rect.x+rect.width) and
		valueInRange(x_s3,rect.x,rect.x+rect.width) and
		valueInRange(x_s3,x1,x2)){
		return false;
	}
	else if(valueInRange(rect.x+rect.width,x1,x2) and 
		rangesOverlap(y1,y2,rect.y,rect.y+rect.height) and
		valueInRange(y_s4,rect.y,rect.y+rect.height) and
		valueInRange(y_s4,y1,y2)){
		return false;
	}
	else{
		return true;
	}
	
}

double rotatedX(double x0, double y0, double theta, double xt){
	return x0*cos(theta)-y0*sin(theta)+xt;
}
double rotatedY(double x0, double y0, double theta, double yt){
	return x0*sin(theta)+y0*cos(theta)+yt;
}


// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles.
// If the square lies outside of all obstacles, return true
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle>& obstacles)
{
	double x1=rotatedX(sideLength/2,sideLength/2,theta,x);
	double y1=rotatedY(sideLength/2,sideLength/2,theta,y);

	double x2=rotatedX(-sideLength/2,sideLength/2,theta,x);
	double y2=rotatedY(-sideLength/2,sideLength/2,theta,y);
	
	double x3=rotatedX(-sideLength/2,-sideLength/2,theta,x);
	double y3=rotatedY(-sideLength/2,-sideLength/2,theta,y);
	
	double x4=rotatedX(sideLength/2,-sideLength/2,theta,x);
	double y4=rotatedY(sideLength/2,-sideLength/2,theta,y);

    for(std::vector<Rectangle>::const_iterator it = obstacles.begin();it!=obstacles.end();it++){

    	Rectangle rect=*it;
    	double X1=rect.x;
    	double X2=rect.x+rect.width;
    	double Y1=rect.y;
    	double Y2=rect.y+rect.height;

    	// If all points of the robot is inside the obstacle (i.e. no intersection)
    	if(valueInRange(x1,X1,X2) and valueInRange(x2,X1,X2) and valueInRange(x3,X1,X2) and valueInRange(x4,X1,X2) and 
    		valueInRange(y1,Y1,Y2) and valueInRange(y2,Y1,Y2) and valueInRange(y3,Y1,Y2) and valueInRange(y4,Y1,Y2)){
    		return false;
    	}
    	


    	// check for intersections
    	if(!isValidSegment(x1,y1,x2,y2,rect)){
    		return false;
    	}
    	else if(!isValidSegment(x2,y2,x3,y3,rect)){
    		return false;
    	}
    	else if(!isValidSegment(x3,y3,x4,y4,rect)){
    		return false;
    	}
    	else if(!isValidSegment(x4,y4,x1,y1,rect)){
    		return false;
    	}
    	else if(!isValidSegment(x4,y4,x2,y2,rect)){
    		return false;
    	}
    	else if(!isValidSegment(x3,y3,x1,y1,rect)){
    		return false;
    	}



    }

    return true;
}

// Add any custom debug / development code here.  This code will be executed instead of the
// statistics checker (Project2.cpp).  Any code submitted here MUST compile, but will not be graded.
void debugMode(const std::vector<Robot>& robots, const std::vector<Rectangle>& obstacles, const std::vector<bool>& valid)
{


	for(unsigned int i=0;i<robots.size();i++){
		double x=robots[i].x;
		double y=robots[i].y;
		double theta=robots[i].theta;
		double length=robots[i].length;

		double x1=rotatedX(length/2,length/2,theta,x);
		double y1=rotatedY(length/2,length/2,theta,y);

		double x2=rotatedX(-length/2,length/2,theta,x);
		double y2=rotatedY(-length/2,length/2,theta,y);
		
		double x3=rotatedX(-length/2,-length/2,theta,x);
		double y3=rotatedY(-length/2,-length/2,theta,y);
		
		double x4=rotatedX(length/2,-length/2,theta,x);
		double y4=rotatedY(length/2,-length/2,theta,y);

		if(isValidSquare(x,y,theta,length,obstacles)!=valid[i]){
				std::cout<<"tst\tcrr"<<std::endl;
				std::cout<<isValidSquare(x,y,theta,length,obstacles)<<"\t"<<valid[i]<<std::endl;

				std::cout<<"robot:"<<std::endl;
				std::cout<<"id\tx\ty\tthe\tlen"<<std::endl;
				std::cout<<i<<"\t"<<x<<"\t"<<y<<"\t"<<theta<<"\t"<<length<<std::endl;
				
				std::cout<<"x\ty"<<std::endl;
				std::cout<<x1<<"\t"<<y1<<std::endl;
				std::cout<<x2<<"\t"<<y2<<std::endl;
				std::cout<<x3<<"\t"<<y3<<std::endl;
				std::cout<<x4<<"\t"<<y4<<std::endl;



				// std::cout<<"obstacles:"<<std::endl;
				// std::cout<<"x\ty"<<std::endl;

				// for(unsigned int j=0;j<obstacles.size();j++){
				// 	std::cout<<obstacles[j].x<<"\t"<<obstacles[j].y<<std::endl;
				// 	std::cout<<obstacles[j].x<<"\t"<<obstacles[j].y+obstacles[j].height<<std::endl;
				// 	std::cout<<obstacles[j].x+obstacles[j].width<<"\t"<<obstacles[j].y+obstacles[j].height<<std::endl;
				// 	std::cout<<obstacles[j].x+obstacles[j].width<<"\t"<<obstacles[j].y<<std::endl;
				// }
		}
	}


}