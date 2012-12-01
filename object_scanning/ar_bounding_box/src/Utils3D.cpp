/** \file Utils3D.cpp
 * \brief Utility functions for working with 3D coordinates
 *
 * This file is part of the RoboEarth ROS ar_bounding_box package.
 *
 * It file was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the European Union Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2011 by <a href="mailto:andreas.koch@ipvs.uni-stuttgart.de">Andreas Koch</a>, University of Stuttgart
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * \author Andreas Koch
 * \version 1.0
 * \date 2011
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 ***********************************************/

#include "Utils3D.h"

using namespace Utils3D;

namespace Utils3D
{



CvMat* createIdentityMatrix()
{
	CvMat * mat = cvCreateMat(4,4,CV_64FC1);
	cvmSet(mat,0,0,1);
    cvmSet(mat,0,1,0);
    cvmSet(mat,0,2,0);
    cvmSet(mat,0,3,0);
    cvmSet(mat,1,0,0);
    cvmSet(mat,1,1,1);
    cvmSet(mat,1,2,0);
    cvmSet(mat,1,3,0);
    cvmSet(mat,2,0,0);
    cvmSet(mat,2,1,0);
    cvmSet(mat,2,2,1);
    cvmSet(mat,2,3,0);
    cvmSet(mat,3,0,0);
    cvmSet(mat,3,1,0);
    cvmSet(mat,3,2,0);
    cvmSet(mat,3,3,1);
	return mat;
}

CvMat* createRotationMatrixX( double angle )
{
	CvMat * mat = cvCreateMat(4,4,CV_64FC1);
	cvmSet(mat,0,0,1);
    cvmSet(mat,0,1,0);
    cvmSet(mat,0,2,0);
    cvmSet(mat,0,3,0);
    cvmSet(mat,1,0,0);
    cvmSet(mat,1,1,cos(angle));
    cvmSet(mat,1,2,-sin(angle));
    cvmSet(mat,1,3,0);
    cvmSet(mat,2,0,0);
    cvmSet(mat,2,1,sin(angle));
    cvmSet(mat,2,2,cos(angle));
    cvmSet(mat,2,3,0);
    cvmSet(mat,3,0,0);
    cvmSet(mat,3,1,0);
    cvmSet(mat,3,2,0);
    cvmSet(mat,3,3,1); 
	return mat;
}


CvMat* createRotationMatrixY( double angle )
{
	CvMat * mat = cvCreateMat(4,4,CV_64FC1);
	cvmSet(mat,0,0,cos(angle));
    cvmSet(mat,0,1,0);
    cvmSet(mat,0,2,sin(angle));
    cvmSet(mat,0,3,0);
    cvmSet(mat,1,0,0);
    cvmSet(mat,1,1,1);
    cvmSet(mat,1,2,0);
    cvmSet(mat,1,3,0);
    cvmSet(mat,2,0,-sin(angle));
    cvmSet(mat,2,1,0);
    cvmSet(mat,2,2,cos(angle));
    cvmSet(mat,2,3,0);
    cvmSet(mat,3,0,0);
    cvmSet(mat,3,1,0);
    cvmSet(mat,3,2,0);
    cvmSet(mat,3,3,1);
	return mat;
}


CvMat* createRotationMatrixZ( double angle )
{
	CvMat * mat = cvCreateMat(4,4,CV_64FC1);
	cvmSet(mat,0,0,cos(angle));
    cvmSet(mat,0,1,-sin(angle));
    cvmSet(mat,0,2,0);
    cvmSet(mat,0,3,0);
    cvmSet(mat,1,0,sin(angle));
    cvmSet(mat,1,1,cos(angle));
    cvmSet(mat,1,2,0);
    cvmSet(mat,1,3,0);
    cvmSet(mat,2,0,0);
    cvmSet(mat,2,1,0);
    cvmSet(mat,2,2,1);
    cvmSet(mat,2,3,0);
    cvmSet(mat,3,0,0);
    cvmSet(mat,3,1,0);
    cvmSet(mat,3,2,0);
    cvmSet(mat,3,3,1);   
	return mat;
}


CvMat* createTranslationMatrix( CvPoint3D32f* trans )
{
	CvMat * mat = cvCreateMat(4,4,CV_64FC1);
	cvmSet(mat,0,0,1);
    cvmSet(mat,0,1,0);
    cvmSet(mat,0,2,0);
    cvmSet(mat,0,3,trans->x);
    cvmSet(mat,1,0,0);
    cvmSet(mat,1,1,1);
    cvmSet(mat,1,2,0);
    cvmSet(mat,1,3,trans->y);
    cvmSet(mat,2,0,0);
    cvmSet(mat,2,1,0);
    cvmSet(mat,2,2,1);
    cvmSet(mat,2,3,trans->z);
    cvmSet(mat,3,0,0);
    cvmSet(mat,3,1,0);
    cvmSet(mat,3,2,0);
    cvmSet(mat,3,3,1);
	return mat;
}

CvMat* createRotationCoordinateSystemMatrixX( double angle )
{
	CvMat * mat = cvCreateMat(4,4,CV_64FC1);
	cvmSet(mat,0,0,1);
    cvmSet(mat,0,1,0);
    cvmSet(mat,0,2,0);
    cvmSet(mat,0,3,0);
    cvmSet(mat,1,0,0);
    cvmSet(mat,1,1,cos(angle));
    cvmSet(mat,1,2,sin(angle));
    cvmSet(mat,1,3,0);
    cvmSet(mat,2,0,0);
    cvmSet(mat,2,1,-sin(angle));
    cvmSet(mat,2,2,cos(angle));
    cvmSet(mat,2,3,0);
    cvmSet(mat,3,0,0);
    cvmSet(mat,3,1,0);
    cvmSet(mat,3,2,0);
    cvmSet(mat,3,3,1); 
	return mat;
}


CvMat* createRotationCoordinateSystemMatrixY( double angle )
{
	CvMat * mat = cvCreateMat(4,4,CV_64FC1);
	cvmSet(mat,0,0,cos(angle));
    cvmSet(mat,0,1,0);
    cvmSet(mat,0,2,-sin(angle));
    cvmSet(mat,0,3,0);
    cvmSet(mat,1,0,0);
    cvmSet(mat,1,1,1);
    cvmSet(mat,1,2,0);
    cvmSet(mat,1,3,0);
    cvmSet(mat,2,0,sin(angle));
    cvmSet(mat,2,1,0);
    cvmSet(mat,2,2,cos(angle));
    cvmSet(mat,2,3,0);
    cvmSet(mat,3,0,0);
    cvmSet(mat,3,1,0);
    cvmSet(mat,3,2,0);
    cvmSet(mat,3,3,1);
	return mat;
}


CvMat* createRotationCoordinateSystemMatrixZ( double angle )
{
	CvMat * mat = cvCreateMat(4,4,CV_64FC1);
	cvmSet(mat,0,0,cos(angle));
    cvmSet(mat,0,1,sin(angle));
    cvmSet(mat,0,2,0);
    cvmSet(mat,0,3,0);
    cvmSet(mat,1,0,-sin(angle));
    cvmSet(mat,1,1,cos(angle));
    cvmSet(mat,1,2,0);
    cvmSet(mat,1,3,0);
    cvmSet(mat,2,0,0);
    cvmSet(mat,2,1,0);
    cvmSet(mat,2,2,1);
    cvmSet(mat,2,3,0);
    cvmSet(mat,3,0,0);
    cvmSet(mat,3,1,0);
    cvmSet(mat,3,2,0);
    cvmSet(mat,3,3,1);   
	return mat;
}

void generatePlaneVectorsFromNormal(CvPoint3D32f*plainNormal,CvPoint3D32f* v1,CvPoint3D32f*v2)
{
	CvPoint3D32f xAxis = cvPoint3D32f(1,0,0);
	CvPoint3D32f yAxis = cvPoint3D32f(0,1,0);
	CvPoint3D32f zAxis = cvPoint3D32f(0,0,1);
	
	CvPoint3D32f vis1 = cvPoint3D32f(0,0,0);
	CvPoint3D32f vis2 = cvPoint3D32f(0,0,0);

	Utils3D::crossProduct(plainNormal,&xAxis,&vis1);
	Utils3D::crossProduct(&vis1,plainNormal,&vis2);


	if ( fabs(plainNormal->x) > .999999f )
	{
		Utils3D::setCoords(&yAxis ,&vis1);
		if ( plainNormal->x < 0)
			Utils3D::scalePoint(&vis1,-1);

		Utils3D::setCoords(&zAxis ,&vis2);
	}

	if ( fabs(plainNormal->y) > .999999f )
	{
		Utils3D::setCoords(&yAxis ,&vis1);
		if ( plainNormal->y < 0)
			Utils3D::scalePoint(&vis1,-1);
		Utils3D::setCoords(&zAxis ,&vis2);
	}
	
	Utils3D::normalize(& vis1);
	Utils3D::normalize(& vis2);


	Utils3D::setCoords(&vis1,v1);
	Utils3D::setCoords(&vis2,v2);

}

CvMat* createTransformationMatrix(CvPoint3D32f* trans, double rotX, double rotY, double rotZ)
{
	CvMat * mat = createIdentityMatrix();
	CvMat * rotMx = createRotationMatrixX(rotX);
	CvMat * rotMy = createRotationMatrixY(rotY);
	CvMat * rotMz = createRotationMatrixZ(rotZ);
	CvMat * transM = createTranslationMatrix(trans);

	cvMatMul(rotMx, mat, mat);
	cvMatMul(rotMy, mat, mat);
	cvMatMul(rotMz, mat, mat);
	cvMatMul(transM, mat, mat);

	cvReleaseMat(&rotMx);
	cvReleaseMat(&rotMy);
	cvReleaseMat(&rotMz);
	cvReleaseMat(&transM);

	return mat;
}

//CvMat* createTransformationMatrix(double transX, double transY, double transZ, double rotX, double rotY, double rotZ)
//{
//	return createTransformationMatrix(&(cvPoint3D32f(transX, transY, transZ)), rotX, rotY, rotZ);
//}

CvMat* createTransformationMatrix(CvPoint3D32f* trans, CvPoint3D32f* rot)
{
	return createTransformationMatrix(trans, rot->x, rot->y, rot->z);
}


//this function needs a homogene 4x4 matrix
void transformPoint(CvPoint3D32f* point, CvMat* transformationMat)
{
	CvMat * pointVec = cvCreateMat(4,1,CV_64FC1);

	cvmSet(pointVec,0,0,point->x);
    cvmSet(pointVec,1,0,point->y);
    cvmSet(pointVec,2,0,point->z);
    cvmSet(pointVec,3,0,1);

	cvMatMul(transformationMat, pointVec, pointVec);

	double w = cvmGet(pointVec,3,0);

	if (w == 0)
	{
		cout << "ERROR in transformPoint... w==0" << endl;
		exit(-2357232);
	}
	else
	{
		point->x = float( cvmGet(pointVec,0,0) / w );
		point->y = float( cvmGet(pointVec,1,0) / w );
		point->z = float( cvmGet(pointVec,2,0) / w );
	}
	cvReleaseMat(&pointVec);
}

//this function needs a homogene 4x4 matrix
void transformDirection(CvPoint3D32f* dir, CvMat* transformationMat)
{

	CvMat * dirVec = cvCreateMat(4,1,CV_64FC1);

	cvmSet(dirVec,0,0,dir->x);
    cvmSet(dirVec,1,0,dir->y);
    cvmSet(dirVec,2,0,dir->z);
    cvmSet(dirVec,3,0,0);

	cvMatMul(transformationMat, dirVec, dirVec);
	
	dir->x = float( cvmGet(dirVec,0,0) );
	dir->y = float( cvmGet(dirVec,1,0) );
	dir->z = float( cvmGet(dirVec,2,0) );

	normalize(dir);

}


void normalize(CvPoint3D32f* vec)
{
	float length = sqrt((vec->x * vec->x) + (vec->y * vec->y) + (vec->z * vec->z));
	if(length > 0)
	{
		vec->x = vec->x / length;
		vec->y = vec->y / length;
		vec->z = vec->z / length;
	}
	else
	{
		/*
		cout << "ERROR Utils3D: normalize with vector length = 0 !!" << endl;
		exit(-23112893);*/

	}
}

void fillStraightLine(CvPoint3D32f loc, CvPoint3D32f dir, STRAIGHTLINE * line)
{
	fillStraightLine(loc.x,loc.y,loc.z,dir.x,dir.y,dir.z, line);
}

void fillStraightLine(float locX, float locY, float locZ, float dirX, float dirY, float dirZ, STRAIGHTLINE * line)
{
	line->loc.x = locX;
	line->loc.y = locY;
	line->loc.z = locZ;

	line->dir.x = dirX;
	line->dir.y = dirY;
	line->dir.z = dirZ;

	normalize(&(line->dir));
}



void fillPlane(CvPoint3D32f loc, CvPoint3D32f dir1, CvPoint3D32f dir2, PLANE * plane)
{
	fillPlane(loc.x, loc.y, loc.z, dir1.x, dir1.y, dir1.z, dir2.x, dir2.y, dir2.z, plane);
}

void fillPlane(float locX, float locY, float locZ, float dir1X, float dir1Y, float dir1Z, float dir2X, float dir2Y, float dir2Z, PLANE * plane)
{
	plane->loc.x = locX;
	plane->loc.y = locY;
	plane->loc.z = locZ;

	plane->dir1.x = dir1X;
	plane->dir1.y = dir1Y;
	plane->dir1.z = dir1Z;

	plane->dir2.x = dir2X;
	plane->dir2.y = dir2Y;
	plane->dir2.z = dir2Z;

	normalize(&(plane->dir1));
	normalize(&(plane->dir2));

}

void transformStraightLine(STRAIGHTLINE* line, CvMat* transformationMat)
{
	transformPoint(&(line->loc), transformationMat);
	transformDirection(&(line->dir), transformationMat);
}


void transformPlane(PLANE* plane, CvMat* transformationMat)
{
	transformPoint(&(plane->loc), transformationMat);
	transformDirection(&(plane->dir1), transformationMat);
	transformDirection(&(plane->dir2), transformationMat);
}


double intersectLinePlane(STRAIGHTLINE line, PLANE plain)
{
#if 0
	double a[3][4];
	/*
	(0) a00*x1 + a01*x2 + a02*x3 = b0 
	(1) a10*x1 + a11*x2 + a12*x3 = b1 
	(2) a20*x1 + a20*x2 + a22*x3 = b2 
	*/

	// initialisiere Gleichungssystem
	a[0][0] = plain.dir1.x; a[0][1] = plain.dir2.x; a[0][2] = - line.dir.x; a[0][3] = line.loc.x - plain.loc.x;
	a[1][0] = plain.dir1.y; a[1][1] = plain.dir2.y; a[1][2] = - line.dir.y; a[1][3] = line.loc.y - plain.loc.y;
	a[2][0] = plain.dir1.z; a[2][1] = plain.dir2.z; a[2][2] = - line.dir.z; a[2][3] = line.loc.z - plain.loc.z;

	// falls a[0][0]=0, vertausche gegebenfalls Gleichungen
	if (a[0][0] == 0.0)
	{ 
		for(int i = 0; i < 4; i++) 
		{
			double t = a[0][i];
			a[0][i] = a[1][i];
			a[1][i] = t;
		}

	}
	if (a[0][0] == 0.0)
	{ 
		for(int i = 0; i < 4; i++) 
		{
			double t = a[0][i];
			a[0][i] = a[2][i];
			a[2][i] = t;
		}
	}
	// multipliziere die 1.Gl mit a10/a00 und ziehe sie von der 2.Gl ab und
	// speichere das Ergebnis als neue 2.Gl ab
	if (a[1][0] != 0.0) 
	{
		double d = a[1][0] / a[0][0];
		for(int i = 0; i < 4; i++) a[1][i] = a[1][i] - (d*a[0][i]);
	}

	// multipliziere die 1.Gl mit a20/a00 und ziehe es von der 3.Gl ab und
	// speichere das Ergebnis als neue 3.Gl ab
	if (a[2][0] != 0.0) 
	{
		double d = a[2][0] / a[0][0];
		for(int i = 0; i < 4; i++) a[2][i] = a[2][i] - (d*a[0][i]);
	}

	// falls a[1][1]=0, vertausche gegebenfalls Gleichungen
	if (a[1][1] == 0.0) 
	{
		for(int i = 0; i < 4; i++) 
		{
			double t = a[1][i];
			a[1][i] = a[2][i];
			a[2][i] = t;
		}
	}

	// multipliziere die 2.Gl mit a21/a11 und ziehe es von der 3.Gl ab und
	// speichere das Ergebnis als neue 3.Gl ab
	if (a[2][1] != 0.0) 
	{
		double d = a[2][1] / a[1][1];
		for(int i = 0; i < 4; i++) a[2][i] = a[2][i] - (d*a[1][i]);
	}

	double x3;
	// berechne nun X3
	if (a[2][3] == 0.0) x3 = 0.0;
	else x3 = a[2][3]/a[2][2];

	return (float) x3;
#else

	CvPoint3D32f n = cvPoint3D32f(0,0,0);
	crossProduct(&plain.dir1,&plain.dir2,&n);
	CvPoint3D32f diff = cvPoint3D32f(0,0,0);
	Utils3D::subPoints(&plain.loc,&line.loc,&diff);
	double scal = dotProd(&n,&line.dir)/getNorm(&n)/getNorm(&line.dir);
	if ( scal == 0)
	{
		// doSomething()
	}
	return dotProd(&n,&diff)/scal;
#endif
}

	STRAIGHTLINE intersectPlanes(PLANE2 P1,PLANE2 P2)
	{
		STRAIGHTLINE line;
		crossProduct(&P1.norm,&P2.norm,&line.dir);
		
		//CvPoint3D32f diff = cvPoint3D32f(0,0,0);

		STRAIGHTLINE tline;

		CvPoint3D32f pPoint = projectPointOnPlane(P1,P2.loc);

		subPoints(&P1.loc,&pPoint,&tline.dir);

		tline.loc = P1.loc;

		double l = intersectLinePlane(tline,P2);

		scalePoint(&tline.dir, l);

		addPoints(&tline.loc,&tline.dir,&line.loc);

		return line;
	}

	double getPointPlaneDistance(PLANE2 P, CvPoint3D32f p)
	{
		CvPoint3D32f diff = cvPoint3D32f(0,0,0);
		subPoints(&P.loc,&p,&diff);
		return dotProd(&diff,&P.norm);
	}

	CvPoint3D32f projectPointOnPlane(PLANE2 P,CvPoint3D32f p)
	{
		double dist = getPointPlaneDistance(P,p);
		CvPoint3D32f n = cvPoint3D32f(0,0,0);
		setCoords(&P.norm,&n);
		scalePoint(&n,-dist);
		CvPoint3D32f r = cvPoint3D32f(0,0,0);
		addPoints(&p,&n,&r);
		return r;
	}

	CvPoint3D32f projectPointOnLine(STRAIGHTLINE line,CvPoint3D32f* p)
	{
		CvPoint3D32f d = cvPoint3D32f(0,0,0);
		subPoints(&line.loc,p,&d);
		float k = (float)dotProd(&d,&line.dir);
		addWPoints(&line.loc,1,&line.dir,-k,&d);
		return d;				
	}

	double intersectLinePlane(STRAIGHTLINE line, PLANE2 plane)
	{
		double d = getPointPlaneDistance(plane,line.loc);
		double scal = dotProd(&line.dir,&plane.norm);
		if ( scal == 0)
		{

		}
		return d * getNorm(&plane.norm)*getNorm(&line.dir) / scal;
	}

	double distPoints(CvPoint3D32f* p1, CvPoint3D32f * p2)
	{
		CvPoint3D32f d = cvPoint3D32f(0,0,0);
		subPoints(p1,p2,&d);
		return getNorm(&d);
	}

//	double distPointLine( CvPoint3D32f* p, STRAIGHTLINE line)
//	{
//		/*
//		CvPoint3D32f d = cvPoint3D32f(0,0,0);
//		subPoints(p,&line.loc,&d);
//		double k = dotProd(&d,&line.dir);
//		scalePoint(&d,k);
//		return distPoints(&d,p);
//		*/
//		return getNorm(&projectPointOnLine(line,p));
//	}

	 double dotProd(CvPoint3D32f* p1, CvPoint3D32f* p2 )
	{
		return p1->x * p2->x + p1->y * p2->y + p1->z * p2->z;
	}

	 double getAngle(CvPoint3D32f* p1, CvPoint3D32f* p2 )
	{
		return acos (dotProd ( p1 , p2 ) / getNorm(p1) / getNorm(p2));
	}
	
	 CvPoint3D32f * cAddPoints(CvPoint3D32f* p1, CvPoint3D32f * p2)
	{
		return cAddWPoints(p1,1,p2,1);
	}

	 void addPoints(CvPoint3D32f* p1, CvPoint3D32f * p2,CvPoint3D32f*res)
	{
		addWPoints(p1,1,p2,1,res);
	}

	 CvPoint3D32f * cAddWPoints(CvPoint3D32f * p1, float w1, CvPoint3D32f * p2, float w2 )
	{
		CvPoint3D32f * np = new CvPoint3D32f;
		np->x = w1* p1->x + w2 * p2->x;
		np->y = w1* p1->y + w2 * p2->y;
		np->z = w1* p1->z + w2 * p2->z;
		return np;
	}

	void addWPoints(CvPoint3D32f * p1, float w1, CvPoint3D32f * p2, float w2, CvPoint3D32f*np )
	{
		np->x = w1* p1->x + w2 * p2->x;
		np->y = w1* p1->y + w2 * p2->y;
		np->z = w1* p1->z + w2 * p2->z;
	}

	 CvPoint3D32f * cSubPoints(CvPoint3D32f* p1, CvPoint3D32f * p2)
	{
		return cAddWPoints(p1,1,p2,-1);
	}

	 void subPoints(CvPoint3D32f* p1, CvPoint3D32f * p2,CvPoint3D32f* res)
	{
		addWPoints(p1,1,p2,-1,res);
	}

	 CvPoint3D32f * cCrossProduct(CvPoint3D32f* p1, CvPoint3D32f* p2)
	{
		CvPoint3D32f * np = new CvPoint3D32f;
		np->x = p1->y * p2->z - p1->z * p2->y;
		np->y = p1->z * p2->x - p1->x * p2->z;
		np->z = p1->x * p2->y - p1->y * p2->x;
		return np;
	}

	 void crossProduct(CvPoint3D32f* p1, CvPoint3D32f* p2,CvPoint3D32f* np)
	{
		np->x = p1->y * p2->z - p1->z * p2->y;
		np->y = p1->z * p2->x - p1->x * p2->z;
		np->z = p1->x * p2->y - p1->y * p2->x;
	}

	 CvPoint3D32f * cGetPoint(float x, float y, float z )
	{
		CvPoint3D32f * p = new CvPoint3D32f;
		p->x = x;
		p->y = y;
		p->z = z;
		return p;
	}

	 CvPoint3D32f * cCopyPoint(CvPoint3D32f* p )
	{
		CvPoint3D32f * o = new CvPoint3D32f ();
		o->x = p->x;
		o->y = p->y;
		o->z = p->z;
		return o;
	}

	 void swapPoints(CvPoint3D32f* p1, CvPoint3D32f* p2)
	{
		CvPoint3D32f* p = cCopyPoint(p1);
		p1->x = p2->x;
		p1->y = p2->y;
		p1->z = p2->z;
		p2->x = p->x;
		p2->y = p->y;
		p2->z = p->z;
		delete p;
	}

	 double getNorm(CvPoint3D32f* p )
	{
		return sqrt(p->x*p->x + p->y*p->y + p->z*p->z );
	}


	 void setCoords (CvPoint3D32f * in, CvPoint3D32f * out)
	{
		out->x = in->x;
		out->y = in->y;
		out->z = in->z;
	}

	 void setCoords (CvPoint3D32f * out, float x, float y, float z)
	{

		out->x = x;
		out->y = y;
		out->z = z;
	}

	 void scalePoint(CvPoint3D32f* p, double scale)
	{
		p->x *= (float)scale;
		p->y *= (float)scale;
		p->z *= (float)scale;
	}

	 void orthonormalizePoints(CvPoint3D32f* p1, CvPoint3D32f* p2,CvPoint3D32f*p3)
	{	
		orthonormalizePoints(p1,p2,p3,1);
	}

	 void orthonormalizePoints(CvPoint3D32f* p1, CvPoint3D32f* p2,CvPoint3D32f*p3,double scale)
	{
		CvPoint3D32f * e1 = cSubPoints(p2,p1);
		CvPoint3D32f * e2 = cSubPoints(p3,p1);

		normalize(e1);
		normalize(e2);

		CvPoint3D32f * e3 = cCrossProduct(e1,e2);
		
		normalize(e3);

		delete e2;

		e2 = cCrossProduct(e3,e1);

		delete e3;

		normalize(e2);

		scalePoint(e1,scale);
		scalePoint(e2,scale);

		addPoints(p1,e1,e1);
		addPoints(p1,e2,e2);
		setCoords(e1,p2);
		setCoords(e2,p3);

		delete e2;
		delete e1;
		
	}


	 CvMat* cGetBasisFrom3Points(CvPoint3D32f* p1, CvPoint3D32f * p2, CvPoint3D32f * p3 )
	{

		orthonormalizePoints(p1,p2,p3);

		CvPoint3D32f * e1 = cSubPoints(p2,p1);
		normalize(e1);
		CvPoint3D32f * e2 = cSubPoints(p3,p1);
		normalize(e2);
		CvPoint3D32f * e3 = cCrossProduct(e1,e2);
		normalize(e3);

		double b[] = {e1->x,e2->x,e3->x, 0,
					  e1->y,e2->y,e3->y, 0,
					  e1->z,e2->z,e3->z, 0,
					    0  ,  0  ,  0  , 1};

		CvMat tmp;

		cvInitMatHeader(&tmp,4,4,CV_64FC1,b);

		CvMat * P = cvCloneMat(&tmp);

		delete e1;
		delete e2;
		delete e3;

		return P;
	}

	CvMat* cGetBasisFrom3Points3x3(CvPoint3D32f* p1, CvPoint3D32f * p2, CvPoint3D32f * p3 )
	{

		orthonormalizePoints(p1,p2,p3);

		CvPoint3D32f * e1 = cSubPoints(p2,p1);
		normalize(e1);
		CvPoint3D32f * e2 = cSubPoints(p3,p1);
		normalize(e2);
		CvPoint3D32f * e3 = cCrossProduct(e1,e2);
		normalize(e3);

		double b[] = {e1->x,e2->x,e3->x, 
					  e1->y,e2->y,e3->y, 
					  e1->z,e2->z,e3->z};

		CvMat tmp;

		cvInitMatHeader(&tmp,3,3,CV_64FC1,b);

		CvMat * P = cvCloneMat(&tmp);

		delete e1;
		delete e2;
		delete e3;

		return P;
	}

	CvMat * createMatrixFrom3Points(CvPoint3D32f* p1, CvPoint3D32f* p2,CvPoint3D32f* p3)
	{
		CvMat * ret = cvCreateMat(3,3,CV_32FC1);
		cvmSet(ret,0,0,p1->x);
		cvmSet(ret,0,1,p1->y);
		cvmSet(ret,0,2,p1->z);

		cvmSet(ret,1,0,p2->x);
		cvmSet(ret,1,1,p2->y);
		cvmSet(ret,1,2,p2->z);

		cvmSet(ret,2,0,p3->x);
		cvmSet(ret,2,1,p3->y);
		cvmSet(ret,2,2,p3->z);

		return ret;
	}

	CvMat * cGetMat32fFrom64d(CvMat* mat64d)
	{
		CvMat * mat32f = cvCreateMat(mat64d->rows,mat64d->cols,CV_32FC1);
		cvCvtScale(mat64d,mat32f);
		return mat32f;
	}

	CvMat * cGetMat64dFrom32f(CvMat* mat32f)
	{
		CvMat * mat64d = cvCreateMat(mat32f->rows,mat32f->cols,CV_64FC1);
		cvCvtScale(mat32f,mat64d);
		return mat64d;
	}


	CvPoint3D32f * cRectifyPoint(CvPoint3D32f * oriPoint, float sensorLengthScale, float sensorWidthScale, float sensorHeightScale, CvMat * map_matrix)
	{
		CvPoint3D32f * retVal = new CvPoint3D32f();

		retVal->y = oriPoint->y * sensorLengthScale;
		retVal->z = (oriPoint->z * 16) * sensorHeightScale;

		CvMat* vec = cvCreateMat( 3, 1, CV_32F );
		cvmSet( vec, 0, 0, oriPoint->x );
		cvmSet( vec, 1, 0, 8192 - (oriPoint->z * 16) );
		cvmSet( vec, 2, 0, 1);

		CvMat* erg = cvCreateMat( 3, 1, CV_32F );
		cvMatMul(map_matrix, vec, erg);

		retVal->x = (float)( cvmGet(erg, 0, 0) / cvmGet(erg, 2, 0));

		cvReleaseMat( &vec );
		cvReleaseMat( &erg );
	
		if(sensorWidthScale != 0)
		{
			retVal->x *= sensorWidthScale;
		}

		return retVal;
	}

	CvPoint3D32f * cInverseRectifyPoint(CvPoint3D32f * oriPoint, float sensorLenghtScale, float sensorWidthScale, float sensorHeightScale, CvMat * map_matrix)
	{
		assert(sensorWidthScale != 0);
		assert(sensorLenghtScale != 0);
		assert(sensorHeightScale != 0);
		assert(map_matrix);

		CvPoint3D32f * retVal = new CvPoint3D32f();

		CvMat * inverse_map_matrix = cvCreateMat( 3, 3, CV_32F );
		cvInvert( map_matrix, inverse_map_matrix, CV_LU );

		retVal->y = oriPoint->y / sensorLenghtScale;
		retVal->z = (oriPoint->z / 16) / sensorHeightScale;

		CvMat* vec = cvCreateMat( 3, 1, CV_32F );
		cvmSet( vec, 0, 0, oriPoint->x / sensorWidthScale );
		cvmSet( vec, 1, 0, 8192 - ((oriPoint->z) / sensorHeightScale));
		cvmSet( vec, 2, 0, 1);

		CvMat* erg = cvCreateMat( 3, 1, CV_32F );
		cvMatMul(inverse_map_matrix, vec, erg);

		retVal->x = (float) (cvmGet(erg, 0, 0) / cvmGet(erg, 2, 0));

		cvReleaseMat( &vec );
		cvReleaseMat( &erg );
		cvReleaseMat( &inverse_map_matrix );
	
		return retVal;
	}

	CvMat* cCalcRectificationMatrix(CvPoint2D32f* oriP1, CvPoint2D32f* oriP2, CvPoint2D32f* oriP3, CvPoint2D32f* oriP4, CvPoint2D32f* dstP1, CvPoint2D32f* dstP2,CvPoint2D32f* dstP3,CvPoint2D32f* dstP4)
	{
		CvMat * map_matrix = cvCreateMat( 3, 3, CV_32F );
		CvPoint2D32f src[4];
		CvPoint2D32f dst[4];

		src[0] = *oriP1;
		src[1] = *oriP2;
		src[2] = *oriP3;
		src[3] = *oriP4;

		dst[0] = *dstP1;
		dst[1] = *dstP2;
		dst[2] = *dstP3;
		dst[3] = *dstP4;

		cvGetPerspectiveTransform( src,dst, map_matrix );
		return map_matrix;
	}


	float getShortestDistanceBetweenTwoLines( STRAIGHTLINE line1, STRAIGHTLINE line2, float * d1, float * d2)
	{
		CvPoint3D32f n;
		crossProduct(&line1.dir,&line2.dir,&n);
		CvMat* m = cvCreateMat(3,3,CV_MAT32F);
		CV_MAT_ELEM(*m,float,0,0) = line1.dir.x;
		CV_MAT_ELEM(*m,float,1,0) = line1.dir.y;
		CV_MAT_ELEM(*m,float,2,0) = line1.dir.z;

		CV_MAT_ELEM(*m,float,0,1) = -line2.dir.x;
		CV_MAT_ELEM(*m,float,1,1) = -line2.dir.y;
		CV_MAT_ELEM(*m,float,2,1) = -line2.dir.z;

		CV_MAT_ELEM(*m,float,0,2) = -n.x;
		CV_MAT_ELEM(*m,float,1,2) = -n.y;
		CV_MAT_ELEM(*m,float,2,2) = -n.z;

		CvPoint3D32f diff;

		subPoints(&line2.loc,&line1.loc,&diff);

		CvMat* p = cvCreateMat(3,1,CV_MAT32F);
		CV_MAT_ELEM(*p,float,0,0) = diff.x;
		CV_MAT_ELEM(*p,float,1,0) = diff.y;
		CV_MAT_ELEM(*p,float,2,0) = diff.z;


		cvInvert(m,m);

		cvMatMul(m,p,p);

		*d1 = CV_MAT_ELEM(*p,float,0,0);
		*d2 = CV_MAT_ELEM(*p,float,1,0);
		float ret = CV_MAT_ELEM(*p,float,2,0);

		cvReleaseMat(&m);
		cvReleaseMat(&p);

		return ret;
	}

	float getLineParameter(STRAIGHTLINE line, CvPoint3D32f p)
	{
		CvPoint3D32f d = cvPoint3D32f(0,0,0);
		subPoints(&line.loc,&p,&d);
		float k = (float) dotProd(&d,&line.dir);
		return -k;
	}

	CvPoint3D32f getLinePoint(STRAIGHTLINE line, float param);

	CvPoint3D32f getLinePoint(STRAIGHTLINE line, float param)
	{
		CvPoint3D32f p;
		addWPoints(&line.loc,1,&line.dir,param,&p);
		return p;
	}
}
