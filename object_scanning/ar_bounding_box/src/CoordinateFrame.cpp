/** \file CoordinateFrame.cpp
 * \brief A helper class for working with coordinate systems.
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

#include "CoordinateFrame.h"

#include <stdio.h>

using namespace Utils3D;


/////////////////////////////////////////////////////////////////////////////
//////////////////////////////Class CoordinateFrame////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

CoordinateFrame::CoordinateFrame(CvPoint3D32f position, CvPoint3D32f rotation)
{
	this->pos = position;
	this->rot = rotation;
}

CoordinateFrame::CoordinateFrame(float posX, float posY, float posZ, float rotX, float rotY, float rotZ)
{
	this->pos = cvPoint3D32f(posX,posY,posZ);
	this->rot = cvPoint3D32f(rotX,rotY,rotZ);
}

CoordinateFrame::CoordinateFrame(CvPoint3D32f position, float rotX, float rotY, float rotZ)
{
	this->pos = position;
	this->rot = cvPoint3D32f(rotX,rotY,rotZ);
}

CoordinateFrame::CoordinateFrame(float posX, float posY, float posZ, CvPoint3D32f rotation)
{
	this->pos = cvPoint3D32f(posX,posY,posZ);
	this->rot = rotation;
}

CoordinateFrame::CoordinateFrame(CvPoint3D32f* p1, CvPoint3D32f* p2, CvPoint3D32f* p3)
{
	orthonormalizePoints(p1,p2,p3);
	CvPoint3D32f vx;
	CvPoint3D32f vy;
	CvPoint3D32f vz;
	
	subPoints(p2,p1,&vx);
	subPoints(p3,p1,&vy);

	crossProduct(&vx,&vy,&vz);
	
	float phi = atan2(vx.y,vx.x);
	float psi = atan2(vy.z,vz.z);
	float theta = atan2(-vx.z,vx.x*cos(phi)+vx.y*sin(phi));

	pos = *p1;
	rot = cvPoint3D32f(psi,theta,phi);
}

CoordinateFrame::CoordinateFrame(cv::Point3f p1, cv::Point3f p2, cv::Point3f p3)
{
    cv::Point3f e1, e2;

    e1 = p2 - p1;
    e2 = p3 - p1;

    cv::norm(e1);
    cv::norm(e2);

    cv::Point3f e3 = e1.cross(e2);
    cv::norm(e3);
    e2 = e3.cross(e1);
    cv::norm(e2);

    e1 += p1;
    e2 += p2;

    // orthonormalized
    cv::Point3f b1(p1), b2(e1), b3(e2);
    cv::Point3f vx = b2 - b1;
    cv::Point3f vy = b3 - b1;
    cv::Point3f vz = vx.cross(vz);

    double phi = atan2(vx.y, vx.x);
    double psi = atan2(vy.z, vz.z);
    double theta = atan2(-vx.z, vx.x*cos(phi)+vx.y*sin(phi));
    pos = b1;
    rot = cv::Point3d(psi, theta, phi);
}


CoordinateFrame::CoordinateFrame(CvMat* trafoMat)
{

	if ( trafoMat )
	{
	CvMat* thisToBaseMat = trafoMat;

	// cvInvert(thisToBaseMat,thisToBaseMat);

	CvPoint3D32f p1 = cvPoint3D32f(0,0,0);
	CvPoint3D32f p2 = cvPoint3D32f(1,0,0);
	CvPoint3D32f p3 = cvPoint3D32f(0,1,0);

	transformPoint(&p1, thisToBaseMat);
	transformPoint(&p2, thisToBaseMat);
	transformPoint(&p3, thisToBaseMat);
 	
	CvPoint3D32f vx;
	CvPoint3D32f vy;
	CvPoint3D32f vz;
	
	subPoints(&p2,&p1,&vx);
	subPoints(&p3,&p1,&vy);

	crossProduct(&vx,&vy,&vz);
	
	float phi = atan2(vx.y,vx.x);
	float psi = atan2(vy.z,vz.z);
	float theta = atan2(-vx.z,vx.x*cos(phi)+vx.y*sin(phi));

	pos = p1;
	rot = cvPoint3D32f(psi,theta,phi);

	}
}

CoordinateFrame::CoordinateFrame()
{
	this->pos = cvPoint3D32f(0,0,0);
	this->rot = cvPoint3D32f(0,0,0);
}


CoordinateFrame::~CoordinateFrame()
{

}

CvMat* CoordinateFrame::createTransformationMatrix()
{
	return Utils3D::createTransformationMatrix(&(this->pos), &(this->rot));
}

CvMat* CoordinateFrame::createTransformationMatrix3x3()
{

	CvMat * mat4x4 = Utils3D::createTransformationMatrix(&(this->pos), &(this->rot));

	CvMat * mat3x3 = cvCreateMat(3,3,CV_64FC1);

	for ( int i = 0; i < 3; i ++ )
	{
		for ( int j = 0; j < 3; j ++ )
		{
			CV_MAT_ELEM(*mat3x3,double,i,j) = CV_MAT_ELEM(*mat4x4,double,i,j);
		}
	}

	cvReleaseMat(&mat4x4);
	
	return mat3x3;
}

void CoordinateFrame::getThreePoints(CvPoint3D32f* p0,CvPoint3D32f* p1,CvPoint3D32f* p2)
{
	assert(p0);
	assert(p1);
	assert(p2);

        CoordinateFrame *retpos = new CoordinateFrame(0.,0.,0.,0.,0.,0.);
	
	CvMat* thisToWorldMat = this->createTransformationMatrix();

	Utils3D::setCoords(p0,0,0,0);
	Utils3D::setCoords(p1,1,0,0);
	Utils3D::setCoords(p2,0,1,0);

	transformPoint(p0, thisToWorldMat);
	transformPoint(p1, thisToWorldMat);
	transformPoint(p2, thisToWorldMat);
}


CoordinateFrame * CoordinateFrame::cInFrame(CoordinateFrame * base)
{
	assert(base);

        CoordinateFrame *retpos = new CoordinateFrame(0.,0.,0.,0.,0.,0.);
	
	CvMat* baseToWorldMat = base->createTransformationMatrix();
	CvMat* thisToWorldMat = this->createTransformationMatrix();

	CvMat * worldToBaseMat = createIdentityMatrix();

	cvInvert(baseToWorldMat,worldToBaseMat);

	CvPoint3D32f p1 = cvPoint3D32f(0,0,0);
	CvPoint3D32f p2 = cvPoint3D32f(1,0,0);
	CvPoint3D32f p3 = cvPoint3D32f(0,1,0);

	transformPoint(&p1, thisToWorldMat);
	transformPoint(&p2, thisToWorldMat);
	transformPoint(&p3, thisToWorldMat);
 	
	transformPoint(&p1, worldToBaseMat);
	transformPoint(&p2, worldToBaseMat);	
	transformPoint(&p3, worldToBaseMat);
	
	CvPoint3D32f vx;
	CvPoint3D32f vy;
	CvPoint3D32f vz;
	
	subPoints(&p2,&p1,&vx);
	subPoints(&p3,&p1,&vy);

	crossProduct(&vx,&vy,&vz);

	float phi = atan2(vx.y,vx.x);
	float psi = atan2(vy.z,vz.z);
	float theta = atan2(-vx.z,vx.x*cos(phi)+vx.y*sin(phi));

	retpos->pos = p1;
	retpos->rot = cvPoint3D32f(psi,theta,phi);

	cvReleaseMat(& thisToWorldMat);
	cvReleaseMat(& baseToWorldMat);

	return retpos;
}


CoordinateFrame * CoordinateFrame::cInWorld(CoordinateFrame * base)
{

	assert(base);

        CoordinateFrame *retpos = new CoordinateFrame(0.,0.,0.,0.,0.,0.);
	
	CvMat* thisToBaseMat = this->createTransformationMatrix();
	CvMat* baseToWorldMat = base->createTransformationMatrix();

	CvMat* thisToWorldMat = createIdentityMatrix();
	cvMatMul(baseToWorldMat,thisToBaseMat,thisToWorldMat);


	CvPoint3D32f p1 = cvPoint3D32f(0,0,0);
	CvPoint3D32f p2 = cvPoint3D32f(1,0,0);
	CvPoint3D32f p3 = cvPoint3D32f(0,1,0);

	transformPoint(&p1, thisToWorldMat);
	transformPoint(&p2, thisToWorldMat);
	transformPoint(&p3, thisToWorldMat);
 	
	CvPoint3D32f vx;
	CvPoint3D32f vy;
	CvPoint3D32f vz;
	
	subPoints(&p2,&p1,&vx);
	subPoints(&p3,&p1,&vy);

	crossProduct(&vx,&vy,&vz);
	
	float phi = atan2(vx.y,vx.x);
	float psi = atan2(vy.z,vz.z);
	float theta = atan2(-vx.z,vx.x*cos(phi)+vx.y*sin(phi));

	retpos->pos = p1;
	retpos->rot = cvPoint3D32f(psi,theta,phi);

	cvReleaseMat(& thisToBaseMat);
	cvReleaseMat(& thisToWorldMat);
	cvReleaseMat(& baseToWorldMat);

	return retpos;
}

CoordinateFrame * CoordinateFrame::cMakeFrame(CoordinateFrame * base)
{
	CvPoint3D32f p0 = cvPoint3D32f(0,0,0);
	CvPoint3D32f p1 = cvPoint3D32f(0,0,0);
	CvPoint3D32f p2 = cvPoint3D32f(0,0,0);
	this->getThreePoints(&p0,&p1,&p2);
	return cMakeFrame(&p0,&p1,&p2);
}

CoordinateFrame * CoordinateFrame::cMakeFrame(CvPoint3D32f* p0,CvPoint3D32f* p1,CvPoint3D32f* p2)
{

	CvPoint3D32f org = cvPoint3D32f(0,0,0);
	CvPoint3D32f rot = cvPoint3D32f(0,0,0);

	Utils3D::setCoords(p0,&org);

	CvPoint3D32f vx = cvPoint3D32f(0,0,0);
	CvPoint3D32f vy = cvPoint3D32f(0,0,0);
	CvPoint3D32f vz = cvPoint3D32f(0,0,0);
	CvPoint3D32f vv = cvPoint3D32f(0,0,0);
	CvPoint3D32f vw = cvPoint3D32f(0,0,0);


	Utils3D::subPoints(p1,p0,&vx);

	Utils3D::normalize(&vx);

	Utils3D::subPoints(p2,p0,&vv);

	Utils3D::crossProduct(&vx,&vv,&vw);

	Utils3D::normalize(&vw);

	Utils3D::crossProduct(&vx,&vw,&vy);

	Utils3D::normalize(&vy);

	Utils3D::crossProduct(&vx,&vy,&vz);

	Utils3D::normalize(&vz);

	float phi = atan2(vx.y,vx.x);
	float psi = atan2(vy.z,vz.z);
	float theta = atan2(-vx.z,vx.x*cos(phi)+vx.y*sin(phi));

	rot.x = psi;
	rot.y = theta;
	rot.z = phi;

        CoordinateFrame* localBase = new CoordinateFrame(org,rot);

	return localBase;
}


char * CoordinateFrame::toString()
{
	int bufferSize = 1250;
        char * format = "{Position: X %0.3f, Y %0.3f, Z %0.3f, A %0.3f, B %0.3f, C %0.3f}";
	char* buffer = new char[bufferSize];
	sprintf(buffer,format,pos.x,pos.y,pos.z,rot.z *(180.f/CV_PI), rot.y *(180.f/CV_PI), rot.x *(180.f/CV_PI));
	return buffer;
}

void CoordinateFrame::setPos(CvPoint3D32f* pos_)
{
	this->pos = * pos_;
}

void CoordinateFrame::setRot(CvPoint3D32f* rot_)
{
	this->rot = * rot_;
}

void CoordinateFrame::getXAxis(CvPoint3D32f* normal)
{
	CvPoint3D32f p0 = cvPoint3D32f(0,0,0);
	CvPoint3D32f p1 = cvPoint3D32f(0,0,0);
	CvPoint3D32f p2 = cvPoint3D32f(0,0,0);
	this->getThreePoints(&p0,&p1,&p2);
	Utils3D::subPoints(&p1,&p0,normal);
	Utils3D::normalize(normal);
}

void CoordinateFrame::getYAxis(CvPoint3D32f* normal)
{
	CvPoint3D32f p0 = cvPoint3D32f(0,0,0);
	CvPoint3D32f p1 = cvPoint3D32f(0,0,0);
	CvPoint3D32f p2 = cvPoint3D32f(0,0,0);
	this->getThreePoints(&p0,&p1,&p2);
	Utils3D::subPoints(&p2,&p0,normal);
	Utils3D::normalize(normal);
}

void CoordinateFrame::getZAxis(CvPoint3D32f* normal)
{
	CvPoint3D32f p0 = cvPoint3D32f(0,0,0);
	CvPoint3D32f p1 = cvPoint3D32f(0,0,0);
	CvPoint3D32f p2 = cvPoint3D32f(0,0,0);
	this->getThreePoints(&p0,&p1,&p2);
	CvPoint3D32f p2p0 = cvPoint3D32f(0,0,0);
	Utils3D::subPoints(&p2,&p0,&p2p0);
	CvPoint3D32f p1p0 = cvPoint3D32f(0,0,0);
	Utils3D::subPoints(&p1,&p0,&p1p0);
	Utils3D::crossProduct(&p1p0,&p2p0,normal);
	Utils3D::normalize(normal);
}

void CoordinateFrame::toggleZDirection()
{
	CvPoint3D32f p0 = cvPoint3D32f(0,0,0);
	CvPoint3D32f p1 = cvPoint3D32f(0,0,0);
	CvPoint3D32f p2 = cvPoint3D32f(0,0,0);
	this->getThreePoints(&p0,&p1,&p2);
	CvPoint3D32f p2p0 = cvPoint3D32f(0,0,0);
	Utils3D::subPoints(&p2,&p0,&p2p0);
	Utils3D::scalePoint(&p2p0,-1);
	Utils3D::addPoints(&p0,&p2p0,&p2);
        CoordinateFrame tmp = CoordinateFrame(&p0,&p1,&p2);
        this->setPos(tmp.getPos());
        this->setRot(tmp.getRot());
}
