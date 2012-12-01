/** \file Utils3D.h
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

#ifndef UTILS3D_DEFINED
#define UTILS3D_DEFINED


#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include <iostream>


using namespace std;

namespace Utils3D
{
	typedef struct _STRAIGHTLINE
	{ //in mm
		CvPoint3D32f loc;
                CvPoint3D32f dir;
	}STRAIGHTLINE;

	typedef struct _PLANE
	{ //in mm
		CvPoint3D32f loc;
		CvPoint3D32f dir1;
		CvPoint3D32f dir2;

	}PLANE;

	typedef struct _PLANE2
	{ //in mm
		CvPoint3D32f loc;
		CvPoint3D32f norm;
	}PLANE2;

        inline int sgn ( float in )
        {
            if ( in > 0)
                return 1;
            if ( in < 0 )
                return -1;
            return 0;
        }

        /// Creates a CvMat Identity-Matrix
	CvMat* createIdentityMatrix();
        /// Creates a CvMat Matrix for Rotation in X-Direction
	CvMat* createRotationMatrixX( double angle );
        /// Creates a CvMat Matrix for Rotation in Y-Direction
	CvMat* createRotationMatrixY( double angle );
        /// Creates a CvMat Matrix for Rotation in Z-Direction
	CvMat* createRotationMatrixZ( double angle );
        /// Creates a CvMat Matrix from a translation Vector
	CvMat* createTranslationMatrix( CvPoint3D32f* trans );

        /// Creates a CvMat Matrix for Rotation of the Coordinate System in X-Direction
	CvMat* createRotationCoordinateSystemMatrixX( double angle );
        /// Creates a CvMat Matrix for Rotation of the Coordinate System in Y-Direction
	CvMat* createRotationCoordinateSystemMatrixY( double angle );
        /// Creates a CvMat Matrix for Rotation of the Coordinate System in Z-Direction
	CvMat* createRotationCoordinateSystemMatrixZ( double angle );

	void generatePlaneVectorsFromNormal(CvPoint3D32f*normal,CvPoint3D32f* v1,CvPoint3D32f*v2);

        /// Creates a CvMat Matrix for translation and rotation
	CvMat* createTransformationMatrix(CvPoint3D32f* trans, double rotX, double rotY, double rotZ);
        /// Creates a CvMat Matrix for translation and rotation
//	CvMat* createTransformationMatrix(double transX, double transY, double transZ, double rotX, double rotY, double rotZ);
        /// Creates a CvMat Matrix for translation and rotation
	CvMat* createTransformationMatrix(CvPoint3D32f* trans, CvPoint3D32f* rot);

        /// fill in values in a given STRAIGHTLINE line
	void fillStraightLine(CvPoint3D32f * loc, CvPoint3D32f * dir, STRAIGHTLINE * line);
        /// fill in values in a given STRAIGHTLINE line
	void fillStraightLine(float locX, float locY, float locZ, float dirX, float dirY, float dirZ, STRAIGHTLINE * line);

        /// fill in values in a given PLANE plane
	void fillPlane(CvPoint3D32f loc, CvPoint3D32f dir1, CvPoint3D32f dir2, PLANE * plane);
        /// fill in values in a given PLANE plane
	void fillPlane(float locX, float locY, float locZ, float dir1X, float dir1Y, float dir1Z, float dir2X, float dir2Y, float dir2Z, PLANE * plane);

        /// normalizes a Vector
	void normalize(CvPoint3D32f* vec);

        /// this Function transforms a direction Vector with a transformation Matrix
	void transformDirection(CvPoint3D32f* dir, CvMat* transformationMat);

        /// this Function transforms a Point with a transformation Matrix
	void transformPoint(CvPoint3D32f* point, CvMat* transformationMat);

        /// this Function transforms a STRAIGHTLINE with a transformation Matrix
	void transformStraightLine(STRAIGHTLINE* line, CvMat* transformationMat);

        /// this Function transforms a PLANE with a transformation Matrix
	void transformPlane(PLANE* plane, CvMat* transformationMat);

        /**
         * this Function intersects a STRAIGHTLINE with PLANE and returns the parameter
         * of the line in the intersection point
         **/
	double intersectLinePlane(STRAIGHTLINE line, PLANE plane);
	
	STRAIGHTLINE intersectPlanes(PLANE2 P1, PLANE2 P2);

	double getPointPlaneDistance(PLANE2 P, CvPoint3D32f p);

	CvPoint3D32f projectPointOnPlane(PLANE2 P,CvPoint3D32f p);

	CvPoint3D32f projectPointOnLine(STRAIGHTLINE L,CvPoint3D32f* p);

	double intersectLinePlane(STRAIGHTLINE line, PLANE2 plane);

	double distPoints(CvPoint3D32f* p1, CvPoint3D32f * p2);

//	double distPointLine( CvPoint3D32f* p, STRAIGHTLINE line);

        /// calculate the dotproduct of two 3D Points
	double dotProd(CvPoint3D32f* p1, CvPoint3D32f* p2 );

        /// calculate a angle between two 3D points
	double getAngle(CvPoint3D32f* p1, CvPoint3D32f* p2 );

        /// create a new 3D Point and store the result of p1 + p2
	CvPoint3D32f * cAddPoints(CvPoint3D32f* p1, CvPoint3D32f * p2);
	
        /// p1 + p2
	void addPoints(CvPoint3D32f* p1, CvPoint3D32f * p2,CvPoint3D32f*res);

        /// create a new 3D Point and calculate a weighted sum of two 3D Points ( w1 * p1 + w2 * p2 )
	CvPoint3D32f * cAddWPoints(CvPoint3D32f * p1, float w1, CvPoint3D32f * p2, float w2 );

        /// calculate a weighted sum of two 3D Points in place ( w1 * p1 + w2 * p2 )
	void addWPoints(CvPoint3D32f * p1, float w1, CvPoint3D32f * p2, float w2, CvPoint3D32f*np );

        /// Create a new 3D Point and store the result of ( p1 - p2 ) in it
	CvPoint3D32f * cSubPoints(CvPoint3D32f* p1, CvPoint3D32f * p2);

        /// res = p1 - p2
	void subPoints(CvPoint3D32f* p1, CvPoint3D32f * p2,CvPoint3D32f* res);

        /// Creates a new 3D Point and calculates the cross product of p1 X p2
	CvPoint3D32f * cCrossProduct(CvPoint3D32f* p1, CvPoint3D32f* p2);

        /// np = p1 X p2
	void crossProduct(CvPoint3D32f* p1, CvPoint3D32f* p2,CvPoint3D32f* np);

        /// Create a new 3D Point and initialize with x, y, and z
	CvPoint3D32f * cGetPoint(float x, float y, float z );

        /// Create a new 3D Point copy the 3D Point p
	CvPoint3D32f * cCopyPoint(CvPoint3D32f* p );

        /// swaps two 3D Points
	void swapPoints(CvPoint3D32f* p1, CvPoint3D32f* p2);

        /// returns the length of vector p
	double getNorm(CvPoint3D32f* p );

        /// set the values of out to the values of in
	void setCoords (CvPoint3D32f * in, CvPoint3D32f * out);

        /// set the values of out to x, y and z
	void setCoords (CvPoint3D32f * out, float x, float y, float z);

        /// scale the 3D Point p with factor scale
	void scalePoint(CvPoint3D32f* p, double scale);

        /// orthonormalize Points
	void orthonormalizePoints(CvPoint3D32f* p1, CvPoint3D32f* p2,CvPoint3D32f*p3);

        /// orthonormalize Points and scale them
	void orthonormalizePoints(CvPoint3D32f* p1, CvPoint3D32f* p2,CvPoint3D32f*p3,double scale);

        /// returns a Matrix corresponding to the basis of the three points p1, p2 and p3
	CvMat* cGetBasisFrom3Points(CvPoint3D32f* p1, CvPoint3D32f * p2, CvPoint3D32f * p3 );

	CvMat* cGetBasisFrom3Points3x3(CvPoint3D32f* p1, CvPoint3D32f * p2, CvPoint3D32f * p3 )	;
	CvMat * createMatrixFrom3Points(CvPoint3D32f* p1, CvPoint3D32f* p2,CvPoint3D32f* p3);

	CvMat * cGetMat32fFrom64d(CvMat* mat64f);
	CvMat * cGetMat64dFrom32f(CvMat* mat64f);

        //Rectification Function for Ranger Calibration
                /// Rectification and inverse Rectification of a single point
		CvPoint3D32f * cRectifyPoint(CvPoint3D32f * oriPoint, float sensorLenghtScale, float sensorWidthScale, float sensorHeightScale, CvMat * map_matrix);
		CvPoint3D32f * cInverseRectifyPoint(CvPoint3D32f * oriPoint, float sensorLenghtScale, float sensorWidthScale, float sensorHeightScale, CvMat * map_matrix);
                /// Calculation of the rectification matrix, input: 4 original points and 4 points, where the original points should be projected to
		CvMat* cCalcRectificationMatrix(CvPoint2D32f* oriP1, CvPoint2D32f* oriP2, CvPoint2D32f* oriP3, CvPoint2D32f* oriP4, CvPoint2D32f* dstP1, CvPoint2D32f* dstP2,CvPoint2D32f* dstP3,CvPoint2D32f* dstP4);


		float getShortestDistanceBetweenTwoLines( STRAIGHTLINE line1, STRAIGHTLINE line2,float *t1, float *t2);

		float getLineParameter(STRAIGHTLINE line, CvPoint3D32f p);

		CvPoint3D32f getLinePoint(STRAIGHTLINE line, float param);
} //of namespace Helper3D



#endif
