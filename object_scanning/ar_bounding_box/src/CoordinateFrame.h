/** \file CoordinateFrame.h
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

#ifndef CoordinateFrame_DEFINED
#define CoordinateFrame_DEFINED


#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include <iostream>
#include "Utils3D.h"

/**
 * Utility class for working with coordinate frames.
 * @note all position units are in [mm]
 * @note all angle units in [rad]
 **/
class CoordinateFrame 
{    // NOTE:
    // In this implementation it is pos->x, pos->y, pos->z, rot->x, rot->y, rot->z    
public:
    ////////////////
    //Constructors//
    ////////////////
    CoordinateFrame(CvPoint3D32f position, CvPoint3D32f rotation);
    CoordinateFrame(float posX, float posY, float posZ, float rotX, float rotY, float rotZ);
    CoordinateFrame(CvPoint3D32f position, float rotX, float rotY, float rotZ);
    CoordinateFrame(float posX, float posY, float posZ, CvPoint3D32f rotation);
    CoordinateFrame(CvPoint3D32f* p0, CvPoint3D32f* p1, CvPoint3D32f* p2);
    CoordinateFrame(cv::Point3f p1, cv::Point3f p2, cv::Point3f p3);
    CoordinateFrame(CvMat* trafoMat);
    CoordinateFrame();

    //////////////
    //Destructor//
    //////////////
    ~CoordinateFrame();

    ////////////////////
    //Methods on CoordinateFrame//
    ////////////////////

    /// Creates a Transformation Matrrix from the CoordinateFrame
    CvMat* createTransformationMatrix();

    /// Creates a Transformation Matrix from the CoordinateFrame just for rotations
    CvMat* createTransformationMatrix3x3();

    /**
     * Stores the base information in three points initialized by the caller.
     * p0 is the vector from origin to the base, p1-p0 is the x-axis, p2-p0 is the y-axis
     **/
    void getThreePoints(CvPoint3D32f* p0,CvPoint3D32f* p1,CvPoint3D32f* p2);

    /**
     * Transforms the CoordinateFrame into an arbitrary base. The result will be
     * saved in a additional created CoordinateFrame instance. The value of the
     * calling-instance will be not modified.
     **/
    CoordinateFrame * cInFrame(CoordinateFrame * base);

    /**
     * Transforms the CoordinateFrame from an arbitrary base to world. The result will be
     * saved in a additional created CoordinateFrame instance. The value of the
     * calling-instance will be not modified.
     **/
    CoordinateFrame * cInWorld(CoordinateFrame * base);

    CoordinateFrame * cMakeFrame(CoordinateFrame * base);
    CoordinateFrame * cMakeFrame(CvPoint3D32f* p0,CvPoint3D32f* p1,CvPoint3D32f* p2);

    //////////
    //access//
    //////////

    /// returns the Position of CoordinateFrame as CvPoint3D32f
    CvPoint3D32f * getPos() { return &(this->pos);}

    /// returns the Rotation of CoordinateFrame as CvPoint3D32f
    CvPoint3D32f * getRot() { return &(this->rot);}

    /**
     * returns the char[] for roblink, Attention: The angles are converted
     * to degree only in this string representation. Also the positions are only
     * converted to mm in this string representation.
     **/
    char * toString();

    /// set the position of the CoordinateFrame
    void setPos(CvPoint3D32f* pos_);

    /// set the rotation of the CoordinateFrame
    void setRot(CvPoint3D32f* rot_);

    void toggleZDirection();


public:
    void getXAxis(CvPoint3D32f* normal);
    void getYAxis(CvPoint3D32f* normal);
    void getZAxis(CvPoint3D32f* normal);


private:
    /// storage of the position in [m]
    CvPoint3D32f pos;
    /// storage of the angles in [rad]
    CvPoint3D32f rot;

};

class CoordinateFrameExt : public CoordinateFrame
{
public:
    int error;
    int prob;
    bool isResult;

public:
    CoordinateFrameExt():CoordinateFrame(){}
    CoordinateFrameExt(CvPoint3D32f* p1,CvPoint3D32f* p2,CvPoint3D32f* p3):CoordinateFrame(p1,p2,p3){}
};

#endif 
