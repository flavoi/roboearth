/** \file MetaFile.h
 * \brief Reads meta.xml files
 *
 * Class to read the metadata of 3D object models, contained in meta.xml files
 * 
 * This file is part of the RoboEarth ROS WP1 package.
 * 
 * It file was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the European Union Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2010 by <a href="mailto:dorian@unizar.es">Dorian Galvez-Lopez</a>, University of Zaragoza
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * \author Dorian Galvez-Lopez
 * \version 1.0
 * \date 2010
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
***********************************************/

/// General structure of meta xml files:
/// (Date: 15 Dec 2010)
///
///  <model>
///  <name>cabinet1</name>
///  <type>planar</type>
///  <faces>1</faces>
///  <dimensions>
///    <scale>0.5</scale>
///    <width>0.35</width>
///    <height>0.69</height>
///    <depth>0.3</depth>
///    <face>
///      <index>0</index>
///      <width>0.35</width>
///      <height>0.35</height>
///      <oTf>1 0 0 0    0 1 0 0    0 0 1 0    0 0 0 1</cTf>
///    </face>
///  </dimensions>
///  </model>
///
/// name: name of the object
/// type: type of objects. Valid types are "planar" or "3D"
/// faces: number of faces the model is composed of (usually 1-6 for 
///        planar objects -actually planar or cube-shaped-, or any number for 
///        3D)
/// dimensions: dimensions of the object. Fields under this node only appear 
///        depending on the type of the object
/// + scale (3D): scale factor applied to the 3D points obtained when Bundler and
///          PMVS are run on the face images. This value is useful to reconstruct
///          the model
/// + width, height, depth (planar): real dimensions in metres of a planar/cuboid
///          object. For actually planar objects, depth is 0 (or may not appear).
///          What dimension is associated with each concept depends on the
///        application that created the model
/// face (planar): contains information about the dimensions of each face. This 
///        information is used to reconstruct the model
/// + index: index of the face [0..N)
/// + width, height: real dimensions in metres of the planar face
/// + oTf: 4x4 transformation in row-major order from the object frame to the
///          face frame. oTf is the identity matrix when the object is actually
///          planar (only one face)
///

#ifndef __META_FILE__
#define __META_FILE__

#include <string>
#include <sstream>
#include <vector>
#include <opencv/cv.h>

#include "rapidxml.hpp"

class MetaFile
{
public:
  struct MetaData
  {
  public:
    struct tFaceDim
    {
      float Width;
      float Height;
      cv::Mat oTf; // transformation from object frame to face frame
    };
    
  public:
    
    MetaData(){ reset(); }
    
  public:
    std::string Name;
    int NFaces;
    std::string Type;
    
    struct tDimensions
    {
      // for 3D objects
      struct tVolume
      {
        float Scale; 
      } Volume;
      
      // for planar objects in metres
      struct tPlanar
      {
        float Width;
        float Height;
        float Depth;
        
        // Dimensions of each face image
        std::vector<tFaceDim> Faces;
        
      } Planar;
      
    } Dimensions;
    
    inline void reset()
    {
      Name = Type = "";
      NFaces = 0;
      Dimensions.Planar.Width = 0;
      Dimensions.Planar.Height = 0;
      Dimensions.Planar.Depth = 0;
    }
  };
  
public:
  
  /**
   * Reads a meta.xml file and returns its content
   * @param filename
   * @param data
   */
  static void readFile(const std::string &filename, MetaData &data);
  
  /**
   * Writes the metadata in filename in xml format
   * @param filename
   * @param data
   */
  static void saveFile(const std::string &filename, const MetaData &data);
  
protected:

  /**
   * Returns a value from a string
   * @param s
   * @return T
   */
  template<class T>
  static T parse(const std::string &s);

  /**
   * Returns several values from a vector of strings
   * @param s
   * @return vector of T's
   */
  template<class T>
  static std::vector<T> parse(const std::vector<std::string> &v);

  /**
   * Converts v into a string
   * @param v
   * @return string version of v
   */  
  template<class T>
  static std::string stringfy(T v);

  /**
   * Converts array v into a string separated by spaces
   * @param v
   * @param N items in v
   * @return string version of v
   */  
  template<class T>
  static std::string stringfy(T* v, int N);

protected:

  /** 
   * Parses the <dimension> node
   * @param meta struct to put the data in
   * @param node <dimension> node
   */
  static void parseDimensions(MetaFile::MetaData &meta, 
    rapidxml::xml_node<> *node);
  
  /** 
   * Parses the <face> node
   * @param meta struct to put the data in
   * @param node <face> node
   */
  static void parseFace(MetaFile::MetaData &meta, 
    rapidxml::xml_node<> *node);

};

// ----------------------------------------------------------------------------

template<class T>
T MetaFile::parse(const std::string &s)
{
  std::stringstream ss(s);
  T v;
  ss >> v;
  return v;
}

// ----------------------------------------------------------------------------

template<class T>
std::vector<T> MetaFile::parse(const std::vector<std::string> &v)
{
  std::vector<T> ret;
  ret.reserve(v.size());
  
  std::vector<std::string>::const_iterator it;
  for(it = v.begin(); it != v.end(); ++it)
  {
    ret.push_back( MetaFile::parse<T>(*it) );
  }
  
  return ret;
}

// ----------------------------------------------------------------------------

template<class T>
std::string MetaFile::stringfy(T v)
{
  std::stringstream ss;
  ss << v;
  return ss.str();
}

// ----------------------------------------------------------------------------

template<class T>
std::string MetaFile::stringfy(T* v, int N)
{
  std::stringstream ss;
  
  for(int i = 0; i < N; ++i)
    ss << v[i] << " ";

  return ss.str();
}

// ----------------------------------------------------------------------------

#endif

