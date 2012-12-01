/** \file MetaFile.cpp
 * \brief Reads meta.xml files
 *
 * Class to read the metadata of object models, contained in meta.xml files
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

#include "MetaFile.h"

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>

#include <opencv/cv.h>

#include "DUtils.h"
#include "DUtilsCV.h"
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"

using namespace std;
using namespace rapidxml;



// ----------------------------------------------------------------------------

void MetaFile::readFile(const std::string &filename, MetaData &data)
{
  data.reset();
  
  fstream f(filename.c_str(), ios::in);
  if(!f.is_open()) throw string("MetaFile: cannot open ") + filename;
  
  f.seekg(0, ios_base::end);
  long length = f.tellg();
  f.seekg(0, ios::beg);
  
  char *buffer = new char [length+1];
  f.read(buffer, length);
  buffer[length] = '\0';
  
  f.close();

  // parse xml
  // Typical format:
  //  <model>
  //  <name>cabinet1</name>
  //  <type>planar</type>
  //  <faces>1</faces>
  //  <dimensions>
  //    <scale>0.5</scale>
  //    <width>0.35</width>
  //    <height>0.69</height>
  //    <depth>0.3</depth>
  //    <face>
  //      <index>0</index>
  //      <width>0.35</width>
  //      <height>0.35</height>
  //      <oTf>...</oTf>
  //    </face>
  //  </dimensions>
  //  </model>

  xml_document<> doc;
  doc.parse<0>(buffer);    // 0 means default parse flags

  vector<xml_node<>* > upnodes;
  
  xml_node<> *node = doc.first_node(); // <model>
  node = node->first_node(); // first "attribute"

  while(node != NULL)
  {
    std::string property = node->name();
    std::string value = node->value();
    
    if(property == "name")
      data.Name = value;
    else if(property == "type")
      data.Type = value;
    else if(property == "faces")
      data.NFaces = MetaFile::parse<int>(value);
    else if(property == "dimensions")
      parseDimensions(data, node);

    node = node->next_sibling();
  }
  
  if(data.Type == "planar" && data.Dimensions.Planar.Faces.empty())
  {
    // old xml version lacked dimensions of the face
    data.Dimensions.Planar.Faces.resize(data.NFaces);
    data.Dimensions.Planar.Faces[0].Width = data.Dimensions.Planar.Width;
    data.Dimensions.Planar.Faces[0].Height = data.Dimensions.Planar.Height;
    data.Dimensions.Planar.Faces[0].oTf = cv::Mat::eye(4, 4, CV_64F);
  }
  
  delete [] buffer;
}

// ----------------------------------------------------------------------------

void MetaFile::parseDimensions(MetaFile::MetaData &data, 
  rapidxml::xml_node<> *node)
{
  // Typical structure:
  //  <dimensions>
  //    <scale>0.5</scale>
  //    <width>0.35</width>
  //    <height>0.69</height>
  //    <depth>0.3</depth>
  //    <face>
  //      <index>0</index>
  //      <width>0.35</width>
  //      <height>0.35</height>
  //      <oTf>...</oTf>
  //    </face>
  //  </dimensions>
  
  node = node->first_node();
  while(node != NULL)
  {
    std::string property = node->name();
    std::string value = node->value();
    
    if(property == "scale") // type == "3D"
      data.Dimensions.Volume.Scale = MetaFile::parse<float>(value);
    else if(property == "width") // type == "planar"
      data.Dimensions.Planar.Width = MetaFile::parse<float>(value);
    else if(property == "height") // type == "planar"
      data.Dimensions.Planar.Height = MetaFile::parse<float>(value);
    else if(property == "depth") // type == "planar"
      data.Dimensions.Planar.Depth = MetaFile::parse<float>(value);
    else if(property == "face") // type == "planar"
      parseFace(data, node);
    
    node = node->next_sibling();
  }
  
}

// ----------------------------------------------------------------------------

void MetaFile::parseFace(MetaFile::MetaData &data, 
  rapidxml::xml_node<> *node)
{
  // Typical structure:
  //    <face>
  //      <index>0</index>
  //      <width>0.35</width>
  //      <height>0.35</height>
  //      <oTf>...</oTf>
  //    </face>
 
  int face_idx = -1;
  float w, h;
  w = h = 0;
  cv::Mat oTf;
  
  node = node->first_node();
  while(node != NULL)
  {
    std::string property = node->name();
    std::string value = node->value();
    
    if(property == "index")
      face_idx = MetaFile::parse<int>(value);
    else if(property == "width")
      w = MetaFile::parse<float>(value);
    else if(property == "height")
      h = MetaFile::parse<float>(value);
    else if(property == "oTf")
    {
      vector<string> vs;
      DUtils::StringFunctions::split(value, vs);
      vector<double> v = MetaFile::parse<double>(vs);
      
      if(!v.empty())
      {
        oTf = (cv::Mat_<double>(4, 4) <<
          v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8],
          v[9], v[10], v[11], v[12], v[13], v[14], v[15]);
      }
    }
    
    node = node->next_sibling();
  }
 
  if(face_idx != -1)
  {
    if((int)data.Dimensions.Planar.Faces.size() <= face_idx)
      data.Dimensions.Planar.Faces.resize(face_idx + 1);
    
    data.Dimensions.Planar.Faces[face_idx].Width = w;
    data.Dimensions.Planar.Faces[face_idx].Height = h;
    data.Dimensions.Planar.Faces[face_idx].oTf = oTf;
  }
 
}

// ----------------------------------------------------------------------------

void MetaFile::saveFile(const std::string &filename, const MetaData &data)
{
  fstream f(filename.c_str(), ios::out);
  if(!f.is_open()) throw string("MetaFile: cannot open ") + filename; 

  xml_document<> doc;
  char *node_name = doc.allocate_string("model"); 
  xml_node<> *root_node = doc.allocate_node(node_element, node_name); 
  doc.append_node(root_node);
  
  string sscale = MetaFile::stringfy(data.Dimensions.Volume.Scale);
  string snfaces = MetaFile::stringfy(data.NFaces);
  string swidth = MetaFile::stringfy(data.Dimensions.Planar.Width);
  string sheight = MetaFile::stringfy(data.Dimensions.Planar.Height);
  string sdepth = MetaFile::stringfy(data.Dimensions.Planar.Depth);
  
  root_node->append_node(doc.allocate_node(node_element, 
    "name", data.Name.c_str() ));
    
  root_node->append_node(doc.allocate_node(node_element, 
    "type", data.Type.c_str() ));
    
  root_node->append_node(doc.allocate_node(node_element, 
    "faces", snfaces.c_str() ));
   
  xml_node<> *dim_node = doc.allocate_node(node_element, "dimensions");
  root_node->append_node(dim_node);
  
  if(data.Type == "planar")
  {
    dim_node->append_node(doc.allocate_node(node_element, 
      "width", swidth.c_str() ));
    dim_node->append_node(doc.allocate_node(node_element, 
      "height", sheight.c_str() ));
    dim_node->append_node(doc.allocate_node(node_element, 
      "depth", sdepth.c_str() ));
    
    for(unsigned int i = 0; i < data.Dimensions.Planar.Faces.size(); ++i)
    {
      xml_node<> *face_node = doc.allocate_node(node_element, "face");
      dim_node->append_node(face_node);
      
      char *sindex = doc.allocate_string(MetaFile::stringfy(i).c_str());
      char *sw = doc.allocate_string(MetaFile::stringfy(
        data.Dimensions.Planar.Faces[i].Width).c_str());
      char *sh = doc.allocate_string(MetaFile::stringfy(
        data.Dimensions.Planar.Faces[i].Height).c_str());
      
      face_node->append_node(doc.allocate_node(node_element, "index", sindex));
      face_node->append_node(doc.allocate_node(node_element, "width", sw));
      face_node->append_node(doc.allocate_node(node_element, "height", sh));
      
      if(!data.Dimensions.Planar.Faces[i].oTf.empty())
      {
        double d[16];
        DUtilsCV::Types::vectorize(data.Dimensions.Planar.Faces[i].oTf, d);
        
        char *sotf = doc.allocate_string(MetaFile::stringfy(d, 16).c_str());
        
        face_node->append_node(doc.allocate_node(node_element, "oTf", sotf));
      }
    }
    
  }
  else if(data.Type == "3D")
  {
    dim_node->append_node(doc.allocate_node(node_element, 
      "scale", sscale.c_str() ));
  }

  f << doc;
  
  f.close();
}

// ----------------------------------------------------------------------------


