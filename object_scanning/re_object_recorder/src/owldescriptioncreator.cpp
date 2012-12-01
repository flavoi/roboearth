/** \file owldescriptioncreator.cpp
 * \brief Utility class to create OWL descriptions from a template
 *
 * This file is part of the RoboEarth ROS re_object_recorder package.
 *
 * It file was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the European Union Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2011 by by <a href="mailto:andreas.koch@ipvs.uni-stuttgart.de">Andreas Koch</a>, University of Stuttgart
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    <UL>
 *     <LI> Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     <LI> Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     <LI> Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *    </UL>
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Andreas Koch
 * \author Daniel Di Marco
 * \version 1.0
 * \date 2011
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
***********************************************/

#include <QString>
#include "owldescriptioncreator.h"

OWLDescriptionCreator::OWLDescriptionCreator(const QString &model_type, const QString &object_class, const QDateTime& creation_date) : owl_description(OWL_TEMPLATE)
{
    replaceFiller("MODEL_TYPE", model_type);
    replaceFiller("OBJECT_CLASS", object_class);

    QString creation_string = creation_date.toString("yy_MM_dd_hh_mm_ss_zzz");

    replaceFiller("DATE_TIME_yy_MM_dd_HH_mm_ss_SSS", creation_string);
}

void OWLDescriptionCreator::replaceFiller(const QString &filler, const QString &content)
{
    owl_description.replace("${" + filler + "}", content);
}

QString OWLDescriptionCreator::getOwlDescription() const
{
    return owl_description;
}

const QString OWLDescriptionCreator::OWL_TEMPLATE("\
<?xml version=\"1.0\" ?>\n\
\n\
<!DOCTYPE rdf:RDF [\n\
    <!ENTITY owl \"http://www.w3.org/2002/07/owl#\" >\n\
    <!ENTITY xsd \"http://www.w3.org/2001/XMLSchema#\" >\n\
    <!ENTITY knowrob \"http://ias.cs.tum.edu/kb/knowrob.owl#\" >\n\
    <!ENTITY roboearth \"http://www.roboearth.org/kb/roboearth.owl#\" >\n\
    <!ENTITY rdfs \"http://www.w3.org/2000/01/rdf-schema#\" >\n\
    <!ENTITY rdf \"http://www.w3.org/1999/02/22-rdf-syntax-ns#\" >\n\
]>\n\
\n\
\n\
<rdf:RDF xml:base=\"http://www.roboearth.org/kb/roboearth.owl\"\n\
  xmlns=\"http://www.roboearth.org/kb/roboearth.owl#\"\n\
  xmlns:comp_cop=\"http://ias.cs.tum.edu/kb/comp_cop.owl#\"\n\
  xmlns:knowrob=\"http://ias.cs.tum.edu/kb/knowrob.owl#\"\n\
  xmlns:owl=\"http://www.w3.org/2002/07/owl#\"\n\
  xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\"\n\
  xmlns:rdfs=\"http://www.w3.org/2000/01/rdf-schema#\"\n\
  xmlns:roboearth=\"http://www.roboearth.org/kb/roboearth.owl#\"\n\
  xmlns:xsd=\"http://www.w3.org/2001/XMLSchema#\">\n\
\n\
    <owl:Ontology rdf:about=\"http://www.roboearth.org/kb/roboearth.owl#\">\n\
      <owl:imports rdf:resource=\"http://ias.cs.tum.edu/kb/knowrob.owl#\"/>\n\
    </owl:Ontology>\n\
\n\
    <owl:ObjectProperty rdf:about=\"&roboearth;providesModelFor\"/>\n\
    <owl:DatatypeProperty rdf:about=\"&roboearth;creationDateTime\"/>\n\
\n\
    <owl:Class rdf:about=\"&roboearth;${MODEL_TYPE}\"/>\n\
    <owl:Class rdf:about=\"&owl;Class\"/>\n\
\n\
    <owl:NamedIndividual rdf:about=\"&roboearth;${MODEL_TYPE}_${DATE_TIME_yy_MM_dd_HH_mm_ss_SSS}\">\n\
        <rdf:type rdf:resource=\"&roboearth;${MODEL_TYPE}\"/>\n\
        <creationDateTime rdf:datatype=\"&xsd;string\">${DATE_TIME_yy_MM_dd_HH_mm_ss_SSS}</creationDateTime>\n\
        <providesModelFor rdf:resource=\"&knowrob;${OBJECT_CLASS}\"/>\n\
    </owl:NamedIndividual>\n\
\n\
    <owl:NamedIndividual rdf:about=\"&knowrob;${OBJECT_CLASS}\">\n\
        <rdf:type rdf:resource=\"&owl;Class\"/>\n\
    </owl:NamedIndividual>\n\
\n\
</rdf:RDF>");
