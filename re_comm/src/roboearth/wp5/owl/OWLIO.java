/* \file OWLIO.java
 * \brief I/O class for OWL ontolgies
 *
 * The OWLIO class provides means to load/save OWL ontologies to/from various sources/targets.
 * 
 * This file is part of the RoboEarth ROS re_comm package.
 * 
 * It was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the 
 * European Union Seventh Framework Programme FP7/2007-2013 
 * under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2010 by 
 * <a href=" mailto:perzylo@cs.tum.edu">Alexander Perzylo</a>
 * Technische Universitaet Muenchen
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
 * \author Alexander Perzylo
 * \version 1.0
 * \date 2010
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 */
package roboearth.wp5.owl;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.UnsupportedEncodingException;
import java.net.UnknownHostException;
import java.util.Map;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.OWLOntologyCreationIOException;
import org.semanticweb.owlapi.io.OWLParser;
import org.semanticweb.owlapi.io.OWLParserException;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.io.UnparsableOntologyException;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyCreationException;
import org.semanticweb.owlapi.model.OWLOntologyFormat;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.model.UnloadableImportException;
import org.semanticweb.owlapi.vocab.PrefixOWLOntologyFormat;

/**
 * 
 * A utility class providing methods to load, save and
 * convert OWL ontologies in various ways.
 * 
 * @author Alexander Perzylo, perzylo@cs.tum.edu
 *
 */
public class OWLIO {


	/**
	 * Ontology format object defining the RDF/XML format
	 */
	public static final OWLOntologyFormat ONTOLOGY_FORMAT_RDFXML = new RDFXMLOntologyFormat();


	/**
	 * Loads an OWL ontology from a given URL.
	 * @param url URL specifing the location of an OWL ontology
	 * @return OWLOntology object containing the parsed ontology
	 */
	public static OWLOntology loadOntologyFromWeb(String url) {

		OWLOntology ontology = null;

		try {
			// Get hold of an ontology manager
			OWLOntologyManager manager = OWLManager.createOWLOntologyManager();

			// Load the ontology from the web
			IRI iri = IRI.create(url);
			ontology = manager.loadOntologyFromOntologyDocument(iri);
		} catch (OWLOntologyCreationIOException e) {
			// IOExceptions during loading get wrapped in an OWLOntologyCreationIOException
			IOException ioException = e.getCause();
			if(ioException instanceof UnknownHostException) {
				System.out.println("Could not load ontology. Unknown host: " + ioException.getMessage());
			}
			else {
				System.out.println("Could not load ontology: " + ioException.getClass().getSimpleName() + " " + ioException.getMessage());    
			}
		} catch (UnparsableOntologyException e) {
			// If there was a problem loading an ontology because there are syntax errors in the document (file) that
			// represents the ontology then an UnparsableOntologyException is thrown
			System.out.println("Could not parse the ontology: " + e.getMessage());
			// A map of errors can be obtained from the exception
			Map<OWLParser, OWLParserException> exceptions = e.getExceptions();
			// The map describes which parsers were tried and what the errors were
			for(OWLParser parser : exceptions.keySet()) {
				System.out.println("Tried to parse the ontology with the " + parser.getClass().getSimpleName() + " parser");
				System.out.println("Failed because: " + exceptions.get(parser).getMessage());
			}
		} catch (UnloadableImportException e) {
			// If our ontology contains imports and one or more of the imports could not be loaded then an
			// UnloadableImportException will be thrown (depending on the missing imports handling policy)
			System.out.println("Could not load import: " + e.getImportsDeclaration());
			// The reason for this is specified and an OWLOntologyCreationException
			OWLOntologyCreationException cause = e.getOntologyCreationException();
			System.out.println("Reason: " + cause.getMessage());
		} catch (OWLOntologyCreationException e) {
			System.out.println("Could not load ontology: " + e.getMessage());
		} catch (Exception e) {
			System.out.println("Could not load ontology due to unknown error: " + e.getMessage());
		}

		return ontology;

	}


	/**
	 * Loads an OWL ontology from a given file.
	 * @param file name of file
	 * @return OWLOntology object containing the parsed ontology
	 */
	public static OWLOntology loadOntologyFromFile(String file) {

		OWLOntology ontology = null;

		try {
			// Build File object
			File f = new File(file);

			// Get hold of an ontology manager
			OWLOntologyManager manager = OWLManager.createOWLOntologyManager();

			// Now load the local copy
			ontology = manager.loadOntologyFromOntologyDocument(f);

		} catch (OWLOntologyCreationIOException e) {
			// IOExceptions during loading get wrapped in an OWLOntologyCreationIOException
			IOException ioException = e.getCause();
			if(ioException instanceof FileNotFoundException) {
				System.err.println("Could not load ontology. File not found: " + ioException.getMessage());
			}
			else {
				System.err.println("Could not load ontology: " + ioException.getClass().getSimpleName() + " " + ioException.getMessage());    
			}
		} catch (UnparsableOntologyException e) {
			// If there was a problem loading an ontology because there are syntax errors in the document (file) that
			// represents the ontology then an UnparsableOntologyException is thrown
			System.err.println("Could not parse the ontology: " + e.getMessage());
			// A map of errors can be obtained from the exception
			Map<OWLParser, OWLParserException> exceptions = e.getExceptions();
			// The map describes which parsers were tried and what the errors were
			for(OWLParser parser : exceptions.keySet()) {
				System.err.println("Tried to parse the ontology with the " + parser.getClass().getSimpleName() + " parser");
				System.err.println("Failed because: " + exceptions.get(parser).getMessage());
			}
		} catch (UnloadableImportException e) {
			// If our ontology contains imports and one or more of the imports could not be loaded then an
			// UnloadableImportException will be thrown (depending on the missing imports handling policy)
			System.err.println("Could not load import: " + e.getImportsDeclaration());
			// The reason for this is specified and an OWLOntologyCreationException
			OWLOntologyCreationException cause = e.getOntologyCreationException();
			System.err.println("Reason: " + cause.getMessage());
		} catch (OWLOntologyCreationException e) {
			System.err.println("Could not load ontology: " + e.getMessage());
		} catch (Exception e) {
			System.err.println("Could not load ontology due to unknown error: " + e.getMessage());
		}

		return ontology;

	}


	/**
	 * Loads an OWL ontology from a given input stream.
	 * @param stream input stream
	 * @return OWLOntology object containing the parsed ontology
	 */
	public static OWLOntology loadOntologyFromStream(InputStream stream) {

		OWLOntology ontology = null;

		try {

			// Get hold of an ontology manager
			OWLOntologyManager manager = OWLManager.createOWLOntologyManager();

			// Now load the local copy
			ontology = manager.loadOntologyFromOntologyDocument(stream);

		} catch (OWLOntologyCreationIOException e) {
			// IOExceptions during loading get wrapped in an OWLOntologyCreationIOException
			IOException ioException = e.getCause();
			if(ioException instanceof FileNotFoundException) {
				System.out.println("Could not load ontology. File not found: " + ioException.getMessage());
			}
			else {
				System.out.println("Could not load ontology: " + ioException.getClass().getSimpleName() + " " + ioException.getMessage());    
			}
		} catch (UnparsableOntologyException e) {
			// If there was a problem loading an ontology because there are syntax errors in the document that
			// represents the ontology then an UnparsableOntologyException is thrown
			System.out.println("Could not parse the ontology: " + e.getMessage());
			// A map of errors can be obtained from the exception
			Map<OWLParser, OWLParserException> exceptions = e.getExceptions();
			// The map describes which parsers were tried and what the errors were
			for(OWLParser parser : exceptions.keySet()) {
				System.out.println("Tried to parse the ontology with the " + parser.getClass().getSimpleName() + " parser");
				System.out.println("Failed because: " + exceptions.get(parser).getMessage());
			}
		} catch (UnloadableImportException e) {
			// If our ontology contains imports and one or more of the imports could not be loaded then an
			// UnloadableImportException will be thrown (depending on the missing imports handling policy)
			System.out.println("Could not load import: " + e.getImportsDeclaration());
			// The reason for this is specified and an OWLOntologyCreationException
			OWLOntologyCreationException cause = e.getOntologyCreationException();
			System.out.println("Reason: " + cause.getMessage());
		} catch (OWLOntologyCreationException e) {
			System.out.println("Could not load ontology: " + e.getMessage());
		} catch (Exception e) {
			System.out.println("Could not load ontology due to unknown error: " + e.getMessage());
		}

		return ontology;

	}


	/**
	 * Parses an OWL ontology from a given string.
	 * @param s String containing an OWL document
	 * @return OWLOntology object containing the parsed ontology
	 */
	public static OWLOntology loadOntologyFromString(String s) {

		InputStream inStream = new ByteArrayInputStream(s.getBytes());
		OWLOntology ontology = OWLIO.loadOntologyFromStream(inStream);

		return ontology;

	}


	/**
	 * Saves an OWL ontology to a file.
	 * @param ontology ontology to be saved
	 * @param file name of target file
	 * @return <tt>true</tt> - if saving was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public static boolean saveOntologyToFile(OWLOntology ontology, String file) {

		boolean ok = false;

		try {
			OWLOntologyFormat format = ontology.getOWLOntologyManager().getOntologyFormat(ontology);
			ok = saveOntologyToFile(ontology, file, format);			
		} catch (NullPointerException e) {
			System.out.println("Could not save ontology: null pointer argument found\n" + e.getMessage());
		}

		return ok; 

	}


	/**
	 * Saves an OWL ontology to a file in a given ontology format.
	 * @param ontology ontology to be saved
	 * @param file name of target file
	 * @param format desired ontology format (@see OWLOntologyFormat)
	 * @return <tt>true</tt> - if saving was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public static boolean saveOntologyToFile(OWLOntology ontology, String file, OWLOntologyFormat format) {

		boolean ok = false;

		try {

			// Build File object
			File f = new File(file);

			// Get hold of the ontology manager
			OWLOntologyManager manager = ontology.getOWLOntologyManager();

			// By default ontologies are saved in the format from which they were loaded.
			// We can get information about the format of an ontology from its manager
			OWLOntologyFormat currFormat = manager.getOntologyFormat(ontology);

			// The Document IRI, where the file should be saved
			IRI documentIRI = IRI.create(f.toURI());

			if (currFormat.equals(format)) {

				// Save a local copy of the ontology.
				manager.saveOntology(ontology, documentIRI);

			} else {

				// Some ontology formats support prefix names and prefix IRIs. When we save the ontology in
				// the new format we will copy the prefixes over so that we have nicely abbreviated IRIs in
				// the new ontology document
				if (format.isPrefixOWLOntologyFormat() && currFormat.isPrefixOWLOntologyFormat()) {
					((PrefixOWLOntologyFormat)format).copyPrefixesFrom(currFormat.asPrefixOWLOntologyFormat());
				}
				manager.saveOntology(ontology, format, documentIRI);

			}

			ok = true;

		} catch (Exception e) {
			System.out.println("Could not save ontology: " + e.getMessage());
		}

		return ok;
	}


	/**
	 * Saves an OWL ontology to a given output stream.
	 * @param ontology ontology to be saved
	 * @param stream output stream
	 * @return <tt>true</tt> - if saving was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public static boolean saveOntologyToStream(OWLOntology ontology, OutputStream stream) {

		boolean ok = false;

		try {
			OWLOntologyFormat format = ontology.getOWLOntologyManager().getOntologyFormat(ontology);
			ok = saveOntologyToStream(ontology, stream, format);			
		} catch (NullPointerException e) {
			System.out.println("Could not save ontology: null pointer argument found\n" + e.getMessage());
		}

		return ok; 

	}


	/**
	 * Saves an OWL ontology to an output stream in a given ontology format.
	 * @param ontology ontology to be saved
	 * @param stream output stream
	 * @param format desired ontology format (@see OWLOntologyFormat)
	 * @return <tt>true</tt> - if saving was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public static boolean saveOntologyToStream(OWLOntology ontology, OutputStream stream, OWLOntologyFormat format) {

		boolean ok = false;

		if (stream != null) {

			try {

				// Get hold of the ontology manager
				OWLOntologyManager manager = ontology.getOWLOntologyManager();

				// By default ontologies are saved in the format from which they were loaded.
				// We can get information about the format of an ontology from its manager
				OWLOntologyFormat currFormat = manager.getOntologyFormat(ontology);

				if (currFormat.equals(format)) {

					// Save a copy of the ontology to the given stream.
					manager.saveOntology(ontology, stream);

				} else {

					// Some ontology formats support prefix names and prefix IRIs. When we save the ontology in
					// the new format we will copy the prefixes over so that we have nicely abbreviated IRIs in
					// the new ontology document
					if (format.isPrefixOWLOntologyFormat() && currFormat.isPrefixOWLOntologyFormat()) {
						((PrefixOWLOntologyFormat)format).copyPrefixesFrom(currFormat.asPrefixOWLOntologyFormat());
					}
					manager.saveOntology(ontology, format, stream);

				}

				ok = true;

			} catch (Exception e) {
				System.out.println("Could not save ontology: " + e.getMessage());
			}

		}

		return ok;
	}


	/**
	 * Saves an OWL ontology to a String object using its current ontology format.
	 * @param ontology ontology to be saved
	 * @return String object containing the string representation of the ontology
	 */
	public static String saveOntologyToString(OWLOntology ontology) {

		return saveOntologyToString(ontology, ontology.getOWLOntologyManager().getOntologyFormat(ontology));

	}	


	/**
	 * Saves an OWL ontology to a String object.
	 * @param ontology ontology to be saved
	 * @param format desired ontology format (@see OWLOntologyFormat)
	 * @return String object containing the string representation of the ontology
	 */
	public static String saveOntologyToString(OWLOntology ontology, OWLOntologyFormat format) {

		String s = null;
		ByteArrayOutputStream os = new ByteArrayOutputStream(4096);

		if (saveOntologyToStream(ontology, os, format)) {
			try {
				s = new String(os.toByteArray(), "UTF-8");
			} catch (UnsupportedEncodingException e) {
				System.out.println("UTF-8 encoding is unsupported: " + e.getMessage());
			}			
		}

		return s;

	}


}
