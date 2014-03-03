﻿// COPYRIGHT 2014 by the Open Rails project.
// 
// This file is part of Open Rails.
// 
// Open Rails is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// Open Rails is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with Open Rails.  If not, see <http://www.gnu.org/licenses/>.

// OR Timetable file is csv file, with following main layout :
// Top Row : train information
// special items in toprow : 
//    #comment : general comment column (ignored except for first cell with row and column set to #comment)
//    <empty>  : continuation of train from previous column
//
// First column : station names
// special items in first column :
//    #comment   : general comment column (ignored except for first cell with row and column set to #comment)
//    #consist   : train consist
//    #path      : train path
//    #direction : Up or Down
//

using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using System.IO;

namespace ORTS.Formats
{
    /// <summary>
    /// class ORTTPreInfo
    /// provides pre-information for menu
    /// extracts only description and list of trains
    /// </summary>
    
    public class TTPreInfo
    {
        public List<String> Trains = new List<string>();
        public String Description;

        private String Separator;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="filePath"></param>

        public TTPreInfo(String filePath)
        {
            Separator = String.Empty;
            try
            {
                using (StreamReader scrStream = new StreamReader(filePath, true))
                {
                    TTFilePreliminaryRead(filePath, scrStream, Separator);
                    scrStream.Close();
                }
            }
            catch (Exception)
            {
                Description = "<" + "load error:" + " " + System.IO.Path.GetFileNameWithoutExtension(filePath) + ">";
            }
        }

        /// <summary>
        /// ORTTFilePreliminaryRead
        /// Read function to obtain pre-info
        /// </summary>
        /// <param name="filePath"></param>
        /// <param name="scrStream"></param>
        /// <param name="Separator"></param>

        public void TTFilePreliminaryRead(String filePath, StreamReader scrStream, String Separator)
        {
            String readLine;
            String restLine;
            int firstCommentColumn = -1;

            // read first line - first character is separator, rest is train info
            readLine = scrStream.ReadLine();
            if (String.IsNullOrEmpty(Separator)) Separator = readLine.Substring(0, 1);

            restLine = readLine.Substring(1);

            String[] SeparatorArray = new String[1] { Separator };
            String[] Parts = restLine.Split(SeparatorArray, System.StringSplitOptions.None);

            int columnIndex = 1;
            foreach (String headerString in Parts)
            {
                if (String.Compare(headerString, "#comment", true) == 0)
                {
                    if (firstCommentColumn < 0) firstCommentColumn = columnIndex;
                }
                else if (!String.IsNullOrEmpty(headerString))
                {
                    Trains.Add(String.Copy(headerString));
                }
                columnIndex++;
            }

            // try and find first comment row - cell at first comment row and column is description

            Description = String.Copy(filePath);

            if (firstCommentColumn > 0)
            {
                bool descFound = false;
                readLine = scrStream.ReadLine();

                while (readLine != null && !descFound)
                {
                    Parts = readLine.Split(SeparatorArray, System.StringSplitOptions.None);
                    if (String.Compare(Parts[0], "#comment", true) == 0)
                    {
                        Description = String.Copy(Parts[firstCommentColumn]);
                        descFound = true;
                    }
                    else
                    {
                        readLine = scrStream.ReadLine();
                    }
                }
            }
        }
    }

/// <summary>
/// class ORMultiTTPreInfo
/// Creates pre-info for Multi TT file
/// returns Description and list of pre-info per file
/// </summary>

    public class MultiTTPreInfo
    {
        public List<TTPreInfo> ORTTInfo = new List<TTPreInfo>();
        public String Description;

        public MultiTTPreInfo(String filePath, String directory)
        {
            Description = String.Empty;
            try
            {
                using (StreamReader scrStream = new StreamReader(filePath, true))
                {
                    MultiTTFilePreliminaryRead(filePath, directory, scrStream);
                    scrStream.Close();
                    if (String.IsNullOrEmpty(Description)) Description = String.Copy(filePath);
                }
            }
            catch (Exception)
            {
                Description = "<" + "load error:" + " " + System.IO.Path.GetFileNameWithoutExtension(filePath) + ">";
            }
        }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="SingleTTInfo"></param>

        public MultiTTPreInfo(TTPreInfo SingleTTInfo)
        {
            Description = String.Copy(SingleTTInfo.Description);
            ORTTInfo.Add(SingleTTInfo);
        }

        /// <summary>
        /// ORMultiTTFilePreliminaryRead
        /// Reads MultiTTfile for preliminary info
        /// </summary>
        /// <param name="filePath"></param>
        /// <param name="directory"></param>
        /// <param name="scrStream"></param>

        void MultiTTFilePreliminaryRead(String filePath, String directory, StreamReader scrStream)
        {
            String readLine;

            // read first line - first character is separator, rest is train info
            readLine = scrStream.ReadLine();

            while (readLine != null)
            {
                if (!String.IsNullOrEmpty(readLine))
                {
                    if (String.Compare(readLine.Substring(0, 1), "#") == 0)
                    {
                        if (String.IsNullOrEmpty(Description)) Description = String.Copy(readLine.Substring(1));
                    }
                    else
                    {
                        String ttfile = System.IO.Path.Combine(directory, readLine);
                        ORTTInfo.Add(new TTPreInfo(ttfile));
                    }
                }
                readLine = scrStream.ReadLine();
            }
        }
    }

    /// <summary>
    /// class MultiTTInfo
    /// extracts filenames from multiTTfile, extents names to full path
    /// </summary>

    public class MultiTTInfo
    {
        public List<string> TTFiles = new List<string>();
        public string Description;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="filePath"></param>
        /// <param name="directory"></param>

        public MultiTTInfo(String filePath, String directory)
        {
            Description = String.Empty;
            try
            {
                using (StreamReader scrStream = new StreamReader(filePath, true))
                {
                    MultiTTFileRead(filePath, directory, scrStream);
                    scrStream.Close();
                    if (String.IsNullOrEmpty(Description)) Description = String.Copy(filePath);
                }
            }
            catch (Exception)
            {
                Description = "<" + "load error:" + " " + System.IO.Path.GetFileNameWithoutExtension(filePath) + ">";
            }
        }

        /// <summary>
        /// MultiTTFileRead
        /// Reads multiTTfile and extracts filenames
        /// </summary>
        /// <param name="filePath"></param>
        /// <param name="directory"></param>
        /// <param name="scrStream"></param>

        void MultiTTFileRead(String filePath, String directory, StreamReader scrStream)
        {
            String readLine;

            // read first line - first character is separator, rest is train info
            readLine = scrStream.ReadLine();

            while (readLine != null)
            {
                if (!String.IsNullOrEmpty(readLine))
                {
                    if (String.Compare(readLine.Substring(0, 1), "#") == 0)
                    {
                        if (String.IsNullOrEmpty(Description)) Description = String.Copy(readLine.Substring(1));
                    }
                    else
                    {
                        String ttfile = System.IO.Path.Combine(directory, readLine);
                        TTFiles.Add(ttfile);
                    }
                }
                readLine = scrStream.ReadLine();
            }
        }
    }

        /// <summary>
        /// class TTContents : extracts full information as unprocessed strings
        /// </summary>

    public class TTContents
    {
        public List<string[]> trainStrings = new List<string[]>();
        public string TTfilename;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="filePath"></param>

        public TTContents(string filePath)
        {
            TTfilename = String.Copy(filePath);
            StreamReader filestream = new StreamReader(filePath, true);

            // read all lines in file
            string readLine = filestream.ReadLine();

            // extract separator from first line
            string[] separator = new string[1] { String.Copy(readLine.Substring(0, 1)) };

            // check : only ";" or "," are allowed as separators
            bool validSeparator = String.Compare(separator[0], ";") == 0 || String.Compare(separator[0], ",") == 0;
            if (!validSeparator)
            {
                throw new InvalidDataException("Invalid separator found in file : " + filePath);
            }

            // extract and store all strings

            while (readLine != null)
            {
                string[] parts = readLine.Split(separator, System.StringSplitOptions.None);
                trainStrings.Add(parts);
                readLine = filestream.ReadLine();
            }
        }
    }
}




