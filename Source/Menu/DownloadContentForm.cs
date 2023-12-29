﻿// COPYRIGHT 2009, 2010, 2011, 2012, 2013, 2014, 2015 by the Open Rails project.
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

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Windows.Forms;
using GNU.Gettext;
using LibGit2Sharp;
using ORTS.Settings;
using System.Linq;
using System.Diagnostics;
using System.Threading;
using System.ComponentModel;
using System.Net;
using System.IO.Compression;
using System.Drawing;

namespace ORTS
{
    public partial class DownloadContentForm : Form
    {
        private readonly GettextResourceManager Catalog;
        private readonly UserSettings Settings;
        private readonly IDictionary<string, RouteSettings.Route> Routes;

        private string RouteName;

        private readonly string ImageTempFilename;
        private Thread ImageThread;
        private readonly string InfoTempFilename;

        public DownloadContentForm(UserSettings settings)
        {
            InitializeComponent();

            Catalog = new GettextResourceManager("Menu");
            Settings = settings;

            Settings.Routes.LoadContentAndInstalled();
            Routes = settings.Routes.Routes;
            for (int index = 0; index < Routes.Count; index++)
            {
                string routeName = Routes.ElementAt(index).Key;
                RouteSettings.Route route = Routes.ElementAt(index).Value;
                dataGridViewDownloadContent.Rows.Add(new string[] { routeName, route.DateInstalled, route.Url });
            }

            dataGridViewDownloadContent.Sort(dataGridViewDownloadContent.Columns[0], ListSortDirection.Ascending);

            InstallPathTextBox.Text = settings.Content.InstallPath;

            ImageTempFilename = Path.GetTempFileName();
            InfoTempFilename = Path.GetTempFileName();
            File.Delete(InfoTempFilename);
            InfoTempFilename = Path.ChangeExtension(ImageTempFilename, "html");
        }

        void dataGridViewDownloadContent_SelectionChanged(object sender, EventArgs e)
        {
            RouteName = dataGridViewDownloadContent.CurrentRow.Cells[0].Value.ToString();

            DownloadContentButton.Enabled = string.IsNullOrWhiteSpace(Routes[RouteName].DateInstalled);

            // picture box handling

            if (pictureBoxRoute.Image != null)
            {
                pictureBoxRoute.Image.Dispose();
                pictureBoxRoute.Image = null;
            }

            if (!string.IsNullOrEmpty(Routes[RouteName].Image))
            {
                if (ImageThread != null)
                {
                    if (ImageThread.IsAlive)
                    {
                        ImageThread.Abort();
                    }
                }

                // use a thread as grabbing the picture from the web might take a few seconds
                ImageThread = new Thread(() =>
                {
                    // wait one second as the user might scroll through the list
                    Stopwatch sw = Stopwatch.StartNew();
                    while (sw.ElapsedMilliseconds <= 1000) { }

                    try
                    {
                        using (WebClient myWebClient = new WebClient())
                        {
                            myWebClient.DownloadFile(Routes[RouteName].Image, ImageTempFilename);
                        }
                    }
                    catch
                    {
                        // route lives whithout a picture, not such a problem
                    }

                    if (File.Exists(ImageTempFilename))
                    {
                        pictureBoxRoute.Image = new Bitmap(ImageTempFilename);
                    }
                });
                ImageThread.Start();
            }


            // text box with description

            textBoxRoute.Text = Routes[RouteName].Description;
        }

        private void InstallPathButton_Click(object sender, EventArgs e)
        {
            using (FolderBrowserDialog folderBrowser = new FolderBrowserDialog())
            {
                folderBrowser.SelectedPath = InstallPathTextBox.Text;
                folderBrowser.Description = Catalog.GetString("Main Path where route is to be installed");
                folderBrowser.ShowNewFolderButton = true;
                if (folderBrowser.ShowDialog(this) == DialogResult.OK)
                {
                    InstallPathTextBox.Text = folderBrowser.SelectedPath;
                }
            }
        }

        private void DownloadContentButton_Click(object sender, EventArgs e)
        {
            string installPath = InstallPathTextBox.Text;
            string installPathRoute = Path.Combine(installPath, RouteName);
            string message;

            // various checks for the directory where the route is installed

            DriveInfo dInfo = new DriveInfo(installPathRoute);

            long size = Routes[RouteName].InstallSize + Routes[RouteName].DownloadSize;

            if (size > (dInfo.AvailableFreeSpace * 1.1))
            {
                message = Catalog.GetStringFmt("Not enough diskspace on drive {0} ({1}), available {2} kB, needed {3} kB, still continue?", 
                    dInfo.Name, 
                    dInfo.VolumeLabel,
                    (dInfo.AvailableFreeSpace / 1024).ToString("N0"), 
                    (size / 1024).ToString("N0"));

                if (MessageBox.Show(message, Catalog.GetString("Attention"), MessageBoxButtons.OKCancel, MessageBoxIcon.Warning) != DialogResult.OK)
                {
                    // cancelled
                    return;
                }
            }

            message = Catalog.GetStringFmt("Route to be installed in \"{0}\", are you sure?", installPathRoute);

            if (MessageBox.Show(message, Catalog.GetString("Attention"), MessageBoxButtons.OKCancel, MessageBoxIcon.Warning) != DialogResult.OK)
            {
                // cancelled
                return;
            }

            if (!Directory.Exists(installPath))
            {
                message = Catalog.GetStringFmt("Directory \"{0}\" does not exist", installPath);
                MessageBox.Show(message, Catalog.GetString("Attention"), MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            if (!Directory.Exists(installPathRoute))
            {
                try
                {
                    Directory.CreateDirectory(installPathRoute);
                }
                catch (Exception createDirectoryException)
                {
                    message = Catalog.GetStringFmt("Directory \"{0}\" cannot be created: {1}",
                        installPathRoute, createDirectoryException.Message);
                    MessageBox.Show(message, Catalog.GetString("Attention"), MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }
            }
            else 
            {
                if ((Directory.GetFiles(installPathRoute).Length != 0) ||
                    (Directory.GetDirectories(installPathRoute).Length != 0))
                {
                    message = Catalog.GetStringFmt("Directory \"{0}\" exists and is not empty", installPathRoute);
                    MessageBox.Show(message, Catalog.GetString("Attention"), MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }
            }

            Settings.Content.InstallPath = installPath;

            // set json route filename

            Settings.Content.RouteJsonName = Path.Combine(installPath, "ORRoute.json");

            // the download

            Cursor.Current = Cursors.WaitCursor;

            dataGridViewDownloadContent.CurrentRow.Cells[1].Value = Catalog.GetString("Installing...");
            Refresh();

            if (!downloadRoute(installPathRoute))
            {
                return;
            }

            // insert row in Options, tab Content

            if (!insertRowInOptions(installPathRoute))
            {
                return;
            }

            string dateTimeNowStr = DateTime.Now.ToString(CultureInfo.CurrentCulture.DateTimeFormat);
            dataGridViewDownloadContent.CurrentRow.Cells[1].Value = dateTimeNowStr;

            Routes[RouteName].DateInstalled = dateTimeNowStr;
            Routes[RouteName].DirectoryInstalledIn = installPathRoute;

            Settings.Folders.Save();
            Settings.Routes.Save();

            MessageBox.Show(Catalog.GetString("Route installed"), Catalog.GetString("Done"), MessageBoxButtons.OK, MessageBoxIcon.Asterisk);

            Close();
        }

        private bool downloadRoute(string installPathRoute)
        {
            bool returnValue = false;

            Thread downloadThread = new Thread(() =>
            {
                if (Routes[RouteName].Url.EndsWith(".git"))
                {
                    returnValue = doTheClone(installPathRoute);
                }
                if (Routes[RouteName].Url.EndsWith(".zip"))
                {
                    returnValue = doTheZipDownload(Routes[RouteName].Url, Path.Combine(installPathRoute, RouteName + ".zip"));
                }
            });
            downloadThread.Start();

            while (downloadThread.IsAlive)
            {
                Stopwatch sw = Stopwatch.StartNew();

                TotalBytes = 0;
                sumMB(installPathRoute);
                dataGridViewDownloadContent.CurrentRow.Cells[1].Value =
                    string.Format("downloaded: {0} kB", (TotalBytes / 1024).ToString("N0"));
                Refresh();

                while ((downloadThread.IsAlive) && (sw.ElapsedMilliseconds <= 3000)) { }
            }

            if (returnValue)
            {
                if (Routes[RouteName].Url.EndsWith(".zip"))
                {
                    Thread installThread = new Thread(() =>
                    {
                        returnValue = doTheUnzipInstall(Path.Combine(installPathRoute, RouteName + ".zip"), installPathRoute);
                    });
                    installThread.Start();

                    long bytesZipfile = TotalBytes;

                    while (installThread.IsAlive)
                    {
                        Stopwatch sw = Stopwatch.StartNew();

                        TotalBytes = -bytesZipfile;
                        sumMB(installPathRoute);
                        dataGridViewDownloadContent.CurrentRow.Cells[1].Value =
                            string.Format("Installed: {0} kB", (TotalBytes / 1024).ToString("N0"));
                        Refresh();

                        while ((installThread.IsAlive) && (sw.ElapsedMilliseconds <= 3000)) { }
                    }
                }
            }

            dataGridViewDownloadContent.CurrentRow.Cells[1].Value = "";

            return returnValue;
        }

        private bool doTheClone(string installPathRoute)
        {
            try
            {
                Repository.Clone(Routes[RouteName].Url, installPathRoute);
            }
            catch (LibGit2SharpException libGit2SharpException)
            {
                {
                    string message = Catalog.GetStringFmt("Error during github download: {0}", libGit2SharpException.Message);
                    MessageBox.Show(message, Catalog.GetString("Attention"), MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return false;
                }
            }

            return true;
        }

        private bool doTheZipDownload(string url, string installPathRouteZipfileName)
        {
            try
            {
                WebClient myWebClient = new WebClient();
                myWebClient.DownloadFile(url, installPathRouteZipfileName);
            }
            catch (Exception error)
            {
                {
                    string message = Catalog.GetStringFmt("Error during download zipfile {0}: {1}", url, error.Message);
                    MessageBox.Show(message, Catalog.GetString("Attention"), MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return false;
                }
            }

            return true;
        }

        private bool doTheUnzipInstall(string installPathRouteZipfileName, string installPathRoute)
        {
            try
            {
                ZipFile.ExtractToDirectory(installPathRouteZipfileName, installPathRoute);
                File.Delete(installPathRouteZipfileName);
            }
            catch (Exception error)
            {
                {
                    string message = Catalog.GetStringFmt("Error during unzip zipfile {0}: {1}", installPathRouteZipfileName, error.Message);
                    MessageBox.Show(message, Catalog.GetString("Attention"), MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return false;
                }
            }

            return true;
        }

        long TotalBytes = 0;

        private void sumMB(string path)
        {
            foreach (string fileName in Directory.GetFiles(path))
            {
                TotalBytes += new FileInfo(fileName).Length;
            }

            foreach (string directoryName in Directory.GetDirectories(path))
            {
                sumMB(directoryName);
            }
        }

        private bool insertRowInOptions(string installPathRoute)
        {
            // check if filesystem is case sensitive
            // ok, this check will be wrong if both upper- and lowercase named route directories exist
            bool directoryCaseInsensitive = Directory.Exists(installPathRoute.ToUpper()) && Directory.Exists(installPathRoute.ToLower());

            // sometimes the route is located one directory level deeper, determine real installPathRoute
            string installPathRouteReal;

            if (Directory.Exists(Path.Combine(installPathRoute, "routes"))) {
                installPathRouteReal = installPathRoute;
            } 
            else
            {
                string[] directories = Directory.GetDirectories(installPathRoute);
                int indexDirectories = 0;
                while ((indexDirectories < directories.Length) &&
                    !Directory.Exists(Path.Combine(directories[indexDirectories], "routes")))
                {
                    indexDirectories++;
                }
                if (indexDirectories < directories.Length)
                {
                    installPathRouteReal = Path.Combine(installPathRoute, directories[indexDirectories]);
                }
                else
                { 
                    string message = Catalog.GetString("Incorrect route, directory \"routes\" not found");
                    MessageBox.Show(message, Catalog.GetString("Attention"), MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return false;
                }
            }

            bool updated = false;
            int index = 0;
            while (!updated)
            {
                string routeName = "";
                bool routeNameFound = false;
                foreach (KeyValuePair<string, string> folderSetting in Settings.Folders.Folders)
                {
                    if (index == 0)
                    {
                        routeName = RouteName;
                    }
                    else
                    {
                        routeName = string.Format("{0} ({1})", RouteName, index);
                    }
                    if (folderSetting.Key == routeName)
                    {
                        if (folderSetting.Value.Equals(installPathRouteReal,
                            directoryCaseInsensitive ? StringComparison.OrdinalIgnoreCase : StringComparison.Ordinal))
                        {
                            updated = true;
                        }
                        else
                        {
                            routeNameFound = true;
                        }
                    }
                }
                if (!updated)
                {
                    if (routeNameFound)
                    {
                        index++;
                    }
                    else
                    {
                        Settings.Folders.Folders[routeName] = installPathRouteReal;
                        updated = true;
                    }
                }
            }
            return true;
        }

        private void InfoButton_Click(object sender, EventArgs e)
        {

            using (StreamWriter outputFile = new StreamWriter(InfoTempFilename))
            {
                RouteSettings.Route route = Routes[RouteName];

                outputFile.WriteLine(string.Format("<h3>{0}:</h3>\n", RouteName));

                string description = route.Description.Replace("\n", "<br />");
                outputFile.WriteLine(string.Format("<p>{0}</p>\n", description));

                outputFile.WriteLine(string.Format("<img height='350' width='600' src = '{0}'/>\n", route.Image));

                if (string.IsNullOrWhiteSpace(route.AuthorUrl))
                {
                    outputFile.WriteLine(string.Format("<p>created by : {0}</p>\n",
                        route.AuthorName));
                }
                else
                {
                    outputFile.WriteLine(string.Format("<p>created by : <a href='{0}'>{1}</a></p>\n",
                        route.AuthorUrl, route.AuthorName));
                }

                if (!string.IsNullOrWhiteSpace(route.Screenshot))
                {
                    outputFile.WriteLine(string.Format("<p>screenshots: <a href='{0}'>{1}</a></p>\n",
                        route.Screenshot, route.Screenshot));
                }

                if (route.Url.EndsWith("git"))
                {
                    outputFile.WriteLine(String.Format("<p>Downloadable: GitHub format<br>\n"));
                    outputFile.WriteLine(String.Format("- From: {0}<br>\n", route.Url));
                    if (route.InstallSize > 0)
                    {
                        outputFile.WriteLine(String.Format("- Install size: {0} GB<br></p>\n", (route.InstallSize / (1024.0 * 1024 * 1024)).ToString("N")));
                    }
                }
                if (route.Url.EndsWith("zip"))
                {
                    outputFile.WriteLine(String.Format("<p>Downloadable: zip format<br>\n"));
                    outputFile.WriteLine(String.Format("- From: {0}<br>\n", route.Url));
                    if (route.InstallSize > 0)
                    {
                        outputFile.WriteLine(String.Format("- Install size: {0} GB<br>\n", (route.InstallSize / (1024.0 * 1024 * 1024)).ToString("N")));
                    }
                    if (route.DownloadSize > 0)
                    {
                        outputFile.WriteLine(String.Format("- Download size: {0} GB<br></p>\n", (route.DownloadSize / (1024.0 * 1024 * 1024)).ToString("N")));
                    }
                }

                if (!string.IsNullOrWhiteSpace(route.DateInstalled))
                {
                    outputFile.WriteLine(String.Format("<p>Installed:<br>\n"));
                    outputFile.WriteLine(String.Format("- at: {0}<br>\n", route.DateInstalled));
                    outputFile.WriteLine(String.Format("- in: \"{0}\"<br></p>\n", route.DirectoryInstalledIn));
                }
            }

            // show html file in default browser
            System.Diagnostics.Process.Start(InfoTempFilename);
        }


        private void DownloadContentForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            try 
            {
                if (pictureBoxRoute.Image != null)
                {
                    pictureBoxRoute.Image.Dispose();
                    pictureBoxRoute.Image = null;
                }
                File.Delete(ImageTempFilename);
                File.Delete(InfoTempFilename);
            }
            catch 
            { 
                // just ignore, the files are the user temp directory anyway
            }
        }
    }

}