﻿using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace ORTS.TrackViewer.UserInterface
{
    /// <summary>
    /// Interaction logic for SceneWindow.xaml
    /// </summary>
    public partial class SceneWindow : Window
    {
        UIElement HostWindow;

        public SceneWindow(UIElement hostWindow)
        {
            InitializeComponent();

            HostWindow = hostWindow;
            //HostVisualElement.Children.Add((SceneViewerVisualHost)HostWindow);
            GraphicsHostElement.Children.Add((SceneViewerHwndHost)hostWindow);
        }

        protected override void OnClosing(CancelEventArgs e)
        {
            e.Cancel = true;
            Hide();
        }
    }
}