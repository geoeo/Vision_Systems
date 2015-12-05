//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Vision.Systems.KinectHealth
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using Vision.Systems.KinectHealth.ViewModels;
    using Vision.Systems.KinectHealth.Models;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window
    {
        private JointVisualizerVM vm;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {

            this.InitializeComponent();
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            this.vm = new JointVisualizerVM(new JointVisualizerModel());

            this.DataContext = vm;

            // set the status text
            vm.StatusText = vm.QueryStatus();
        }


        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            this.vm.Closing();
        }

        private void Calibrate_Click(object sender, RoutedEventArgs e)
        {
            this.vm.Calibrate();
        }

        private void CheckBox_Checked(object sender, RoutedEventArgs e)
        {
            this.vm.showAngles = true;
        }

        private void CheckBox_Unchecked(object sender, RoutedEventArgs e)
        {
            this.vm.showAngles = false;
        }

    }
}
