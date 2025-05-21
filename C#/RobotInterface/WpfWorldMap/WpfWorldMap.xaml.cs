using SciChart.Charting.Visuals;
using SciChart.Charting.Visuals.Annotations;
using SciChart.Core.Extensions;
using SciChart.Data.Model;
using System;
using System.Reflection;
using System.Security.AccessControl;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace WpfWorldMap_NS
{
    public partial class WpfWorldMap : UserControl
    {
        private CustomAnnotation _robot;
        private DispatcherTimer _timer;
        private double _angle;
        // Déplace le cercle en cercle
        
        private double pos_X = 20;
        private double pos_Y = 50;

        public WpfWorldMap()
        {
            InitializeComponent();
            Loaded += OnLoaded;
        }

        private void OnLoaded(object sender, RoutedEventArgs e)
        {
            // Crée un cercle initial à (20, 30)
            var cercle = new Ellipse
            {
                Width = 20,
                Height = 20,
                Fill = Brushes.Red,
                Stroke = Brushes.Black,
                StrokeThickness = 1
            };

            this._robot = new CustomAnnotation
            {
                X1 = 20,
                Y1 = 30,
                Content = cercle,
                HorizontalAnchorPoint = HorizontalAnchorPoint.Center,
                VerticalAnchorPoint = VerticalAnchorPoint.Center
            };

            sciChartSurface.Annotations.Add(_robot);

           
            
            double w = sciChartSurface.XAxis.VisibleRange.Max.ToDouble();
            var mur1 = new CustomAnnotation
            {
                X1 = w/2,
                Y1 = 0,
                Content = new Rectangle
                {
                    Width = 1000,
                    Height = 10,
                    Fill = Brushes.Green,
                    Stroke = Brushes.Black,
                    StrokeThickness = 0,
                },
                HorizontalAnchorPoint = HorizontalAnchorPoint.Center,
                VerticalAnchorPoint = VerticalAnchorPoint.Center
            };
            sciChartSurface.Annotations.Add(mur1);

            var mur2 = new CustomAnnotation
            {
                X1 = w / 2,
                Y1 = w ,
                Content = new Rectangle
                {
                    Width = 1000,
                    Height = 10,
                    Fill = Brushes.Green,
                    Stroke = Brushes.Black,
                    StrokeThickness = 0,
                },
                HorizontalAnchorPoint = HorizontalAnchorPoint.Center,
                VerticalAnchorPoint = VerticalAnchorPoint.Center
            };
            sciChartSurface.Annotations.Add(mur2);

            var mur3 = new CustomAnnotation
            {
                X1 = 0,
                Y1 = w / 2,
                Content = new Rectangle
                {
                    Width = 10,
                    Height = 1000,
                    Fill = Brushes.Green,
                    Stroke = Brushes.Black,
                    StrokeThickness = 0,
                },
                HorizontalAnchorPoint = HorizontalAnchorPoint.Center,
                VerticalAnchorPoint = VerticalAnchorPoint.Center
            };
            sciChartSurface.Annotations.Add(mur3);

            var mur4 = new CustomAnnotation
            {
                X1 = w ,
                Y1 = w / 2,
                Content = new Rectangle
                {
                    Width = 10,
                    Height = 1000,
                    Fill = Brushes.Green,
                    Stroke = Brushes.Black,
                    StrokeThickness = 0,
                },
                HorizontalAnchorPoint = HorizontalAnchorPoint.Center,
                VerticalAnchorPoint = VerticalAnchorPoint.Center
            };
            sciChartSurface.Annotations.Add(mur4);




            //var contour = new BoxAnnotation
            //{
            //    X1 = 0,
            //    Y1 = 0,
            //    X2 = 10,  // Remplace par ta taille max en X
            //    Y2 = 10,  // Remplace par ta taille max en Y
            //    BorderThickness = new Thickness(3),
            //    BorderBrush = Brushes.Green,
            //    Background = Brushes.Transparent,
            //    CoordinateMode = AnnotationCoordinateMode.Relative, // ou Absolute selon usage
            //    IsEditable = false
            //};

            //sciChartSurface.Annotations.Add(contour);

            // Timer pour faire bouger le cercle
            _timer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(50)
            };
            _timer.Tick += MoveRobot;
            _timer.Start();
        }

        private void MoveRobot(object? sender, EventArgs e)
        {
            this._robot.X1 = this.pos_X;
            this._robot.Y1 = this.pos_Y;
        }

        public void UpdatePosRobot(double X,  double Y)
        {
            this.pos_X = X;
            this.pos_Y = Y;
        }
    }
}
