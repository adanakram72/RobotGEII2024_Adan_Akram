using SciChart.Charting.Visuals;
using SciChart.Charting.Visuals.Annotations;
using SciChart.Core.Extensions;
using SciChart.Data.Model;
using System;
using System.Numerics;
using System.Reflection;
using System.Security.AccessControl;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;


namespace WpfWorldMap_NS
{
    public partial class WpfWorldMap : UserControl
    {
        private RotateTransform _rotation;
        private CustomAnnotation _robot;
        private DispatcherTimer _timer;
        private double _angle;

        private RotateTransform _ghostRotation;
        private CustomAnnotation _ghost;

        private double ghostTargetX;
        private double ghostTargetY;

        private double pos_X = 20;
        private double pos_Y = 50;

        private double ghost_X = 20;
        private double ghost_Y = 50;
        private double ghost_Angle = 0;

        public double LastClickedX { get; private set; } = 0;
        public double LastClickedY { get; private set; } = 0;
        public bool HasNewClick { get; set; } = false;

        public WpfWorldMap()
        {
            InitializeComponent();
            Loaded += OnLoaded;
            sciChartSurface.MouseLeftButtonDown += SciChartSurface_MouseLeftButtonDown;
        }

        private void OnLoaded(object sender, RoutedEventArgs e)
        {
            var triangle = new Polygon
            {
                Points = new PointCollection
    {
        new Point(0, -10),
        new Point(-7, 7),
        new Point(7, 7)
    },
                Fill = Brushes.Red,
                Stroke = Brushes.Black,
                StrokeThickness = 1,
                RenderTransformOrigin = new Point(0.5, 0.5)
            };

            var rotation = new RotateTransform(0);
            triangle.RenderTransform = rotation;
            this._rotation = rotation;

            this._robot = new CustomAnnotation
            {
                X1 = 20,
                Y1 = 30,
                Content = triangle,
                HorizontalAnchorPoint = HorizontalAnchorPoint.Center,
                VerticalAnchorPoint = VerticalAnchorPoint.Center
            };


            sciChartSurface.Annotations.Add(_robot);

            var ghostTriangle = new Polygon
            {
                Points = new PointCollection
                {
                    new Point(0, -10),
                    new Point(-7, 7),
                    new Point(7, 7)
                },
                Fill = Brushes.Blue,
                Stroke = Brushes.Black,
                StrokeThickness = 1,
                RenderTransformOrigin = new Point(0.5, 0.5)
            };
            _ghostRotation = new RotateTransform(0);
            ghostTriangle.RenderTransform = _ghostRotation;

            _ghost = new CustomAnnotation
            {
                X1 = ghost_X,
                Y1 = ghost_Y,
                Content = ghostTriangle,
                HorizontalAnchorPoint = HorizontalAnchorPoint.Center,
                VerticalAnchorPoint = VerticalAnchorPoint.Center
            };
            sciChartSurface.Annotations.Add(_ghost);

            double w = sciChartSurface.XAxis.VisibleRange.Max.ToDouble();
            var mur1 = new CustomAnnotation
            {
                X1 = w / 2,
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
                Y1 = w,
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
                X1 = w,
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

            _timer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(50)
            };
            _timer.Tick += MoveRobot;
            _timer.Start();
        }
        /*private void MoveRobot(object? sender, EventArgs e)
        {
            this._robot.X1 = this.pos_X;
            this._robot.Y1 = this.pos_Y;
            this._rotation.Angle = this._angle;
        }*/

        private void MoveRobot(object? sender, EventArgs e)
        {
            this._robot.X1 = this.pos_X;
            this._robot.Y1 = this.pos_Y;
            this._rotation.Angle = this._angle;
        }


        public void UpdatePosRobot(double X, double Y, double angleDegrees)
        {
            this.pos_X = X;
            this.pos_Y = Y;
            this._angle = angleDegrees;
        }

        private void SciChartSurface_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            var sciChartSurface = sender as SciChart.Charting.Visuals.SciChartSurface;
            if (sciChartSurface == null) return;

            Point mousePos = e.GetPosition(sciChartSurface);

            double xDataValue = sciChartSurface.XAxis.GetDataValue(mousePos.X).ToDouble();
            double yDataValue = sciChartSurface.YAxis.GetDataValue(mousePos.Y).ToDouble();

            LastClickedX = xDataValue;
            LastClickedY = yDataValue;
            HasNewClick = true;
        }



        private double NormalizeAngle(double angle)
        {
            while (angle > 180) angle -= 360;
            while (angle <= -180) angle += 360;
            return angle;
        }

        public void SetGhostTarget(double x, double y)
        {
            ghostTargetX = x;
            ghostTargetY = y;
        }

    }
}

