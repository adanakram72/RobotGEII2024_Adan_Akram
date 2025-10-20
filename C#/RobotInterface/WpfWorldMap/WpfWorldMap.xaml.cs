using SciChart.Charting.Visuals;
using SciChart.Charting.Visuals.Annotations;
using SciChart.Charting.Visuals.Axes;
using SciChart.Core.Extensions;
using SciChart.Data.Model;
using System;
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

        private double pos_X = 1.0;   // centre X de la carte (0..2)
        private double pos_Y = 1.5;   // centre Y de la carte (0..3)

        private double ghost_X = 0.3;
        private double ghost_Y = 0.2;
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
            // Carte 2m x 3m
            sciChartSurface.XAxis.AutoRange = AutoRange.Never;
            sciChartSurface.YAxis.AutoRange = AutoRange.Never;
            sciChartSurface.XAxis.VisibleRange = new DoubleRange(0.0, 2.0);
            sciChartSurface.YAxis.VisibleRange = new DoubleRange(0.0, 3.0);

            // --- Robot (triangle rouge) au CENTRE ---
            var triangle = new Polygon
            {
                Points = new PointCollection { new Point(0, -10), new Point(-7, 7), new Point(7, 7) },
                Fill = Brushes.Red,
                Stroke = Brushes.Black,
                StrokeThickness = 1,
                RenderTransformOrigin = new Point(0.5, 0.5)
            };
            _rotation = new RotateTransform(0);
            triangle.RenderTransform = _rotation;

            _robot = new CustomAnnotation
            {
                X1 = pos_X,   // 1.0
                Y1 = pos_Y,   // 1.5
                Content = triangle,
                HorizontalAnchorPoint = HorizontalAnchorPoint.Center,
                VerticalAnchorPoint = VerticalAnchorPoint.Center
            };
            sciChartSurface.Annotations.Add(_robot);

            // --- Ghost (triangle bleu) optionnel ---
            var ghostTriangle = new Polygon
            {
                Points = new PointCollection { new Point(0, -10), new Point(-7, 7), new Point(7, 7) },
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

            // Cadre 2m x 3m (vert)
            sciChartSurface.Annotations.Add(new LineAnnotation { X1 = 0, Y1 = 0, X2 = 2, Y2 = 0, Stroke = Brushes.Green, StrokeThickness = 2 });
            sciChartSurface.Annotations.Add(new LineAnnotation { X1 = 0, Y1 = 3, X2 = 2, Y2 = 3, Stroke = Brushes.Green, StrokeThickness = 2 });
            sciChartSurface.Annotations.Add(new LineAnnotation { X1 = 0, Y1 = 0, X2 = 0, Y2 = 3, Stroke = Brushes.Green, StrokeThickness = 2 });
            sciChartSurface.Annotations.Add(new LineAnnotation { X1 = 2, Y1 = 0, X2 = 2, Y2 = 3, Stroke = Brushes.Green, StrokeThickness = 2 });

            // Timer d'update (ne modifie PAS les positions)
            _timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(20) };
            _timer.Tick += MoveRobot;
            _timer.Start();
        }

        private void MoveRobot(object? sender, EventArgs e)
        {
            // On applique juste les valeurs courantes
            _robot.X1 = pos_X;   // restera 1.0 tant que tu n'appelles pas UpdatePosRobot
            _robot.Y1 = pos_Y;   // restera 1.5
            _rotation.Angle = _angle;

            _ghost.X1 = ghost_X;
            _ghost.Y1 = ghost_Y;
            _ghostRotation.Angle = ghost_Angle;
        }

        // Appelle cette méthode si tu veux déplacer le robot (sinon il reste au centre)
        public void UpdatePosRobot(double X, double Y, double angleDegrees)
        {
            pos_X = X;
            pos_Y = Y;
            _angle = angleDegrees;
        }

        public void UpdateGhost(double x, double y, double angleDegrees)
        {
            ghost_X = x;
            ghost_Y = y;
            ghost_Angle = angleDegrees;
        }

        public event EventHandler<Point>? WorldPointSelected;

        private void SciChartSurface_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            var s = (SciChart.Charting.Visuals.SciChartSurface)sender;
            var p = e.GetPosition(s);
            double x = s.XAxis.GetDataValue(p.X).ToDouble();
            double y = s.YAxis.GetDataValue(p.Y).ToDouble();

            // clamp clics à la carte
            if (x < 0) x = 0; else if (x > 2) x = 2;
            if (y < 0) y = 0; else if (y > 3) y = 3;

            LastClickedX = x;
            LastClickedY = y;
            HasNewClick = true;

            WorldPointSelected?.Invoke(this, new Point(x, y));
        }
    }
}
