﻿using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Runtime.Serialization;
using System.Windows.Shapes;
using ZeroFormatter;

namespace Utilities
{
    [DataContract]
    public class PointD
    {
        [DataMember]
        public double X;// { get; set; }
        [DataMember]
        public double Y;// { get; set; }
        public PointD(double x, double y)
        {
            X = x;
            Y = y;
        }
    }

    public class PointDExtended
    {
        public PointD Pt;
        public Color Color;
        public double Width;

        public PointDExtended(PointD pt, Color c, double size)
        {
            Pt = pt;
            Color = c;
            Width = size;
        }
    }

    [ZeroFormattable]
    public class SegmentD
    {

        [Index(0)]
        virtual public double X1 { get; set; }

        [Index(1)]
        virtual public double Y1 { get; set; }

        [Index(2)]
        virtual public double X2 { get; set; }

        [Index(3)]
        virtual public double Y2 { get; set; }

        [IgnoreFormat]
        public PointD PtDebut
        {
            get { return new PointD(X1, Y1); }
            private set { X1 = PtDebut.X; Y1 = PtDebut.Y; }
        }

        [IgnoreFormat]
        public PointD PtFin
        {
            get { return new PointD(X2, Y2); }
            private set { X2 = PtFin.X; Y2 = PtFin.Y; }
        }

        [IgnoreFormat]
        public double Angle
        {
            get { return Math.Atan2(Y2 - Y1, X2 - X1); }
            //private set { }
        }

        public SegmentD(PointD ptDebut, PointD ptFin)
        {
            X1 = ptDebut.X;
            Y1 = ptDebut.Y;
            X2 = ptFin.X;
            Y2 = ptFin.Y;
        }

        public SegmentD()
        {
        }
    }

    public class Point3D
    {
        public double X;// { get; set; }
        public double Y;// { get; set; }
        public double Z;// { get; set; }
        public Point3D(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
        }
    }

    public class RectangleD
    {
        public double Xmin;
        public double Xmax;// { get; set; }
        public double Ymin;
        public double Ymax;// { get; set; }
        public RectangleD(double xMin, double xMax, double yMin, double yMax)
        {
            Xmin = xMin;
            Xmax = xMax;
            Ymin = yMin;
            Ymax = yMax;
        }
    }

    public class ConvexPolygonD
    {
        public List<PointD> PtList;
        public List<SegmentD> SegmentList;
        public bool isConvex = true;

        public ConvexPolygonD(List<PointD> ptList)
        {
            PtList = ptList;
            SegmentList = new List<SegmentD>();

            ///On génère l'ensemble des segments du polygone dans l'ordre
            for (int i = 0; i < PtList.Count - 1; i++)
            {
                SegmentList.Add(new SegmentD(ptList[i], ptList[i + 1]));
            }
            if(PtList.Count>1)
                SegmentList.Add(new SegmentD(ptList[PtList.Count-1], ptList[0]));

            isConvex = IsConvex();
        }

        private bool IsConvex()
        {
            bool SegmentAngleMustBePositive = false;
            if (SegmentList.Count >= 3)
            {
                if (Toolbox.Angle(SegmentList[SegmentList.Count - 1], SegmentList[0]) > 0)
                    SegmentAngleMustBePositive = true;
                else
                    SegmentAngleMustBePositive = false;

                for (int i = 0; i < SegmentList.Count - 1; i++)
                {
                    double angle = Toolbox.Angle(SegmentList[i], SegmentList[i+1]);
                    if (angle < 0 && SegmentAngleMustBePositive == true)
                        return false;
                    if (angle > 0 && SegmentAngleMustBePositive == false)
                        return false;
                }

                return true;
            }
            else 
            {
                return false;
            }
        }

        public bool IsInside(PointD pt)
        {
            bool positiveSide = false;
            var ptSegment = new SegmentD(SegmentList[0].PtDebut, pt);
            double angle = Toolbox.VectorProduct(SegmentList[0], ptSegment);

            if (angle > 0)
                positiveSide = true;
            else
                positiveSide = false;

            for (int i = 1; i < SegmentList.Count; i++)
            {
                ptSegment = new SegmentD(SegmentList[i].PtDebut, pt);
                angle = Toolbox.VectorProduct(SegmentList[i], ptSegment);
                if (angle < 0 && positiveSide == true)
                    return false;
                if (angle > 0 && positiveSide == false)
                    return false;
            }

            return true;
        }
    }

    public class PolarPoint
    {
        public double Distance;
        public double Angle;

        public PolarPoint(double angle, double distance)
        {
            Distance = distance;
            Angle = angle;
        }
    }
    [ZeroFormattable]
    public class PolarPointRssi
    {
        [Index(0)]
        public virtual double Distance { get; set; }
        [Index(1)]
        public virtual double Angle { get; set; }
        [Index(2)]
        public virtual double Rssi { get; set; }

        public PolarPointRssi(double angle, double distance, double rssi)
        {
            Distance = distance;
            Angle = angle;
            Rssi = rssi;
        }
        public PolarPointRssi()
        {

        }
        public PolarPoint ToPolarPoint()
        {
            return new PolarPoint(Angle, Distance);
        }
    }

    public class PolarPointRssiExtended
    {
        public PolarPointRssi Pt { get; set; }
        public double Width { get; set; }
        public Color Color { get; set; }

        public PolarPointRssiExtended(PolarPointRssi pt, double width, Color c)
        {
            Pt = pt;
            Width = width;
            Color = c;
        }
    }

    public class PolarCourbure
    {
        public virtual double Courbure { get; set; }
        public virtual double Angle { get; set; }
        public virtual bool Discontinuity { get; set; }
        public PolarCourbure(double angle, double courbure, bool discontinuity)
        {            
            Angle = angle;
            Courbure = courbure;
            Discontinuity = discontinuity;
        }
    }

    [ZeroFormattable][Serializable]
    public class Location
    {
        [Index(0)]
        public virtual double X { get; set; }
        [Index(1)]
        public virtual double Y { get; set; }
        [Index(2)]
        public virtual double Theta { get; set; }
        [Index(3)]
        public virtual double Vx { get; set; }
        [Index(4)]
        public virtual double Vy { get; set; }
        [Index(5)]
        public virtual double Vtheta { get; set; }

        public Location()
        {

        }
        public Location(double x, double y, double theta, double vx, double vy, double vtheta)
        {
            X = x;
            Y = y;
            Theta = theta;
            Vx = vx;
            Vy = vy;
            Vtheta = vtheta;
        }
        public Location(double x, double y, double theta)
        {
            X = x;
            Y = y;
            Theta = theta;
            Vx = 0;
            Vy = 0;
            Vtheta = 0;
        }

        public PointD ToPointD()
        {
            return new PointD(X, Y);
        }
    }

    //Pose probleme
    [ZeroFormattable]
    public class LocationExtended
    {
        [Index(0)]
        public virtual double X { get; set; }
        [Index(1)]
        public virtual double Y { get; set; }
        [Index(2)]
        public virtual double Theta { get; set; }
        [Index(3)]
        public virtual double Vx { get; set; }
        [Index(4)]
        public virtual double Vy { get; set; }
        [Index(5)]
        public virtual double Vtheta { get; set; }
        [Index(6)]
        public virtual ObjectType Type { get; set; }
        [Index(7)]
        public virtual int Id { get; set; }

        public LocationExtended()
        {

        }
        public LocationExtended(double x, double y, double theta, double vx, double vy, double vtheta, ObjectType type, int id=0)
        {
            X = x;
            Y = y;
            Theta = theta;
            Vx = vx;
            Vy = vy;
            Vtheta = vtheta;
            Type = type;
            Id = id;
        }
        public LocationExtended(double x, double y, double theta, ObjectType type)
        {
            X = x;
            Y = y;
            Theta = theta;
            Vx = 0;
            Vy = 0;
            Vtheta = 0;
            Type = type;
        }
        public LocationExtended(Location l, ObjectType type)
        {
            X = l.X;
            Y = l.Y;
            Theta = l.Theta;
            Vx = l.Vx;
            Vy = l.Vy;
            Vtheta = l.Vtheta;
            Type = type;
        }
        public PointD ToPointD()
        {
            return new PointD(X, Y);
        }
        public Location ToLocation()
        {
            return new Location(X, Y, Theta, Vx, Vy, Vtheta);
        }
    }
    public class PotentialLocationFromBalises
    {
        public Location location;
        public PointD Balise1Theorique;
        public PointD Balise2Theorique;
        public PointD Balise3Theorique;
        public double DistanceBalise1;
        public double AngleBalise1;
        public double DistanceBalise2;
        public double AngleBalise2;
        public double DistanceBalise3;
        public double AngleBalise3;
        public double Score;

        public PotentialLocationFromBalises()
        {
            location = new Location();
        }
    }

    public class PolygonD
    {
        public List<PointD> Points = new List<PointD>();
    }

    public class PolygonExtended
    {
        public PolygonD polygon = new PolygonD();
        public float borderWidth = 1;
        public System.Drawing.Color borderColor = System.Drawing.Color.FromArgb(0xFF, 0xFF, 0xFF, 0xFF);
        public double borderOpacity = 1;
        public double[] borderDashPattern = new double[] { 1.0 };
        public System.Drawing.Color backgroundColor = System.Drawing.Color.FromArgb(0x66, 0xFF, 0xFF, 0xFF);
        public PolygonExtended Copy()
        {
            PolygonExtended polygonCopy = new PolygonExtended();
            polygonCopy.polygon = polygon;
            polygonCopy.borderWidth = borderWidth;
            polygonCopy.borderOpacity = borderOpacity;
            polygonCopy.borderDashPattern = borderDashPattern;
            polygonCopy.borderColor = borderColor;
            polygonCopy.backgroundColor = backgroundColor;
            return polygonCopy;
        }
    }

    [ZeroFormattable]
    public class SegmentExtended
    {
        [Index(0)]
        public virtual SegmentD Segment { get; set; }

        [Index(1)]
        public virtual double Width { get; set; }

        [IgnoreFormat]
        public virtual System.Drawing.Color Color { get; set; } 

        [Index(3)]
        public virtual double Opacity { get; set; }

        [Index(4)]
        public virtual double[] DashPattern { get; set; }

        public SegmentExtended(PointD ptDebut, PointD ptFin, System.Drawing.Color color, double width = 1)
        {
            Segment = new SegmentD(ptDebut, ptFin);
            Color = color;
            Width = width;
            Opacity = 1;
            DashPattern = new double[] { 1.0 };
        }
        public SegmentExtended()
        {

        }
    }

    public class PolarPointListExtended
    {
        public List<PolarPointRssi> polarPointList;
        public ObjectType type;
        //public System.Drawing.Color displayColor;
        //public double displayWidth=1;
    }

    public class CircularZone
    {
        public PointD center;
        public double radius; //Le rayon correspond à la taille la zone - à noter que l'intensité diminuera avec le rayon
        public double centerStrength; //La force correspond à l'intensité du point central de la zone
        public double radialStrength; //La force correspond à l'intensité du point central de la zone
        public CircularZone(PointD center, double radius, double centerStrength = 1.0, double radialStrength = 0.0)
        {
            this.radius = radius;
            this.center = center;
            this.centerStrength = centerStrength ;
            this.radialStrength = radialStrength ;
        }
    }
    public class RectangleZone
    {
        public RectangleD rectangularZone;
        public double strength; //La force correspond à l'intensité du point central de la zone
        public RectangleZone(RectangleD rect, double strength = 1)
        {
            this.rectangularZone = rect;
            this.strength = strength;
        }
    }

    public class RectanglePointZone : PolygonConvexPointZone
    {
        /// ATTENTION : dans cette classe l'ordre des points est très important !
        /// Les points doivent être déclarés dans le sens trigonométrique
        /// 
        //public List<PointD> ptList = new List<PointD>();
        //public double strength; //La force correspond à l'intensité du point central de la zone

        public RectanglePointZone(PointD p1, PointD p2, PointD p3, PointD p4, double strength = 1) : base(new List<PointD> { p1, p2, p3, p4 }, strength)
        {
            this.ptList.Add(p1);
            this.ptList.Add(p2);
            this.ptList.Add(p3);
            this.ptList.Add(p4);
            this.strength = strength;
        }
        public RectanglePointZone(List<PointD> pList, double strength = 1) : base(pList, strength)
        {
            this.ptList=pList;
            this.strength = strength;
        }
    }


    public class PolygonConvexPointZone
    {
        /// ATTENTION : dans cette classe l'ordre des points est très important !
        /// Les points doivent être déclarés dans le sens trigonométrique
        /// 
        public List<PointD> ptList = new List<PointD>();
        public double strength; //La force correspond à l'intensité du point central de la zone
        
        public PolygonConvexPointZone(List<PointD> pList, double strength = 1)
        {
            this.ptList = pList;
            this.strength = strength;
        }

        public void GetBoundary(out List<PointD> upperPtList, out List<PointD> lowerPtList)
        {
            upperPtList = new List<PointD>();
            lowerPtList = new List<PointD>();

            //On récupère le pt le plus à gauche
            var minPt = ptList.OrderBy(x => x.X).First();
            var minPtIndex = ptList.IndexOf(minPt);

            //On récupère le pt le plus à droite
            var maxPt = ptList.OrderByDescending(x => x.X).First();
            var maxPtIndex = ptList.IndexOf(maxPt);

            if (minPtIndex < maxPtIndex)
            {
                for (int i = maxPtIndex; i < ptList.Count; i++)
                    upperPtList.Add(ptList[i]);
                for (int i = 0; i <= minPtIndex; i++)
                    upperPtList.Add(ptList[i]);
                for (int i = minPtIndex; i <= maxPtIndex; i++)
                    lowerPtList.Add(ptList[i]);
            }
            else
            {
                for (int i = minPtIndex; i < ptList.Count; i++)
                    lowerPtList.Add(ptList[i]);
                for (int i = 0; i <= maxPtIndex; i++)
                    lowerPtList.Add(ptList[i]);
                for (int i = maxPtIndex; i <= minPtIndex; i++)
                    upperPtList.Add(ptList[i]);
            }
        }
    }


    public class ConicalZone
    {
        public PointD InitPoint;
        public PointD Cible;
        public double Radius;
        public ConicalZone(PointD initPt, PointD ciblePt, double radius)
        {
            InitPoint = initPt;
            Cible = ciblePt;
            Radius = radius;
        }
    }

    public class ConicalPolygonalZone
    {
        public PointD InitPoint;
        public PolygonD Polygone;
        public double Radius;
        public ConicalPolygonalZone(PointD initPt, PolygonD polygon, double radius)
        {
            InitPoint = initPt;
            Polygone = polygon;
            Radius = radius;
        }
    }

    public class PolygonElargedZone
    {
        public PolygonConvexPointZone Polygon;
        public double Radius;
        public PolygonElargedZone(PolygonConvexPointZone polygon, double radius)
        {
            Polygon = polygon;
            Radius = radius;
        }
    }

    public class AngularZone
    {
        public PointD InitPoint;
        public PointD Cible;
        public double DispersionAngle;
        public double Strength;
        public AngularZone(PointD initPt, PointD ciblePt, double dispersion, double strength = 1)
        {
            InitPoint = initPt;
            Cible = ciblePt;
            DispersionAngle = dispersion;
            Strength = strength;
        }
    }

    public class SegmentZone
    {
        public PointD PointA;
        public PointD PointB;
        public double Radius;
        public double Strength;
        public SegmentZone(PointD ptA, PointD ptB, double radius, double strength)
        {
            PointA = ptA;
            PointB = ptB;
            Radius = radius;
            Strength = strength;
        }
    }

    public class ObjectType
    {

    }
}
