using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HKX2Builders
{
    public struct NavmeshBuilderConfig
    {
        public float CellSize;
        public float CellHeight;
        public float WalkableSlopeAngle;
        public float WalkableHeight;
        public float WalkableClimb;
        public float WalkableRadius;
        public int MinRegionArea;
        public float DetailSampleDist;
        public float DetailSampleMaxError;

        public float CostYScale;
        public int KMeansClusteringK;
        //public float GraphMinGrouping;
        //public float GraphEdgeRadius;
        public float StreamingSetSearchRadius;

        public static NavmeshBuilderConfig Default()
        {
            return new NavmeshBuilderConfig
            {
                CellSize = 0.05f,
                CellHeight = 0.05f,
                WalkableSlopeAngle = 30.0f,
                WalkableHeight = 1.0f,
                WalkableClimb = 0.5f,
                WalkableRadius = 0.001f,
                MinRegionArea = 3,
                DetailSampleDist = 5,
                DetailSampleMaxError = 0.001f,

                CostYScale = 2f,
                KMeansClusteringK = 30,
                //GraphMinGrouping = 8f,
                //GraphEdgeRadius = 0.5f,s
                StreamingSetSearchRadius = 2f,
            };
        }
    }
}
