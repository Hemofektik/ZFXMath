
#ifndef _ZFXMATH_INCLUDE_POLYGON_H_
#define _ZFXMATH_INCLUDE_POLYGON_H_

#include "Geometry.h"

namespace ZFXMath
{
	template<typename T>
	struct TPolygon2D
	{
	public:

	private:

		struct Edge
		{
			TVector2D<T> center;
			TVector2D<T> direction;
			T			 extent;
		};

		TVector2D<T>*	vertices;
		TVector2D<T>*	normals;
		Edge*			edges;
		uint32_t		numVertices;
		uint32_t		numVerticesReserved;

	public:

		TPolygon2D()
			: vertices(NULL)
			, normals(NULL)
			, edges(NULL)
			, numVertices(0)
			, numVerticesReserved(0)
		{}

		TPolygon2D(TPolygon2D&& polygon)
			: vertices(polygon.vertices)
			, normals(polygon.normals)
			, edges(polygon.edges)
			, numVertices(polygon.numVertices)
			, numVerticesReserved(polygon.numVerticesReserved)
		{
			polygon.vertices = NULL;
			polygon.normals = NULL;
			polygon.edges = NULL;
		} // copy constructor is implicitly forbidden due to user-defined move constructor (use Clone() instead)

		TPolygon2D<T>& operator= (TPolygon2D<T>&& polygon)
		{
			this->vertices = polygon.vertices;
			this->normals = polygon.normals;
			this->edges = polygon.edges;
			this->numVertices = polygon.numVertices;
			this->numVerticesReserved = polygon.numVerticesReserved;
			polygon.vertices = NULL;
			polygon.normals = NULL;
			polygon.edges = NULL;
			return *this;
		} // copy assignment is implicitly forbidden due to user-defined move assignment (use Clone() instead)

		~TPolygon2D() 
		{
			delete[] vertices;
			delete[] normals;
			delete[] edges;
		}

		template<typename TargetType = T>
		TPolygon2D<TargetType> Clone(TargetType scale = (TargetType)1.0) const
		{
			TPolygon2D<TargetType> copy;
			copy.SetNumVertices(numVertices);
			for (uint32_t v = 0; v < copy.GetNumVertices(); v++)
			{
				copy.SetVertex(v, TVector2D<TargetType>(
					vertices[v].x * scale,
					vertices[v].y * scale));
			}
			if (normals)
			{
				copy.CloseRing();
			}
			return copy;
		}

		void AddVertex( const TVector2D<T>& v )
		{
			if(numVertices == numVerticesReserved)
			{
				numVerticesReserved = Max<uint32_t>(10, numVertices * 2);
				TVector2D<T>* newVertices = new TVector2D<T>[numVerticesReserved];
				if (vertices)
				{
					memcpy(newVertices, vertices, numVertices * sizeof(TVector2D<T>));
					delete[] vertices;
				}
				vertices = newVertices;
			}

			vertices[numVertices] = v;
			numVertices++;
		}

		void SetVertex( uint32_t index, const TVector2D<T>& v )
		{
			assert( index >= 0 && index < numVertices );

			vertices[index] = v;
		}

		// removes latest vertex if it is equal to the first vertex
		// creates normal and edge information needed for any call to ComputeSqrDistance
		void CloseRing()
		{
			// "delete" last vertex if it is a duplicate of the first one
			{
				const TVector2D<T>& firstVertex = vertices[0];
				const TVector2D<T>& lastVertex = vertices[numVertices - 1];

				const TVector2D<T> delta = firstVertex - lastVertex;

				if (delta.x == 0 && delta.y == 0)
				{
					numVertices--;
				}
			}

			// TODO: recreating all these buffers (edges+normals) should be avoided if possible (see ReserveNumVertices)

			// create edges
			delete[] edges;
			const uint32_t numEdges = GetNumEdges();
			edges = new Edge[numEdges];
			for (uint32_t n = 0; n < numEdges; n++)
			{
				auto& v0 = vertices[n];
				auto& v1 = vertices[(n < numVertices - 1) ? n + 1 : 0];

				Edge& edge = edges[n];
				edge.center = (v0 + v1) * (T)0.5;
				edge.direction = v1 - v0;
				edge.extent = edge.direction.Normalize() * (T)0.5;
			}	

			// compute normals
			delete[] normals;
			const uint32_t numNormals = numVertices;
			normals = new TVector2D<double>[numVertices];
			for (uint32_t n = 0; n < numNormals; n++)
			{
				auto& e0 = edges[(n > 0) ? n - 1 : numVertices - 1];
				auto& e1 = edges[n];

				normals[n] = -(e0.direction + e1.direction).GetOrthogonal();
				normals[n].Normalize();
			}
		}

		// Reserves memory for the given amount of vertices.
		// Deletes all existing data and the number of vertices is reset to 0.
		void ReserveNumVertices(uint32_t numVertices)
		{
			delete[] vertices;
			vertices = new TVector2D<T>[numVertices];
			this->numVerticesReserved = numVertices;
			this->numVertices = 0;
		}

		// Conditionally reserves memory for the given amount of vertices.
		// Deletes all existing data and the number of vertices is reset to the given number.
		void SetNumVertices(uint32_t numVertices)
		{
			if (numVertices > numVerticesReserved)
			{
				ReserveNumVertices(numVertices);
			}
			this->numVertices = numVertices;
		}

		MathResult Check( const TVector2D<T>& v ) const
		{
			uint32_t numCrossingEdges = 0;
			for(uint32_t n = 0; n < GetNumEdges(); n++ )
			{
				TVector2D<T> v0;
				TVector2D<T> v1;

				GetEdge( n, v0, v1 );
				
				TVector2D<T> vMin = v0;
				TVector2D<T> vMax = v1;

				vMin.Min( v1 );
				vMax.Max( v0 );

				if( v.x > vMin.x && v.y > vMin.y && v.y < vMax.y )
				{
					numCrossingEdges++;
				}
			}

			return ( numCrossingEdges & 1 ) ? INSIDE : OUTSIDE;
		}

		// Returns the signed area of this polygon.
		// Area is positive if its winding order is clockwise (rightwards+;downwards+), negative otherwise.
		template<typename AreaType = T>
		AreaType ComputeArea() const
		{
			AreaType area = (AreaType)0;
			TVector2D<T> v0 = vertices[numVertices - 2];
			TVector2D<T> v1 = vertices[numVertices - 1];
			for (uint32_t v = 0; v < numVertices; v++)
			{
				TVector2D<T> v2 = vertices[v];
				area += (AreaType)(v1.x * (v2.y - v0.y));
				v0 = v1;
				v1 = v2;
			}
			area /= (AreaType)2;

			return area;
		}

		T ComputeSqrDistance(const TVector2D<T>& v, /*out*/ bool& pointIsRightOfEdge, /*out*/ TVector2D<T>* closestPoint = NULL) const
		{
			T minSqrDistance = numeric_limits<T>::max();
			bool minDistancePointIsRightOfEdge = false;
			TVector2D<T> closestPointLocal;
			TVector2D<T>* pClosestPointLocal = closestPoint ? &closestPointLocal : NULL;
			for (uint32_t n = 0; n < GetNumEdges(); n++)
			{
				bool pointIsRightOfTestEdge = false;
				T sqrDistance = SqrDistanceToEdge(v, n, pointIsRightOfTestEdge, pClosestPointLocal);
				if (sqrDistance < minSqrDistance)
				{
					minSqrDistance = sqrDistance;
					minDistancePointIsRightOfEdge = pointIsRightOfTestEdge;
					if (closestPoint)
					{
						*closestPoint = closestPointLocal;
					}
				}
			}

			pointIsRightOfEdge = minDistancePointIsRightOfEdge;
			return minSqrDistance;
		}

		// assumes that the polygon is convex
		void Split(uint32_t edgeIndex, T lerp, TPolygon2D<T>& newPolygon0, TPolygon2D<T>& newPolygon1 ) const
		{
			TVector2D<T> edgeV0;
			TVector2D<T> edgeV1;

			GetEdge( edgeIndex, edgeV0, edgeV1 );

			TVector2D<T> rayOrigin = edgeV0.Interpolate( edgeV1, lerp );
			TVector2D<T> rayDir = ( edgeV0 - edgeV1 ).GetOrthogonal();

			// find the other intersection point
			for(uint32_t n = 0; n < GetNumEdges(); n++ )
			{
				if( n == edgeIndex ) continue;

				TVector2D<T> v0;
				TVector2D<T> v1;

				GetEdge( n, v0, v1 );

				TVector2D<T> intersection;
				if( IntersectsLineRay( v0, v1, rayOrigin, rayDir, intersection ) )
				{
					uint32_t minIndex = Min( n, edgeIndex );
					uint32_t maxIndex = Max( n, edgeIndex );

					newPolygon0.SetNumVertices( maxIndex - minIndex + 2 );
					newPolygon1.SetNumVertices( GetNumVertices() - maxIndex + minIndex + 2 );

					// left polygon
					uint32_t originVertexIndex = minIndex;
					for(uint32_t vi = 0; vi < newPolygon0.GetNumVertices() - 2; vi++)
					{
						originVertexIndex++;

						newPolygon0.SetVertex( vi, vertices[originVertexIndex] );
					}
					newPolygon0.SetVertex( newPolygon0.GetNumVertices() - 2, intersection );
					newPolygon0.SetVertex( newPolygon0.GetNumVertices() - 1, rayOrigin );

					//test winding
					{
						TVector2D<T> edge0V0;
						TVector2D<T> edge0V1;
						TVector2D<T> edge1V0;
						TVector2D<T> edge1V1;
						newPolygon0.GetEdge( newPolygon0.GetNumEdges() - 2, edge0V0, edge0V1 );
						newPolygon0.GetEdge( newPolygon0.GetNumEdges() - 1, edge1V0, edge1V1 );

						TVector2D<T> edge0Dir = ( edge0V0 - edge0V1 );
						edge0Dir.Normalize();
						TVector2D<T> edge1Dir = ( edge1V0 - edge1V1 );
						edge1Dir.Normalize();
						
						if( edge0Dir.GetOrthogonal().DotProduct( edge1Dir ) > 0.0f )
						{	// correct winding order
							newPolygon0.SetVertex( newPolygon0.GetNumVertices() - 2, rayOrigin );
							newPolygon0.SetVertex( newPolygon0.GetNumVertices() - 1, intersection );
						}
					}

					// right polygon
					originVertexIndex = maxIndex;
					for(uint32_t vi = 0; vi < newPolygon1.GetNumVertices() - 2; vi++)
					{
						originVertexIndex = ( originVertexIndex + 1 ) % GetNumVertices();

						newPolygon1.SetVertex( vi, vertices[originVertexIndex] );
					}
					newPolygon1.SetVertex( newPolygon1.GetNumVertices() - 2, rayOrigin );
					newPolygon1.SetVertex( newPolygon1.GetNumVertices() - 1, intersection );

					//test winding
					{
						TVector2D<T> edge0V0;
						TVector2D<T> edge0V1;
						TVector2D<T> edge1V0;
						TVector2D<T> edge1V1;
						newPolygon1.GetEdge( newPolygon1.GetNumEdges() - 2, edge0V0, edge0V1 );
						newPolygon1.GetEdge( newPolygon1.GetNumEdges() - 1, edge1V0, edge1V1 );

						TVector2D<T> edge0Dir = ( edge0V0 - edge0V1 );
						edge0Dir.Normalize();
						TVector2D<T> edge1Dir = ( edge1V0 - edge1V1 );
						edge1Dir.Normalize();
						
						if( edge0Dir.GetOrthogonal().DotProduct( edge1Dir ) > 0.0f )
						{	// correct winding order
							newPolygon1.SetVertex( newPolygon1.GetNumVertices() - 2, intersection );
							newPolygon1.SetVertex( newPolygon1.GetNumVertices() - 1, rayOrigin );
						}
					}

					return;
				}
			}
		}

		uint32_t GetNumEdges() const
		{
			return numVertices;
		}

		uint32_t GetNumVertices() const
		{
			return numVertices;
		}

		const TVector2D<T>* GetVertices() const
		{
			return vertices;
		}

		TVector2D<T>* GetVertices()
		{
			return vertices;
		}

		const TVector2D<T>& GetVertex(uint32_t index) const
		{
			assert( index >= 0 && index < numVertices );
			return vertices[index];
		}

		TVector2D<T>& GetVertex(uint32_t index)
		{
			assert( index >= 0 && index < numVertices );
			return vertices[index];
		}

		void GetEdge(uint32_t index, TVector2D<T>& v0, TVector2D<T>& v1) const
		{
			assert( index >= 0 && index < GetNumEdges() );
			v0 = vertices[index];
			v1 = ( index < numVertices - 1 ) ? vertices[index + 1] : vertices[0];
		}

	private:

		double SqrDistanceToEdge(const TVector2D<T>& point, uint32_t edgeIndex, /*out*/ bool& pointIsRightOfEdge, /*out*/ TVector2D<T>* closestPoint = NULL) const
		{
			const Edge& edge = edges[edgeIndex];

			TVector2D<T> diff = point - edge.center;
			T segmentParameter = edge.direction.DotProduct(diff);
			TVector2D<T> segmentClosestPoint;
			TVector2D<T> segmentClosestPointNormal;
			if (-edge.extent < segmentParameter)
			{
				if (segmentParameter < edge.extent)
				{
					segmentClosestPoint = edge.center + edge.direction * segmentParameter;
					segmentClosestPointNormal = -edge.direction.GetOrthogonal();
				}
				else
				{
					// Vertex 1 of Edge
					segmentClosestPoint = vertices[(edgeIndex < numVertices - 1) ? edgeIndex + 1 : 0];
					segmentClosestPointNormal = normals[(edgeIndex < numVertices - 1) ? edgeIndex + 1 : 0];
				}
			}
			else
			{
				// Vertex 0 of Edge
				segmentClosestPoint = vertices[edgeIndex];
				segmentClosestPointNormal = normals[edgeIndex];
			}

			diff = point - segmentClosestPoint;
			T sqrDistance = diff.DotProduct(diff);

			pointIsRightOfEdge = segmentClosestPointNormal.DotProduct(diff) < 0.0;
			if (closestPoint)
			{
				*closestPoint = segmentClosestPoint;
			}

			return sqrDistance;
		}

        bool IntersectsLineRay( TVector2D<T> linePoint1, TVector2D<T> linePoint2,
                                TVector2D<T> rayOrigin,  TVector2D<T> rayDir, TVector2D<T>& intersection) const
        { // Based on the 2d line intersection method from "comp.graphics.algorithms Frequently Asked Questions" 

            /* 
                    (Ay-Cy)(Dx-Cx)-(Ax-Cx)(Dy-Cy) 
                r = -----------------------------  (eqn 1) 
                    (Bx-Ax)(Dy-Cy)-(By-Ay)(Dx-Cx) 
            */

            float q = (linePoint1.y - rayOrigin.y) * (rayDir.x) -
                      (linePoint1.x - rayOrigin.x) * (rayDir.y);
            float d = (linePoint2.x - linePoint1.x) * (rayDir.y) -
                      (linePoint2.y - linePoint1.y) * (rayDir.x);

            if (d == 0) // parallel lines so no intersection anywhere in space (in curved space, maybe, but not here in Euclidian space.) 
            {
                return false;
            }

            float r = q / d;

            /* 
                    (Ay-Cy)(Bx-Ax)-(Ax-Cx)(By-Ay) 
                s = -----------------------------  (eqn 2) 
                    (Bx-Ax)(Dy-Cy)-(By-Ay)(Dx-Cx) 
            */

            q = (linePoint1.y - rayOrigin.y) * (linePoint2.x - linePoint1.x) -
                (linePoint1.x - rayOrigin.x) * (linePoint2.y - linePoint1.y);
            float s = q / d;

            /* 
                    If r>1, P is located on extension of AB 
                    If r<0, P is located on extension of BA 
                    If s>1, P is located on extension of CD 
                    If s<0, P is located on extension of DC 

                    The above basically checks if the intersection is located at an extrapolated 
                    point outside of the line segments. To ensure the intersection is only within 
                    the line segments then the above must all be false, ie r between 0 and 1 
                    and s between 0 and 1. 
            */

            if (r < 0 || r > 1 || s < 0 /*|| s > 1*/ )
            {
                return false;
            }

            /* 
                    Px=Ax+r(Bx-Ax)
                    Py=Ay+r(By-Ay)
            */

            intersection.x = linePoint1.x + r * (linePoint2.x - linePoint1.x);
            intersection.y = linePoint1.y + r * (linePoint2.y - linePoint1.y);
            return true;
        }
	};
}

#endif
