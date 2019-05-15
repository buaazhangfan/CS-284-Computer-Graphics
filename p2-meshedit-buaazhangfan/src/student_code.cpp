#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
  void BezierCurve::evaluateStep()
  {
    std::vector<Vector2D>& LatestLevel = evaluatedLevels.back();
    int nums = LatestLevel.size();
    if (nums == 1)
      return;
    std::vector<Vector2D>cur_points;
    for (int i = 0; i < nums - 1; i++)
    {
      Vector2D new_point = lerp(t, LatestLevel[i], LatestLevel[i + 1]);
      cur_points.push_back(new_point);
    }
    evaluatedLevels.push_back(cur_points);
    // TODO Part 1.
    // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
    // Store all of the intermediate control points into the 2D vector evaluatedLevels.
    return;
  }


  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    vector<Vector3D>row_bezier;
    for (int i = 0; i < 4; i++)
    {
      Vector3D cur = evaluate1D(controlPoints[i], u);
      row_bezier.push_back(cur);
    }
    
    return evaluate1D(row_bezier, v);
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
  {
    Vector3D p0_1 = lerp(t, points[0], points[1]);
    Vector3D p1_1 = lerp(t, points[1], points[2]);
    Vector3D p2_1 = lerp(t, points[2], points[3]);
    Vector3D p0_2 = lerp(t, p0_1, p1_1);
    Vector3D p1_2 = lerp(t, p1_1, p2_1);
    
    return lerp(t, p0_2, p1_2);
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.
 }



  Vector3D Vertex::normal( void ) const
  {
  	Vector3D n(0, 0, 0);
    HalfedgeCIter h = this -> halfedge();
  	h = h -> twin();
  
    HalfedgeCIter h_orig = h;
    
    do
    {
      Vector3D vec1 = h -> vertex() -> position - position;
      h = h -> next() -> twin();
      Vector3D vec2 = h -> vertex() -> position - position;
      n += cross(vec2, vec1);
    }
    while (h != h_orig);
    
    return n.unit();
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
    // return Vector3D();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
  	if (e0 -> isBoundary())
  	  return e0;
  
    HalfedgeIter h0 = e0 -> halfedge();
    HalfedgeIter h1 = h0 -> next();
    HalfedgeIter h2 = h1 -> next();
    HalfedgeIter h3 = h0 -> twin();
    HalfedgeIter h4 = h3 -> next();
    HalfedgeIter h5 = h4 -> next();
    HalfedgeIter h6 = h5 -> twin();
    HalfedgeIter h7 = h4 -> twin();
    HalfedgeIter h8 = h2 -> twin();
    HalfedgeIter h9 = h1 -> twin();
  
    VertexIter v0 = h0 -> vertex();
    VertexIter v1 = h3 -> vertex();
    VertexIter v2 = h5 -> vertex();
    VertexIter v3 = h2 -> vertex();
  
    EdgeIter e1 = h5 -> edge();
    EdgeIter e2 = h4 -> edge();
    EdgeIter e3 = h2 -> edge();
    EdgeIter e4 = h1 -> edge();
  
    FaceIter f0 = h0 -> face();
    FaceIter f1 = h3 -> face();
  
    v0 -> halfedge() = h4;
    v1 -> halfedge() = h1;
    v2 -> halfedge() = h5;
    v3 -> halfedge() = h2;
  
    e0 -> halfedge() = h0;
    e1 -> halfedge() = h5;
    e2 -> halfedge() = h4;
    e3 -> halfedge() = h2;
    e4 -> halfedge() = h1;
  
    f0 -> halfedge() = h0;
    f1 -> halfedge() = h3;
  
  
    h0 -> setNeighbors(h2, h3, v2, e0, f0);
    h1 -> setNeighbors(h3, h9, v1, e4, f1);
    h2 -> setNeighbors(h4, h8, v3, e3, f0);
    h3 -> setNeighbors(h5, h0, v3, e0, f1);
    h4 -> setNeighbors(h0, h7, v0, e2, f0);
    h5 -> setNeighbors(h1, h6, v2, e1, f1);
    
    h6 -> setNeighbors(h6 -> next(), h5, v1, e1, h6 -> face());
    h7 -> setNeighbors(h7 -> next(), h4, v2, e2, h7 -> face());
    h8 -> setNeighbors(h8 -> next(), h2, v0, e3, h8 -> face());
    h9 -> setNeighbors(h9 -> next(), h1, v3, e4, h9 -> face());
    
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.
    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
  
    if (e0 -> isBoundary())
      return e0 -> halfedge() -> vertex();
      
    HalfedgeIter h0 = e0 -> halfedge();
    HalfedgeIter h1 = h0 -> next();
    HalfedgeIter h2 = h1 -> next();
    HalfedgeIter h3 = h0 -> twin();
    HalfedgeIter h4 = h3 -> next();
    HalfedgeIter h5 = h4 -> next();
    HalfedgeIter h6 = h5 -> twin();
    HalfedgeIter h7 = h4 -> twin();
    HalfedgeIter h8 = h2 -> twin();
    HalfedgeIter h9 = h1 -> twin();
  
    VertexIter v0 = h0 -> vertex();
    VertexIter v1 = h3 -> vertex();
    VertexIter v2 = h5 -> vertex();
    VertexIter v3 = h2 -> vertex();
  
    EdgeIter e1 = h5 -> edge();
    EdgeIter e2 = h4 -> edge();
    EdgeIter e3 = h2 -> edge();
    EdgeIter e4 = h1 -> edge();
  
    FaceIter f0 = h0 -> face();
    FaceIter f1 = h3 -> face();
  
    v0 -> halfedge() = h4;
    v1 -> halfedge() = h1;
    v2 -> halfedge() = h5;
    v3 -> halfedge() = h2;
  
    e0 -> halfedge() = h0;
    e1 -> halfedge() = h5;
    e2 -> halfedge() = h4;
    e3 -> halfedge() = h2;
    e4 -> halfedge() = h1;
  
    f0 -> halfedge() = h0;
    f1 -> halfedge() = h3;
  
    VertexIter v4 = newVertex();
    v4 -> position = (v0 -> position + v1 -> position) * 0.5;
  
    EdgeIter e5 = newEdge();
    EdgeIter e6 = newEdge();
    EdgeIter e7 = newEdge();
  
    FaceIter f2 = newFace();
    FaceIter f3 = newFace();
  
    HalfedgeIter h10 = newHalfedge();
    HalfedgeIter h11 = newHalfedge();
    HalfedgeIter h12 = newHalfedge();
    HalfedgeIter h13 = newHalfedge();
    HalfedgeIter h14 = newHalfedge();
    HalfedgeIter h15 = newHalfedge();
    
    h0 -> setNeighbors(h1, h3, v4, e0, f0);
    h1 -> setNeighbors(h10, h9, v1, e4, f0);
    h2 -> setNeighbors(h15, h8, v3, e3, f2);
    h3 -> setNeighbors(h12, h0, v1, e0, f1);
    h4 -> setNeighbors(h13, h7, v0, e2, f3);
    h5 -> setNeighbors(h3, h6, v2, e1, f1);
    h6 -> setNeighbors(h6 -> next(), h5, v1, e1, h6 -> face());
    h7 -> setNeighbors(h7 -> next(), h4, v2, e2, h7 -> face());
    h8 -> setNeighbors(h8 -> next(), h2, v0, e3, h8 -> face());
    h9 -> setNeighbors(h9 -> next(), h1, v3, e4, h9 -> face());
    h10 -> setNeighbors(h0, h11, v3, e5, f0);
    h11 -> setNeighbors(h2, h10, v4, e5, f2);
    h12 -> setNeighbors(h5, h13, v4, e6, f1);
    h13 -> setNeighbors(h14, h12, v2, e6, f3);
    h14 -> setNeighbors(h4, h15, v4, e7, f3);
    h15 -> setNeighbors(h11, h14, v0, e7, f2);
  
    v0 -> halfedge() = h4;
    v1 -> halfedge() = h1;
    v2 -> halfedge() = h5;
    v3 -> halfedge() = h2;
    v4 -> halfedge() = h0;
  
    e0 -> halfedge() = h0;
    e1 -> halfedge() = h5;
    e2 -> halfedge() = h4;
    e3 -> halfedge() = h2;
    e4 -> halfedge() = h1;
    e5 -> halfedge() = h10;
    e6 -> halfedge() = h12;
    e7 -> halfedge() = h15;
  
    f0 -> halfedge() = h0;
    f1 -> halfedge() = h3;
    f2 -> halfedge() = h15;
    f3 -> halfedge() = h14;
  
  
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    return v4;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
		// TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++)
    {
      v -> isNew = false;
      int n = v -> degree();
      float u;
      if (n == 3)
        u = 3 / (float)16;
      else
        u = 3 / (8 * (float)n);
      
      Vector3D n_p = (1 - (n * u)) * (v -> position);
      Vector3D neigh_sum;
      HalfedgeIter h = v -> halfedge();
      do{
        HalfedgeIter h_twin = h -> twin();
        VertexIter v = h_twin -> vertex();
        neigh_sum += v -> position;
        h = h_twin -> next();
      } while(h != v->halfedge());
      
      n_p += (u * neigh_sum);
      v -> newPosition = n_p;
    }
    
    vector<EdgeIter>old_edges;
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++)
    {
      e -> isNew = false;
      HalfedgeIter h = e -> halfedge();
      VertexIter v0 = h -> vertex();
      HalfedgeIter h_twin = h -> twin();
      VertexIter v1 = h_twin -> vertex();
      VertexIter v3 = h -> next() -> twin() -> vertex();
      VertexIter v2 = h_twin -> next() -> twin() -> vertex();
      
      Vector3D new_position = (3 / (float)8) * ((v0 -> position) + (v1 -> position)) + (1 / (float)8) * ((v2 -> position) + (v3 -> position));
      e -> newPosition = new_position;
      old_edges.push_back(e);
    }
    
    for (auto e : old_edges)
    {
      VertexIter new_v = mesh.splitEdge(e);
      new_v -> isNew = true;
      new_v -> newPosition = (e) -> newPosition;
      HalfedgeIter h  = new_v -> halfedge();
      EdgeIter n_e_1 = h -> next() -> next() -> edge();
      n_e_1 -> isNew = true;
      HalfedgeIter h_twin = h -> twin();
      EdgeIter n_e_2 = h_twin -> next() -> edge();
      n_e_2 -> isNew = true;
    }
    
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++)
    {
      if (e -> isNew == true)
      {
        HalfedgeIter h = e -> halfedge();
        VertexIter v0 = h -> vertex();
        VertexIter v1 = h -> twin() -> vertex();
        
        if ((v0 -> isNew == true && v1 -> isNew == false) || (v0 -> isNew == false && v1 -> isNew == true))
          mesh.flipEdge(e);
      }
    }
    
    
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++)
    {
      v -> position = v -> newPosition;
    }

    

    // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // TODO a vertex of the original mesh.


    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.


    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)


    // TODO Now flip any new edge that connects an old and new vertex.


    // TODO Finally, copy the new vertex positions into final Vertex::position.

    return;
  }
}
