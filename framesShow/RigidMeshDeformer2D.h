#ifndef __RMSIMPLICIT_RIGID_MESH_DEFORMER_2D_H__
#define __RMSIMPLICIT_RIGID_MESH_DEFORMER_2D_H__

//#include "stdafx.h"
#include <map>
#include <set>
#include <limits>
#include <algorithm>
#include "TriangleMesh.h"
#include "LinearSystem.h"

//using namespace cv;
//using namespace std;

typedef unsigned int size_tt;



class RigidMeshDeformer2D
{
public:
	RigidMeshDeformer2D( );
	~RigidMeshDeformer2D() {};


	void ForceValidation() { ValidateSetup(); }
	void RemoveHandle( unsigned int nHandle );

/*
 * interface stuff
 */
	//unsigned int GetNumHandles();

	//const Vec2f & GetInitialHandle(unsigned int nHandle); 
	//const Vec2f & GetDeformedHandle( unsigned int nHandle );

	//! nHandle is vertex ID
	void SetDeformedHandle( unsigned int nHandle, const Vec2f & vHandle );
	
	//void TransformPoint( Vec2f & vTransform );
	void UnTransformPoint(Vec2f & vTransform );

/*
 * mesh handling
 */
	void InitializeFromMesh( TriangleMesh * pMesh );
	void UpdateDeformedMesh( TriangleMesh * pMesh, bool bRigid );


/*
 * debug
 */
	const Vec2f * GetTriangleVerts( unsigned int nTriangle ) { return m_vTriangles[nTriangle].vScaled; }
protected:

	struct Vertex {
		Vec2f vPosition;
	};

public:
	struct Triangle {
		unsigned int nVerts[3];

		// definition of each vertex in triangle-local coordinate system
		Vec2f vTriCoords[3];

		// un-scaled triangle
		Vec2f vScaled[3];

		// pre-computed matrices for triangle scaling step
		Mat mF, mC;
	};

protected:
	std::vector<Vertex> m_vInitialVerts;
	std::vector<Vertex> m_vDeformedVerts;
	
	std::vector<Triangle> m_vTriangles;


	struct Constraint {
		unsigned int nVertex;
		Vec2f vConstrainedPos;

		Constraint() { nVertex = 0; vConstrainedPos = Vec2f(0,0); }
		Constraint( unsigned int nVert, const Vec2f & vPos ) { nVertex = nVert; vConstrainedPos = vPos; }

		bool operator<(const Constraint & c2) const
			{ return nVertex < c2.nVertex; }
	};

	std::set<Constraint> m_vConstraints;
	void UpdateConstraint( Constraint & cons );


	bool m_bSetupValid;
	void InvalidateSetup() { m_bSetupValid = false; }
	void ValidateSetup();


	Mat m_mFirstMatrix;
	std::vector<unsigned int> m_vVertexMap;
	Mat m_mHXPrime, m_mHYPrime;
	Mat m_mDX, m_mDY;

	LinearSystem::LUData m_mLUDecompX, m_mLUDecompY;

	void PrecomputeOrientationMatrix();
	void PrecomputeScalingMatrices( unsigned int nTriangle );
	void PrecomputeFittingMatrices();

	void ValidateDeformedMesh( bool bRigid );
	void UpdateScaledTriangle( unsigned int nTriangle );
	void ApplyFittingStep();

	Vec2f GetInitialVert( unsigned int nVert ) 
	  { return Vec2f( m_vInitialVerts[ nVert ].vPosition[0], m_vInitialVerts[ nVert ].vPosition[1]); }
};




#endif // __RMSIMPLICIT_RIGID_MESH_DEFORMER_2D_H__
