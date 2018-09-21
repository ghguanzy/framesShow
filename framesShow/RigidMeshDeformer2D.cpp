#include "stdafx.h"
#include "RigidMeshDeformer2D.h"

//using namespace rmsmesh;
//using namespace std;


RigidMeshDeformer2D::RigidMeshDeformer2D()
{
	InvalidateSetup();
}

void RigidMeshDeformer2D::SetDeformedHandle( unsigned int nHandle, const Vec2f & vHandle )
{
	Constraint c(nHandle, vHandle);
	UpdateConstraint(c);
}

void RigidMeshDeformer2D::RemoveHandle( unsigned int nHandle )
{	
	Constraint c(nHandle, Vec2f(0,0));
	m_vConstraints.erase(c);
	m_vDeformedVerts[nHandle].vPosition = m_vInitialVerts[nHandle].vPosition;
	InvalidateSetup();
}

float SquaredLength(Vec2f vec)
{
	return vec.dot(vec);
}

float CalLength(Vec2f vec)
{
	return sqrt(vec.dot(vec));
}

void DebugBreak()
{
	abort();
}

void RigidMeshDeformer2D::InitializeFromMesh( TriangleMesh * pMesh )
{
	m_vConstraints.clear();
	m_vInitialVerts.resize(0);
	m_vDeformedVerts.resize(0);
	m_vTriangles.resize(0);

	// copy vertices
	unsigned int nVerts = pMesh->GetNumVertices();
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Vec3f vVertex;
		pMesh->GetVertex(i, vVertex);

		Vertex v;
		v.vPosition = Vec2f( vVertex[0], vVertex[1] );
		m_vInitialVerts.push_back( v );
		m_vDeformedVerts.push_back( v );
	}

	// copy triangles
	unsigned int nTris = pMesh->GetNumTriangles();
	for ( unsigned int i = 0; i < nTris; ++i ) {
		Triangle t;
		pMesh->GetTriangle(i, t.nVerts );
		m_vTriangles.push_back(t);
	}


	// set up triangle-local coordinate systems
	for ( unsigned int i = 0; i < nTris; ++i ) {
		Triangle & t = m_vTriangles[i];
		
		for ( int j = 0; j < 3; ++j ) {
			unsigned int n0 = j;
			unsigned int n1 = (j+1)%3;
			unsigned int n2 = (j+2)%3;

			Vec2f v0 = GetInitialVert( t.nVerts[n0] );
			Vec2f v1 = GetInitialVert( t.nVerts[n1] );
			Vec2f v2 = GetInitialVert( t.nVerts[n2] );

			// find coordinate system
			Vec2f v01( v1 - v0 );
			Vec2f v01N( v01 );
			//v01N.Normalize();
			v01N = normalize(v01N);
			Vec2f v01Rot90( v01[1], -v01[0] );
			Vec2f v01Rot90N( v01Rot90 );
			//v01Rot90N.Normalize();
			v01Rot90N = normalize(v01Rot90N);
			// express v2 in coordinate system
			Vec2f vLocal(v2 - v0);
			float fX = vLocal.dot(v01) /  SquaredLength(v01);
			float fY = vLocal.dot(v01Rot90) / SquaredLength(v01Rot90);

			// sanity check
			Vec2f v2test(v0 + fX * v01 + fY * v01Rot90);
			float fLength = CalLength((v2test - v2));
			if ( fLength > 0.001f )
			{
				cout << "vec rotate error" << endl;
				abort();
			}

			t.vTriCoords[j] = Vec2f(fX,fY);
		}
	}
}


void RigidMeshDeformer2D::UpdateDeformedMesh( TriangleMesh * pMesh, bool bRigid )
{
	ValidateDeformedMesh(bRigid);

	std::vector<Vertex> & vVerts = (m_vConstraints.size() > 1) ? m_vDeformedVerts : m_vInitialVerts;

	unsigned int nVerts = pMesh->GetNumVertices();
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Vec2f vNewPos( vVerts[i].vPosition );
		pMesh->SetVertex( i, Vec3f( vNewPos[0], vNewPos[1], 0.0f ) );
	}
}



void RigidMeshDeformer2D::UpdateConstraint( Constraint & cons )
{
	std::set<Constraint>::iterator found( m_vConstraints.find(cons) );
	if ( found != m_vConstraints.end() ) {
#if 0
            (*found).vConstrainedPos = cons.vConstrainedPos;
#else
            const Constraint & foo = *found;
            Constraint & bar = const_cast<Constraint&>(foo);
            bar.vConstrainedPos = cons.vConstrainedPos;
#endif
		m_vDeformedVerts[cons.nVertex].vPosition = cons.vConstrainedPos;

	} else {
		m_vConstraints.insert( cons );
		m_vDeformedVerts[cons.nVertex].vPosition = cons.vConstrainedPos;
		InvalidateSetup();
	} 

}




void ExtractSubMatrix( Mat & mFrom, int nRowOffset, int nColOffset, Mat & mTo )
{
	int nRows = mTo.rows;
	int nCols = mTo.cols;

	for ( int i = 0; i < nRows; ++i ) {
		for ( int j = 0; j < nCols; ++j ) {
			mTo.at<float>(i,j) = mFrom.at<float>( i + nRowOffset, j + nColOffset);
		}
	}
}


void RigidMeshDeformer2D::ValidateSetup()
{
	if  ( m_bSetupValid || m_vConstraints.size() < 2)
		return;

	printf("Computing matrices for mesh with %d verts....this might take a while...\n", m_vInitialVerts.size() );

	PrecomputeOrientationMatrix();

	// ok, now scale triangles
	size_tt nTris = m_vTriangles.size();
	for ( unsigned int i = 0; i < nTris; ++i )
		PrecomputeScalingMatrices(i);

	PrecomputeFittingMatrices();

	printf("Done!\n" );


	m_bSetupValid = true;
}




void RigidMeshDeformer2D::PrecomputeFittingMatrices()
{
	// put constraints into vector (will be useful)
	std::vector<Constraint> vConstraintsVec;
	std::set<Constraint>::iterator cur(m_vConstraints.begin()), end(m_vConstraints.end());
	while ( cur != end )
		vConstraintsVec.push_back( *cur++ );

	// resize matrix and clear to zero
	unsigned int nVerts = (unsigned int)m_vDeformedVerts.size();
	size_tt nConstraints = vConstraintsVec.size();
	unsigned int nFreeVerts = nVerts - nConstraints;

	// figure out vertex ordering. first do free vertices, then constraints
	unsigned int nRow = 0;
	m_vVertexMap.resize(nVerts);
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Constraint c(i, Vec2f(0,0));
		if ( m_vConstraints.find(c) != m_vConstraints.end() )
			continue;
		m_vVertexMap[i] = nRow++;
	}
	if ( nRow != nFreeVerts )	DebugBreak();
	for ( unsigned int i = 0 ; i < nConstraints; ++i )
		m_vVertexMap[vConstraintsVec[i].nVertex ] = nRow++;
	if ( nRow != nVerts )	DebugBreak();		// bad!
	

	// test vector...
	//Wml::GVectord gUTestX( nVerts ), gUTestY(nVerts);
	vector<float> gUTestX(nVerts), gUTestY(nVerts);
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Constraint c(i,Vec2f(0,0));
		if ( m_vConstraints.find(c) != m_vConstraints.end() )
			continue;
		int nRow = m_vVertexMap[i];
		gUTestX[nRow] = m_vInitialVerts[i].vPosition[0];
		gUTestY[nRow] = m_vInitialVerts[i].vPosition[1];
	}
	for ( unsigned int i = 0; i < nConstraints; ++i ) {
		int nRow = m_vVertexMap[ vConstraintsVec[i].nVertex ];
		gUTestX[nRow] = vConstraintsVec[i].vConstrainedPos[0];
		gUTestY[nRow] = vConstraintsVec[i].vConstrainedPos[1];
	}


	// make Hy and Hx matrices
	Mat mHX( nVerts, nVerts, CV_32FC1);
	Mat mHY( nVerts, nVerts, CV_32FC1);
	for ( unsigned int i = 0; i < nVerts; ++i )
		for ( unsigned int j = 0; j < nVerts; ++j )
			mHX.at<float>(i,j) = mHY.at<float>(i,j) = 0.0;

	// ok, now fill matrix
	size_tt nTriangles = m_vTriangles.size();
	for ( unsigned int i = 0; i < nTriangles; ++i ) {
		Triangle & t = m_vTriangles[i];

//		_RMSInfo("Triangle %d: \n", i);
		double fTriSumErr = 0;
		for ( int j = 0; j < 3; ++j ) {
			double fTriErr = 0;

			int nA = m_vVertexMap[ t.nVerts[j] ];
			int nB = m_vVertexMap[ t.nVerts[(j+1)%3] ];

			// X elems
			mHX.at<float>(nA,nA) += 2;
			mHX.at<float>(nA,nB) += -2;
			mHX.at<float>(nB,nA) += -2;
			mHX.at<float>(nB,nB) += 2;

			//  Y elems
			mHY.at<float>(nA,nA) += 2;
			mHY.at<float>(nA,nB) += -2;
			mHY.at<float>(nB,nA) += -2;
			mHY.at<float>(nB,nB) += 2;
		}
	}

	// extract HX00 and  HY00 matrices
	Mat mHX00( (int)nFreeVerts, (int)nFreeVerts, CV_32FC1);
	Mat mHY00( (int)nFreeVerts, (int)nFreeVerts, CV_32FC1);
	ExtractSubMatrix( mHX, 0, 0, mHX00 );
	ExtractSubMatrix( mHY, 0, 0, mHY00 );

	// Extract HX01 and HX10 matrices
	Mat mHX01( (int)nFreeVerts, (int)nConstraints, CV_32FC1);
	Mat mHX10( (int)nConstraints, (int)nFreeVerts, CV_32FC1);
	ExtractSubMatrix( mHX, 0, nFreeVerts, mHX01 );
	ExtractSubMatrix( mHX, nFreeVerts, 0, mHX10 );

	// Extract HY01 and HY10 matrices
	Mat mHY01( (int)nFreeVerts, (int)nConstraints, CV_32FC1);
	Mat mHY10( (int)nConstraints, (int)nFreeVerts, CV_32FC1);
	ExtractSubMatrix( mHY, 0, nFreeVerts, mHY01 );
	ExtractSubMatrix( mHY, nFreeVerts, 0, mHY10 );

	// now compute HXPrime = HX00 + Transpose(HX00) (and HYPrime)
	m_mHXPrime = ( mHX00 + mHX00.t() );	//不使用LU分解
	m_mHYPrime = ( mHY00 + mHY00.t() );
	//m_mHXPrime = mHX00;
	//m_mHYPrime = mHY00;

	// and then D = HX01 + Transpose(HX10)
	m_mDX = mHX01 + mHX10.t();
	m_mDY = mHY01 + mHY10.t();
	//m_mDX = mHX01;
	//m_mDY = mHY01;

	//// pre-compute LU decompositions
	//bool bResult = LinearSystem::LUDecompose( m_mHXPrime, m_mLUDecompX );
	//if (!bResult)
	//	DebugBreak();
	//bResult = LinearSystem::LUDecompose( m_mHYPrime, m_mLUDecompY );
	//if (!bResult)
	//	DebugBreak();

}

//template<typename T> vector<T> MatMulVec(Mat mat, vector<T> vec)
//{
//	assert(mat.cols == vec.size());
//	vector<T> retVec(mat.rows);
//	T sum;
//
//	for (int i = 0; i < mat.rows; i++)
//	{
//		sum = 0;
//		for (int j = 0; j < mat.cols; j++)
//		{
//			sum += mat<float>(i,j) * vec[j];
//		}
//		retVec[i] = sum;
//	}
//
//	return retVec;
//}

template<typename T> vector<T> operator*(Mat mat, vector<T> vec)
{
	assert(mat.cols == vec.size());
	vector<T> retVec(mat.rows);
	T sum;

	for (int i = 0; i < mat.rows; i++)
	{
		sum = 0;
		for (int j = 0; j < mat.cols; j++)
		{
			sum += mat.at<float>(i,j) * vec[j];
		}
		retVec[i] = sum;
	}

	return retVec;
}

template<typename T> void operator*=(vector<T> vec, T val)
{
	int size = vec.size();
	for (int i = 0; i < size; i++)
	{
		{
			vec[i] = vec[i] * val;
		}
	}

}

template<typename T> void operator+=(vector<T> vec, vector<T> vec_add)
{
	int size = vec.size();
	for (int i = 0; i < size; i++)
	{
		{
			vec[i] = vec[i] + vec_add[i];
		}
	}

}

template<typename T> T operator*(vector<T> vec1, vector<T> vec2)
{
	assert(vec1.size() == vec2.size());
	T dot = 0;
	int size = vec1.size();
	for (int i = 0; i < size; i++)
	{
		{
			dot += vec1[i] * vec2[i];
		}
	}
	return dot;
}

void RigidMeshDeformer2D::ApplyFittingStep()
{
	// put constraints into vector (will be useful)
	std::vector<Constraint> vConstraintsVec;
	std::set<Constraint>::iterator cur(m_vConstraints.begin()), end(m_vConstraints.end());
	while ( cur != end )
		vConstraintsVec.push_back( *cur++ );

	unsigned int nVerts = (unsigned int)m_vDeformedVerts.size();
	size_tt nConstraints = vConstraintsVec.size();
	unsigned int nFreeVerts = nVerts - nConstraints;

	// make vector of deformed vertex weights
	vector<float> vFX( nVerts );
	vector<float> vFY( nVerts );
	for ( int i = 0; i < (int)nVerts; ++i )
		vFX[i] = vFY[i] = 0.0;

	size_tt nTriangles = m_vTriangles.size();
	for ( unsigned int i = 0; i < nTriangles; ++i ) {
		Triangle & t = m_vTriangles[i];

		for ( int j = 0; j < 3; ++j ) {

			int nA = m_vVertexMap[ t.nVerts[j] ];
			int nB = m_vVertexMap[ t.nVerts[(j+1)%3] ];

			Vec2f vDeformedA( t.vScaled[j] );
			Vec2f vDeformedB( t.vScaled[(j+1)%3] );

			// X elems
			vFX[nA] += -2 * vDeformedA[0] + 2 * vDeformedB[0];
			vFX[nB] += 2 * vDeformedA[0] - 2 * vDeformedB[0];

			//  Y elems
			vFY[nA] += -2 * vDeformedA[1] + 2 * vDeformedB[1];
			vFY[nB] += 2 * vDeformedA[1] - 2 * vDeformedB[1];
		}
	}

	// make F0 vectors
	vector<float> vF0X( nFreeVerts ), vF0Y( nFreeVerts );
	for ( int i = 0; i < (int)nFreeVerts; ++i ) {
		vF0X[i] = vFX[i];
		vF0Y[i] = vFY[i];
	}

	// make Q vectors (vectors of constraints)
	vector<float> vQX( (int)nConstraints ),  vQY( (int)nConstraints );
	for ( int i = 0; i < (int)nConstraints; ++i ) {
		vQX[i] = vConstraintsVec[i].vConstrainedPos[0];
		vQY[i] = vConstraintsVec[i].vConstrainedPos[1];
	}

	// ok, compute RHS for X and solve
	vector<float> vRHSX( m_mDX * vQX );
	vRHSX += vF0X;
	vRHSX *= (float)-1;
	vector<float> vSolutionX( (int)nFreeVerts );

	////Wml::LinearSystemd::Solve( m_mHXPrime, vRHSX, vSolutionX );
	//bool bResult = LinearSystem::LUBackSub( m_mLUDecompX, vRHSX, vSolutionX );
	//if (!bResult)
	//	DebugBreak();
	vSolutionX = m_mHXPrime.inv() * vRHSX;

	// now for Y
	vector<float> vRHSY( m_mDY * vQY );
	vRHSY += vF0Y;
	vRHSY *= (float)-1;
	vector<float> vSolutionY( (int)nFreeVerts );
//	Wml::LinearSystemd::Solve( m_mHYPrime, vRHSY, vSolutionY );
	//bResult = LinearSystem::LUBackSub( m_mLUDecompY, vRHSY, vSolutionY );
	//if (!bResult)
	//	DebugBreak();
	vSolutionY = m_mHYPrime.inv() * vRHSY;

	// done!
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Constraint c(i,Vec2f(0,0));
		if ( m_vConstraints.find(c) != m_vConstraints.end() )
			continue;
		int nRow = m_vVertexMap[i];
		m_vDeformedVerts[i].vPosition[0] = (float)vSolutionX[nRow];
		m_vDeformedVerts[i].vPosition[1] = (float)vSolutionY[nRow];
	}

}












void RigidMeshDeformer2D::PrecomputeOrientationMatrix()
{
	// put constraints into vector (will be useful)
	std::vector<Constraint> vConstraintsVec;
	std::set<Constraint>::iterator cur(m_vConstraints.begin()), end(m_vConstraints.end());
	while ( cur != end )
		vConstraintsVec.push_back( *cur++ );

	// resize matrix and clear to zero
	unsigned int nVerts = (unsigned int)m_vDeformedVerts.size();
	//m_mFirstMatrix.SetSize( 2*nVerts, 2*nVerts);
	m_mFirstMatrix = Mat(2 * nVerts, 2 * nVerts, CV_32FC1);
	for ( unsigned int i = 0; i < 2*nVerts; ++i )
		for ( unsigned int j = 0; j < 2*nVerts; ++j )
			m_mFirstMatrix.at<float>(i,j) = 0.0;

	size_tt nConstraints = vConstraintsVec.size();
	unsigned int nFreeVerts = nVerts - nConstraints;

	// figure out vertex ordering. first do free vertices, then constraints
	unsigned int nRow = 0;
	m_vVertexMap.resize(nVerts);
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Constraint c(i, Vec2f(0,0));
		if ( m_vConstraints.find(c) != m_vConstraints.end() )
			continue;
		m_vVertexMap[i] = nRow++;
	}
	if ( nRow != nFreeVerts )	DebugBreak();
	for ( unsigned int i = 0 ; i < nConstraints; ++i )
		m_vVertexMap[vConstraintsVec[i].nVertex ] = nRow++;
	if ( nRow != nVerts )	DebugBreak();		// bad!


	// test vector...
	vector<float> gUTest( nVerts * 2 );
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Constraint c(i,Vec2f(0,0));
		if ( m_vConstraints.find(c) != m_vConstraints.end() )
			continue;
		int nRow = m_vVertexMap[i];
		gUTest[2*nRow] = m_vInitialVerts[i].vPosition[0];
		gUTest[2*nRow+1] = m_vInitialVerts[i].vPosition[1];
	}
	for ( unsigned int i = 0; i < nConstraints; ++i ) {
		int nRow = m_vVertexMap[ vConstraintsVec[i].nVertex ];
		gUTest[2*nRow] = vConstraintsVec[i].vConstrainedPos[0];
		gUTest[2*nRow+1] = vConstraintsVec[i].vConstrainedPos[1];
	}


	// ok, now fill matrix (?)
	size_tt nTriangles = m_vTriangles.size();
	for ( unsigned int i = 0; i < nTriangles; ++i ) {
		Triangle & t = m_vTriangles[i];
		
//		_RMSInfo("Triangle %d: \n", i);
		double fTriSumErr = 0;
		for ( int j = 0; j < 3; ++j ) {
			double fTriErr = 0;

			int n0x = 2 * m_vVertexMap[ t.nVerts[j] ];
			int n0y = n0x + 1;
			int n1x = 2 * m_vVertexMap[ t.nVerts[(j+1)%3] ];
			int n1y = n1x + 1;
			int n2x = 2 * m_vVertexMap[ t.nVerts[(j+2)%3] ];
			int n2y = n2x + 1;
			float x = t.vTriCoords[j][0];
			float y = t.vTriCoords[j][1];


			// DEBUG
			Vec2f v0( (float)gUTest[n0x], (float)gUTest[n0y] );
			Vec2f v1( (float)gUTest[n1x], (float)gUTest[n1y] );
			Vec2f v2( (float)gUTest[n2x], (float)gUTest[n2y] );
			Vec2f v01( v1 - v0 );
			Vec2f v01Perp( v01[1], -v01[0] );
			Vec2f vTest( v0 + x * v01 + y * v01Perp );
			float fDist = (vTest - v2).dot(vTest - v2);
			//if ( fDist > 0.0001f )
			//	DebugBreak();
			//DEBUG

			//double dTest = 
			//(1 - 2*x + (x*x) + (y*y))*pow(gUTest[n0x],2) + (1 - 2*x + (x*x) + (y*y))*pow(gUTest[n0y],2) + 
			//	((x*x) + (y*y))*pow(gUTest[n1x],2) + ((x*x) + (y*y))*pow(gUTest[n1y],2) + 
			//	pow(gUTest[n2x],2) + pow(gUTest[n2y],2) + gUTest[n1y]*(-2*y*gUTest[n2x] - 2*x*gUTest[n2y]) + 
			//	gUTest[n0y]*(-2*y*gUTest[n1x] + (2*x - 2*(x*x) - 2*(y*y))*gUTest[n1y] + 2*y*gUTest[n2x] + 
			//	(-2 + 2*x)*gUTest[n2y]) + gUTest[n0x]*
			//	((2*x - 2*(x*x) - 2*(y*y))*gUTest[n1x] + 2*y*gUTest[n1y] + (-2 + 2*x)*gUTest[n2x] - 
			//	2*y*gUTest[n2y]) + gUTest[n1x]*(-2*x*gUTest[n2x] + 2*y*gUTest[n2y]);
			//_RMSInfo("TEST IS %f %f\n", dTest, fDist);
			
			// n0x,n?? elems
			m_mFirstMatrix.at<float>(n0x,n0x) += 1 - 2*x + x*x + y*y;
			m_mFirstMatrix.at<float>(n0x,n1x) += 2*x - 2*x*x - 2*y*y;		//m_mFirstMatrix[n1x][n0x] += 2*x - 2*x*x - 2*y*y;
			m_mFirstMatrix.at<float>(n0x,n1y) += 2*y;						//m_mFirstMatrix[n1y][n0x] += 2*y;
			m_mFirstMatrix.at<float>(n0x,n2x) += -2 + 2*x;					//m_mFirstMatrix[n2x][n0x] += -2 + 2*x;
			m_mFirstMatrix.at<float>(n0x,n2y) += -2 * y;						//m_mFirstMatrix[n2y][n0x] += -2 * y;

			fTriErr += (1 - 2*x + x*x + y*y)  * gUTest[n0x] * gUTest[n0x];
			fTriErr += (2*x - 2*x*x - 2*y*y)  * gUTest[n0x] * gUTest[n1x];
			fTriErr += (2*y)                  * gUTest[n0x] * gUTest[n1y];
			fTriErr += (-2 + 2*x )            * gUTest[n0x] * gUTest[n2x];
			fTriErr += (-2 * y)               * gUTest[n0x] * gUTest[n2y];

			// n0y,n?? elems
			m_mFirstMatrix.at<float>(n0y,n0y) += 1 - 2*x + x*x + y*y;
			m_mFirstMatrix.at<float>(n0y,n1x) += -2*y;						//m_mFirstMatrix[n1x,n0y] += -2*y;
			m_mFirstMatrix.at<float>(n0y,n1y) += 2*x - 2*x*x - 2*y*y;		//m_mFirstMatrix[n1y,n0y] += 2*x - 2*x*x - 2*y*y;
			m_mFirstMatrix.at<float>(n0y,n2x) += 2*y;						//m_mFirstMatrix[n2x,n0y] += 2*y;
			m_mFirstMatrix.at<float>(n0y,n2y) += -2 + 2*x;					//m_mFirstMatrix[n2y,n0y] += -2 + 2*x;

			fTriErr += (1 - 2*x + x*x + y*y)   * gUTest[n0y] * gUTest[n0y];
			fTriErr += (-2*y)                  * gUTest[n0y] * gUTest[n1x];
			fTriErr += (2*x - 2*x*x - 2*y*y)   * gUTest[n0y] * gUTest[n1y];
			fTriErr += (2*y)                   * gUTest[n0y] * gUTest[n2x];
			fTriErr += (-2 + 2*x)              * gUTest[n0y] * gUTest[n2y];

			// n1x,n?? elems
			m_mFirstMatrix.at<float>(n1x,n1x) += x*x + y*y;
			m_mFirstMatrix.at<float>(n1x,n2x) += -2*x;						//m_mFirstMatrix[n2x,n1x] += -2*x;
			m_mFirstMatrix.at<float>(n1x,n2y) += 2*y;						//m_mFirstMatrix[n2y,n1x] += 2*y;

			fTriErr += (x*x + y*y)            * gUTest[n1x] * gUTest[n1x];
			fTriErr += (-2*x)                 * gUTest[n1x] * gUTest[n2x];
			fTriErr += (2*y)                  * gUTest[n1x] * gUTest[n2y];

			//n1y,n?? elems
			m_mFirstMatrix.at<float>(n1y,n1y) += x*x + y*y;
			m_mFirstMatrix.at<float>(n1y,n2x) += -2*y;						//m_mFirstMatrix[n2x,n1y] += -2*y;
			m_mFirstMatrix.at<float>(n1y,n2y) += -2*x;						//m_mFirstMatrix[n2y,n1y] += -2*x;


			fTriErr += (x*x + y*y)            * gUTest[n1y] * gUTest[n1y];
			fTriErr += (-2*y)                 * gUTest[n1y] * gUTest[n2x];
			fTriErr += (-2*x)                 * gUTest[n1y] * gUTest[n2y];

			// final 2 elems
			m_mFirstMatrix.at<float>(n2x,n2x) += 1;
			m_mFirstMatrix.at<float>(n2y,n2y) += 1;

			fTriErr += gUTest[n2x] * gUTest[n2x]  +  gUTest[n2y] * gUTest[n2y] ;

			//_RMSInfo("  Error for vert %d (%d) - %f\n", j, t.nVerts[j], fTriErr);
			fTriSumErr += fTriErr;
		}
		//_RMSInfo("  Total Error: %f\n", fTriSumErr);
	}


	// test...
	vector<float> gUTemp = m_mFirstMatrix * gUTest;
	double fSum = gUTemp * gUTest;
	printf("    (test) Residual is %f\n", fSum);

/*
	// just try printing out matrix...
	for ( unsigned int i = 0; i < 2*nFreeVerts; ++i ) {
		for ( unsigned int j = 0 ; j < 2*nFreeVerts; ++j )
			_RMSInfo("%5.2f ", m_mFirstMatrix(i,j));
		_RMSInfo("| ");
		for ( unsigned int j = 0; j < 2*nConstraints; ++j )
			_RMSInfo("%5.2f ", m_mFirstMatrix(i, 2*nFreeVerts+j));
		_RMSInfo("\n");
	}
	_RMSInfo("-------\n");
	for ( unsigned int i = 0; i < 2*nConstraints; ++i ) {
		for ( unsigned int j = 0 ; j < 2*nFreeVerts; ++j )
			_RMSInfo("%5.2f ", m_mFirstMatrix(i+2*nFreeVerts,j));
		_RMSInfo("| ");
		for ( unsigned int j = 0; j < 2*nConstraints; ++j )
			_RMSInfo("%5.2f ", m_mFirstMatrix(i+2*nFreeVerts, 2*nFreeVerts+j));
		_RMSInfo("\n");
	}
	_RMSInfo("\n\n");
*/

	// extract G00 matrix
	Mat mG00( 2*nFreeVerts, 2*nFreeVerts, CV_32FC1);
	ExtractSubMatrix( m_mFirstMatrix, 0, 0, mG00 );

	// extract G01 and G10 matrices
	Mat mG01( 2 * (int)nFreeVerts, 2 * (int)nConstraints , CV_32FC1);
	ExtractSubMatrix( m_mFirstMatrix, 0, 2*nFreeVerts, mG01 );
	Mat mG10( 2 * (int)nConstraints, 2 * (int)nFreeVerts , CV_32FC1);
	ExtractSubMatrix( m_mFirstMatrix, 2*nFreeVerts, 0, mG10 );

	// ok, now compute GPrime = G00 + Transpose(G00) and B = G01 + Transpose(G10)
	Mat mGPrime = mG00 + mG00.t();
	Mat mB = mG01 + mG10.t();

	// ok, now invert GPrime
	Mat mGPrimeInverse( mGPrime.rows, mGPrime.cols, CV_32FC1);
	mGPrimeInverse = mGPrime.inv();
	//bool bInverted = LinearSystem::Inverse( mGPrime, mGPrimeInverse );
	//if (!bInverted)
	//	DebugBreak();

	// now compute -GPrimeInverse * B
	Mat mFinal = mGPrimeInverse * mB;
	mFinal *= -1;

	m_mFirstMatrix = mFinal;		// [RMS: not efficient!]
}

void RigidMeshDeformer2D::PrecomputeScalingMatrices( unsigned int nTriangle )
{
	// ok now fill matrix
	Triangle & t = m_vTriangles[nTriangle];

	// create matrices and clear to zero
	t.mF = Mat(4,4, CV_32FC1);
	t.mC = Mat(4,6, CV_32FC1);

	// precompute coeffs
	double x01 = t.vTriCoords[0][0];
	double y01 = t.vTriCoords[0][1];
	double x12 = t.vTriCoords[1][0];
	double y12 = t.vTriCoords[1][1];
	double x20 = t.vTriCoords[2][0];
	double y20 = t.vTriCoords[2][1];

	double k1 = x12*y01 + (-1 + x01)*y12;
	double k2 = -x12 + x01*x12 - y01*y12;
	double k3 = -y01 + x20*y01 + x01*y20;
	double k4 = -y01 + x01*y01 + x01*y20;
	double k5 = -x01 + x01*x20 - y01*y20 ;

	double a = -1 + x01;
	double a1 = pow(-1 + x01,2) + pow(y01,2);
	double a2 = pow(x01,2) + pow(y01,2);
	double b =  -1 + x20;
	double b1 = pow(-1 + x20,2) + pow(y20,2);
	double c2 = pow(x12,2) + pow(y12,2);

	double r1 = 1 + 2*a*x12 + a1*pow(x12,2) - 2*y01*y12 + a1*pow(y12,2);
	double r2 = -(b*x01) - b1*pow(x01,2) + y01*(-(b1*y01) + y20);
	double r3 = -(a*x12) - a1*pow(x12,2) + y12*(y01 - a1*y12);
	double r5 = a*x01 + pow(y01,2);
	double r6 = -(b*y01) - x01*y20;
	double r7 = 1 + 2*b*x01 + b1*pow(x01,2) + b1*pow(y01,2) - 2*y01*y20;

	//  set up F matrix

	// row 0 mF
	t.mF.at<float>(0,0) = 2*a1 + 2*a1*c2 + 2*r7;
	t.mF.at<float>(0,1) = 0;
	t.mF.at<float>(0,2) = 2*r2 + 2*r3 - 2*r5;
	t.mF.at<float>(0,3) = 2*k1 + 2*r6 + 2*y01;

	// row 1
	t.mF.at<float>(1,0) = 0;
	t.mF.at<float>(1,1) = 2*a1 + 2*a1*c2 + 2*r7;
	t.mF.at<float>(1,2) = -2*k1 + 2*k3 - 2*y01;
	t.mF.at<float>(1,3) = 2*r2 + 2*r3 - 2*r5;

	// row 2
	t.mF.at<float>(2,0) = 2*r2 + 2*r3 - 2*r5;
	t.mF.at<float>(2,1) = -2*k1 + 2*k3 - 2*y01;
	t.mF.at<float>(2,2) = 2*a2 + 2*a2*b1 + 2*r1;
	t.mF.at<float>(2,3) = 0;

	//row 3
	t.mF.at<float>(3,0) = 2*k1 - 2*k3 + 2*y01;
	t.mF.at<float>(3,1) = 2*r2 + 2*r3 - 2*r5;
	t.mF.at<float>(3,2) = 0;
	t.mF.at<float>(3,3) = 2*a2 + 2*a2*b1 + 2*r1;

	// ok, now invert F
	Mat mFInverse(4,4, CV_32FC1);
	mFInverse = t.mF.inv();
	//bool bResult = LinearSystemd::Inverse(t.mF, mFInverse);
	//mFInverse *= -1.0;
	//if (!bResult) {
	//	DebugBreak();
	//}
	t.mF =  mFInverse;

	// set up C matrix

	// row 0 mC
	t.mC.at<float>(0,0) = 2*k2;
	t.mC.at<float>(0,1) = -2*k1;
	t.mC.at<float>(0,2) = 2*(-1-k5);
	t.mC.at<float>(0,3) = 2*k3;
	t.mC.at<float>(0,4) = 2*a;
	t.mC.at<float>(0,5) = -2*y01;

	// row 1 mC
	t.mC.at<float>(1,0) = 2*k1;
	t.mC.at<float>(1,1) = 2*k2;
	t.mC.at<float>(1,2) = -2*k3;
	t.mC.at<float>(1,3) = 2*(-1-k5);
	t.mC.at<float>(1,4) = 2*y01;
	t.mC.at<float>(1,5) = 2*a;

	// row 2 mC
	t.mC.at<float>(2,0) = 2*(-1-k2);
	t.mC.at<float>(2,1) = 2*k1;
	t.mC.at<float>(2,2) = 2*k5;
	t.mC.at<float>(2,3) = 2*r6;
	t.mC.at<float>(2,4) = -2*x01;
	t.mC.at<float>(2,5) = 2*y01;

	// row 3 mC
	t.mC.at<float>(3,0) = 2*k1;
	t.mC.at<float>(3,1) = 2*(-1-k2);
	t.mC.at<float>(3,2) = -2*k3;
	t.mC.at<float>(3,3) = 2*k5;
	t.mC.at<float>(3,4) = -2*y01;
	t.mC.at<float>(3,5) = -2*x01;


}

void Scale(Vec2f & vTriV0,
	Vec2f & vTriV1,
	Vec2f & vTriV2,
	float fScale)
{
	// find center of mass
	Vec2f vCentroid(vTriV0 + vTriV1 + vTriV2);
	vCentroid *= (float)1.0 / (float)3.0;

	// convert to vectors, scale and restore
	vTriV0 -= vCentroid;	vTriV0 *= fScale;	vTriV0 += vCentroid;
	vTriV1 -= vCentroid;	vTriV1 *= fScale;	vTriV1 += vCentroid;
	vTriV2 -= vCentroid;	vTriV2 *= fScale;	vTriV2 += vCentroid;
}

void RigidMeshDeformer2D::UpdateScaledTriangle( unsigned int nTriangle )
{
	// ok now fill matrix
	Triangle & t = m_vTriangles[nTriangle];

	// multiply mC by deformed vertex position
	const Vec2f & vDeformedV0 = m_vDeformedVerts[ t.nVerts[0] ].vPosition;
	const Vec2f & vDeformedV1 = m_vDeformedVerts[ t.nVerts[1] ].vPosition;
	const Vec2f & vDeformedV2 = m_vDeformedVerts[ t.nVerts[2] ].vPosition;
	//double tmp[6] = { vDeformedV0[0], vDeformedV0[1], 
	//				  vDeformedV1[0], vDeformedV1[1], 
	//				  vDeformedV2[0], vDeformedV2[1] };
	vector<double> vDeformed = { vDeformedV0[0], vDeformedV0[1],
		vDeformedV1[0], vDeformedV1[1],
		vDeformedV2[0], vDeformedV2[1] };
	vector<double> mCVec = t.mC * vDeformed;

	// compute -MFInv * mC
	vector<double> vSolution = t.mF * mCVec;

	// ok, grab deformed v0 and v1 from solution vector
	Vec2f vFitted0( (float)vSolution[0], (float)vSolution[1] );
	Vec2f vFitted1( (float)vSolution[2], (float)vSolution[3] );

	// figure out fitted2
	float x01 = t.vTriCoords[0][0];
	float y01 = t.vTriCoords[0][1];
	Vec2f vFitted01( vFitted1 - vFitted0 );
	Vec2f vFitted01Perp( vFitted01[1], -vFitted01[0] );
	Vec2f vFitted2( vFitted0 + (float)x01 * vFitted01 + (float)y01 * vFitted01Perp );

	// ok now determine scale
	Vec2f & vOrigV0 = m_vInitialVerts[ t.nVerts[0] ].vPosition;
	Vec2f & vOrigV1 = m_vInitialVerts[ t.nVerts[1] ].vPosition;
	float fScale = CalLength( vOrigV1 - vOrigV0 ) / CalLength(vFitted01);

	// now scale triangle
	Scale( vFitted0, vFitted1, vFitted2, fScale );

	t.vScaled[0] = vFitted0;
	t.vScaled[1] = vFitted1;
	t.vScaled[2] = vFitted2;
}


void RigidMeshDeformer2D::ValidateDeformedMesh( bool bRigid )
{
	size_tt nConstraints = m_vConstraints.size();
	if ( nConstraints < 2 )
		return;

	ValidateSetup();

	// make q vector of constraints
	vector<float> vQ( 2 * (int)nConstraints );
	int k = 0;
	std::set<Constraint>::iterator cur(m_vConstraints.begin()), end(m_vConstraints.end());
	while ( cur != end ) {
#if 0
		Constraint & c = (*cur++);
#else
                Constraint & c = const_cast<Constraint &>((*cur++));
#endif
		vQ[ 2*k ] = c.vConstrainedPos[0];
		vQ[ 2*k + 1] = c.vConstrainedPos[1];
		++k;
	}

	vector<float> vU = m_mFirstMatrix * vQ;
	size_tt nVerts = m_vDeformedVerts.size();
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Constraint c(i, Vec2f(0,0));
		if ( m_vConstraints.find(c) != m_vConstraints.end() )
			continue;		
		int nRow = m_vVertexMap[i];

		double fX = vU[ 2*nRow ];
		double fY = vU[ 2*nRow + 1];
		m_vDeformedVerts[i].vPosition = Vec2f( (float)fX, (float)fY );
	}

	if ( bRigid ) {
		// ok, now scale triangles
		size_tt nTris = m_vTriangles.size();
		for ( unsigned int i = 0; i < nTris; ++i )
			UpdateScaledTriangle(i);


		ApplyFittingStep();
	}
}
