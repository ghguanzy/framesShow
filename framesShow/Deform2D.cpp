// Deform2D.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"   // only used for OpenFileDialog below
#include "glut.h"
#include <afxdlgs.h>
#include "Deform2D.h"

#include "RigidMeshDeformer2D.h"
using namespace std;


TriangleMesh m_mesh;
float m_bounds[6];
Vec2f m_vTranslate;
float m_fScale;
TriangleMesh m_deformedMesh;

RigidMeshDeformer2D m_deformer;
bool m_bConstraintsValid;

std::set<unsigned int> m_vSelected;
unsigned int m_nSelected;

int m_nViewport[4];


// predecls
void InitializeDeformedMesh();
void UpdateDeformedMesh();
void InvalidateConstraints();
void ValidateConstraints();
unsigned int FindHitVertex( float nX, float nY );


void MakeSquareMesh()
{
	m_mesh.Clear();

	const unsigned int nRowLen = 5;

	float fYStep = 2.0f / (float)(nRowLen-1);
	float fXStep = 2.0f / (float)(nRowLen-1);

	for ( unsigned int yi = 0; yi < nRowLen; ++yi ) {
		float fY = -1.0f + (float)yi * fYStep;
		for ( unsigned int xi = 0; xi < nRowLen; ++xi ) {
			float fX = -1.0f + (float)xi * fXStep;

			Vec3f vVert(fX,fY,0);
			m_mesh.AppendVertexData(&vVert, NULL, false);
		}
	}

	for ( unsigned int yi = 0; yi < nRowLen-1; ++yi ) {
		unsigned int nRow1 = yi * nRowLen;
		unsigned int nRow2 = (yi+1) * nRowLen;

		for ( unsigned int xi = 0; xi < nRowLen-1; ++xi ) {

			unsigned int nTri1[3] = { nRow1 + xi, nRow2 + xi + 1, nRow1 + xi + 1 };
			unsigned int nTri2[3] = { nRow1 + xi, nRow2 + xi, nRow2 + xi + 1 };

			m_mesh.AppendTriangleData( nTri1 );
			m_mesh.AppendTriangleData( nTri2 );
		}
	}
			
	InitializeDeformedMesh();
	glutPostRedisplay();
}



void UpdateScale() 
{
	glGetIntegerv(GL_VIEWPORT, m_nViewport);
	float fViewCenterX = (float)m_nViewport[2] / 2;
	float fViewCenterY = (float)m_nViewport[3] / 2;

	m_mesh.GetBoundingBox( m_bounds );
	m_vTranslate[0] = fViewCenterX  -  0.5f * ( m_bounds[0] + m_bounds[1] );
	m_vTranslate[1] = fViewCenterY  -  0.5f * ( m_bounds[2] + m_bounds[3] );

	float fWidth = m_bounds[1] - m_bounds[0];
	float fHeight = m_bounds[3] - m_bounds[2];
	float fSizeObject = std::max(fWidth,fHeight);
	float fSizeView = std::min( m_nViewport[2], m_nViewport[3] );

	m_fScale = 0.5f * (fSizeView / fSizeObject);
}
Vec2f ViewToWorld( const Vec2f & vPoint )
{
	return (vPoint - m_vTranslate) / m_fScale;
}
Vec2f WorldToView( const Vec2f & vPoint )
{
	return ( vPoint * m_fScale ) + m_vTranslate;
}




void InitializeDeformedMesh()
{
	m_deformedMesh.Clear();
	
	unsigned int nVerts = m_mesh.GetNumVertices();
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Vec3f vVertex;
		m_mesh.GetVertex(i, vVertex);
		m_deformedMesh.AppendVertexData(&vVertex, NULL, false);
	}

	unsigned int nTris = m_mesh.GetNumTriangles();
	for ( unsigned int i = 0; i < nTris; ++i ) {
		unsigned int nTriangle[3];
		m_mesh.GetTriangle(i,nTriangle);
		m_deformedMesh.AppendTriangleData(nTriangle);
	}

	m_deformer.InitializeFromMesh( &m_mesh );
	InvalidateConstraints();
}



void UpdateDeformedMesh() 
{
	ValidateConstraints();
	m_deformer.UpdateDeformedMesh( &m_deformedMesh, true );
}


// deformer stuff
void InvalidateConstraints() 
{ 
	m_bConstraintsValid = false; 
}

void ValidateConstraints()
{
	if ( m_bConstraintsValid )
		return;

	size_tt nConstraints = m_vSelected.size();
	std::set<unsigned int>::iterator cur(m_vSelected.begin()), end(m_vSelected.end());
	while ( cur != end ) {
		unsigned int nVertex = *cur++;
		Vec3f vVertex;
		m_deformedMesh.GetVertex( nVertex, vVertex);
		m_deformer.SetDeformedHandle( nVertex, Vec2f( vVertex[0], vVertex[1] ) );
	}

	m_deformer.ForceValidation();

	m_bConstraintsValid = true;
}


// selection stuff

unsigned int FindHitVertex( float nX, float nY )
{
	unsigned int nVerts = m_deformedMesh.GetNumVertices();
	for ( unsigned int i = 0; i < nVerts; ++i ) {

		Vec3f vVertex;
		m_deformedMesh.GetVertex(i, vVertex);
		Vec2f vView = WorldToView( Vec2f(vVertex[0], vVertex[1]) );
		float fX = vView[0];
		float fY = vView[1];

		double fDist = sqrt( 
			(double)((nX - fX)*(nX - fX) + (nY-fY)*(nY-fY) ));
		if ( fDist < 5 ) 
			return i;
	}

	return std::numeric_limits<unsigned int>::max();
}






void OnMouseClick(int button, int state, int x, int y) 
{
	if ( button == GLUT_LEFT_BUTTON ) {

		if ( state == GLUT_DOWN )
			m_nSelected = FindHitVertex( (float)x, (float)(m_nViewport[3] - 1 - y) );
		else
			m_nSelected = std::numeric_limits<unsigned int>::max();
		glutPostRedisplay();

	} else if ( button == GLUT_RIGHT_BUTTON && state == GLUT_UP ) {

		unsigned int nHit = FindHitVertex( (float)x, (float)(m_nViewport[3] - 1 - y) );
		if ( nHit != std::numeric_limits<unsigned int>::max() ) {

			if ( m_vSelected.find(nHit) == m_vSelected.end() )
				m_vSelected.insert(nHit);
			else {
				m_vSelected.erase(nHit);
				m_deformer.RemoveHandle(nHit);

				// restore position
				Vec3f vVertex;
				m_mesh.GetVertex(nHit,vVertex);
				m_deformedMesh.SetVertex(nHit, vVertex);
			}

			InvalidateConstraints();
			glutPostRedisplay();
		}

	}

}



void OnMouseMove(int x, int y)
{
	if ( m_nSelected != std::numeric_limits<unsigned int>::max() ) {
		Vec2f vNewPosView( (float)x, (float)(m_nViewport[3] - 1 - y) );
		Vec2f vNewPosWorld = ViewToWorld( vNewPosView );
		Vec3f vNewPos( vNewPosWorld[0], vNewPosWorld[1], 0.0f );
		m_deformedMesh.SetVertex( m_nSelected, vNewPos );
		InvalidateConstraints();

		UpdateDeformedMesh();
		//glutPostRedisplay();
	}
}


void OnKeyboard(unsigned char key, int x, int y)
{
	if ( key == 'f' ) {
		CFileDialog openDlg(TRUE, (LPCTSTR)"obj", (LPCTSTR)"mesh.obj", 0, (LPCTSTR)"OBJ Files (*.obj)|*.obj|");
		if ( openDlg.DoModal() == IDOK ) {
			m_mesh.Clear();
			m_mesh.read( (char *)(openDlg.GetPathName().GetBuffer()) );
			m_vSelected.clear();
			InitializeDeformedMesh();

			glutPostRedisplay();
		}
	}
}





void OnRender()
{
	//UpdateDeformedMesh();

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear( GL_COLOR_BUFFER_BIT );
	glDisable(GL_DEPTH_TEST);

	UpdateScale();
	glLoadIdentity();
	glTranslatef(m_vTranslate[0], m_vTranslate[1], 0.0f);
	glScalef(m_fScale, m_fScale, 1.0f);

	glLineWidth(2.0f);
	glColor3f(0.0f, 0.0f, 0.0f);

	unsigned int nTris = m_deformedMesh.GetNumTriangles();
	for ( unsigned int i = 0; i < nTris; ++i ) {
		Vec3f vVerts[3];
		m_deformedMesh.GetTriangle(i, vVerts);

		glBegin(GL_LINE_LOOP);
		glVertex3fv((GLfloat*)&vVerts[0] );
		glVertex3fv((GLfloat*)&vVerts[1] );
		glVertex3fv((GLfloat*)&vVerts[2] );
		glEnd();
	}

	glLoadIdentity();
	glColor3f(1.0f, 0.0f, 0.0f);

	std::set<unsigned int>::iterator cur(m_vSelected.begin()), end(m_vSelected.end());
	while ( cur != end ) {
		unsigned int nSelected = *cur++;

		Vec3f vSelected;
		m_deformedMesh.GetVertex( nSelected, vSelected );
		Vec2f vView = WorldToView( Vec2f(vSelected[0],vSelected[1]) );

		glBegin(GL_QUADS);
		glVertex2f( vView[0] - 5, vView[1] - 5 );
		glVertex2f( vView[0] + 5, vView[1] - 5 );
		glVertex2f( vView[0] + 5, vView[1] + 5 );
		glVertex2f( vView[0] - 5, vView[1] + 5 );
		glEnd();
	}

	glutSwapBuffers();
}




void OnReshape( int width, int height ) 
{
	glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, width, 0, height);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}



int deform2D(int argc, char ** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowPosition(120,260);
	glutInitWindowSize(600,600);
	glutCreateWindow("Deform2D");

	glutDisplayFunc( OnRender );
	glutReshapeFunc( OnReshape );
	glutMouseFunc( OnMouseClick );
	glutMotionFunc( OnMouseMove );
	glutKeyboardFunc( OnKeyboard );

	MakeSquareMesh();

	glutMainLoop();
	return 0;
}

