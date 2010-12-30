#ifndef VIEWER_H
#define VIEWER_H

//#define EIGEN_DONT_ALIGN
// TODO Fix Eigen Alignment issues: http://eigen.tuxfamily.org/dox/UnalignedArrayAssert.html
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <Tools/GL/glcommon.h>
#include <Tools/Model3DS.h>

using namespace Eigen;

class Viewer: public wxGLCanvas {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Transform<double, 3, Eigen::Affine> camT, prevCamT;
	Matrix3d camRotT;
	Vector3d worldV, prevWorldV;

	Model3DS* model;

	Viewer(wxWindow * parent, wxWindowID id, const wxPoint & pos,
			const wxSize& size, long style = 0, const wxString & name =
					_("GLCanvas"), int * attribList = 0,
			const wxPalette & palette = wxNullPalette) :
		wxGLCanvas(parent, id, pos, size, style, name, attribList, palette) {
	}

	virtual ~Viewer() {
	}


	void OnIdle(wxIdleEvent & evt) {
		//draw();
		evt.RequestMore();
	}

	void InitGL();
	void setClearColor();
	void UpdateCamera();
	int  DrawGLScene();
	void ResetGL();
	void addGrid();

	long x, y, xInit, yInit;
	int w, h;

	bool existsUpdate;
	bool doCollisions;
	bool Move;
	bool pflag;
	int threadCounter;
	bool gridActive;
	double camRadius;
	bool keys[256];
	bool active;
	bool renderLoopActive;
	bool redrawFlag;
	bool showSpheres;
	bool fullscreen;
	///////////////////////
	bool mouseLDown;
	bool mouseRDown;
	bool mouseMDown;
	int redrawCount;
	bool loading;

	Vector3d gridColor;
	Vector3d backColor;

	bool mouseCaptured;

	void resized(wxSizeEvent& evt);
	void shown(wxShowEvent& evt);

	int getWidth() {
		return GetSize().x;
	}
	int getHeight() {
		return GetSize().y;
	}

	void render(wxPaintEvent& evt);

	// events
	void OnMouse(wxMouseEvent& evt);
	void OnCaptureLost(wxMouseCaptureLostEvent& evt);
	void mouseWheelMoved(wxMouseEvent& evt);

DECLARE_EVENT_TABLE()
};

#endif
