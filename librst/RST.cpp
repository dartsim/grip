#include "GUI/RSTFrame.h"
#include "AllTabs.h"
#include "GUI/GUI.h"
#include <wx/icon.h>

#include "GUI/icons/robot.xpm"
#include "RSTApp.h"

IMPLEMENT_APP(RSTApp)

bool RSTApp::OnInit()
{
#ifdef WIN32
	// --- Console ---
	FILE* pFile;
    AllocConsole();
    SetConsoleTitle("RST Console");
    freopen_s(&pFile,"conin$", "r", stdin);
    freopen_s(&pFile,"conout$", "w", stdout);
    freopen_s(&pFile,"conout$", "w", stderr);
#endif
	// --- Console ---

    if ( !wxApp::OnInit() )
        return false;
    frame = new RSTFrame(wxT("RST"));
	frame->SetFocus();

	wxInitAllImageHandlers();
	wxImage rstimg = wxImage(robot_xpm);
	char r,g,b;
	rstimg.InitAlpha();
	for(int i=0; i<16; i++)for(int j=0; j<16; j++){
			r=rstimg.GetRed(i,j);g=rstimg.GetBlue(i,j);b=rstimg.GetGreen(i,j);
			if(r == g && g == b) rstimg.SetAlpha(i,j,255-r);
	}
	wxIcon ico;
	ico.CopyFromBitmap(wxBitmap(rstimg));
	frame->SetIcon(ico);
    frame->Show(true);

    // AddTab();
    return true;
}

