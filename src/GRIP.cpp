/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * 
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <GUI/GRIPFrame.h>
#include <Tabs/AllTabs.h>
#include <GUI/GUI.h>
#include <wx/icon.h>
#include <wx/cmdline.h>

#include "GUI/icons/robot.xpm"
#include "GRIPApp.h"

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

/**
 * @function setConfiguration
 * @brief Reads the joint indices and angles from a file. The first line is indices,
 * the second is angles.
 * @date 2014-02-10
 */
void GRIPApp::setConfiguration (const char* filepath) {

  // Open the file
  fstream file (filepath);
  assert(file.is_open() && "Could not open the file!");

  // Read the first line
  std::string line;
	assert(getline(file, line) && "Two lines: first indices, then angles.");
	int index;
	vector <int> dofs;
	stringstream stream (line, stringstream::in);
	while(stream >> index) dofs.push_back(index);

	// Read the second line
	Eigen::VectorXd q (dofs.size());
	double angle;
	size_t i = 0;
	assert(getline(file, line) && "Two lines: first indices, then angles.");
	stringstream stream2 (line, stringstream::in);
	while(stream2 >> angle) q(i++) = angle;
	
	// Set the robot configuration
	mWorld->getSkeleton("Krang")->setConfig(dofs, q);
}

/**
 * @function processArgs 
 * @brief Processes the -f argument to get the input scene file
 * @date 2014-01-29
 */
void GRIPApp::processArgs (){
 
	// Setup the items to check in the command line
  wxCmdLineEntryDesc cmdLineDesc[] = {
   { wxCMD_LINE_OPTION, wxT("f"), },
   { wxCMD_LINE_OPTION, wxT("q"), },
   { wxCMD_LINE_NONE }
  };
 
	// Parse the arguments
  wxCmdLineParser parser (cmdLineDesc, argc, argv);
  parser.Parse();
 
	// Check if the file option is given; if so, load the file
  wxString optf;
	bool loadedFile = false;
  if(parser.Found( wxT("f"), &optf)) {
		std::cout << "Will try to load the file: '" << optf.mb_str() << "'" << std::endl;
		frame->DoLoad(std::string(optf.mb_str()));
		loadedFile = true;
	}
  if(parser.Found( wxT("q"), &optf)) {
		assert(loadedFile && "Can not load configuration unless urdf file is given with 'f'");
		std::cout << "Will try to load the configuration file: '" << optf.mb_str() << "'" << std::endl;
		setConfiguration(optf.mb_str());
	}
	
 }
 
/**
 * @function OnInit
 * @brief Initialize GRIP window
 * @date 2011-10-13
 */
bool GRIPApp::OnInit()
{
#ifdef WIN32
	// --- Console ---
	FILE* pFile;
    AllocConsole();
    SetConsoleTitle(L"GRIP Console");
    freopen_s( &pFile, "conin$", "r", stdin );
    freopen_s( &pFile, "conout$", "w", stdout );
    freopen_s( &pFile, "conout$", "w", stderr );
#endif
	// --- Console ---

 // if ( !wxApp::OnInit() ) return false;
  frame = new GRIPFrame(wxT("GRIP"));
	frame->SetFocus();

	AddTabs();

	wxInitAllImageHandlers();
	wxImage gripimg = wxImage(robot_xpm);
	char r,g,b;
	gripimg.InitAlpha();
	for(int i=0; i<16; i++)for(int j=0; j<16; j++){
			r=gripimg.GetRed(i,j);g=gripimg.GetBlue(i,j);b=gripimg.GetGreen(i,j);
			if(r == g && g == b) gripimg.SetAlpha(i,j,255-r);
	}
	wxIcon ico;
	ico.CopyFromBitmap(wxBitmap(gripimg));
	frame->SetIcon(ico);
    frame->Show(true);

	processArgs();
	return true;
}

