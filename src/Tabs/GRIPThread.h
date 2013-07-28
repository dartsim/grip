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

/*
 * GRIPThread.h
 *
 *  Desc: GRIPThreads to provide 1 worker thread for each GRIPTab.
 *  Usage: Create an GRIPThread Object and call CreateThread().
 *         Thread() in GRIPTab is called implicitly.
 *         onThreadComplete() in GRIPTab is called implicitly.
 *      Author: pushkar
 */

#ifndef GRIPTHREAD_H_
#define GRIPTHREAD_H_

#include <iostream>
#include <wx/thread.h>
#include <Tabs/GRIPTab.h>

enum GRIPThreadError
{
	GRIPTHREAD_NO_ERROR = 0,      // No error
	GRIPTHREAD_RUNNING,           // The thread is already running
	GRIPTHREAD_NOT_RUNNING,       // The thread isn't running
	GRIPTHREAD_MISC_ERROR         // Some other error
};

static const char* gripThreadErrorToString(GRIPThreadError _num) {
	switch(_num) {
	case GRIPTHREAD_NO_ERROR: return "GripThread - No error"; break;
	case GRIPTHREAD_RUNNING: return "GripThread - Running"; break;
	case GRIPTHREAD_NOT_RUNNING: return "GripThread - Not Running"; break;
	case GRIPTHREAD_MISC_ERROR: return "GripThread - Misc Error"; break;
	default: return "GripThread - Default, no Error";
	}
}

class GRIPThread: public wxThread {
	GRIPTab* tab;
	GRIPThreadError state;
public:
	GRIPThread(GRIPTab* _tab) : wxThread (wxTHREAD_JOINABLE) {
		this->tab = _tab;
		state = GRIPTHREAD_NOT_RUNNING;
	}

	~GRIPThread() {
		if(tab != NULL) free(tab);
	};

	GRIPThreadError getThreadState() {
		return state;
	}

	GRIPThreadError CreateThread() {
		if(wxThread::Create() != wxTHREAD_NO_ERROR) {
			std::cout << "Error creating GRIP Thread" << std::endl;
			return GRIPTHREAD_MISC_ERROR;
		}

		if(wxThread::Run() != wxTHREAD_NO_ERROR) {
			std::cout<< "Error starting GRIP Thread" << std::endl;
			return GRIPTHREAD_MISC_ERROR;
		}

		return GRIPTHREAD_NO_ERROR;
	}

	void StopThread() {
		if(state == GRIPTHREAD_RUNNING) {
			wxThread::Delete();
			state = GRIPTHREAD_NOT_RUNNING;
		}
		// Even though this is a Joinable Thread, cannot call Wait(). Use Delete() or Wait().
	}

	bool CheckPoint() {
		return wxThread::TestDestroy();
	}

	virtual wxThread::ExitCode Entry()
	{
		state = GRIPTHREAD_RUNNING;
		tab->Thread();
		state = GRIPTHREAD_NOT_RUNNING;
		tab->onThreadComplete();
		return (wxThread::ExitCode)0;
	}
};

#endif /* GRIPTHREAD_H_ */
