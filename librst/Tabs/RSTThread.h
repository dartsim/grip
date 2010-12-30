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
 * RSTThread.h
 *
 *  Desc: RSTThreads to provide 1 worker thread for each RSTTab.
 *  Usage: Create an RSTThread Object and call CreateThread().
 *         Thread() in RSTTab is called implicitly.
 *         onThreadComplete() in RSTTab is called implicitly.
 *      Author: pushkar
 */

#ifndef RSTTHREAD_H_
#define RSTTHREAD_H_

#include <iostream>
#include <wx/thread.h>
#include <Tabs/RSTTab.h>

enum RSTThreadError
{
	RSTTHREAD_NO_ERROR = 0,      // No error
	RSTTHREAD_RUNNING,           // The thread is already running
	RSTTHREAD_NOT_RUNNING,       // The thread isn't running
	RSTTHREAD_MISC_ERROR         // Some other error
};

class RSTThread: public wxThread {
	RSTTab* tab;
	RSTThreadError state;
public:
	RSTThread(RSTTab* _tab) : wxThread (wxTHREAD_JOINABLE) {
		this->tab = _tab;
		state = RSTTHREAD_NOT_RUNNING;
	}

	~RSTThread() {
		if(tab != NULL) free(tab);
	};

	RSTThreadError getThreadState() {
		return state;
	}

	RSTThreadError CreateThread() {
		if(wxThread::Create() != wxTHREAD_NO_ERROR) {
			cout << "Error creating RST Thread" << endl;
			return RSTTHREAD_MISC_ERROR;
		}

		if(wxThread::Run() != wxTHREAD_NO_ERROR) {
			cout<< "Error starting RST Thread" << endl;
			return RSTTHREAD_MISC_ERROR;
		}

		return RSTTHREAD_NO_ERROR;
	}

	void StopThread() {
		if(state == RSTTHREAD_RUNNING) {
			wxThread::Delete();
			state = RSTTHREAD_NOT_RUNNING;
		}
		// Even though this is a Joinable Thread, cannot call Wait(). Use Delete() or Wait().
	}

	bool CheckPoint() {
		return wxThread::TestDestroy();
	}

	virtual wxThread::ExitCode Entry()
	{
		state = RSTTHREAD_RUNNING;
		tab->Thread();
		state = RSTTHREAD_NOT_RUNNING;
		tab->onThreadComplete();
		return (wxThread::ExitCode)0;
	}
};

#endif /* RSTTHREAD_H_ */
