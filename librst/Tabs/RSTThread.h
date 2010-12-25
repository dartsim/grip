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
