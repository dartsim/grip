#include "RSTApp.h"
#include "EmptyTab.h"

extern wxNotebook* tabView;

class EmptyTabApp : public RSTApp {
	virtual void AddTabs() {
		tabView->AddPage(new EmptyTab(tabView), wxT("EmptyTab"));
	}
};

IMPLEMENT_APP(EmptyTabApp)