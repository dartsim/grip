#include "RSTApp.h"
#include "EmptyTab.h"
#include <Tabs/PlanningTab.h>

extern wxNotebook* tabView;

class EmptyTabApp : public RSTApp {
	virtual void AddTabs() {
		tabView->AddPage(new EmptyTab(tabView), wxT("EmptyTab"));
		tabView->AddPage(new PlanningTab(tabView), wxT("PlanningTab"));
	}
};

IMPLEMENT_APP(EmptyTabApp)