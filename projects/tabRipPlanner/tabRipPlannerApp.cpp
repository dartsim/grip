#include "GRIPApp.h"
#include "tabRipPlanner.h"

extern wxNotebook* tabView;

class RipPlannerTabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new RipPlannerTab(tabView), wxT("RIP Planner"));
	}
};

IMPLEMENT_APP(RipPlannerTabApp)
